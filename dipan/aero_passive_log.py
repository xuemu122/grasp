"""SDK logging with optional idle-only feedback reads; never sends motion itself."""
import atexit
import json
import math
import os
from pathlib import Path
import queue
import threading
import time
import uuid


class PassiveLog:
    def __init__(self, directory):
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=True)
        self.path = root / ('hand_' + time.strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:8] + '.jsonl')
        self.stream = self.path.open('x', encoding='utf-8')
        self.queue = queue.Queue(maxsize=2048)
        self.closed = threading.Event()
        self.dropped = 0
        self.error = None
        self.thread = threading.Thread(target=self._write, daemon=True, name='aero-passive-log')
        self.thread.start()
        atexit.register(self.close)
        self.emit({'event': 'log_start', 'time_basis': 'host_monotonic_seconds',
                   'note': 'Commands are not measured position. Optional idle reads require a separate enable event.'})

    def emit(self, row):
        if self.closed.is_set() or self.error:
            return
        try:
            self.queue.put_nowait(dict(row, dropped_records=self.dropped))
        except queue.Full:
            self.dropped += 1

    def _write(self):
        try:
            while not self.closed.is_set() or not self.queue.empty():
                try:
                    row = self.queue.get(timeout=.1)
                except queue.Empty:
                    continue
                self.stream.write(json.dumps(row, ensure_ascii=True, allow_nan=False)+'\n')
                self.stream.flush()
        except Exception as exc:
            self.error = str(exc)
            print('[Aero log] Disk logging failed; original control remains unchanged:', exc, flush=True)
        finally:
            self.stream.close()

    def close(self):
        self.closed.set()
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=2.)


class ObservedHand:
    METHODS = {'get_actuator_currents': 'current_mA',
               'get_actuator_temperatures': 'temperature_C',
               'get_actuations': 'motor_position',
               'set_joint_positions': 'command_joint_deg'}

    def __init__(self, hand, logger):
        self._hand, self._logger = hand, logger
        self._last_command = time.monotonic()
        self._feedback_stop = threading.Event()
        self._feedback_thread = None

    def start_idle_feedback(self, period_s=1., quiet_s=1., slow_read_s=.05):
        if self._feedback_thread is not None:
            raise RuntimeError('Feedback sampler already started')
        if not all(math.isfinite(x) for x in (period_s, quiet_s, slow_read_s)) or period_s < 1 or quiet_s < .5 or slow_read_s <= 0:
            raise ValueError('Require period >= 1s, quiet >= 0.5s, positive read budget')
        lock = getattr(self._hand, '_serial_lock', None)
        # The SDK takes this same lock internally; it MUST be reentrant.
        if lock is None or not lock.acquire(blocking=False):
            raise RuntimeError('Shared SDK lock unavailable; idle sampler refused')
        try:
            if not lock.acquire(blocking=False):
                raise RuntimeError('SDK lock is not reentrant; idle sampler refused')
            lock.release()
        finally:
            lock.release()
        self._feedback_stop.clear()
        self._feedback_thread = threading.Thread(target=self._idle_feedback,
            args=(lock, period_s, quiet_s, slow_read_s), daemon=True, name='aero-idle-feedback')
        self._feedback_thread.start()
        self._logger.emit({'event': 'idle_feedback_enabled', 'period_per_group_s': period_s,
                           'quiet_required_s': quiet_s, 'slow_read_budget_s': slow_read_s,
                           'note': 'Pauses during repeated commands; NOT a real-time protection system'})
        print('[Aero log] Idle feedback enabled; pauses during repeated position commands.', flush=True)

    def _idle_feedback(self, lock, period, quiet, budget):
        methods = ('get_actuations', 'get_actuator_currents', 'get_actuator_temperatures')
        index = 0
        while not self._feedback_stop.wait(period/3):
            if time.monotonic()-self._last_command < quiet:
                continue
            if not lock.acquire(blocking=False):
                continue
            started = time.monotonic()
            try:
                if self._feedback_stop.is_set() or started-self._last_command < quiet:
                    continue
                name = methods[index % 3]
                values = getattr(self, name)()
                elapsed = time.monotonic()-started
                valid = values is not None and len(values) == 7 and all(math.isfinite(float(x)) for x in values)
                self._logger.emit({'event': 'idle_feedback_timing', 'method': name,
                    'host_monotonic_s': time.monotonic(), 'elapsed_s': elapsed, 'valid': valid})
                if not valid or elapsed > budget:
                    raise RuntimeError('Invalid feedback or diagnostic read exceeded %.3fs budget (%.3fs)' % (budget, elapsed))
                index += 1
            except Exception as exc:
                self._logger.emit({'event': 'idle_feedback_stopped', 'host_monotonic_s': time.monotonic(), 'error': str(exc)})
                print('[Aero log] Diagnostic reads stopped; original control unchanged:', exc, flush=True)
                self._feedback_stop.set()
            finally:
                lock.release()

    def _stop_feedback(self):
        self._feedback_stop.set()
        if self._feedback_thread is not None and self._feedback_thread is not threading.current_thread():
            self._feedback_thread.join(timeout=2.)

    def __getattr__(self, name):
        original = getattr(self._hand, name)
        if name == 'close':
            def close(*args, **kwargs):
                self._stop_feedback()
                try:
                    return original(*args, **kwargs)
                finally:
                    self._logger.emit({'event': 'serial_close', 'host_monotonic_s': time.monotonic(),
                                       'note': 'Not motor power-off'})
                    self._logger.close()
            return close
        if name not in self.METHODS:
            return original

        def observed(*args, **kwargs):
            started = time.monotonic()
            if name.startswith('set_'):
                self._last_command = started
            try:
                result = original(*args, **kwargs)
            except Exception as exc:
                self._logger.emit({'event': 'sdk_error', 'method': name,
                                   'host_monotonic_s': time.monotonic(), 'error': str(exc)})
                raise
            # Logging must not change the SDK result or exception semantics.
            try:
                row = dict(event='command' if name.startswith('set_') else 'feedback',
                           method=name, request_start_s=started, host_monotonic_s=time.monotonic())
                row['call_elapsed_s'] = row['host_monotonic_s']-started
                value = (args[0] if args else kwargs.get('positions')) if name.startswith('set_') else result
                row[self.METHODS[name]] = None if value is None else [float(x) for x in value]
                if row[self.METHODS[name]] is not None and not all(math.isfinite(x) for x in row[self.METHODS[name]]):
                    raise ValueError('Nonfinite SDK values omitted')
                if name.startswith('set_'):
                    row['reported_success'] = result
                self._logger.emit(row)
            except Exception:
                self._logger.dropped += 1
            return result
        return observed


def observe_hand(hand):
    directory = os.getenv('AERO_PASSIVE_LOG_DIR', '').strip()
    if not directory:
        return hand
    try:
        logger = PassiveLog(directory)
        print('[Aero log] Passive recording:', logger.path, flush=True)
        observed = ObservedHand(hand, logger)
        if os.getenv('AERO_IDLE_FEEDBACK_ENABLE', '0').lower() in ('1', 'true', 'yes', 'on'):
            try:
                observed.start_idle_feedback()
            except Exception as exc:
                print('[Aero log] Idle feedback refused; passive logging retained:', exc, flush=True)
        return observed
    except Exception as exc:
        print('[Aero log] Cannot start recording; original control retained:', exc, flush=True)
        return hand
