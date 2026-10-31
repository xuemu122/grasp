"""Read saved CSV/JSONL logs and preview K/D schedules. Never accesses hardware."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics

CHANNELS = ['thumb_cmc_abd', 'thumb_cmc_flex', 'thumb_tendon',
            'index', 'middle', 'ring', 'pinky']


def number(value):
    try:
        x = float(value)
        return x if math.isfinite(x) else None
    except (ValueError, TypeError):
        return None


def load(path):
    # Snapshot the source once; no writes to the input or serial/device paths.
    content = path.read_bytes()
    rows, skipped = [], 0
    text = content.decode('utf-8-sig')
    if path.suffix.lower() == '.csv':
        import io
        for row in csv.DictReader(io.StringIO(text)):
            rows.append(dict(time=number(row.get('cycle_end_s', row.get('time_s'))),
                phase=row.get('phase', ''),
                **{g: [number(row.get(g+'_'+c)) for c in CHANNELS]
                   if row.get(g+'_ok', '1') == '1' else [None]*7
                   for g in ('current_mA', 'actuator_deg', 'temperature_C')}))
    else:
        for line in text.splitlines():
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                skipped += 1
                continue
            row = event.get('feedback', event)
            if not any(key in row for key in ('current_mA', 'temperature_C', 'motor_position')):
                continue
            result = dict(time=number(row.get('host_monotonic_s')), phase=event.get('event', ''))
            for target, key in [('current_mA', 'current_mA'), ('actuator_deg', 'motor_position'),
                                ('temperature_C', 'temperature_C')]:
                values = row.get(key, [])
                result[target] = [number(x) for x in values] if isinstance(values, list) and len(values) == 7 else [None]*7
            rows.append(result)
    valid = [r for r in rows if r['time'] is not None]
    skipped += len(rows)-len(valid)
    if not valid:
        raise ValueError('No readable samples. Supply samples.csv or hand_console.jsonl.')
    return valid, skipped, hashlib.sha256(content).hexdigest()


def analyze(rows):
    times = [r['time'] for r in rows]
    gaps = [b-a for a, b in zip(times, times[1:])]
    positive = [x for x in gaps if x > 0]
    metrics = dict(samples=len(rows), duration_s=times[-1]-times[0],
                   strictly_increasing=all(x > 0 for x in gaps),
                   median_sample_gap_s=statistics.median(positive) if positive else None,
                   max_sample_gap_s=max(positive) if positive else None, channels=[])
    for i, name in enumerate(CHANNELS):
        curr = [(r['time'], r['current_mA'][i]) for r in rows if r['current_mA'][i] is not None]
        temps = [r['temperature_C'][i] for r in rows if r['temperature_C'][i] is not None]
        peak = max(curr, key=lambda x: abs(x[1])) if curr else None
        metrics['channels'].append(dict(channel=name, valid_current_samples=len(curr),
            missing_current_samples=len(rows)-len(curr), peak_abs_current_mA=abs(peak[1]) if peak else None,
            peak_time_s=peak[0]-times[0] if peak else None,
            mean_abs_current_mA=statistics.mean(abs(x[1]) for x in curr) if curr else None,
            max_temperature_C=max(temps) if temps else None))
    return metrics


def gains(t, config, method):
    high, low, rate, tau0, mass, zeta, switch = [float(config[k]) for k in
        ('K_high_N_per_m', 'K_low_N_per_m', 'K_rate_max_N_per_m_s',
         'tau0_s', 'equivalent_mass_kg', 'zeta', 'switch_time_s')]
    if not all(math.isfinite(x) for x in (high, low, rate, tau0, mass, zeta, switch)) or not high > low > 0 or min(rate, tau0, mass, zeta) <= 0 or switch < 0:
        raise ValueError('Invalid schedule configuration')
    tau = max(tau0, 4*(high-low)/(3*math.sqrt(3)*rate))
    elapsed = t-switch
    if elapsed < 0:
        K = high
    elif method == 'direct':
        K = low
    elif method == 'linear':
        K = high-(high-low)*min(1., elapsed/(3*math.sqrt(3)*tau/4))
    else:
        K = low+(high-low)*(1-math.tanh(elapsed/tau)**2)
    return K, 2*zeta*math.sqrt(mass*K)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('input', type=Path, nargs='?')
    p.add_argument('--demo', action='store_true', help='Synthetic software check, NOT an experiment')
    p.add_argument('--output', type=Path, required=True, help='New output directory')
    p.add_argument('--gain-config', type=Path, help='Optional existing schedule JSON; no motor commands generated')
    args = p.parse_args()
    if bool(args.input) == args.demo:
        p.error('Choose an input log OR --demo')
    if args.demo:
        rows = [dict(time=i*.02, phase='SYNTHETIC_DEMO', current_mA=[100+j*10+5*math.sin(i*.02) for j in range(7)],
                     actuator_deg=[j+math.sin(i*.02) for j in range(7)], temperature_C=[30]*7) for i in range(301)]
        skipped, digest = 0, None
    else:
        rows, skipped, digest = load(args.input)
    result = analyze(rows)
    result.update(synthetic_demo=args.demo, skipped_rows=skipped, input_sha256=digest,
                  hardware_access=False, contact_force_peak_N=None, task_success=None,
                  note='Current is NOT contact force. Schedule preview is NOT an executed experiment.')
    config = json.loads(args.gain_config.read_text(encoding='utf-8-sig')) if args.gain_config else None
    times = [r['time']-rows[0]['time'] for r in rows]
    preview = []
    if config:
        for t in times:
            for method in ('direct', 'linear', 'smooth'):
                K, D = gains(t, config, method)
                preview.append(dict(time_s=t, method=method, K_N_per_m=K, D_N_s_per_m=D))
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'summary.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if config:
        (args.output/'gain_config.json').write_text(json.dumps(config, indent=2), encoding='utf-8')
        with (args.output/'gain_preview.csv').open('w', newline='', encoding='utf-8') as f:
            w = csv.DictWriter(f, fieldnames=list(preview[0])); w.writeheader(); w.writerows(preview)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for ax, group, label in zip(axes, ('current_mA', 'actuator_deg', 'temperature_C'),
                               ('Motor current (mA)', 'SDK actuation position (deg)', 'Temperature (C)')):
        for i, ch in enumerate(CHANNELS):
            ax.plot(times, [r[group][i] if r[group][i] is not None else math.nan for r in rows], label=ch)
        ax.set_ylabel(label); ax.grid(alpha=.2)
    axes[0].legend(ncol=3, fontsize=8)
    axes[-1].set_xlabel('Time from first saved sample (s)')
    fig.suptitle('SYNTHETIC SOFTWARE DEMO - NOT EXPERIMENTAL DATA' if args.demo else 'Saved motor feedback - no hardware access')
    fig.tight_layout(); fig.savefig(args.output/'feedback.png', dpi=160); plt.close(fig)
    if config:
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for method in ('direct', 'linear', 'smooth'):
            data = [r for r in preview if r['method'] == method]
            for ax, key in zip(axes, ('K_N_per_m', 'D_N_s_per_m')):
                ax.plot(times, [r[key] for r in data], label=method)
        axes[0].set_ylabel('K (N/m)'); axes[1].set_ylabel('D (N s/m)')
        axes[0].legend(); axes[1].set_xlabel('Time from first saved sample (s)')
        fig.suptitle('Offline schedule calculation - not motor execution')
        fig.tight_layout(); fig.savefig(args.output/'gain_preview.png', dpi=160); plt.close(fig)
    print('Finished. No serial port or SDK accessed. Output:', args.output.resolve())
    print('Samples:', len(rows), '| Timestamp order valid:', result['strictly_increasing'])
    if not config:
        print('K/D preview omitted: no gain configuration supplied.')


if __name__ == '__main__':
    main()
