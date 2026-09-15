python3 -c "
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from lhandpro_gripper import LHandProGripper

hand = LHandProGripper(port='/dev/ttyUSB0')
hand.connect(enable_motors=True, home_motors=False)  # ʹ�ܵ���������
sdk = hand.controller.sdk_handle

# �����ܲ�������������ֱ���˶�
print('�������� set_move_no_home(1) ����������...')
sdk.set_move_no_home(1)

# �ֶ����㵥����
print('������������:')
for axis in range(1, 7):
    print(f'  home axis{axis}...')
    sdk.home_motors(axis)
    time.sleep(3)
    pos = sdk.get_now_position(axis)
    status = sdk.get_now_status(axis)
    print(f'    position={pos}, status={status}')

print('�������')
hand.disconnect()
"
