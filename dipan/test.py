python3 -c "
import os, sys, time
sys.path.insert(0, '/home/nvidia/grasp/dipan')
from lhandpro_gripper import LHandProGripper

hand = LHandProGripper(port='/dev/ttyUSB0')
hand.connect(enable_motors=True, home_motors=False)  # 使能但跳过回零
sdk = hand.controller.sdk_handle

# 看看能不能跳过回零检查直接运动
print('尝试设置 set_move_no_home(1) 跳过回零检查...')
sdk.set_move_no_home(1)

# 手动回零单个轴
print('尝试逐个轴回零:')
for axis in range(1, 7):
    print(f'  home axis{axis}...')
    sdk.home_motors(axis)
    time.sleep(3)
    pos = sdk.get_now_position(axis)
    status = sdk.get_now_status(axis)
    print(f'    position={pos}, status={status}')

print('测试完成')
hand.disconnect()
"
