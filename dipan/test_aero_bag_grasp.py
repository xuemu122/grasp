#!/usr/bin/env python3
"""非膨化袋装物品柔性抓取测试.

复用 test_aero_small_grasp.py 的流程, 只换一套初始位与参数
(dipan/config/bag_grasp.json):
  0 thumb_abduction  80.3  固定
  1 thumb_flex       35.2  固定
  2 thumb_tendon      0.0  跟随递增
  3 index_finger     32.0  柔性抓取
  4 middle_finger    36.0  柔性抓取
  5 ring_finger      36.9  柔性抓取
  6 pinky_finger     22.2  柔性抓取

先空手采同一姿态曲线:
  python3 test_aero_bag_grasp.py --record-profile runtime/aero_bag_empty.csv \
      --max 90 --thumb-max 30 --step 2 --fast --restore-after
再抓:
  python3 test_aero_bag_grasp.py
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("AERO_SMALL_CONFIG", os.path.join(HERE, "config", "bag_grasp.json"))

from test_aero_small_grasp import main  # noqa: E402


if __name__ == "__main__":
    main()
