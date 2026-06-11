#!/usr/bin/env python3
"""
手眼标定可视化 — 查看基座/法兰/相机/盒子的相对位置
用法: python3 debug_visualize.py
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def rpy_to_rot(rx_deg, ry_deg, rz_deg):
    """ZYX extrinsic"""
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz),  np.cos(rz), 0],
                    [0, 0, 1]])
    Ry = np.array([[ np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
    Rx = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx),  np.cos(rx)]])
    return Rz @ Ry @ Rx


def draw_axes(ax, T, size=0.1, alpha=0.8):
    """在 4x4 变换矩阵位置画 RGB=XYZ 轴"""
    origin = T[:3, 3]
    ax.quiver(*origin, *T[:3, 0], color='r', length=size, alpha=alpha, arrow_length_ratio=0.2)
    ax.quiver(*origin, *T[:3, 1], color='g', length=size, alpha=alpha, arrow_length_ratio=0.2)
    ax.quiver(*origin, *T[:3, 2], color='b', length=size, alpha=alpha, arrow_length_ratio=0.2)


def main():
    # ── 参数 (从调试输出搬来) ──
    flange_pose = (-344, -6, 676, 142, 84, -51)  # mm, 度
    T_flange_to_cam = np.array([
        [ 0.05119370554924858,  0.984925105924302,  0.16523238251668163, -0.49634003057303844],
        [ 0.009795782454369852,  0.1649461496107451, -0.9862539279387924,  0.667787464500021],
        [-0.9986406997305104,   0.05210857365723904,-0.001203907456826879, 0.3855367282009634],
        [ 0.0,                   0.0,                 0.0,                   1.0],
    ])
    p_cam = np.array([-0.170, 0.011, 0.291])  # 盒子在相机坐标系
    hand_length = 0.35  # 法兰 → 指尖

    # ── 变换 ──
    T_base_flange = np.eye(4)
    T_base_flange[:3, :3] = rpy_to_rot(*flange_pose[3:])
    T_base_flange[:3, 3] = np.array(flange_pose[:3]) / 1000.0

    T_base_cam = T_base_flange @ T_flange_to_cam
    cam_world = T_base_cam[:3, 3]

    p_cam_h = np.append(p_cam, 1.0)
    box_world = (T_base_cam @ p_cam_h)[:3]

    print("=" * 50)
    print("  手眼标定可视化")
    print("=" * 50)
    print(f"基座原点:    (0, 0, 0)m")
    print(f"法兰在基座:  {T_base_flange[:3,3]*1000} mm")
    print(f"相机在基座:  {cam_world*1000} mm")
    print(f"盒子在基座:  {box_world*1000} mm")
    print(f"相机→盒子距离: {np.linalg.norm(p_cam):.3f}m")
    print(f"法兰→盒子距离: {np.linalg.norm((T_flange_to_cam @ p_cam_h)[:3]):.3f}m")

    # ── 3D 图 ──
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # 基座 (世界原点)
    ax.scatter([0], [0], [0], c='black', s=80, marker='s', label='Base')
    draw_axes(ax, np.eye(4), size=0.15, alpha=0.9)

    # 法兰
    ax.scatter(*T_base_flange[:3, 3], c='orange', s=60, marker='^', label='Flange')
    draw_axes(ax, T_base_flange, size=0.1, alpha=0.6)

    # 相机 (蓝点 + 光轴)
    ax.scatter(*cam_world, c='blue', s=60, label='Camera')
    draw_axes(ax, T_base_cam, size=0.08, alpha=0.7)

    # 盒子
    ax.scatter(*box_world, c='red', s=100, marker='D', label='Box')
    ax.text(*box_world, ' BOX', fontsize=10, color='red')

    # 连线
    # 基座 → 法兰
    ax.plot([0, T_base_flange[0,3]], [0, T_base_flange[1,3]], [0, T_base_flange[2,3]],
            'orange', linestyle='--', linewidth=1, alpha=0.5)
    # 法兰 → 相机
    ax.plot([T_base_flange[0,3], cam_world[0]],
            [T_base_flange[1,3], cam_world[1]],
            [T_base_flange[2,3], cam_world[2]],
            'blue', linestyle='--', linewidth=1, alpha=0.5)
    # 相机 → 盒子
    ax.plot([cam_world[0], box_world[0]],
            [cam_world[1], box_world[1]],
            [cam_world[2], box_world[2]],
            'red', linewidth=1.5, alpha=0.7)

    # Nova5 工作空间球（臂展 ~817mm, 基座底部以上）
    u = np.linspace(0, 2*np.pi, 20)
    v = np.linspace(np.pi/4, np.pi/2, 10)  # 上半球，基座下方不可达
    R = 0.817
    xs = R * np.outer(np.cos(u), np.sin(v))
    ys = R * np.outer(np.sin(u), np.sin(v))
    zs = R * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_wireframe(xs, ys, zs, color='gray', alpha=0.15, linewidth=0.3)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('hand-eye calib visualization\nNova5 reach = 817mm sphere')
    ax.legend()
    ax.set_box_aspect([1, 1, 1])

    # 固定视角: 侧视 (XZ 平面)
    ax.view_init(elev=20, azim=-90)

    plt.tight_layout()
    plt.savefig('/home/nvidia/grasp/dipan/calib_debug_3d.png', dpi=150)
    print(f"\n已保存图片: dipan/calib_debug_3d.png")

    # 俯视图
    fig2, ax2 = plt.subplots(figsize=(8, 8))
    ax2.set_aspect('equal')
    ax2.scatter([0], [0], c='black', s=100, marker='s', label='Base (0,0)')
    ax2.scatter(T_base_flange[0,3], T_base_flange[1,3], c='orange', s=60, marker='^', label='Flange')
    ax2.scatter(cam_world[0], cam_world[1], c='blue', s=60, label='Camera')
    ax2.scatter(box_world[0], box_world[1], c='red', s=100, marker='D', label='Box')
    # 工作空间圆
    circle = plt.Circle((0, 0), 0.817, fill=False, color='gray', linestyle='--', alpha=0.5)
    ax2.add_patch(circle)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('Top View (XY plane)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('/home/nvidia/grasp/dipan/calib_debug_top.png', dpi=150)
    print(f"已保存图片: dipan/calib_debug_top.png")

    plt.show()


if __name__ == "__main__":
    main()
