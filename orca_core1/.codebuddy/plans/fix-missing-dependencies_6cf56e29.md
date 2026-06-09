---
name: fix-missing-dependencies
overview: 修复 pyproject.toml 中缺失的 pyserial 依赖声明，并指导用户安装所有项目依赖
todos:
  - id: add-pyserial-dep
    content: 在 pyproject.toml 的 dependencies 中添加 pyserial (>=3.5,<4.0.0)
    status: completed
---

## 用户需求

运行 `python scripts/tension.py` 时遇到两个 `ModuleNotFoundError`：

1. `No module named 'dynamixel_sdk'` — 已在 pyproject.toml 声明但未安装到当前环境
2. `No module named 'serial'` — 即 pyserial 包，代码中使用了但未在 pyproject.toml 中声明为依赖

## 核心修复

- 在 `pyproject.toml` 的 dependencies 中添加缺失的 `pyserial` 依赖声明
- 确保用户重新安装项目依赖以解决两个缺失模块的问题

## 技术栈

- 项目：Python 包，使用 hatchling 构建
- 依赖管理：`pyproject.toml` 声明式依赖

## 实现方案

在 `pyproject.toml` 的 `dependencies` 列表中添加 `pyserial (>=3.5,<4.0.0)`。

`pyserial` 被以下位置使用：

- `orca_core/hardware/feetech/port_handler.py:4` — 顶级 `import serial`（Feetech 电机通信底层）
- `orca_core/utils/utils.py:215` — `auto_detect_port()` 内懒导入 `serial.tools.list_ports`
- `orca_core/utils/utils.py:242` — `get_and_choose_port()` 内懒导入 `serial.tools.list_ports`

pyserial 3.5 是目前最新的 3.x 稳定版，且项目使用了其标准 API（`serial.tools.list_ports`），兼容性无风险。

## 修改文件

```
d:\orca_core\
└── pyproject.toml  # [MODIFY] 第11-17行 dependencies 列表，添加 pyserial 依赖
```

## 实施备注

- `dynamixel_sdk` 已在 pyproject.toml 中声明，只是未安装。添加 pyserial 后用户需运行 `pip install -e .` 一次性安装所有依赖
- pyserial 版本约束 `>=3.5,<4.0.0` 与项目现有风格一致