# Gimbal_2026

二自由度舵机云台 + K230 人体目标跟踪（云台主控板）。

> 实验室学习项目《STM32 双模式智能小车》的**云台主控**部分：
> 只负责「云台人体跟踪」与「K230 视觉识别人体」。

## 基本信息

| 项目 | 内容 |
|---|---|
| 主控 | STM32F103C8T6（64 KB Flash / 20 KB RAM，Cortex-M3） |
| 执行机构 | 二自由度舵机云台：左右旋转 270° + 上下俯仰 180° |
| 视觉 | 正点原子 K230D BOX，YOLOv5n 人体检测 |
| 通信 | K230 → STM32：USART2，115200-8-N-1，13 字节定长帧；→ 循迹板：USART1；← 蓝牙：USART3（JDY-31） |
| 显示 | 0.96" OLED（SSD1306，软件 I2C） |
| 构建 | CMake + Ninja + arm-none-eabi-gcc + OpenOCD |

## 目录

```text
Gimbal_2026/
├─ application/     应用逻辑（云台跟踪任务 GIMBAL_TASK）
├─ modules/         设备/算法模块（视觉协议、云台控制律、OLED）
├─ bsp/             片上外设封装（USART2、TIM1 舵机 PWM、软件 I2C）
├─ config/          板级与参数配置（board / gimbal / vision）
├─ Core/            main.c 与中断向量实现
├─ Drivers/         CMSIS + ST 标准外设库（仅用到的子集）
├─ System/          Delay（SysTick 延时 + 毫秒时基）
├─ cmake/           交叉编译工具链文件
├─ docs/            规范、技术交付文档、引脚与协议说明
├─ tests/           模块测试与台架测试记录
└─ .vscode/         VS Code 工程设置
```

## 快速开始

```powershell
# 编译（生成 build-cmake/Gimbal_2026.elf|.hex|.bin）
cmake --preset f103
cmake --build --preset f103

# 或者直接用 Make 包装
make            # 等价于上面两步
make download   # 编译 + OpenOCD/ST-Link 烧录
```

VS Code 里：`Ctrl+Shift+B` 运行默认任务 `CMake: build`；F5 用 `Debug STM32` 调试。

## 开发规范（强制）

1. 分层依赖：`application -> modules -> bsp -> 寄存器`，**禁止反向依赖**；
2. `application/` **禁止**出现 `GPIO_*` / `TIM_*` / `USART_*` 等寄存器或外设库调用；
3. 所有物理量变量名/注释必须带单位（`_deg` / `_px` / `_ms` / `_s`）；
4. 禁止魔法值，参数一律集中到 `config/`；
5. 控制输出必须限幅（角度行程 + 脉宽双层限幅）；
6. 修改公共接口、控制参数、安全逻辑必须同步更新 `docs/技术交付文档.md`；
7. 先给计划，人工确认后再改代码；小步修改，编译通过再上板。

## Agent 开发前必须阅读

- `docs/电控C_Cpp开发规范_v0.1.md`（或上级目录 `../docs/`）
- `docs/在Agent情况下如何进行电控开发.md`
- `docs/技术交付文档.md`
- `docs/引脚分配.md`
- `docs/K230通信协议.md`

## 当前状态

- ✅ 三层结构 + CMake 构建（`-Wall -Wextra -Werror`，0 warning 0 error）
- ✅ 位置式 PID：低通 + 软死区 + 行程限幅，参数沿用已验证的 `Gimbal_1.0`
- ✅ 13 字节协议滑窗解析（抗丢字节 / 抗噪声重同步）
- ✅ 目标丢失：保持角度不动（不盲转、不回中）
- ✅ OLED 显示角度/偏差/状态/帧数
- ✅ **双板链路发送**：每 30 ms 把「云台 yaw 偏角」发给循迹板，供模式 2 转向
- ✅ **速率限幅**：每轴 60°/s，防甩头把 K230 甩丢目标
- ✅ **输出死区 0.3°（累积式）**：消 Kd 放大的画面抖动，又不留稳态误差
- ✅ **限位反向扫视**：撞端点后反向扫，上电 8 s 内 / 没见过人之前不扫，且**仅模式 1** 才扫
- ✅ **底盘零点标定** `GIMBAL_CHASSIS_ZERO_DEG = 109.1`（摄像头装歪 6.8° 的补偿）
- ✅ **蓝牙命令**：`mode1/mode2/stop`、`dump/get/set/f/save/reset`、`bz on|off`、`yaw/pitch <角度>`
- ✅ **蜂鸣器**（PB4，低电平触发，已关 JTAG）：起跑/丢线/十字停车/断链 4 种音型
- ✅ **协议扩展**：0x03 调参 / 0x04 应答 / 0x05 提示音 / 0x06 模式转发，USART1 上文本与二进制解复用
- ⏳ 已知限制：yaw 行程 3~267°，配 65° 视场仍缺约 31° 盲区

## 与 Chassis_2026 的分工

本板**不含**循迹、电机驱动、模式切换按键、超声测距。
两块板的分工与通信约定见上级目录 `../docs/双板分工与协议.md`。
