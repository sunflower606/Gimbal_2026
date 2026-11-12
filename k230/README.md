# k230 归档

K230D BOX 侧运行的脚本，本目录为**归档副本**，方便与 STM32 侧协议配套保存。

| 文件 | 说明 |
|---|---|
| `person_detection_gimbal.py` | 人体检测 + 13 字节坐标回传，与 `modules/vision_protocol.c` 严格对应 |

## 来源

`D:\Hilab考核\person_detection_gimbal.py`（2026-11-05 归档）

## 在 K230 上运行

1. 用 CanMV IDE 打开本目录的 `person_detection_gimbal.py`；
2. K230D BOX 用 PH2.0 接口2（GPIO44/45）接 STM32：
   - `UART2_TX` (GPIO44) → STM32 `PA3`（USART2_RX）
   - `UART2_RX` (GPIO45) ← STM32 `PA2`（USART2_TX）（只发不收可不接）
   - `GND` ↔ `GND`（**必须共地**）
3. 保存为 `/sdcard/main.py` 后重启，或直接在 IDE 里运行；
4. IDE 串口终端里应看到 `FPS: xx.x`。

## 协议要点（详见 `../docs/K230通信协议.md`）

- 13 字节定长帧，大端序
- 帧头 `AA 55 01 08`
- `x`/`y` 为人体框中心像素坐标（1280×960）
- 未检测到人时发 `(-1, -1)`
- 校验 = 前 12 字节逐字节异或

## ⚠️ 配套修改约束

改动本脚本里的**任何**以下内容，都必须同步改 STM32 侧：

| 改动 | STM32 侧同步点 |
|---|---|
| 帧头 / 帧长 / 校验方式 | `config/vision_config.h` |
| 图像分辨率 `rgb888p_size` | `VISION_IMAGE_WIDTH_PX` / `VISION_IMAGE_HEIGHT_PX` |
| 丢失约定（-1 或其它） | `vision_protocol.c` 的 `x_px < 0 \|\| y_px < 0` 判断 |
| 发送间隔 | `GIMBAL_LOSE_TIMEOUT_MS` |

改完必须更新 `docs/K230通信协议.md`。
