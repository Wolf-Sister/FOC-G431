# FOC_v1 - STM32G431 磁场定向控制 (FOC) 电机控制器

[![MCU](https://img.shields.io/badge/MCU-STM32G431CBT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
[![Core](https://img.shields.io/badge/Core-Cortex--M4%20with%20FPU-purple)](https://developer.arm.com/Processors/Cortex-M4)
[![Toolchain](https://img.shields.io/badge/Toolchain-Keil%20MDK--ARM%20v5-green)](https://www.keil.com/mdk5/)
[![Language](https://img.shields.io/badge/Language-C99-orange)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

> English | [中文](#foc_v1---stm32g431-磁场定向控制-foc-电机控制器)

## Overview

**FOC_v1** is an embedded BLDC motor Field Oriented Control (FOC) project based on the STM32G431CBT6 microcontroller. It implements precision motor control using a 14-bit AS5047P magnetic rotary encoder for position feedback and dual ADC for phase current sensing.

The project is currently in the **sensor validation phase** — verifying AS5047P encoder communication via SPI+DMA with low-pass filtered angle output over UART.

### Key Features

- **MCU**: STM32G431CBT6 (ARM Cortex-M4, 170 MHz, FPU, 128 KB Flash, 32 KB RAM)
- **Position Sensor**: AS5047P 14-bit magnetic rotary encoder (SPI1 + DMA)
- **Current Sensing**: Dual ADC (ADC1 + ADC2) for phase currents
- **Motor Driver**: 3-phase PWM (TIM1), DRV8313-compatible interface
- **Protections**: Overcurrent comparator (NCOMPO), fault/sleep/reset control
- **Debug Output**: USART2 @ 115200 baud, angle data streaming at 10 Hz
- **Libraries**: STM32G4xx HAL, CMSIS DSP, SimpleFOC port (in progress)

---

## Hardware

| Component              | Pin / Peripheral       | Description                    |
|------------------------|------------------------|--------------------------------|
| **Encoder SPI**        | SPI1 (PA4-PA7)         | AS5047P 14-bit magnetic encoder|
| **Debug UART**         | USART2 (PA2-PA3)       | 115200 baud angle output       |
| **Motor PWM**          | TIM1 (PA8-PA11, PB0/PB1)| 6-channel complementary PWM   |
| **Driver Enable**      | PB13, PB14, PB15       | EN1 / EN2 / EN3                |
| **Driver Control**     | PB2, PB8, PB12         | NFAULT / NSLEEP / NRESET       |
| **Overcurrent**        | PB9 (NCOMPO)           | Comparator input               |
| **Current Sense ADC**  | ADC1 / ADC2            | Phase current measurement      |

### Pinout Quick Reference

```
                STM32G431CBTx (LQFP-48)
         ┌─────────────────────────────────┐
         │                                 │
  PA2 TX ┤                                 ├─ PA3 RX (UART Debug)
  PA4 CS ┤                                 ├─ PA5 SCK
  PA6 SO ┤    SPI1 (AS5047P Encoder)       ├─ PA7 SI
  PA8 CH1┤                                 ├─ PA9 CH2
 PA10 CH3┤        TIM1 (3-Phase PWM)       ├─ PA11 CH4
  PB0 CH5┤                                 ├─ PB1 CH6
 PB13 EN1┤                                 ├─ PB14 EN2
 PB15 EN3┤                                 ├─ PB2  NFAULT
  PB8 NSLP├                                ├─ PB12 NRESET
  PB9 CMP┤                                 ├─ ...
         │                                 │
         └─────────────────────────────────┘
```

---

## Directory Structure

```
FOC_v1/
├── Core/
│   ├── Inc/                  # Application headers
│   │   ├── main.h            # Main application config
│   │   ├── as5047p.h         # AS5047P encoder driver
│   │   ├── dma.h             # DMA configuration
│   │   ├── spi.h             # SPI configuration
│   │   ├── adc.h             # ADC configuration
│   │   ├── tim.h             # Timer/PWM configuration
│   │   ├── gpio.h            # GPIO pin definitions
│   │   ├── usart.h           # UART debug interface
│   │   └── stm32g4xx_hal_conf.h  # HAL module configuration
│   └── Src/                  # Application sources
│       ├── main.c            # Main loop + FOC routines
│       ├── as5047p.c         # AS5047P SPI+DMA driver
│       ├── dma.c             # DMA init & callbacks
│       ├── spi.c             # SPI init
│       ├── adc.c             # ADC init
│       ├── tim.c             # Timer/PWM init
│       ├── gpio.c            # GPIO init
│       ├── usart.c           # UART init
│       ├── system_stm32g4xx.c # System clock init (170 MHz)
│       ├── stm32g4xx_hal_msp.c # HAL MSP (peripheral init)
│       └── stm32g4xx_it.c    # Interrupt handlers
├── Drivers/
│   ├── CMSIS/                # ARM CMSIS Core + Device
│   └── STM32G4xx_HAL_Driver/ # STM32G4 HAL Library
├── MDK-ARM/
│   ├── FOC_v1.uvprojx        # Keil uVision project file
│   ├── FOC_v1.uvoptx         # Keil project options
│   └── startup_stm32g431xx.s # Startup assembly (vector table)
├── MCU_Develop-main/         # Reference: STM32 tutorial collection
├── 参考/                     # Reference: SimpleFOC & AS5047P drivers
├── .gitignore
└── README.md
```

---

## Quick Start

### Prerequisites

- **Keil MDK-ARM v5** (v5.38 or later recommended)
- **Device Pack**: `Keil.STM32G4xx_DFP.2.2.0` (install via Pack Installer)
- **ST-Link** debug probe (or compatible)
- **Hardware**: STM32G431 + AS5047P encoder + DRV8313 motor driver

### Build & Flash

1. **Open the project** in Keil uVision:
   ```
   MDK-ARM/FOC_v1.uvprojx
   ```

2. **Select target**: `FOC_v1` (default)

3. **Build**: Press `F7` or click `Project → Build Target`

4. **Flash & Debug**: Press `Ctrl+F5` or click `Debug → Start/Stop Debug Session`

### Serial Monitor

Connect to USART2 (PA2/PA3) at **115200 baud** to view angle data:

```
Angle: 12345  Smooth: 12340  Raw: 12345  Count: 1001
```

Data format: raw 14-bit angle (0-16383), low-pass filtered angle, zero-crossing corrected value.

---

## Development Status

### Completed
- [x] System clock: HSE bypass → PLL @ 170 MHz
- [x] USART2 debug serial output (115200)
- [x] SPI1 DMA for AS5047P (non-blocking read)
- [x] AS5047P driver: parity check, angle extraction
- [x] Low-pass filter with zero-crossing handling
- [x] 10 Hz angle streaming over UART

### In Progress
- [ ] FOC current loop (ADC phase current reading → Clarke/Park → PI → SVPWM)
- [ ] 3-phase PWM output (TIM1 complementary channels with dead-time)
- [ ] SimpleFOC library port to STM32G4 (C++)
- [ ] Motor calibration & alignment routines
- [ ] Velocity & position closed-loop control

---

## Key Code Reference

### Angle Reading Pipeline (`Core/Src/main.c`)

```c
// Main loop: read AS5047P via SPI DMA
while (1) {
    AS5047P_TriggerRead();       // Start SPI DMA transfer
    // ... wait for DMA complete callback ...
    if (data_ready) {
        raw_angle = AS5047P_ReadAngle();    // Extract 14-bit angle
        smooth_angle = FOC_GetSmoothAngle(raw_angle);  // LPF
        // Output over UART at 10 Hz
    }
}
```

### FOC Parameters (in `Core/Inc/main.h`)

```c
#define FOC_LPF_ALPHA   0.15f        // Low-pass filter coefficient
#define FOC_ANGLE_MAX   16383.0f     // 14-bit encoder resolution
#define FOC_PWM_FREQ    20000        // PWM frequency (Hz)
```

---

## Dependencies

All dependencies are vendored in the project tree — no external package manager required:

| Library | Path | License |
|---------|------|---------|
| STM32G4xx HAL | `Drivers/STM32G4xx_HAL_Driver/` | ST SLA (BSD-like) |
| CMSIS Core | `Drivers/CMSIS/` | Apache 2.0 |
| SimpleFOC (reference) | `参考/Arduino-FOC-master/` | MIT |
| AS5047P Driver (reference) | `参考/AS5047P-Driver-master/` | Open Source |

---

## Reference Projects

The `MCU_Develop-main/` directory contains 21 incremental SimpleFOC projects from STM32F103 to STM32F405, plus bootloader tutorials and HAL/LL library examples. These are included for learning reference.

The `参考/` directory contains the original Arduino SimpleFOC library and AS5047P driver that this project ports to bare-metal STM32.

---

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Commit your changes (`git commit -m 'Add some feature'`)
4. Push to the branch (`git push origin feature/your-feature`)
5. Open a Pull Request

---

## License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

Note: The STM32 HAL/CMSIS libraries in `Drivers/` are distributed under STMicroelectronics' own license terms (see `Drivers/CMSIS/LICENSE.txt` and `Drivers/STM32G4xx_HAL_Driver/LICENSE.txt`).

---

## Acknowledgments

- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) — Open-source FOC library by Antun Skuric et al.
- [STMicroelectronics](https://www.st.com/) — STM32G4 HAL & CMSIS libraries
- [ams-OSRAM](https://ams.com/) — AS5047P magnetic rotary encoder

---

# FOC_v1 - STM32G431 磁场定向控制 (FOC) 电机控制器

## 项目简介

**FOC_v1** 是一个基于 STM32G431CBT6 微控制器的嵌入式无刷直流电机磁场定向控制 (FOC) 项目。使用 14 位 AS5047P 磁旋转编码器进行位置反馈，双 ADC 进行相电流采样，实现精密电机控制。

项目当前处于 **传感器验证阶段** —— 通过 SPI+DMA 验证 AS5047P 编码器通信，经过低通滤波后通过串口输出角度数据。

### 主要特性

- **主控**: STM32G431CBT6 (ARM Cortex-M4, 170 MHz, FPU, 128 KB Flash, 32 KB RAM)
- **位置传感器**: AS5047P 14 位磁旋转编码器 (SPI1 + DMA 非阻塞读取)
- **电流采样**: 双 ADC (ADC1 + ADC2) 相电流检测
- **电机驱动**: 3 相 PWM (TIM1)，兼容 DRV8313 驱动芯片接口
- **保护功能**: 过流比较器 (NCOMPO)、故障/休眠/复位控制
- **调试输出**: USART2 @ 115200 bps，10 Hz 角度数据流
- **软件库**: STM32G4xx HAL、CMSIS DSP、SimpleFOC 移植（进行中）

---

## 硬件连接

| 功能           | 引脚 / 外设           | 说明                          |
|----------------|-----------------------|-------------------------------|
| **编码器 SPI** | SPI1 (PA4-PA7)        | AS5047P 14 位磁编码器         |
| **调试串口**   | USART2 (PA2-PA3)      | 115200 bps 角度输出           |
| **电机 PWM**   | TIM1 (PA8-PA11, PB0/PB1)| 6 路互补 PWM                |
| **驱动使能**   | PB13, PB14, PB15      | EN1 / EN2 / EN3               |
| **驱动控制**   | PB2, PB8, PB12        | NFAULT / NSLEEP / NRESET      |
| **过流检测**   | PB9 (NCOMPO)          | 比较器输入                     |
| **电流采样**   | ADC1 / ADC2           | 相电流测量                     |

---

## 目录结构

```
FOC_v1/
├── Core/
│   ├── Inc/                  # 应用层头文件
│   │   ├── main.h            # 主程序配置
│   │   ├── as5047p.h         # AS5047P 编码器驱动
│   │   ├── dma.h             # DMA 配置
│   │   ├── spi.h             # SPI 配置
│   │   ├── adc.h             # ADC 配置
│   │   ├── tim.h             # 定时器/PWM 配置
│   │   ├── gpio.h            # GPIO 引脚定义
│   │   ├── usart.h           # 串口调试接口
│   │   └── stm32g4xx_hal_conf.h  # HAL 模块配置
│   └── Src/                  # 应用层源码
│       ├── main.c            # 主循环 + FOC 算法
│       ├── as5047p.c         # AS5047P SPI+DMA 驱动
│       ├── dma.c             # DMA 初始化与回调
│       ├── spi.c             # SPI 初始化
│       ├── adc.c             # ADC 初始化
│       ├── tim.c             # 定时器/PWM 初始化
│       ├── gpio.c            # GPIO 初始化
│       ├── usart.c           # 串口初始化
│       ├── system_stm32g4xx.c # 系统时钟初始化 (170 MHz)
│       ├── stm32g4xx_hal_msp.c # HAL MSP 层
│       └── stm32g4xx_it.c    # 中断服务函数
├── Drivers/
│   ├── CMSIS/                # ARM CMSIS 核心 + 设备支持
│   └── STM32G4xx_HAL_Driver/ # STM32G4 HAL 驱动库
├── MDK-ARM/
│   ├── FOC_v1.uvprojx        # Keil uVision 工程文件
│   ├── FOC_v1.uvoptx         # Keil 工程选项
│   └── startup_stm32g431xx.s # 启动汇编文件 (向量表)
├── MCU_Develop-main/         # 参考：STM32 教程合集
├── 参考/                     # 参考：SimpleFOC 原始库 & AS5047P 驱动
├── .gitignore
└── README.md
```

---

## 快速开始

### 环境要求

- **Keil MDK-ARM v5** (推荐 v5.38 及以上)
- **器件包**: `Keil.STM32G4xx_DFP.2.2.0` (通过 Pack Installer 安装)
- **ST-Link** 调试器 (或兼容版本)
- **硬件**: STM32G431 + AS5047P 编码器 + DRV8313 电机驱动

### 编译与烧录

1. 在 Keil uVision 中 **打开工程**:
   ```
   MDK-ARM/FOC_v1.uvprojx
   ```

2. **选择目标**: `FOC_v1` (默认)

3. **编译**: 按 `F7` 或点击 `Project → Build Target`

4. **烧录与调试**: 按 `Ctrl+F5` 或点击 `Debug → Start/Stop Debug Session`

### 串口监视

连接到 USART2 (PA2/PA3)，波特率 **115200**，查看角度数据:

```
Angle: 12345  Smooth: 12340  Raw: 12345  Count: 1001
```

数据格式: 原始 14 位角度 (0-16383)、低通滤波角度、过零修正值。

---

## 开发状态

### 已完成
- [x] 系统时钟: HSE 旁路 → PLL @ 170 MHz
- [x] USART2 调试串口 (115200)
- [x] SPI1 DMA 驱动 AS5047P (非阻塞读取)
- [x] AS5047P 驱动: 奇偶校验、角度提取
- [x] 低通滤波器 + 过零处理
- [x] 10 Hz 串口角度数据流

### 进行中
- [ ] FOC 电流环 (ADC 相电流 → Clarke/Park 变换 → PI → SVPWM)
- [ ] 3 相 PWM 输出 (TIM1 互补通道 + 死区)
- [ ] SimpleFOC 库 C++ 移植到 STM32G4
- [ ] 电机校准与对齐程序
- [ ] 速度 & 位置闭环控制

---

## 关键代码参考

### 角度读取流程 (`Core/Src/main.c`)

```c
// 主循环: 通过 SPI DMA 读取 AS5047P
while (1) {
    AS5047P_TriggerRead();       // 启动 SPI DMA 传输
    // ... 等待 DMA 完成回调 ...
    if (data_ready) {
        raw_angle = AS5047P_ReadAngle();    // 提取 14 位角度
        smooth_angle = FOC_GetSmoothAngle(raw_angle);  // 低通滤波
        // 通过串口以 10 Hz 频率输出
    }
}
```

### FOC 参数 (在 `Core/Inc/main.h` 中定义)

```c
#define FOC_LPF_ALPHA   0.15f        // 低通滤波器系数
#define FOC_ANGLE_MAX   16383.0f     // 14 位编码器分辨率
#define FOC_PWM_FREQ    20000        // PWM 频率 (Hz)
```

---

## 依赖库

所有依赖库均已包含在工程目录中，无需外部包管理器:

| 库 | 路径 | 许可证 |
|-----|------|--------|
| STM32G4xx HAL | `Drivers/STM32G4xx_HAL_Driver/` | ST SLA (类 BSD) |
| CMSIS Core | `Drivers/CMSIS/` | Apache 2.0 |
| SimpleFOC (参考) | `参考/Arduino-FOC-master/` | MIT |
| AS5047P 驱动 (参考) | `参考/AS5047P-Driver-master/` | 开源 |

---

## 参考工程

`MCU_Develop-main/` 目录包含 21 个从 STM32F103 到 STM32F405 逐步递进的 SimpleFOC 工程，以及 Bootloader 教程和 HAL/LL 库示例，供学习参考。

`参考/` 目录包含 Arduino SimpleFOC 原始库和 AS5047P 驱动，本工程将其移植到裸机 STM32。

---

## 参与贡献

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/新功能`)
3. 提交修改 (`git commit -m '添加新功能'`)
4. 推送到分支 (`git push origin feature/新功能`)
5. 创建 Pull Request

---

## 许可证

本项目采用 **MIT 许可证** — 详见 [LICENSE](LICENSE)。

注意: `Drivers/` 目录下的 STM32 HAL/CMSIS 库遵循 STMicroelectronics 的许可条款 (参见 `Drivers/CMSIS/LICENSE.txt` 和 `Drivers/STM32G4xx_HAL_Driver/LICENSE.txt`)。

---

## 致谢

- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) — Antun Skuric 等人开发的开源 FOC 库
- [STMicroelectronics](https://www.st.com/) — STM32G4 HAL & CMSIS 库
- [ams-OSRAM](https://ams.com/) — AS5047P 磁旋转编码器
