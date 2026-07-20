# FOC_v1 - STM32G431 磁场定向控制 (FOC) 电机控制器

[![MCU](https://img.shields.io/badge/MCU-STM32G431CBT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
[![Core](https://img.shields.io/badge/Core-Cortex--M4%20with%20FPU-purple)](https://developer.arm.com/Processors/Cortex-M4)
[![Toolchain](https://img.shields.io/badge/Toolchain-Keil%20MDK--ARM%20v5-green)](https://www.keil.com/mdk5/)
[![Language](https://img.shields.io/badge/Language-C99-orange)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 项目简介

**FOC_v1** 是一个基于 STM32G431CBT6 微控制器的嵌入式无刷直流电机磁场定向控制 (FOC) 项目。使用 14 位 AS5047P 磁旋转编码器进行位置反馈，双 ADC 进行相电流采样，硬件 CORDIC 加速三角函数计算，实现完整的三环级联 FOC 控制（位置环 → 速度环 → 电流环），同时通过 USART2 DMA 与上位机 VOFA+ 进行实时数据交互与在线调参。

### 主要特性

- **主控**: STM32G431CBT6 (ARM Cortex-M4, 170 MHz, FPU, 硬件 CORDIC)
- **位置传感器**: AS5047P 14 位磁旋转编码器 (SPI1 + DMA，非阻塞流水线读取，多圈累积)
- **电流采样**: 双 ADC (ADC1 + ADC2) 同步注入采样，自动零点校准 (2000 样本)
- **控制算法**:
  - Clarke + Park 变换，SVPWM 空间矢量调制
  - 三环级联控制：位置环 (1 kHz P 控制) → 速度环 (2 kHz PI 控制) → 电流环 (20 kHz Iq/Id PI)
  - Iq/Id 双轴 PI 闭环控制（Tustin 离散化 + 双向抗饱和 + 输出斜率限制）
  - 交叉解耦 + 反电动势前馈补偿 (>1000 rad/s 时自动启用)
  - CORDIC 硬件加速 sin/cos 计算，一个 PWM 周期的流水线角度补偿
- **电机驱动**: TIM1 3 相互补 PWM，兼容 DRV8313 驱动芯片
- **传感器对准**: 自动零电角度校准流程（静态电压矢量锁定 + 平均采样）
- **上位机通信**: VOFA+ JustFloat 协议发送 10 通道遥测数据，文本命令在线调整全部参数
- **环路频率**: PWM/电流环 20 kHz，速度环 2 kHz，位置环 1 kHz，遥测 10 Hz
- **编码器接口**: 双缓冲无锁缓存 (TIM2 ISR 写入, FOC 电流环读取)，10 kHz 固定速率滑动窗口速度估算

---

## 硬件连接

功能           | 引脚 / 外设              | 说明
---------------|--------------------------|-----------------------------
**编码器 SPI** | SPI1 (PA4-PA7)           | AS5047P 14 位磁编码器
**UART2 遥测** | USART2 (PA15 RX, PB3 TX) | 115200 bps，VOFA+ 协议
**电机 PWM**   | TIM1 (PA8-PA11, PB0-PB1) | 6 路互补 PWM
**驱动使能**   | PB13, PB14, PB15         | EN1 / EN2 / EN3
**驱动控制**   | PB2, PB8, PB12           | NFAULT / NSLEEP / NRESET
**过流检测**   | PB9 (NCOMPO)             | 比较器输入
**电流采样**   | ADC1 / ADC2              | 双 ADC 注入同步采样

---

## 目录结构

```
FOC_v1/
├── Core/
│   ├── Inc/                       # 应用层头文件
│   │   ├── main.h                 # 主程序配置
│   │   ├── foc.h                  # FOC 核心 — SVPWM、电流环、传感器对准
│   │   ├── pid.h                  # PID 控制器 — Tustin 积分器 + 输出限幅 + 默认参数
│   │   ├── vofa.h                 # VOFA+ JustFloat 遥测 + 命令接收
│   │   ├── utils.h                # 工具函数 — DWT 微秒计时、LPF、角度归一化
│   │   ├── as5047p.h              # AS5047P 编码器 SPI+DMA 底层驱动
│   │   ├── as5047p_ext.h          # AS5047P 高层接口 (角度/速度/多圈累积)
│   │   ├── cordic.h               # CORDIC 硬件加速器配置 (CubeMX)
│   │   ├── adc.h                  # ADC 配置 (CubeMX)
│   │   ├── dma.h                  # DMA 配置 (CubeMX)
│   │   ├── spi.h                  # SPI 配置 (CubeMX)
│   │   ├── tim.h                  # 定时器/PWM 配置 (CubeMX)
│   │   ├── usart.h                # 串口配置 (CubeMX)
│   │   ├── gpio.h                 # GPIO 引脚定义 (CubeMX)
│   │   ├── stm32g4xx_it.h         # 中断服务函数声明 (CubeMX)
│   │   └── stm32g4xx_hal_conf.h   # HAL 模块配置 (CubeMX)
│   └── Src/                       # 应用层源码
│       ├── main.c                 # 主循环 + 状态机 + ADC 中断回调 + 遥测
│       ├── foc.c                  # FOC 算法：Clarke/Park、三环控制、前馈、SVPWM
│       ├── pid.c                  # PID 控制器实现 + 参数初始化
│       ├── vofa.c                 # VOFA+ 协议：遥测发送 + 16 键命令解析
│       ├── utils.c                # 工具函数实现
│       ├── as5047p.c              # AS5047P SPI+DMA 驱动 (临界区保护)
│       ├── as5047p_ext.c          # AS5047P 高层接口：速度窗口/多圈/编码器缓存
│       ├── cordic.c               # CORDIC 硬件初始化 (CubeMX)
│       ├── adc.c                  # ADC 初始化 (CubeMX)
│       ├── dma.c                  # DMA 初始化与回调 (CubeMX)
│       ├── spi.c                  # SPI 初始化 (CubeMX)
│       ├── tim.c                  # 定时器/PWM 初始化 (CubeMX)
│       ├── usart.c                # 串口初始化 (CubeMX)
│       ├── gpio.c                 # GPIO 初始化 (CubeMX)
│       ├── system_stm32g4xx.c     # 系统时钟初始化 (170 MHz)
│       ├── stm32g4xx_hal_msp.c    # HAL MSP 层 (CubeMX)
│       └── stm32g4xx_it.c         # 中断服务函数 (CubeMX)
├── Drivers/
│   ├── CMSIS/                     # ARM CMSIS 核心 + 设备支持
│   └── STM32G4xx_HAL_Driver/      # STM32G4 HAL 驱动库
├── MDK-ARM/
│   ├── FOC_v1.uvprojx             # Keil uVision 工程文件
│   ├── FOC_v1.uvoptx              # Keil 工程选项
│   └── startup_stm32g431xx.s      # 启动汇编文件 (向量表)
├── .gitignore
└── README.md
```

---

## 快速开始

### VOFA+ 上位机连接

连接 USART2 (PA15 RX, PB3 TX)，波特率 **115200**：

- **遥测接收**: 打开 VOFA+，选择 JustFloat 协议，添加 10 通道数据显示
- **命令发送**: 在 VOFA+ 终端输入文本命令，格式为逗号分隔的键值对

支持的遥测通道 (10 通道, 100 Hz):

通道    | 变量              | 说明
--------|-------------------|------------------
[0]     | id_target         | D 轴目标电流 (A)
[1]     | id_meas           | D 轴实测电流 (A)
[2]     | iq_target         | Q 轴目标电流 / 转矩指令 (A)
[3]     | iq_meas           | Q 轴实测电流 (A)
[4]     | velocity_filt     | 速度环滤波后角速度 (rad/s)
[5]     | status_flag       | 步进同步标志 (0/1)
[6]     | set_speed         | 速度目标值 (rad/s)
[7]     | set_position      | 位置目标值 (多圈 rad)
[8]     | pos_meas          | 实测多圈位置 (rad)
[9]     | mode              | 控制模式 (0=转矩, 1=速度, 2=位置)

支持的命令 (16 键):

命令  | 说明                        | 示例
------|-----------------------------|--------
T=V   | 转矩 / Iq 电流指令 (A)      | `T=0.5`
D=V   | D 轴电流目标 (A)            | `D=0.0`
P=V   | Q 轴电流环 P 增益           | `P=1.5`
I=V   | Q 轴电流环 I 增益           | `I=200`
DP=V  | D 轴电流环 P 增益           | `DP=1.5`
DI=V  | D 轴电流环 I 增益           | `DI=200`
S=V   | 速度目标值 (rad/s)          | `S=50`
SP=V  | 速度环 P 增益               | `SP=0.1`
SI=V  | 速度环 I 增益               | `SI=0.02`
PS=V  | 位置目标值 (多圈 rad)       | `PS=6.28`
PP=V  | 位置环 P 增益               | `PP=5`
PL=V  | 位置环速度限幅 (rad/s)      | `PL=100`
CVL=V | 电流环电压矢量限幅 (V)      | `CVL=6`
SIL=V | 速度环电流限幅 (A)          | `SIL=0.5`
PSL=V | 位置环速度限幅 (rad/s)      | `PSL=600`
M=V   | 控制模式 (0=转矩,1=速度,2=位置) | `M=1`

示例: `M=2,PS=6.28,PP=5,PL=100` — 切换到位置模式，目标 1 圈，P=5，限速 100 rad/s

---

## 控制架构

```
  位置外环 (1 kHz, TIM3)      速度外环 (2 kHz, 降采样)      电流内环 (20 kHz, ADC ISR)
  ┌─────────────────────┐   ┌──────────────────────┐   ┌────────────────────────────┐
  │ pos_setpoint        │   │ speed_setpoint       │   │                            │
  │     │               │   │     │                │   │  ┌───────────────────────┐ │
  │  ┌──▼───┐           │   │  ┌──▼───┐            │   │  │ CORDIC 硬件 sin/cos   │ │
  │  │ P 控制│──speed_cmd┼──►│ PI 控制│──Iq_cmd────┼──►│  │ (Q31 写入, 下帧读取)    │ │
  │  └──────┘           │   │ └──────┘            │   │  └───────────────────────┘ │
  │     ▲               │   │     ▲               │   │                            │
  │     │ pos_meas      │   │     │ vel_filt      │   │  Iq_cmd──►[PI Iq]──►[+]──┐ │
  └─────┼───────────────┘   └─────┼───────────────┘   │            ▲        ▲    │ │
        │                         │                   │   iq_meas  │  Vq_ff │    │ │
        │                         │                   │            │        │    │ │
  ┌─────┴─────────────────────────┴───────────────────┴────────────┴────────┴────┴─┴─┐
  │                            编码器双缓冲缓存 (TIM2 ISR)                             │
  │  单圈角度 [0,2π)  ·  滑动窗口速度 [rad/s]  ·  多圈累积总角度 [rad]               │
  └────────────────────────────┬──────────────────────────────────────────────────────┘
                               │
  ┌────────────────────────────┴──────────────────────────────────────────────────────┐
  │  AS5047P SPI+DMA (10 kHz)    │  双 ADC 注入采样    │  交叉解耦+反电动势前馈        │
  │  非阻塞流水线读取            │  A,B,C 三相电流       │  Vd_ff=-ωLq·Iq (>1000rad/s)  │
  └────────────────────────────┬──────────────────────────────────────────────────────┘
                               │
  Id_cmd──►[PI Id]──►[+]──┐    │             ┌──────────┐
            ▲        ▲    │    │             │ 反Park   │◄── Vd,Vq
   id_meas  │  Vd_ff │    │    │             │ Vd,Vq→αβ │
            │        │    │    │             └────┬─────┘
            │   ┌────┴────┴────┴──┐               │
            │   │ 电压饱和限制      │          ┌────┴─────┐
            │   │ (SVM内切圆限幅)   │          │  SVM     │
            │   └─────────────────┘          │  αβ→占空比│
            │                                └────┬─────┘
            │                                     │
      ┌─────┴──────┐                          ┌────┴─────┐
      │ Park 变换   │                          │ TIM1 PWM │
      │ Iα,Iβ→Id,Iq │                          │ 3相输出   │
      └─────▲──────┘                          └──────────┘
            │
      ┌─────┴──────┐
      │ Clarke 变换 │
      │ Ib,Ic→Iα,Iβ │
      └─────▲──────┘
            │
      ┌─────┴──────┐
      │ LPF α=0.05 │
      │ fc≈80Hz    │
      └─────▲──────┘
            │
       双ADC 相电流
```

---

## 开发状态

### 已完成

- [x] 系统时钟: HSE 旁路 → PLL @ 170 MHz
- [x] AS5047P 编码器 SPI+DMA 流水线驱动 (临界区保护)
- [x] 编码器高层接口: 角度/速度/多圈累积 + 双缓冲无锁缓存
- [x] 双 ADC 注入同步采样 + 自动零点校准 (2000 样本)
- [x] 3 相互补 PWM 输出 (TIM1)，DRV8313 兼容
- [x] CORDIC 硬件加速器 sin/cos 计算
- [x] Clarke + Park 变换，12 扇区 SVPWM
- [x] Iq/Id 双轴 PI 电流闭环 (Tustin 离散化 + 双向抗饱和 + 斜率限制)
- [x] 交叉解耦 + 反电动势前馈补偿 (>1000 rad/s)
- [x] 速度闭环 — 2 kHz PI 控制，级联在电流环之上
- [x] 位置闭环 — 1 kHz P 控制，硬限幅速度输出
- [x] 三模式控制: 转矩 / 速度 / 位置，在线切换
- [x] 传感器自动对准 (静态电压矢量 + 平均采样零电角度校准)
- [x] VOFA+ JustFloat 10 通道遥测数据发送 (100 Hz)
- [x] VOFA+ 16 键文本命令接收与解析 (PID 在线调参、模式切换)
- [x] DWT 微秒级计时
- [x] 默认 PID 参数预设 (pid.h 集中管理)

### 计划中

- [ ] 参数自动保存到 Flash
- [ ] 过流 / 过压 / 欠压保护
- [ ] CAN 总线通信

---

## 依赖库

所有依赖库均已包含在工程目录中，无需外部包管理器:

| 库 | 路径 | 许可证 |
|-----|------|--------|
| STM32G4xx HAL | `Drivers/STM32G4xx_HAL_Driver/` | ST SLA |
| CMSIS Core | `Drivers/CMSIS/` | Apache 2.0 |
| CMSIS DSP | `Middlewares/ST/ARM/DSP/` | Apache 2.0 |
| CORDIC 硬件加速 | STM32G4 片上外设 | — |

---

## 许可证

本项目采用 **MIT 许可证** — 详见 [LICENSE](LICENSE)。

注意: `Drivers/` 目录下的 STM32 HAL/CMSIS 库遵循 STMicroelectronics 的许可条款 (参见 `Drivers/CMSIS/LICENSE.txt` 和 `Drivers/STM32G4xx_HAL_Driver/LICENSE.txt`)。

---

## 致谢

- [TinyFoc](https://github.com/JiuXu01/TinyFoc) — 简洁高效的 FOC 参考实现
- [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) — Antun Skuric 等人开发的开源 FOC 库
- [VOFA+](https://www.vofa.plus/) — 伏特加电子，优秀的串口数据可视化工具
- [STMicroelectronics](https://www.st.com/) — STM32G4 HAL & CMSIS 库
- [ams-OSRAM](https://ams.com/) — AS5047P 磁旋转编码器

---

## 安全启动与维护约定

- CubeMX 中 `NSLEEP`、`NRESET`、`DRV_EN1`、`DRV_EN2`、`DRV_EN3` 的上电初始电平必须保持为 `LOW`。
- 电流零点校准完成后，程序按 `NRESET → NSLEEP → DRV_EN1~3 → 编码器对齐` 的顺序使能驱动，每一步保留 1 ms 稳定时间。
- `foc_alignSensor()` 不直接开启电流环；只有电机状态、目标值和全部 PID 状态初始化完成后，`main()` 才同时置位 `current_loop_enable` 和 `motor_ready`。
- 一帧 VOFA 命令先完整解析到局部变量，再在临界区统一提交，因此 `M=1,S=4` 与 `S=4,M=1` 语义一致。
- dq 电压矢量被 SVM 线性区限幅后，会把实际施加量回算到 Id/Iq 积分器，避免耦合饱和造成积分累积。
- 前馈默认总开关为 `FOC_FEEDFORWARD_ENABLE=0U`。需要高速前馈时改为 `1U`；电角速度低于 1000 rad/s 时仍为零，并在 1000～1300 rad/s 之间平滑接入。
- CubeMX 重新生成代码后，应首先执行下面的源码契约检查，确认 `.ioc` 与生成代码没有恢复为不安全的 GPIO 初始状态。
      
