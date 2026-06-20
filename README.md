# FOC_v1 - STM32G431 磁场定向控制 (FOC) 电机控制器

[![MCU](https://img.shields.io/badge/MCU-STM32G431CBT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
[![Core](https://img.shields.io/badge/Core-Cortex--M4%20with%20FPU-purple)](https://developer.arm.com/Processors/Cortex-M4)
[![Toolchain](https://img.shields.io/badge/Toolchain-Keil%20MDK--ARM%20v5-green)](https://www.keil.com/mdk5/)
[![Language](https://img.shields.io/badge/Language-C99-orange)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 项目简介

**FOC_v1** 是一个基于 STM32G431CBT6 微控制器的嵌入式无刷直流电机磁场定向控制 (FOC) 项目。使用 14 位 AS5047P 磁旋转编码器进行位置反馈，双 ADC 进行相电流采样，实现完整电流闭环 FOC 控制，同时通过 USART2 DMA 与上位机 VOFA+ 进行实时数据交互。

项目移植并融合了 TinyFoc 的核心算法，在此基础上加入了去耦前馈补偿、VOFA+ 协议通信、Python 上位机调参等实用功能。

### 主要特性

- **主控**: STM32G431CBT6 (ARM Cortex-M4, 170 MHz, FPU)
- **位置传感器**: AS5047P 14 位磁旋转编码器 (SPI1 + DMA，非阻塞流水线读取)
- **电流采样**: 双 ADC (ADC1 + ADC2) 同步注入采样，自动零点校准
- **控制算法**:
  - Clarke + Park 变换，SVPWM 空间矢量调制
  - Iq/Id 双轴 PI 闭环控制（Tustin 离散化 + 抗饱和 + 输出斜率限制）
  - 交叉解耦 + 反电动势前馈补偿
  - 12 扇区 SVM 线性调制
- **电机驱动**: TIM1 3 相互补 PWM，兼容 DRV8313 驱动芯片
- **传感器对准**: 自动零电角度校准流程
- **上位机通信**: VOFA+ JustFloat 协议发送遥测数据，文本命令接收控制参数
- **环路频率**: PWM / 电流环 20 kHz，遥测 10 Hz
- **Python 工具**: `foc_auto_tuner.py` 自动扫参 / 阶跃响应分析 / PID 调优

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
│   │   ├── pid.h                  # PID 控制器 — Tustin 积分器 + 输出限幅
│   │   ├── vofa.h                 # VOFA+ JustFloat 遥测 + 命令接收
│   │   ├── utils.h                # 工具函数 — DWT 微秒计时、LPF、角度归一化
│   │   ├── as5047p.h              # AS5047P 编码器 SPI+DMA 底层驱动
│   │   ├── as5047p_ext.h          # AS5047P 高层接口 (角度/速度/多圈累计)
│   │   ├── adc.h                  # ADC 配置
│   │   ├── dma.h                  # DMA 配置
│   │   ├── spi.h                  # SPI 配置
│   │   ├── tim.h                  # 定时器/PWM 配置
│   │   ├── usart.h                # 串口配置
│   │   ├── gpio.h                 # GPIO 引脚定义
│   │   └── stm32g4xx_hal_conf.h   # HAL 模块配置
│   └── Src/                       # 应用层源码
│       ├── main.c                 # 主循环 + 状态机 + ADC 中断回调
│       ├── foc.c                  # FOC 算法：Clarke/Park、PI、前馈、SVPWM
│       ├── pid.c                  # PID 控制器实现
│       ├── vofa.c                 # VOFA+ 协议：遥测发送 + 命令解析
│       ├── utils.c                # 工具函数实现
│       ├── as5047p.c              # AS5047P SPI+DMA 驱动
│       ├── as5047p_ext.c          # AS5047P 高层接口实现
│       ├── adc.c                  # ADC 初始化
│       ├── dma.c                  # DMA 初始化与回调
│       ├── spi.c                  # SPI 初始化
│       ├── tim.c                  # 定时器/PWM 初始化
│       ├── usart.c                # 串口初始化
│       ├── gpio.c                 # GPIO 初始化
│       ├── system_stm32g4xx.c     # 系统时钟初始化 (170 MHz)
│       ├── stm32g4xx_hal_msp.c    # HAL MSP 层
│       └── stm32g4xx_it.c         # 中断服务函数
├── Drivers/
│   ├── CMSIS/                     # ARM CMSIS 核心 + 设备支持
│   └── STM32G4xx_HAL_Driver/      # STM32G4 HAL 驱动库
├── MDK-ARM/
│   ├── FOC_v1.uvprojx             # Keil uVision 工程文件
│   ├── FOC_v1.uvoptx              # Keil 工程选项
│   └── startup_stm32g431xx.s      # 启动汇编文件 (向量表)
├── TinyFoc-main/                  # 参考：TinyFoc 原始实现 (STM32F401)
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

### VOFA+ 上位机连接

连接 USART2 (PA15 RX, PB3 TX)，波特率 **115200**：

- **遥测接收**: 打开 VOFA+，选择 JustFloat 协议，添加 8 通道数据显示
- **命令发送**: 在 VOFA+ 终端输入文本命令，格式为逗号分隔的键值对

支持的遥测通道:

通道 | 变量         | 说明
------|-------------|------------------
[0]   | id_target   | D 轴目标电流 (A)
[1]   | id_meas     | D 轴实测电流 (A)
[2]   | iq_target   | Q 轴目标电流 (A)
[3]   | iq_meas     | Q 轴实测电流 (A)
[4]   | vd_cmd      | D 轴电压指令 (V)
[5]   | vq_cmd      | Q 轴电压指令 (V)
[6]   | vbus        | 母线电压 (V)
[7]   | status_flag | 步进同步标志

支持的命令:

命令 | 说明            | 示例
-----|----------------|--------
T=V  | 扭矩/电流指令 (A) | `T=0.5`
D=V  | D 轴电流目标 (A)  | `D=0.0`
P=V  | Q 轴 P 增益      | `P=1.5`
I=V  | Q 轴 I 增益      | `I=200`
DP=V | D 轴 P 增益      | `DP=1.5`
DI=V | D 轴 I 增益      | `DI=200`

---

## 控制架构

```
                    ┌─────────────────────────────────────────┐
                    │           foc_current_loop() @ 20kHz     │
                    │                                          │
  set_torque ──►[+]──►[ PI Iq ]──►[+]──►[ 饱和 ]──┐          │
                 ▲                  ▲                │          │
                 │ iq_meas          │ Vq_ff          │          │
                 │                  │                │          │
  id_target  ──►[+]──►[ PI Id ]──►[+]──►[ 饱和 ]──┐│          │
                 ▲                  ▲              ││          │
                 │ id_meas          │ Vd_ff        ││          │
                 │                  │              ││          │
            ┌────┴────┐      ┌──────┴──────┐      ││          │
            │ 低通滤波  │      │ 前馈补偿     │      ││          │
            │ α=0.05   │      │ -ωLq·Iq     │      ││          │
            │          │      │ +ω(Ld·Id+ψ) │      ││          │
            └────▲─────┘      └──────▲──────┘      ││          │
                 │                    │             ││          │
            ┌────┴────┐        ┌─────┴─────┐       ││          │
            │ Park    │        │ 电角速度   │       ││          │
            │ Iα,Iβ→dq│        │ _elecVel()│       ││          │
            └────▲─────┘        └─────▲─────┘       ││          │
                 │                     │             ││          │
            ┌────┴────┐         ┌──────┴──────┐     ││          │
            │ Clarke  │         │ 编码器       │     ││          │
            │ Ib,Ic→αβ│         │ AS5047P     │     ││          │
            └────▲─────┘         └────────────┘     ││          │
                 │                                   ││          │
            ┌────┴────┐                              ││          │
            │ 双ADC    │                              ││          │
            │ A,B,C相 │                              ││          │
            └─────────┘                              ││          │
                                                     ▼▼          │
                                               ┌──────────┐     │
                                               │ 反Park   │◄────┘
                                               │ Vd,Vq→αβ │
                                               └────┬─────┘
                                                    │
                                               ┌────┴─────┐
                                               │  SVM     │
                                               │  αβ→占空比│
                                               └────┬─────┘
                                                    │
                                               ┌────┴─────┐
                                               │ TIM1 PWM │
                                               │ 3相输出   │
                                               └──────────┘
```

---

## 开发状态

### 已完成

- [x] 系统时钟: HSE 旁路 → PLL @ 170 MHz
- [x] AS5047P 编码器 SPI+DMA 流水线驱动
- [x] 编码器高层接口: 角度/速度/多圈累计
- [x] 双 ADC 注入同步采样 + 自动零点校准 (2000 样本)
- [x] 3 相互补 PWM 输出 (TIM1)
- [x] Clarke + Park 变换，12 扇区 SVPWM
- [x] Iq/Id 双轴 PI 电流闭环 (Tustin 离散化 + 抗饱和 + 斜率限制)
- [x] 交叉解耦 + 反电动势前馈补偿
- [x] 传感器自动对准 (零电角度校准)
- [x] VOFA+ JustFloat 遥测数据发送
- [x] VOFA+ 文本命令接收与解析 (PID 在线调参)
- [x] DWT 微秒级计时

### 计划中

- [ ] 速度闭环 (PI 速度环外环)
- [ ] 位置闭环
- [ ] Python 自动扫参 / 阶跃响应分析脚本
- [ ] 参数自动保存到 Flash
- [ ] 过流 / 过压 / 欠压保护

---

## 关键代码参考

### 电流环主流程 (`Core/Src/foc.c`)

```c
void foc_current_loop(void)
{
    // 1. 读取电角度 & 电角速度 (仅一次)
    float angle_el = _electricalAngle();
    float elec_vel = _electricalVelocity();

    // 2. Clarke + Park: B,C 相电流 → Id, Iq
    // 3. 低通滤波 (α=0.05)
    // 4. 交叉解耦 + 反电动势前馈 (仅速度 > 1 rad/s 时开启)
    // 5. Iq / Id PI 控制 (死区 0.04A)
    // 6. 电压饱和限制 (SVM 内切圆)
    // 7. 计算延迟角度补偿
    // 8. 反 Park + SVM → PWM 占空比输出
}
```

### PID 控制器 (`Core/Src/pid.c`)

```c
float PIDController_Update(struct PIDController *pid, float error)
{
    // Tustin 梯形积分器
    // 积分抗饱和 (clamp to ±limit)
    // 输出斜率限制 (output ramp)
    // 返回限幅后的控制量
}
```

### 主循环状态机 (`Core/Src/main.c`)

```c
// Phase 0: 等待 ADC 电流零点校准完成 → 传感器对准 → 闭环启动
// Phase 2: 电流闭环运行 → 10 Hz VOFA+ 遥测 + 命令处理
```

---

## 依赖库

所有依赖库均已包含在工程目录中，无需外部包管理器:

| 库 | 路径 | 许可证 |
|-----|------|--------|
| STM32G4xx HAL | `Drivers/STM32G4xx_HAL_Driver/` | ST SLA |
| CMSIS Core | `Drivers/CMSIS/` | Apache 2.0 |
| CMSIS DSP | `Middlewares/ST/ARM/DSP/` | Apache 2.0 |
| TinyFoc (参考) | `TinyFoc-main/` | MIT |

---

## 参考工程

`TinyFoc-main/` 是 TinyFoc 在 STM32F401 上的原始实现，本项目将其核心算法移植到 STM32G431 平台并做了以下改进：

- 将 C++ 类重写为 C 结构体 + 函数，适配 Keil C99 编译环境
- 加入电流低通滤波，消除高频噪声
- 加入交叉解耦 + 反电动势前馈，提升动态响应
- 引入 VOFA+ 协议，实现无需额外硬件的在线调参
- PID 改用 Tustin 离散化，消除采样频率变化对积分项的影响

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
