# FOC_v1 - STM32G431 磁场定向控制 (FOC) 电机控制器

[![MCU](https://img.shields.io/badge/MCU-STM32G431CBT6-blue)](https://www.st.com/en/microcontrollers-microprocessors/stm32g431cb.html)
[![Core](https://img.shields.io/badge/Core-Cortex--M4%20with%20FPU-purple)](https://developer.arm.com/Processors/Cortex-M4)
[![GateDriver](https://img.shields.io/badge/GateDriver-DRV8323H-red)](https://www.ti.com/product/DRV8323)
[![Toolchain](https://img.shields.io/badge/Toolchain-Keil%20MDK--ARM%20v5%20%2B%20CubeMX-green)](https://www.keil.com/mdk5/)
[![Language](https://img.shields.io/badge/Language-C99-orange)](#)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## 项目简介

**FOC_v1** 是一个基于 STM32G431CBT6 微控制器的嵌入式无刷直流电机磁场定向控制 (FOC) 项目。功率级为 **DRV8323H 三相智能栅极驱动器 + 6 颗外置 N-MOSFET**，使用 14 位 AS5047P 磁旋转编码器进行位置反馈，**INA240A2 电流采样放大器 + 10 mΩ 分流电阻** 进行相电流采样，硬件 CORDIC 加速三角函数计算，实现完整的三环级联 FOC 控制（位置环 → 速度环 → 电流环），同时通过 USART2 DMA 与上位机 VOFA+ 进行实时数据交互与在线调参。

### 主要特性

- **主控**: STM32G431CBT6 (ARM Cortex-M4, 170 MHz, FPU, 硬件 CORDIC)
- **功率级**: DRV8323H 栅极驱动器 (3x PWM 模式) + 6× 外置 N-MOSFET，内置电荷泵支持 100% 占空比
- **相电流采样**: 双 INA240A2 (50 V/V) + 10 mΩ 分流，双 ADC 同步注入采样 + 16× 硬件过采样，自动零点校准 (2000 样本)
- **母线监测**: INA219 经 I2C1 读取母线电压/电流，5 Hz 更新；有效实测值参与 FOC 调制比换算，异常或过旧时禁能
- **位置传感器**: AS5047P 14 位磁旋转编码器 (SPI1 + DMA，非阻塞流水线读取，多圈累积)
- **控制算法**:
  - Clarke + Park 变换，SVPWM 空间矢量调制
  - 三环级联控制：位置环 (1 kHz P 控制) → 速度环 (2 kHz PI 控制) → 电流环 (20 kHz Iq/Id PI)
  - Iq/Id 双轴 PI 闭环控制（Tustin 离散化 + 双向抗饱和 + dq 矢量抗饱和回灌）
  - 交叉解耦 + 反电动势前馈补偿 (>1000 rad/s 时自动启用)
  - CORDIC 硬件加速 sin/cos 计算，一个 PWM 周期的流水线角度补偿
- **传感器对准**: 自动零电角度校准流程（静态电压矢量锁定 + 平均采样）
- **编码器失效保护**: 两级判定 — 瞬时陈旧则冻结角度并折减电流限幅，连续 20 帧 (1 ms) 则锁存停机
- **安全停机**: `foc_safe_stop()` 统一关断入口，可在任意上下文调用（清 CCR → 关 MOE → 关驱动器使能 → 清 PID 积分器）
- **上位机通信**: VOFA+ 遥测 14 通道 @100 Hz，文本命令在线调整全部参数（**460800 bps**）

串口总开关位于 `Core/Inc/app_config.h`：`APP_UART_ENABLE=1` 开启 UART2 命令、遥测和日志；`=0` 编译跳过 UART2 初始化，并由用户钩子关闭其 DMA 中断。默认开启；修改后需重新编译。也可在所有源文件的编译宏中统一定义此值。CAN 控制目前尚未实现，关闭串口后没有外部运动指令入口；上电在保护检查通过后仍自动电气对准和闭环启动。CubeMX 重新生成后应核对用户钩子并重新验证两种配置。

`APP_COMMAND_TIMEOUT_MS`默认`0`，表示人工值守台架保持最后目标。远程控制集成时应配置有限超时，并周期发送有效运动命令或`HB=1\n`保活；无效帧和单纯调参不续期。超时锁存停机，需要复位。保护阈值尚待电机额定与实测核定，软件配置和验证结果见[修复记录](docs/fix-results-2026-10-09.md)。

关节机械零点与 FOC 电气零点不同：轮子无需机械回零，但所有电机仍需有效的电气零点及电流采样偏置。当前 `foc_alignSensor()` 实现电气对准；关节机械零点设置属于 CAN 改造方案，尚未实现。
- **环路频率**: PWM 40 kHz、电流环 20 kHz，速度环 2 kHz，位置环 1 kHz，遥测 10 ms 周期
- **编码器接口**: 双缓冲缓存 (TIM2 ISR 写入, FOC 电流环读取)，约4 ms滑动窗口速度估算

---

## 硬件架构

当前默认母线为12 V。INA219在现有网表中直接接母线，器件输入额定及母线瞬态会约束整板范围；驱动器自身支持的电压范围不等于整板额定。整板最高安全电压尚未核定，不能按原图中的6–60 V接入电源。

```
        VBAT (当前软件默认 12V，整板额定待核定)
            │
      ┌─────┴──────────────────────────────┐
      │        DRV8323H (U9, QFN-40)       │
      │  MODE = 47kΩ→AGND  ⇒  3x PWM 模式  │
      │  ENABLE ← PB10 (DRV_EN, 高有效)     │
      │  INH1/2/3 ← PA9/PA10/PA8 (PWM)     │
      │  INL1/2/3 ← PB13/14/15 (静态高)    │
      │  nFAULT → PB9 (10kΩ 外部上拉)      │
      └───┬────────┬────────┬──────────────┘
        GH_A     GH_B     GH_C   (经 2.2Ω)
        GL_A     GL_B     GL_C   (经 2.2Ω)
          │        │        │
      ┌───┴──┐ ┌───┴──┐ ┌───┴──┐
      │Q1 Q2 │ │Q3 Q4 │ │Q5 Q6 │   三相半桥
      └───┬──┘ └───┬──┘ └───┬──┘
          │        │        │
          └────────┼────────┘
                   │
   MOTOR_A ──── R4 (10mΩ) ──── MOTOR_A_CS ─┐
   MOTOR_B ──── R23 (10mΩ) ────────────────┼──→ U4/U5 (INA240A2, 50V/V)
   MOTOR_C ──── R5 (10mΩ) ──── MOTOR_C_CS ─┘        │
                                                    │ SENSOR_A → PA0 (ADC1_IN1)
                                                    │ SENSOR_C → PA1 (ADC2_IN2)
```

> **3x PWM 模式说明**: `MODE` 引脚经 47 kΩ 接 AGND，DRV8323H 进入 3x PWM 模式，由器件内部根据 `INHx` 生成互补高低边驱动并插入死区。`INLx` 在此模式下不参与逻辑，但**必须保持确定电平以避免输入级悬空误触发**，故 PB13/PB14/PB15 上电即置高并全程保持。
>
> **B 相电流**由基尔霍夫定律推导 (`I_B = -(I_A + I_C)`)，不做独立采样。

### 引脚分配

功能           | 引脚 / 外设                       | 说明
--------------|-----------------------------------|-----------------------------
**编码器 SPI** | SPI1: PA4(CS) PA5(SCK) PA6(MISO) PA7(MOSI) | AS5047P 14 位磁编码器，5.3125 MHz
**UART2 遥测** | USART2: PB3 TX, PB4 RX            | **460800 bps**，VOFA+ 协议
**电机 PWM**   | TIM1: PA8(CH1) PA9(CH2) PA10(CH3) | 3 路，驱动 DRV8323H 的 INH1/2/3
**驱动使能**   | PB10 (DRV_EN)                     | DRV8323H `ENABLE`，高有效
**驱动低边绑带** | PB13, PB14, PB15                | 3x 模式 INL1/2/3，静态高，运行期不写
**驱动故障**   | PB9 (NFAULT)                      | 输入，外部 10 kΩ 上拉（**当前固件未读取**）
**相电流采样** | PA0 (ADC1_IN1), PA1 (ADC2_IN2)    | 双 ADC 注入同步采样 + 16× 过采样
**母线监测**   | I2C1: PA15(SCL) PB7(SDA)          | INA219，地址 0x40
**调试串口**   | SWD: PA13(CLK) PA14(DIO)          | 测试点 CLK / DIO
**状态指示灯** | PC13 → LED2                       | 低电平点亮（当前固件未驱动）
**板载预留**   | CAN 收发器 + 120Ω 终端            | 收发器 STBY 接地，固件未启用

### 时钟与电源

| 项目 | 配置 |
|------|------|
| HSE | **8 MHz 有源振荡器**（4 脚 OSC1）→ `RCC_HSE_BYPASS` |
| PLL | M=2, N=85, R=2 → **SYSCLK 170 MHz** |
| Flash 等待周期 | 4 WS |
| 稳压器 | `PWR_REGULATOR_VOLTAGE_SCALE1_BOOST` |
| ADC 时钟 | SYSCLK / 4 = 42.5 MHz |

> 使用有源振荡器而非晶振，频率精度优于 ±50 ppm，是 460800 bps 可靠通信的物理基础（波特率寄存器误差仅 −0.0038%）。

---

## 目录结构

```
FOC_v1/
├── Core/
│   ├── Inc/                       # 应用层头文件
│   │   ├── main.h                 # 主程序配置、引脚宏
│   │   ├── foc.h                  # FOC 核心 — SVPWM、电流环、安全停机、限值常量
│   │   ├── pid.h                  # PID 控制器 — Tustin 积分器 + 输出限幅 + 默认参数
│   │   ├── vofa.h                 # VOFA+ 遥测 + 命令接收
│   │   ├── uart_tx.h              # UART2 发送环形缓冲（单生产者）
│   │   ├── utils.h                # 工具函数 — DWT 微秒计时、LPF、角度归一化
│   │   ├── as5047p.h              # AS5047P 编码器 SPI+DMA 底层驱动
│   │   ├── as5047p_ext.h          # AS5047P 高层接口 (角度/速度/多圈/陈旧判定)
│   │   ├── ina219.h               # 母线电压/电流监测 (I2C1)
│   │   ├── cordic.h               # CORDIC 硬件加速器配置 (CubeMX)
│   │   ├── adc.h / dma.h / spi.h / tim.h / usart.h / i2c.h / gpio.h
│   │   ├── stm32g4xx_it.h         # 中断服务函数声明 (CubeMX)
│   │   └── stm32g4xx_hal_conf.h   # HAL 模块配置 (CubeMX)
│   └── Src/                       # 应用层源码
│       ├── main.c                 # 主循环 + 状态机 + ADC 中断回调 + 遥测 + Error_Handler
│       ├── foc.c                  # FOC 算法：Clarke/Park、三环控制、前馈、SVPWM、安全停机
│       ├── pid.c                  # PID 控制器实现 + 参数初始化
│       ├── vofa.c                 # VOFA+ 协议：遥测发送 + 命令解析
│       ├── uart_tx.c              # 发送环形缓冲实现
│       ├── utils.c                # 工具函数实现
│       ├── as5047p.c              # AS5047P SPI+DMA 驱动 (临界区保护)
│       ├── as5047p_ext.c          # AS5047P 高层接口：速度窗口/多圈/缓存/陈旧判定
│       ├── ina219.c               # INA219 寄存器读写与换算
│       ├── cordic.c / adc.c / dma.c / spi.c / tim.c / usart.c / i2c.c / gpio.c
│       ├── system_stm32g4xx.c     # 系统时钟初始化 (170 MHz)
│       ├── stm32g4xx_hal_msp.c    # HAL MSP 层 (CubeMX)
│       └── stm32g4xx_it.c         # 中断服务函数 (CubeMX)
├── Drivers/
│   ├── CMSIS/                     # ARM CMSIS 核心 + 设备支持
│   └── STM32G4xx_HAL_Driver/      # STM32G4 HAL 驱动库
├── Middlewares/
│   └── ST/ARM/DSP/                # CMSIS-DSP (arm_cortexM4lf_math.lib)
├── MDK-ARM/
│   ├── FOC_v1.uvprojx             # Keil uVision 工程文件
│   ├── FOC_v1.uvoptx              # Keil 工程选项
│   └── startup_stm32g431xx.s      # 启动汇编文件 (向量表)
├── docs/
│   ├── code-review-2026-07-17.md  # 代码审查报告与整改方案
│   └── superpowers/               # 电流环自动整定工具的设计与计划
├── 资料/
│   └── Netlist_PCB1_2_2026-09-30.tel   # 原理图网表
├── FOC_v1.ioc                     # CubeMX 工程配置
├── .gitignore
└── README.md
```

---

## 快速开始

### VOFA+ 上位机连接

连接 USART2 (PB3 TX, PB4 RX)，波特率 **460800**：

- **遥测接收**: 打开 VOFA+，添加 14 通道数据显示
- **命令发送**: 在 VOFA+ 终端输入文本命令，格式为逗号分隔的键值对；发送设置勾选“追加换行”，推荐 LF 或 CRLF，也支持单独 CR。未带结束符的内容会等待后续字节，不会执行。

> ⚠️ **波特率必须设为 460800**。115200 下 14 通道遥测帧的带宽需求 (17.8 kB/s) 超过链路能力 (11.52 kB/s)，会导致发送队列持续积压。

支持的遥测通道 (14 通道, 100 Hz)：

通道    | 变量              | 说明
--------|-------------------|------------------
[0]     | id_target         | D 轴目标电流 (A)
[1]     | id_meas           | D 轴实测电流 (A)
[2]     | iq_target         | Q 轴目标电流 / 转矩指令 (A)
[3]     | iq_meas           | Q 轴实测电流 (A)
[4]     | velocity_raw      | 滑动窗口原始速度 (rad/s)
[5]     | velocity_filt     | 速度环滤波后角速度 (rad/s)
[6]     | speed_setpoint    | 速度目标值 (rad/s)
[7]     | speed_kp_active   | 速度环当前生效的 P 增益（增益调度）
[8]     | status_flag       | 命令同步标志 (收到命令置 1，发送后清零)
[9]     | mode              | 控制模式 (0=转矩, 1=速度, 2=位置)
[10]    | position_target   | 位置目标值 (多圈 rad)
[11]    | position_meas     | 实测多圈位置 (rad)
[12]    | bus_voltage       | INA219 母线电压 (V)
[13]    | bus_current       | INA219 母线电流 (A)

支持的命令：

命令  | 说明                              | 示例
------|-----------------------------------|--------
T=V   | 转矩 / Iq 电流指令 (A)            | `T=0.5`
D=V   | D 轴电流目标 (A)                  | `D=0.0`
P=V   | Q 轴电流环 P 增益                 | `P=10.485`
I=V   | Q 轴电流环 I 增益                 | `I=371.25`
DP=V  | D 轴电流环 P 增益                 | `DP=10.485`
DI=V  | D 轴电流环 I 增益                 | `DI=371.25`
S=V   | 速度目标值 (rad/s)                | `S=50`
SP=V  | 速度环 P 增益（会同时关闭增益调度） | `SP=0.09`
SI=V  | 速度环 I 增益                     | `SI=0.1`
SG=V  | 增益调度开关 (0=手动 SP, 1=调度)  | `SG=1`
SPL=V | 调度低速区 P 增益                 | `SPL=0.09`
SPH=V | 调度高速区 P 增益                 | `SPH=0.02`
PS=V  | 位置目标值 (多圈 rad)             | `PS=6.28`
PR=V  | 相对位置增量 (rad，以实测位置为基准，限幅 ±2π) | `PR=1.57`
PP=V  | 位置环 P 增益 (rad/s per rad)     | `PP=10`
PL=V  | 位置环速度限幅 (rad/s，`PSL` 的别名) | `PL=100`
PSL=V | 位置环速度限幅 (rad/s)            | `PSL=100`
PAL=V | 位置轨迹加减速度限幅 (rad/s²)     | `PAL=40`
CVL=V | 电流环 dq 电压矢量限幅 (V)        | `CVL=6`
SIL=V | 速度环 Iq 电流限幅 (A)            | `SIL=0.6`
M=V   | 控制模式 (0=转矩, 1=速度, 2=位置) | `M=1`
HB=1  | 外部控制保活（启用超时配置时续期） | `HB=1`

示例: `M=2,PS=6.28,PP=10,PSL=100` — 切换到位置模式，目标 1 圈，P=10，限速 100 rad/s

> **命令语义**: 必须以 LF（`0A`）、CRLF（`0D 0A`）或 CR（`0D`）结束；一帧先完整校验再在临界区内提交，`M=1,S=4`与`S=4,M=1`等价。未知键、畸形数值、非有限值、越界值或未就绪/故障状态拒绝整帧，不执行前缀。只有接受的命令置`status_flag=1`。同模式标签不重置动态状态。`T/D`共用dq矢量限值，`SIL`不能超过当前候选硬上限1.5 A。

连接排查可在闭环遥测开始后发送 `HB=1` 并追加 LF：该指令不改变运动目标，接受后遥测通道[8]会有一帧为1。不带结束符时既不会执行，也不会出现`[CMD] rejected`；若出现拒绝日志，检查命令格式、数值范围及启动/故障状态。脚本应发送实际控制字符，例如 Python 的 `port.write(b"HB=1\n")`。

---

## 控制架构

```
  位置外环 (1 kHz, TIM3)      速度外环 (2 kHz, 降采样)      电流内环 (20 kHz, ADC ISR)
  ┌─────────────────────┐   ┌──────────────────────┐   ┌────────────────────────────┐
  │ pos_setpoint        │   │ speed_setpoint       │   │                            │
  │     │               │   │     │                │   │  ┌───────────────────────┐ │
  │  ┌──▼───┐           │   │  ┌──▼───┐            │   │  │ CORDIC 硬件 sin/cos   │ │
  │  │ P 控制│──speed_cmd┼──►│ PI 控制│──Iq_cmd────┼──►│  │ (Q31 写入, 同帧读取)   │ │
  │  └──────┘           │   │ └──────┘            │   │  └───────────────────────┘ │
  │     ▲               │   │     ▲               │   │                            │
  │     │ pos_meas      │   │     │ vel_filt      │   │  Iq_cmd──►[PI Iq]──►[+]──┐ │
  └─────┼───────────────┘   └─────┼───────────────┘   │            ▲        ▲    │ │
        │                         │                   │   iq_meas  │  Vq_ff │    │ │
        │                         │                   │            │        │    │ │
  ┌─────┴─────────────────────────┴───────────────────┴────────────┴────────┴────┴─┴─┐
  │                 编码器双缓冲无锁缓存 (TIM2 ISR, 20 kHz)                            │
  │  单圈角度 [0,2π) · 滑动窗口速度 [rad/s] · 多圈累积总角度 [rad] · 陈旧标志          │
  └────────────────────────────┬──────────────────────────────────────────────────────┘
                               │
  ┌────────────────────────────┴──────────────────────────────────────────────────────┐
  │  AS5047P SPI+DMA (20 kHz)   │  双 ADC 注入采样     │  交叉解耦 + 反电动势前馈        │
  │  非阻塞流水线读取            │  INA240A2 ×2         │  Vd_ff=-ωLq·Iq (>1000rad/s)   │
  └────────────────────────────┬──────────────────────────────────────────────────────┘
                               │
  Id_cmd──►[PI Id]──►[+]──┐    │             ┌──────────┐
            ▲        ▲    │    │             │ 反Park   │◄── Vd,Vq
   id_meas  │  Vd_ff │    │    │             │ Vd,Vq→αβ │
            │        │    │    │             └────┬─────┘
            │   ┌────┴────┴────┴──┐               │
            │   │ dq 矢量抗饱和    │          ┌────┴─────┐
            │   │ (幅值 + 占空比)  │          │  SVM     │
            │   └─────────────────┘          │ αβ→占空比 │
            │                                └────┬─────┘
            │                                     │
      ┌─────┴──────┐                          ┌────┴─────┐
      │ Park 变换   │                          │ TIM1 PWM │
      │ Iα,Iβ→Id,Iq │                          │ 3 路输出  │
      └─────▲──────┘                          └──────────┘
            │
      ┌─────┴──────┐
      │ Clarke 变换 │
      │ Ib,Ic→Iα,Iβ │
      └─────▲──────┘
            │
      ┌─────┴──────┐
      │ LPF α=0.05 │
      │ fc≈163 Hz  │
      └─────▲──────┘
            │
    双 ADC 相电流 (INA240A2)
```

### 环路时序

| 环路 | 触发源 | 频率 | 备注 |
|------|--------|------|------|
| PWM 载波 | TIM1 中心对齐 | 39.98 kHz | ARR=2125, RCR=3 |
| 电流环 | TIM1 TRGO → ADC 注入完成 | 20 kHz (50 µs) | 在 ADC ISR 中执行 |
| 编码器 | TIM2 更新中断 | 20 kHz (50 µs) | SPI+DMA 流水线读取 |
| 速度环 | 电流环降采样 | 2 kHz (500 µs) | `SPEED_DECIMATION=10` |
| 位置环 | TIM3 更新中断 | 1 kHz (1 ms) | 仅位置模式生效 |
| 遥测 | 主循环节流 | 100 Hz (10 ms) | 14 通道 |

### 中断优先级

| 优先级 | 中断 | 说明 |
|--------|------|------|
| 0 | ADC1_2, TIM1_UP, CORDIC | FOC 热路径，`BASEPRI` 临界区不屏蔽 |
| 1 | TIM2, SPI1, DMA1_CH1/2 | 编码器与 SPI 流水线 |
| 2 | TIM3 | 位置环 |
| 3 | USART2, DMA1_CH3/4 | 通信 |
| 15 | SysTick | HAL 时基 |

---

## 安全机制

### 安全启动顺序

1. **上电保持关断**: `DRV_EN` (PB10) 上电为低；先进行编码器诊断，最多10轮重试，连续两组有效诊断/幅值才通过。失败时保留关断，不启动PWM、ADC或电流校准
2. **电流零点校准**: 驱动器关断状态下采集2000个有效样本，检查偏置、峰峰值及量程余量；500 ms内未完成则锁存故障
3. **驱动器使能**: 校准、母线和编码器诊断通过后，取得新鲜角度再置高`DRV_EN`；等待nFAULT连续5 ms为高，总等待上限50 ms
4. **传感器对准**: `foc_alignSensor()` 从零渐升静态电压矢量 (0 → 3 V, 100 ms)，保持 300 ms，降回 1 V 后采样零电角度，再平滑撤除电压
5. **进入闭环**: 对准总预算1 s、零角平均采样预算100 ms；结果须稳定且新鲜。状态/PID初始化后再次检查故障，由统一入口原子设置`current_loop_enable`和`motor_ready`

### 编码器失效两级保护

`AS5047P_EncoderCache_Read()` 在读取时刻计算角度年龄（阈值 2 ms）。电流环据此分级响应：

| 级别 | 条件 | 动作 |
|------|------|------|
| 0 | 正常 | 正常角度外推（上限 200 µs） |
| 1 | 数据陈旧，连续帧数 < 20 | **停止外推**（冻结在最后已知角度）+ 电流限幅折减至 30% |
| 2 | 数据陈旧，连续 ≥ 20 帧 (1 ms) | 置位`encoder_lost_latched`，锁存编码器故障并停止驱动 |

连续帧计数在角度年龄超过2 ms之后开始。另在DMA请求中每400次插入诊断读取，诊断响应不发布成角度；诊断缺少就绪位或磁场/运算状态异常会锁存故障。正常角度0仍合法。运行诊断间隔约20 ms，不能宣称对所有通信故障立即响应。

串口`[FAULT] reason=2`表示编码器故障。对应的`[ENC] cause=...`行提供最近诊断：`SPI`/`PARITY`/`EF`为通信错误，`LF_NOT_READY`为尚未就绪，`COF`为运算异常，`MAG_LOW`/`MAG_HIGH`为磁场异常，`MAG_ZERO`为幅值为零，`NO_DATA`为全1响应，`UNSTABLE`为未取得连续两组有效诊断。`dia`/`mag`保留完整16位响应，包含校验和EF位；`spi`为两个读取的HAL状态（0=OK、1=ERROR、2=BUSY、3=TIMEOUT），`err`斜杠后为ERRFL记录是否有效。运行期`mag`仍是启动读数；最近诊断正常也不能排除角度帧陈旧。

### 安全停机 `foc_safe_stop()`

`foc_safe_stop()`保留兼容签名，软件故障由`motor_fault_trip()`统一处理：

```
1. 锁存首次故障原因
2. DRV_EN = 0，清TIM1 MOE    → 先禁能，再清CCR；不依赖CCR预装载生效
3. 保存首次故障快照          → 主循环异步记录，ISR不格式化日志
4. 清目标、PID积分/历史输出与滤波状态
5. current_loop_enable、motor_ready、alignment_in_progress = 0
```

锁存后所有使能和PWM输出入口拒绝恢复。CPU异常入口与`Error_Handler`先调用最小寄存器关断，不调用完整控制器清理。复位后会重新校准和对准；IWDG及异常复位保持禁能策略尚未实施。

> ⚠️ **当前无自动恢复机制**：故障停机或 `Error_Handler` 触发后，需**复位 MCU** 才能重新使能。这是刻意选择——DRV8323H 的 OCP/OTW 等故障具有自恢复特性，自动重入闭环会在功率级瞬间施加积分器里积攒的满幅指令。

### 其他保护

- **对齐阶段过流**: `FOC_ALIGN_CURRENT_LIMIT_A = 1.0 A`，持续 3 ms 即中止对齐并关断
- **占空比限幅抗饱和**: 占空比被限幅时，通过 `foc_duty_to_dq()` 严格反算实际施加的 dq 电压并回灌给积分器
- **dq 电压矢量抗饱和**: SVM 线性区限幅后把实际施加量回算到 Id/Iq 积分器
- **ADC与相电流**: 未滤波三相电流保护在ADC回调入口执行，对准期间同样生效；检查可信码窗口、单帧严重过流及各相连续过流。TIM1监测ADC停更（2 ms预算），nFAULT运行期轮询
- **阈值状态**: 默认参考0.6 A；新增1.5/1.8/2.4 A等配置为初始候选，尚未验证电机、整板额定及实测噪声，不代表安全额定。软件保护不能覆盖所有短时尖峰、nFAULT短脉冲或ISR卡死

---

## 开发状态

### 已完成

- [x] 系统时钟: 8 MHz 有源振荡器 → PLL @ 170 MHz
- [x] AS5047P 编码器 SPI+DMA 流水线驱动 (临界区保护)
- [x] 编码器高层接口: 角度/速度/多圈累积 + 双缓冲无锁缓存 + 陈旧判定
- [x] 双 ADC 注入同步采样 + 16× 硬件过采样 + 自动零点校准 (2000 样本)
- [x] DRV8323H 3x PWM 栅极驱动 (TIM1 三路，死区由器件内部生成)
- [x] INA240A2 相电流采样通路定标 (10 mΩ + 50 V/V，已对网表核实)
- [x] INA219 母线电压/电流监测 (I2C1, 5 Hz，有效性与时间戳)
- [x] CORDIC 硬件加速器同帧 sin/cos 计算
- [x] Clarke + Park 变换，6 扇区 SVPWM
- [x] Iq/Id 双轴 PI 电流闭环 (Tustin 离散化 + 双向抗饱和 + dq 矢量与占空比双重抗饱和)
- [x] 交叉解耦 + 反电动势前馈补偿 (>1000 rad/s，默认关闭)
- [x] 速度闭环 — 2 kHz PI + 两段 P 增益调度 (8~9 rad/s 切换，bumpless 传递)
- [x] 位置闭环 — 1 kHz P 控制 + 梯形速度轨迹 (加速度约束 + 剩余距离制动)
- [x] 三模式控制: 转矩 / 速度 / 位置，在线切换
- [x] 传感器自动对准 (静态电压矢量 + 平均采样零电角度校准)
- [x] `foc_safe_stop()` 统一安全停机 + `Error_Handler` 关断功率级
- [x] 编码器失效两级保护
- [x] UART2 发送环形缓冲 (单生产者，TX暂存区保持至完成或确认中止)
- [x] VOFA+ 14 通道遥测 @100 Hz (460800 bps)
- [x] VOFA+ 文本命令接收与解析 (21 键在线调参、模式切换、轨迹参数)
- [x] DWT 微秒级计时
- [x] 默认 PID 参数集中管理 (pid.h)

### 已知限制 / 待办

- [ ] **保护阈值与ISR时序上板核定**：软件已有参考限值、实测过流、ADC窗口和校准资格检查；参数仍为候选，关断后续流及母线瞬态需测量
- [ ] **nFAULT短脉冲/硬件直达关断**：当前已接入ADC入口轮询；EXTI、模拟看门狗及TIM1 Break路由尚未实施
- [ ] **无看门狗**: `Error_Handler` 的"等待复位"目前没有复位源，实际行为是停住（功率级已关断，安全但需人工干预）
- [ ] **CubeMX实际再生成回归**：用户钩子已保存条件行为，但仍需在副本中实际再生成并验收
- [ ] 参数自动保存到 Flash
- [ ] CAN 总线通信（收发器与 120Ω 终端已在板上，但 STBY 接地且固件未启用）
- [ ] PC13 状态指示灯未驱动
- [ ] DRV8323H 内置三路低边 CSA 处于闲置状态（当前用外置 INA240A2）

---

## 已知未修复缺陷

旧审查报告中的事项如下；最新修复与验证边界见[2026-10-09修复记录](docs/fix-results-2026-10-09.md)。

| 级别 | 内容 |
|------|------|
| 高 | `CVL` 上限允许进入占空比限幅区（已加抗饱和，但限值本身仍偏大） |
| 中 | `SVM()` 返回值为 -1 时未使用上一帧占空比（当前因限幅配置不会触发，属脆弱依赖） |
| 中 | 速度环误差乘 `motor_config.dir`，使 `dir=-1` 时抗饱和判据语义反转 |
| 中 | 速度估计器 4 ms 定长窗口导致低速 0.096 rad/s 量化阶梯 |
| 中 | 命令键当前最多3字符；超长/未知键明确拒绝，扩展协议时需更新解析器 |
| 中 | INA219仍为阻塞式I2C，每次读写预算3 ms；多个事务合计延迟仍需实测 |
| 低 | `SVPWM_Update()` 无调用者；`AS5047P_ReadAnglePipeline()` 有声明无定义 |
| 低 | `motor_control_t` 中多个字段定义未使用 |
| 低 | `.ioc` 的 `Dma.USART2_RX.3.Mode=DMA_CIRCULAR` 被 `vofa.c` 运行时改为 `DMA_NORMAL`，CubeMX 重新生成后会失效 |

---

## CubeMX 维护约定

重新生成代码后，请确认以下内容未被回退（**全部位于 `USER CODE` 区间内，理论上不会被清空，但仍建议核对**）：

| 文件 | 位置 | 内容 |
|------|------|------|
| `usart.c` | `USER CODE BEGIN 0` | `extern volatile uint8_t huart2_ready;` |
| `usart.c` | `USER CODE BEGIN USART2_Init 2` | `huart2_ready = 1U;` |
| `usart.c` | 生成区 | `huart2.Init.BaudRate = 460800;` |
| `tim.c` | `USER CODE BEGIN 0` | `extern volatile uint8_t htim1_ready;` |
| `tim.c` | `USER CODE BEGIN TIM1_Init 2` | `htim1_ready = 1U;` |

**CubeMX 中的关键配置**（改动后将影响固件行为）：

| 外设 | 配置 | 原因 |
|------|------|------|
| USART2 | 波特率 **460800** | 14 通道遥测带宽需求 |
| TIM1 | 中心对齐1, ARR=2125, RCR=3, 死区=0 | 40 kHz 载波 / 20 kHz 触发；死区由 DRV8323H 内部生成 |
| ADC1/ADC2 | 双模式注入同步, 16× 过采样, 右移 4 | 与 `CURRENT_FACTOR` 定标配套 |
| PB9 (NFAULT) | 输入，无上下拉 | 外部已有 10 kΩ 上拉 |
| PB13/14/15 | 输出，上电置**高** | DRV8323H 3x 模式 INLx 需确定电平 |
| PB10 (DRV_EN) | 输出，上电置**低** | 保证上电时功率级关断 |

> ⚠️ **不要**将 PB13/PB14/PB15 的上电电平改为低——它们必须为高才能保证 3x 模式输入级不悬空。

**新增源文件时**: `uart_tx.c` 需手动加入 Keil 工程的 `Application/User/Core` 分组（CubeMX 不管理非生成文件）。

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
- [Texas Instruments](https://www.ti.com/) — DRV8323H 栅极驱动器、INA240A2 电流采样放大器、INA219 母线监测
- [ams-OSRAM](https://ams.com/) — AS5047P 磁旋转编码器
