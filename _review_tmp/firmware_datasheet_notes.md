# Cubli-F407ZGT6 主控板原理图审查 — 固件引脚配置提取 + 数据手册调研

> 生成日期：2026-10-07。任务A固件事实全部来自磁盘上实际存在的文件（`无刷闭环联调/Cubli-ST`、`Cubli-ESP`、`Software`），已逐文件核对，与 git 历史无关。
> 任务B所有数据手册事实均注明来源 URL；未能查到的项已明确标注"查不到"，无任何编造。

---

# 任务A：固件引脚配置提取

## A.0 固件目录实际状态（以磁盘为准）

```
无刷闭环联调/
├── Cubli-ST/          STM32F407ZGT6 Keil MDK 工程（含 CubeMX.ioc）
│   ├── CubeMX.ioc     ★ 完整 CubeMX 工程描述（360 行，纯文本）
│   ├── Core/Src       main.c gpio.c tim.c i2c.c usart.c stm32f4xx_it.c stm32f4xx_hal_msp.c system_stm32f4xx.c
│   ├── Core/Inc       main.h MyProject.h gpio.h i2c.h tim.h usart.h stm32f4xx_hal_conf.h stm32f4xx_it.h
│   ├── Hardware/      delay.c/h enc_quad.c/h lqr_balance.c/h lsm6dsr.c/h mpu6050.c/h spi.c/h timer.c/h
│   ├── SimpleFOC/     BLDCMotor.c/h FOCMotor.c/h CurrentSense.c/h foc_utils.c/h
│   └── MDK-ARM/       Keil 工程 + CubeMX 输出
├── Cubli-ESP/         PlatformIO + Arduino 工程（platformio.ini, src/main.cpp）
├── Software/          PyQt5 上位机（main.py, README.md, requirements.txt, run_upper_computer.bat）
├── Netlist_MB_Schematic_2026-10-07.net  板级网表（编码器正交解码注释中引用）
└── 修改说明_审查修复与无线化升级.md / 改动文件说明.md
```

## A.1 CubeMX.ioc 完整解析（Mcu.Pin 列表，引脚→Signal）

MCU：STM32F407ZGT6，封装 LQFP144，`MxCube.Version=6.17.0`，固件包 STM32Cube FW_F4 V1.28.3，工具链 MDK-ARM V5.32。
**.ioc 中没有任何 `GPIO_Label`（用户标签）条目** —— 所有引脚均为无标签状态；工程内"标签"只存在于源码注释与宏定义中（见 A.6）。

| 引脚 | Signal (ioc) | 模式/备注 |
|---|---|---|
| PH0-OSC_IN | RCC_OSC_IN | HSE-External-Oscillator |
| PH1-OSC_OUT | RCC_OSC_OUT | HSE-External-Oscillator |
| PA0-WKUP | S_TIM5_CH1 | TIM5 编码器 CH1 |
| PA1 | S_TIM5_CH2 | TIM5 编码器 CH2 |
| PA5 | S_TIM2_CH1_ETR | TIM2 编码器 CH1 |
| PA6 | S_TIM3_CH1 | TIM3 PWM CH1 |
| PA7 | S_TIM3_CH2 | TIM3 PWM CH2 |
| PB0 | S_TIM3_CH3 | TIM3 PWM CH3 |
| PB3 | S_TIM2_CH2 | TIM2 编码器 CH2 |
| PB6 | I2C1_SCL | I2C，上拉，LOW speed |
| PB7 | I2C1_SDA | I2C，上拉，LOW speed |
| PB10 | GPIO_Output | VeryHigh speed（运行时被 MPU6050 软 I2C 重配为开漏，见 A.4） |
| PB11 | GPIO_Output | VeryHigh speed（同上） |
| PB12 | GPIO_Output | VeryHigh speed（源码用作 LSM6DSR 软件 CS） |
| PB13 | SPI2_SCK | 上拉，VeryHigh speed |
| PB14 | SPI2_MISO | 上拉，VeryHigh speed |
| PB15 | SPI2_MOSI | 上拉，VeryHigh speed |
| PC6 | S_TIM8_CH1 | TIM8 编码器 CH1 |
| PC7 | S_TIM8_CH2 | TIM8 编码器 CH2 |
| PC9 | I2C3_SDA | I2C，上拉，LOW speed |
| PA8 | I2C3_SCL | I2C，上拉，LOW speed |
| PA9 | USART1_TX | 异步 |
| PA10 | USART1_RX | 异步 |
| PA13 | SYS_JTMS-SWDIO | Serial_Wire |
| PA14 | SYS_JTCK-SWCLK | Serial_Wire |
| PD0 | GPIO_Output | Low speed（源码：M1_EN） |
| PD1 | GPIO_Output | Low speed（源码：M2_EN） |
| PD2 | GPIO_Output | Low speed（源码：M3_EN） |
| PD5 | USART2_TX | 异步 |
| PD6 | USART2_RX | 异步 |
| PD7 | GPIO_Input | 无上/下拉 |
| PD12 | S_TIM4_CH1 | TIM4 PWM CH1 |
| PD13 | S_TIM4_CH2 | TIM4 PWM CH2 |
| PD14 | S_TIM4_CH3 | TIM4 PWM CH3 |
| PE9 | S_TIM1_CH1 | TIM1 PWM CH1 |
| PE11 | S_TIM1_CH2 | TIM1 PWM CH2 |
| PE13 | S_TIM1_CH3 | TIM1 PWM CH3 |
| PF0 | I2C2_SDA | I2C，上拉，LOW speed |
| PF1 | I2C2_SCL | I2C，上拉，LOW speed |
| PG0~PG8 (9 根) | GPIO_Output | 推挽、无上拉、Low speed，初始电平 RESET（低） |
| VP_SYS_VS_Systick | SYS_VS_Systick | SysTick |

启用外设（Mcu.IP）：I2C1、I2C2、I2C3、NVIC、RCC、SYS、TIM1、TIM2、TIM3、TIM4、TIM5、TIM8、USART1、USART2。
NVIC：`USART2_IRQn=true`（生成中断入口，实际 `stm32f4xx_it.c` 中 USART2_IRQHandler 与用户添加的 USART1_IRQHandler 都调用 `USART_Link_IRQHandler()`，作为调试链路共用处理）。

## A.2 TIM 配置（PWM / 编码器）

### PWM 定时器（`CubeMX.ioc` + `Core/Src/tim.c`）

| 定时器 | 通道/引脚 | 模式 | Prescaler | Period(ARR) | 计数模式 | 极性/死区 | 实际 PWM 频率 |
|---|---|---|---|---|---|---|---|
| TIM1 | CH1=PE9, CH2=PE11, CH3=PE13 | PWM Generation 1/2/3 | 0 | 3360 | CENTERALIGNED1 | OCMode=PWM1, OCPolarity=HIGH, OCIdleState=RESET, DeadTime=0, Break 禁用, RepetitionCounter=0 | 168 MHz/(2×3360) ≈ **25.0 kHz**（APB2 定时器时钟 168MHz） |
| TIM3 | CH1=PA6, CH2=PA7, CH3=PB0 | PWM Generation 1/2/3 | 0 | 1679 | CENTERALIGNED1 | 同上（无 BDTR，通用定时器） | 84 MHz/(2×1679) ≈ **25.0 kHz**（APB1 定时器时钟 84MHz） |
| TIM4 | CH1=PD12, CH2=PD13, CH3=PD14 | PWM Generation 1/2/3 | 0 | 1679 | CENTERALIGNED1 | 同上 | ≈ **25.0 kHz** |

- TIM1 在 `MX_TIM1_Init()` 内由用户代码调用 `HAL_TIM_MspPostInit` 并启动 PWM 且 `TIM1->BDTR |= TIM_BDTR_MOE`（主输出使能）。
- TIM3/TIM4 的 PWM Start 在 main.c USER CODE 2 中执行。
- 电机映射（main.c 注释 + Software/README）：**M1=TIM1(PE9/11/13)，M2=TIM3(PA6/PA7/PB0)，M3=TIM4(PD12/13/14)**。
- ⚠️ 审查注意：`Hardware/timer.c` 头注释写 "TIM1 ... 25KHz, 中心对齐, deadtime=60ns"，但 `tim.c` 实际 `DeadTime = 0`（无死区），且 `timer.h` 注释用 `PWM_Period_TIM1=3359` 而实际 ARR=3360 —— 注释与代码不一致（频率结论不受影响，死区结论必须以代码为准：**死区=0**）。

### 编码器定时器

`.ioc` 中 TIM2/TIM5/TIM8 声明为 `Encoder_Interface`（SH.S_TIMx_CHx 条目），但 CubeMX 只生成了 PWM 的 tim.c —— **编码器初始化实际是 `Hardware/enc_quad.c` 的寄存器级用户代码**（由宏开关选择，见 A.4）：

| 定时器 | 电机 | A 相 | B 相 | AF | 配置（enc_quad.c） |
|---|---|---|---|---|---|
| TIM2 | M1 | PA5 | PB3 | AF1_TIM2 | SMS=011（编码器模式3：TI1&TI2 双边沿 ×4），IC1→TI1、IC2→TI2，IC1F/IC2F=N=8 滤波，CCER=0（均不反相），ARR=0xFFFF |
| TIM5 | M2 | PA0 | PA1 | AF2_TIM5 | 同上 |
| TIM8 | M3 | PC6 | PC7 | AF3_TIM8 | 同上 |

即：TIM2/TIM5/TIM8 = **Encoder Mode TI1+TI2（×4 正交解码）**，`ENC_QUAD_COUNTS_PER_REV = 1024`（enc_quad.h，注释说明若 MT6701 ABZ 出厂配置不同需改此处）。

### 其它定时器（用户代码，`Hardware/timer.c`）
- **TIM6**：1ms 时基中断（PSC=84-1→1MHz，ARR=1000-1，TIM6_DAC_IRQn 优先级 1）。
- **TIM5（默认模式，ENC_USE_HW_QUAD=0）**：32 位自由计数微秒时基（PSC=84-1，ARR=0xFFFFFFFF，约 71.5 分钟回绕）。
- **TIM7（ENC_USE_HW_QUAD=1 时替代 TIM5）**：16 位 1MHz 微秒时基 + 溢出中断扩展（PSC=84-1，ARR=0xFFFF，TIM7_IRQn 优先级 0）。
- **conflict 提示**：`.ioc` 里 TIM5 是 M2 编码器；默认固件 `ENC_USE_HW_QUAD=0` 时 TIM5 又被 micros() 占用 —— 两种方式互斥，由编译期宏保证。

## A.3 时钟树（ioc RCC 段 + main.c SystemClock_Config 一致）

| 项 | 值 |
|---|---|
| HSE | **25 MHz** 外部晶振（HSE_VALUE=25000000） |
| PLL | M=25, N=336, P=2, Q=4, 源=HSE |
| SYSCLK | **168 MHz**（PLLCLK） |
| AHB | 168 MHz（/1） |
| APB1 | 42 MHz（/4），**APB1 定时器时钟 = 84 MHz** |
| APB2 | 84 MHz（/2），**APB2 定时器时钟 = 168 MHz** |
| PLLQ | 84 MHz；I2S=96 MHz；LSE=32.768k；LSI=32k |

## A.4 软件 I2C 与编码器 I2C（回答任务 A 第 2 项）

### 硬件 I2C ×3（编码器 MT6701，`Core/Src/i2c.c` + `Core/Inc/i2c.h`）
- **I2C1：PB6=SCL，PB7=SDA → MT6701 #1（M1）**；400kHz Fast Mode，DutyCycle 2，7bit，NoStretch 禁用；MSP 中 GPIO 为 AF_OD + **内部上拉**。
- **I2C2：PF1=SCL，PF0=SDA → MT6701 #2（M2）**；配置同上。
- **I2C3：PA8=SCL，PC9=SDA → MT6701 #3（M3）**；配置同上。
- 从机地址：`MT6701_SLAVE_ADDR = (0x06 << 1) = 0x0C`（即 7 位地址 0x06，与 MT6701 数据手册 b'0000110 完全一致，见任务 B）。
- 角度读取：`HAL_I2C_Mem_Read(bus, 0x0C, MT6701_REG_ANGLE_14b=0x03, 8bit, 2字节)`，解析 `angle = (temp[0]<<6)|(temp[1]>>2)`，`deg = angle*360/16384` —— 与 MT6701 手册"先读 0x03 再读 0x04"（0x03=Angle[13:6]，0x04 高 6 位=Angle[5:0]）完全吻合。
- 每条总线都有 `i2c_bus_recover()`：检测 SDA 被拉低或 BUSY 后 SWRST 复位 → 切开漏 GPIO → 发 9 个 SCL 脉冲驱逐从机 → 伪造 STOP → 恢复复用。读超时 `MT6701_READ_TIMEOUT = 10 ms`。
- main.c 打印确认映射："Encoder mode: I2C absolute (I2C1 PB6/PB7, I2C2 PF1/PF0, I2C3 PA8/PC9)"。

### 软件 I2C ×1（仅用于备用 IMU MPU-6050，与编码器无关）
- 位置：`Hardware/mpu6050.c`（纯 GPIO 位操作 + 忙等延时，无硬件外设）。
- **PB10 = SCL，PB11 = SDA**（开漏 GPIO_MODE_OUTPUT_OD + 内部上拉，VeryHigh speed）。
  - `SCL_H/L()` 写 GPIOB BSRR pin10；`SDA_H/L()` pin11；`SDA_READ()` 读 IDR pin11。
  - `i2c_delay()`：`for (volatile int i=0; i<25; i++);` 忙等。
- 从机：MPU-6050，AD0 接地 → 地址 0x68（mpu6050.h）。
- ⚠️ 审查注意：`.ioc`/`MX_GPIO_Init` 把 PB10/PB11 初始化为**推挽输出**（初始 ODR=0 → 输出低），只有当 `MPU6050_Init()` 被调用时才改为开漏；默认固件 `ACTIVE_IMU=IMU_TYPE_LSM6DSR`，**MPU6050_Init 不会被调用**，PB10/PB11 将停留在推挽低电平。若 MPU6050 实际焊接并共享这两根线，上电期间总线被钳低（LSM6DSR 主用时无影响，但原理图审查需确认 PB10/PB11 是否与其它网络冲突）。

### 编码器第二方式（编译期宏 `ENC_USE_HW_QUAD = 0`，enc_quad.h）
- 0 = 三路 I2C 绝对式读取（**当前默认**）；1 = TIM2/TIM5/TIM8 硬件正交解码（ABZ 增量输入，对应引脚见 A.2 表）。
- 依据网表注释：M1 A=PA5→TIM2_CH1(AF1)，B=PB3→TIM2_CH2；M2 A=PA0→TIM5_CH1(AF2)，B=PA1→TIM5_CH2；M3 A=PC6→TIM8_CH1(AF3)，B=PC7→TIM8_CH2。

## A.5 与 ESP32 通信的串口（任务 A 第 5 项）

- **USART1：PA9=TX，PA10=RX，115200-8-N-1**，AF7，内部上拉，使能 RXNEIE 中断。
  - main.c：`MX_USART1_UART_Init(); /* 板载 ESP32-WROOM32E 无线上位机链路 (PA9/PA10) */`
  - usart.c 注释：**ESP32 IO26 ← F407 PA9(TX)；ESP32 IO25 → F407 PA10(RX)**，115200-8-N-1。
- USART2：PD5=TX，PD6=RX，115200-8-N-1（有线 PC 调试后备通道；`DEBUG_LINK_USART=1` 时 printf/命令解析走 USART1，USART2 中断入口仍在但仅为兼容转发）。
- printf 通过环形 TX 缓冲（1024B）+ TXE 中断异步发送；接收以 \r\n 组包到 `USART_RX_BUF`。
- 上位机协议：`$TELE,...#` 遥测帧（25Hz 周期，40ms 分频）+ ASCII 命令（M/M1/M2/M3/STOP/EN/MODE/ALIGN/PID/TELE/IMU/HALL/SCAN/SENSOR/TEST/BAL/BMODE/LQR/CAL_IMU/ZERO/ATT/PP/U/L）。

## A.6 其它 GPIO（按键/LED/DRV8313 使能等）及"标签"

**.ioc 无 User Label；以下名称全部来自源码宏/注释，原样保留：**

| 引脚 | 名称（源码原文） | 方向/配置 | 代码出处 |
|---|---|---|---|
| PD0 | **M1_Enable / M1_Disable**（注释 "PD0=M1_EN"） | 推挽输出，低初始 | MyProject.h 宏 `HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, ...)`；gpio.c 注释 "(PD0=M1_EN, PD1=M2_EN, PD2=M3_EN)" |
| PD1 | **M2_Enable / M2_Disable** | 同上 | 同上 |
| PD2 | **M3_Enable / M3_Disable** | 同上 | 同上 |
| PG0 | M1 对应 LED（"PG0对应M1"） | 推挽输出低 | main.c LED 翻转逻辑 |
| PG1 | M2 对应 LED | 同上 | 同上 |
| PG2 | M3 对应 LED | 同上 | 同上 |
| PG4 | 启动/状态 LED（`Boot_PG4_Blink_5s`、自检指示） | 推挽输出低 | main.c |
| PG3, PG5, PG6, PG7, PG8 | （无名称，预留输出） | 推挽输出低 | gpio.c（除上述外无代码引用） |
| PD7 | （无名称）输入 | 输入，无上拉 | gpio.c（无代码引用；**用途需原理图确认**） |
| PB12 | LSM_CS（LSM6DSR 软件 SPI 片选） | 推挽输出（spi.c 配置，Pull=UP）；ioc 中为 GPIO_Output | spi.h `LSM_CS_LOW/HIGH()`（BSRR pin12）；Software/README：PB12 CS "板载 3.3V 上拉 R49" |
| PB10/PB11 | MPU-6050 软 I2C SCL/SDA | 见 A.4 | mpu6050.c |
| PA9/PA10 | ESP32 链路 TX/RX | 见 A.5 | usart.c |
| PA13/PA14 | SWDIO/SWCLK | SWD 调试口 | ioc |

**DRV8313 相关要点（审查重点）：**
- 固件中 **没有任何 nSLEEP / nRESET / nFAULT 的 GPIO 配置或读取代码**。
- 使能引脚 PD0/PD1/PD2（M1/M2/M3）语义为 "DRV8313 Enable"（MyProject.h 注释 "DRV8313 使能引脚: PD0=M1, PD1=M2, PD2=M3"）。**PD0/1/2 在板上究竟接每片 DRV8313 的 EN1/2/3（三脚并联）、还是 nSLEEP、还是 nRESET，固件无法区分——必须对照原理图**。
- 由于 DRV8313 的 ENx/INx/nSLEEP/nRESET 均为内置 100kΩ 下拉（见任务 B），PD0/1/2 上电前的低电平默认对应"禁用/休眠"，与固件 `set_motor_enable(0,0)` 上电安全锁一致。
- 硬件无 IMU INT 引脚、无按键、无拨码开关的任何配置。

## A.7 ESP32 工程（Cubli-ESP）

- **模组**：经典 ESP32（ESP32-WROOM-32E，main.cpp 注释"ESP32-WROOM-32E 无线上位机通信桥"）；PlatformIO `board = mhetesp32devkit`（MHET ESP32 DevKit），framework=arduino，monitor 115200 / upload 921600。
- **与 STM32 通信**：`Serial2.begin(115200, SERIAL_8N1, RX=IO26, TX=IO25)` —— IO26=RX（接 F407 PA9），IO25=TX（接 F407 PA10）。透明双向桥（UART ↔ WiFi TCP）。
- **其它 IO**：`STATUS_LED_PIN = 2`（板载 LED，STA 连接中快闪 / 有客户端常亮 / 无客户端 1Hz 慢闪）。
- WiFi：优先 STA（SSID=Cubli-Lab），20s 失败切 AP（Cubli-ESP / cubli12345，IP 192.168.4.1）；TCP 服务器端口 **3333**，最多 2 客户端。
- **Software 子目录**：PyQt5 + pyserial + pyqtgraph 的 PC 上位机（三电机调速、3D 姿态立方体、转速波形、遥测解析、有线 USART2/无线 TCP 双链路）。

---

# 任务B：数据手册调研（全部注明来源）

## B.1 IP5306（英集芯 Injoinic，eSOP8）

来源：官方数据手册 V1.01 英文版镜像 PDF（Laskakit，Injoinic 版权页）https://www.laskakit.cz/user/related_files/ip5306.pdf ；补充 https://done.land/components/power/powersupplies/battery/chargers/charge-discharge/ip5306/ ；https://w2.electrodragon.com/Chip-cn-dat/injoinic-dat/IP5306-dat/IP5306-dat.md

### 管脚定义（eSOP8，Datasheet 第 2 页 "4 Pin definitions"）

| 引脚号 | 名称 | 功能（原文） |
|---|---|---|
| 1 | VIN | DC5V Charge input pin（5V 充电输入） |
| 2 | LED1 | led Drive Pins（电量显示 LED 驱动） |
| 3 | LED2 | led Drive Pins |
| 4 | LED3 | led Drive Pins |
| 5 | KEY | Key input, Multiplexing driving lights（按键输入，复用照明驱动） |
| 6 | BAT | Boost input pins, 接锂电池正极 |
| 7 | SW | DC-DC Switch pin（开关节点，接电感） |
| 8 | VOUT | 5V Boost output pin（5V 升压输出） |
| 底部 | PowerPAD | Connect to GND（**必须接 GND**） |

（注意：只有 3 个 LED 引脚，4 灯方案是 LED1/2/3 + 由 KEY 复用或按手册 Map 1 的 D1~D4 二极管矩阵接法；1/2/3/4 灯均可，见手册第 6/7 页。）

### 典型应用电路外围要求（手册 Map 7 / Map 1 / BOM 表 第 7、9 页）
- **电感**：1 μH @ 500kHz，接 SW–BAT。推荐型号 SPM70701R0（DARFON，1.0μH ±20%，Idc 12A，Isat 15A）；BOM 备注：**饱和电流、温升电流需 >4.5A，DCR < 0.1Ω**。
- **电容**：C1(VIN 入口) 10μF、C2(BAT) 10μF、C7(VOUT 后) 10μF —— 0805 10μF **耐压 ≥16V 陶瓷**；C3/C4/C5(VOUT) 22μF×3 —— **耐压 ≥16V 陶瓷**。
- **电阻**：R3=2Ω（VIN 串阻）、R4=2Ω（BAT 侧）、R2=20Ω（照明 LED 限流）、R1=10kΩ（KEY 串阻到按键）。
- **LED 接法**：D1~D4 电量灯由 LED1/LED2/LED3+KEY 复用驱动（二极管隔离矩阵），驱动电流 4mA/灯（I_L1/L2/L3=4mA，手册第 5 页）；电量阈值：4 灯模式 C≥75% 全亮、50~75% 亮 3 灯、25~50% 亮 2 灯、3~25% 亮 1 灯、0~3% 1.5Hz 闪（第 6 页；done.land 给出的分压点 3.36/3.57/3.65/3.91V）。
- **KEY 接法**：KEY 经 10kΩ 串联电阻到按键再到 GND；短按(30ms~2s)开机/点亮电量、长按(>2s)照明灯、1 秒内双击关机（ElectroDragon/手册行为）；不用按键时 KEY 可悬空（ElectroDragon："PIN5 can float"）。照明功能（R2+D5）可省略。
- 开关频率 fs=500kHz（ buck/boost 共用一个电感）；PMOS rDS(on)=35mΩ、NMOS=30mΩ。

### 轻载自动关机（第 5 页 "Automatic load detection time T_loadD"）
- **连续负载电流 < 45mA 持续 32 秒 → 自动进入待机/关断 5V 输出**（手册原文 "Continuous load current of less than 45mA" → 32 s）。
- 待机电流 100μA；空载自动睡眠后静态电流降至 50μA（第 1 页 "quiescent current drops 50uA"）。
- **避免方法**：① 周期性(<32s)短暂拉低 KEY ≥50ms 唤醒；② 挂常亮假负载（保持 >45mA）；③ 选常开版 **IP5306_CK**（始终输出 5V，pin-to-pin 兼容）；④ 选 I2C 版并置 `reg0x00[1]=1` 关闭自动关机（来源：ElectroDragon IP5306-dat、done.land）。

### 边充边放 / 电流 / 电压（第 1、4、5 页）
- **支持边充边放**：第 1 页 "Built-in power path management, support charge while discharge"（单电感分时复用，充放电路径切换瞬间输出会有短暂中断，done.land 建议输出加大电容防止 MCU 掉电重启）。
- **充电电流**：典型 2.1A，最大 2.4A（I_CHRG）；涓流 100mA（BAT=2.7V），涓流截止 2.9V；充电目标 4.2V（±0.5%），充满回差 4.1V，充电超时 24h；型号分档 4.20/4.30/4.35/4.40V（IP5306_4.30V 等）。
- **最大输出电流**：同步升压 2.4A（12W 档），5.0V 输出（大电流线补 5.15V @ >1A），纹波 50mV；推荐工况负载电流 0~2.4A（最大 3A，第 7 页 Recommended Operating）。
- **BAT 电压范围**：3.0~4.4V（VBAT，第 5 页 Boost system）；VIN 工作范围 4.5~5.5V（abs max −0.3~5.5V），输入欠压 4.5V + 200mV 回差；短路检测 >3.5A / 150~200μs；输出过流检测（VOUT 持续 <4.2V 30ms）；过温 125°C + 40°C 回差；ESD 4kV。
- 不支持 QC/PD 快充协议（ElectroDragon）。

## B.2 IP2326（英集芯 Injoinic）

来源（可用公开资料有限，以下均注明）：
- 立创商城产品页摘要（item.szlcsc.com，IP2326，VQFN-24-EP）："IP2326是一款支持15W快充的2节/3节串联锂电池升压充电IC。集成功率MOS，采用同步开关架构，使其在应用时仅需极少的外围器件…"。
- 代理页汇总（icanic.cn / zrsc-ic.com "Boost Charger IP2326" / eeworld）：升压开关充电频率 **500kHz**；最大输入充电功率 15W；5V 输入时 8V/1.0A 输出效率约 94%、8V/1.5A 约 92%；为 3 串电池组提供 1.2A 充电电流；**外接电阻设定充电电流、充电电压、输入欠压/过压阈值、充电超时**；**集成充电均衡**（充电时逐节检测并平衡 2 节电芯电压）；**引脚选择 2 串 / 3 串充电**；输入限压/欠压自适应，智能调节充电电流防止拉挂适配器。
- 封装：VQFN-24-EP（4×4mm）。

### 本任务问题逐项回答
- **是什么芯片**：2S/3S 串锂电池**同步升压型充电管理 IC**（Boost Charger，把 5V USB 输入升压到 2S(8.4V)/3S(12.6V) 给电池充电），内置功率 MOS，带充电均衡。
- **是否带 USB D+/D- 检测**：**公开渠道未能证实**。搜索到的规格描述均未提及 D+/D-（BC1.2/苹果分压/插入检测）引脚或功能；英集芯带协议识别的是其它系列。**若原理图把 D+/D- 接到 IP2326，需向原厂确认该引脚真实定义（查不到，禁止臆断）**。
- **管脚定义（1~24 逐脚）**：免费渠道只能确认功能归类中出现过 CHG（充电指示）、BAT、TS（NTC 温度检测）、ISET（充电电流设定）等字样；**完整逐脚表查不到**（立创商城需登录下载 PDF，英集芯官网/代理站均未公开全文）。审查时应以原理图上的实际网络为准并与原厂 PDF 核对。
- **典型应用要求**：外接电阻设充电电流/电压/输入欠压阈值/超时；500kHz 升压电感+输入输出电容（具体感值/容值未能从公开渠道核实，查不到）；引脚选择 2S/3S。

## B.3 SH367103（中颖电子 Sino Wealth，3/4 串锂电池 Pack 保护）

来源：
- 数据手册第 1~3 页（Sino Wealth《SH367103系列 Preliminary V1.0》，经 datasheet4u 页面图片版获取）https://datasheet4u.com/datasheets/Sino-Wealth-Microelectronic/SH367103/1087500
- 《SH367103 Application Notices V2.2》（Scribd 节选文本）https://www.scribd.com/document/909758368/SH367103-Application-Notices-V2-2
- 立创商城中文资料摘要（item.szlcsc.com，SH367103X/016XY-AAE00，TSSOP-16）
- alldatasheet 同系列 SH367107（VOV 3.6~4.5V/10mV 步进、±25mV）：https://www.alldatasheet.com/datasheet-pdf/pdf/2288580/SINOWEALTH/SH367107.html

### 这颗芯片是什么
- **3/4 串锂电池 Pack 保护芯片（1 级保护，纯保护 IC/前级，非电量计，无 I2C/AFE 通信）**；SEL 管脚选择 3 串或 4 串应用；适用于磷酸铁锂 Pack。
- 功能：过充（4 级电压等级可选）、放电过流 1/2/3、短路、充放电高低温（TS 外接 NTC）、0V 充电、正常/休眠模式（低容量自动休眠）。
- 内部框图：电压比较器模块（VC1~VC4）、充/放电电流检测模块（输入 **VM、CHSE**）、温度检测模块（**TS**）、逻辑模块、保护延时/封锁模块（输出 **DSG、COC**）、MOSFET 驱动模块（输入 **CTL**，输出 **CHG、DSG**）。
- **无内置 MOS，需外接充/放电 MOSFET**（低边 N-MOS 驱动；Application Notices 介绍了基于标准方案拓展的 PMOS 方案）。

### 管脚定义（TSSOP-16，手册第 3 页管脚图）

| 引脚号 | 名称 |
|---|---|
| 1 | CTL |
| 2 | CHSE |
| 3 | CHG |
| 4 | VM |
| 5 | DSG |
| 6 | DSD |
| 7 | CDC |
| 8 | VI |
| 9 | TS |
| 10 | SEL |
| 11 | GND |
| 12 | VC4 |
| 13 | VC3 |
| 14 | VC2 |
| 15 | VC1 |
| 16 | VDD |

功能（手册/应用指导可确认的部分）：VC1~VC4=各节电芯电压采样；SEL=3/4 串选择；TS=温度检测（NTC）；**CTL=优先控制 CHG/DSG 输出**（"CTL 管脚优先控制 CHG/DSG 管脚的输出，且 CTL 控制 CHG/DSG 管脚的优先级高于芯片内部保护电路"；CTL=VDD 电平时 CHG/DSG 取决于内部保护电路）；**VI=充/放电状态判断**（"SH367103 由 VI 电平判断系统充放电状态，当 VI 管脚电平高于放电状态检测电压时…"）；VM、CHSE 为充/放电电流检测输入（框图）；CHG/DSG 为充电/放电 MOSFET 驱动输出。**DSD、CDC、VM、CHSE 的逐脚文字定义在免费渠道查不到**（手册第 3 页之后 datasheet4u 未收录；需原厂 PDF）。

### 典型参数（手册第 1 页 "1 特点"）
- 过充保护电压 Vocp：3.6~4.35V（10mV 步进，精度 ±25mV）；Vocp2：3.1~4.35V（10mV，±50mV）；Vocp3/Vocp4：2.0~3.7V（100mV，±100mV）。
- 放电过流 Vdocp1：0.025~0.35V（25mV 步进，±10mV）；Vdocp2：2×Vdocp1（50mV，±20mV）；Vdocp3：320mV；短路 Vscp=4.5×Vdocp1（112.5mV，±45mV）。
- 温度保护：充电高温 50°C、充电低温 0°C、放电高温 70°C（均 ±4°C Max）；外接充放电管经 NTC 电阻时影响过流 1/2 保护延时。
- **工作电压范围：3.0V~26V**（图片扫描件该行数字有压缩伪影，标注为 30V~26V，按 3/4 串应用与同系列资料判断应为 3.0~26V —— 建议以原厂 PDF 复核）；工作温度 −40~85°C。
- 功耗：正常 15μA Max，休眠 4μA Max。封装 16-pin TSSOP。低 N-MOSFET 驱动。
- **外部 MOS 接法**：DSG/CHG 分别驱动放电/充电低边 N-MOS 栅极（PMOS 拓展方案见 Application Notices V2.2）；电流经 VM/CHSE 检测；CDC/DSD 为过流检测相关端（逐脚说明查不到）。

## B.4 MT6701（麦歌恩 MagnTek，差分霍尔磁角度传感器）

来源：官方数据手册 v1.8（2022.12）PDF https://uploadcdn.oneyac.com/attachments/files/brand_pdf/magntek/1B/81/MT6701CTSTD.pdf （magntek.com.cn 版权）

### I2C 从机地址（7.7.2 节）
- **7 位地址 = b'0000110 = 0x06**（可编程改为 b'1000110 = 0x46）。固件 `(0x06<<1)` 写法正确。

### ABZ/UVW 与串行接口能否同时工作（7.1 节）
- **不能同时**：SOP-8 封装上 ABZ/UVW（非差分）、I2C、SSI **共用引脚 6（A/SDA/DO）、7（B/SCL/CLK）、8（Z/CSN/W）**，由 MODE 引脚在 ABZ(=0) 与 I2C/SSI(=1) 间二选一。QFN-16 同理（6/7/8 共用，9/11/12 为 UVW/差分）。
- OUT（脚 3，模拟/PWM）与 PUSH（脚 5）是独立引脚，可与 I2C 并存。

### SDA/SCL 是否开漏、要不要上拉（7.7 节 + 电气参数）
- 数据手册 I2C 参考电路（图-17/18）中 **SDA、SCL 都画了 4.7kΩ 上拉到 VDD**；正文注明 "SCL 是数字输入引脚，是否需要上拉取决于 MCU"。
- 电气特性表中数字 I/O 为 **推挽输出**（"数字I/O参数(推挽输出)"，VOH=VDD−0.5V @2mA，VOL=0.5V @2mA）。结论：**SDA 建议按参考电路加 4.7k 上拉**（推挽输出特性下读时序依赖上拉，且手册参考电路明确画了上拉）；SCL 上拉视主机而定。
- MODE（SOP-8 脚 2）：数字输入，**内置 200kΩ 上拉**（悬空默认 = I2C/SSI 模式；拉地 = ABZ/UVW 模式）；Z 脚同样内置 200kΩ 上拉。

### VDD 供电范围（第 1、5、32 页）
- 工作电压 **3.3~5.0V**（特性页；电气参数表 VDD min 3.0 / max 5.5V）；**EEPROM 烧写时必须 4.5V < VDD < 5.5V**。Idd 典型 10mA / 最大 14mA。I2C 时序：SCL 周期最小 1μs（≈1MHz 上限）。

### MT6701CT 与 "MT6701CT-ST" 型号区别（第 5 页型号列表）
- 数据手册只定义两种封装：**MT6701CT = SOP-8（MSL-3），MT6701QT = QFN3×3-16L（MSL-1）**；后缀 -STD/-AKD/-ACD/-A200… 仅是出厂 EEPROM 预设不同（如 CT-STD：AB=1 脉冲/圈、Z=1LSB、逆时针角度增加；CT-ACD：AB=1024 脉冲/圈、Z=4LSB；CT-STV：顺时针版本）。
- **官方手册中不存在 "SOT23-6 封装" 的 MT6701 变体**；"MT6701CT-ST" 极可能是商家对 "MT6701CT-STD" 的截断简写。若采购平台确实出现 SOT23-6 实物，需向麦歌恩核实（查不到官方依据）。

### I2C 模式下角度寄存器读取方式（7.7.2 节，图-20）
- 14 位绝对角度存于 **0x03（Angle[13:6]）与 0x04（Angle[5:0]，bit1:0 无效）**；**必须先读 0x03 再读 0x04**；θ = (Σ Angle[i]·2^i)/16384 × 360°。
- 写寄存器：Slave ID(0x06+W) → 寄存器地址 → 数据（图-21）。EEPROM 编程流程：写目标寄存器 → 写 0xB3 到 0x09 → 写 0x05 到 0x0A → 保持 4.5~5.5V 等待 ≥600ms → 断电回读校验（8.2 节）。
- 其余常用寄存器：0x25[7] UVW_MUX、0x29[6] ABZ_MUX、0x29[1] DIR、0x30/0x31 ABZ_RES[9:0]（1~1024 脉冲/圈）、0x32 Z_PULSE_WIDTH/ZERO[11:8]、0x33 ZERO[7:0]、0x34 HYST、0x38 PWM_FREQ/PWM_POL/OUT_MODE、0x3E~0x40 A_START/A_STOP。ABZ 上电默认 50ms 内无输出（可配置为输出上电绝对位置脉冲序列）。

## B.5 DRV8313（TI，Triple 1/2-H Bridge）

来源：官方数据手册 SLVSBA5D（2012.10，2016.04 修订）https://www.ti.com/lit/ds/symlink/drv8313.pdf

### 封装
- **PWP = 28 引脚 HTSSOP（PowerPAD，9.70×4.40mm）**；RHH = 36 引脚 VQFN（6×6mm）。两种均量产（Package Addendum）。

### 管脚图（PWP 28 脚，Top View；Pin Functions 表）

| 引脚号 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 1 | CPL | PWR | 电荷泵负端。CPH–CPL 间接 **VM 耐压 0.01μF 陶瓷** |
| 2 | CPH | PWR | 电荷泵正端（同上 0.01μF） |
| 3 | VCP | PWR | 电荷泵输出。对 VM 接 **16V、0.1μF 陶瓷** |
| 4 | VM | PWR | 电机电源 1（两个 VM 引脚必须接同一电源） |
| 5 | OUT1 | O | 半桥输出 1 |
| 6 | PGND1 | PWR | 低边 FET 源极。接 GND 或低边检流电阻 |
| 7 | PGND2 | PWR | 同上 |
| 8 | OUT2 | O | 半桥输出 2 |
| 9 | OUT3 | O | 半桥输出 3 |
| 10 | PGND3 | PWR | 同上 |
| 11 | VM | PWR | 电机电源 2 |
| 12 | COMPP | I | 比较器正输入（未提交比较器） |
| 13 | COMPN | I | 比较器负输入 |
| 14 | GND | PWR | 地 |
| 15 | V3P3 | PWR | 内部 3.3V LDO。对 GND 接 **6.3V、0.47μF**；最多对外供 10mA |
| 16 | nRESET | I | 低有效复位：复位故障、禁输出；**内置下拉** |
| 17 | nSLEEP | I | 睡眠控制：高=工作，低=休眠；**内置下拉** |
| 18 | nFAULT | OD | 故障指示，**开漏，需外部上拉**（典型 10kΩ） |
| 19 | nCOMPO | OD | 比较器输出，**开漏，需外部上拉** |
| 20 | GND | PWR | 地 |
| 21 | NC | NC | 无内部连接 |
| 22 | EN3 | I | 通道 3 使能，高有效，**内置下拉** |
| 23 | IN3 | I | 通道 3 输入（高=OUT3 高；ENx 低时无效），**内置下拉** |
| 24 | EN2 | I | 通道 2 使能 |
| 25 | IN2 | I | 通道 2 输入 |
| 26 | EN1 | I | 通道 1 使能 |
| 27 | IN1 | I | 通道 1 输入 |
| 28 | GND | PWR | 地 |
| 底部 | PowerPAD | PWR | **必须接 GND**（散热） |

（VQFN-36 对应关系见数据手册 Pin Functions 表 RHH 列。）

### 关键行为与接法要求
- **nSLEEP 悬空行为**：内置 100kΩ 下拉 → 悬空 = 休眠（电荷泵关、输出 Hi-Z）。要正常工作必须拉高。进入休眠再唤醒后约 1ms 才完全恢复（7.4.1）。
- **nFAULT**：开漏输出，故障时拉低，**必须外部上拉到 VMCU**（典型应用图 Figure 13/15 用 10kΩ 上拉）。VM 欠压、过流、过温均拉低 nFAULT。
- **逻辑电平**：VIL ≤ 0.6~0.7V，**VIH ≥ 2.2V**，数字引脚最高 5.5V → **3.3V 逻辑完全兼容**。
- **VM 电压范围**：**8~60V**（推荐工作），abs max 65V；UVLO 6.3V（上升，typ，max 8V）。⚠️ **12V/7.4V 锂电供电低于 8V 时芯片处于 UVLO，不工作** —— 本板 3S(12.6V) OK，2S(7.4V) 不行，审查时注意供电架构。PWM 频率上限 250kHz（ENx/INx），死区固定 90ns（tDEAD），rDS(on) 每管 0.24Ω typ（25°C）/0.39Ω max（85°C）。
- **CP1/CP2（=CPH/CPL）外围**：CPH–CPL 之间 0.01μF（VM 耐压陶瓷）；VCP–VM 之间 0.1μF（16V）；V3P3–GND 0.47μF（6.3V）；每个 VM 引脚 0.1μF 旁路 + 一颗 VM 大容量 bulk 电容（典型应用 0.1μF+100μF，Figure 13/15）。nSLEEP 低时电荷泵关闭。
- **电流检测运放：DRV8313 没有 SPx/SNx/SOx 电流检测放大器**（那是 DRV8302/8303 的架构）。DRV8313 提供的是：3 个独立 PGND 脚（可各接一个低边检流电阻，PGND 电压须保持 ±500mV 内）+ **1 个未提交比较器**（COMPP/COMPN/nCOMPO，响应 2μs，共模 0~5V）。**不用比较器时官方布局指南明确 COMPP、COMPN、nCOMPO 接 GND**（10.1 节 "In Figure 19 and Figure 20 ... COMPP, COMPN, and COMPO pins are tied to GND"）。典型过流监测方案：RSENSE 200mΩ、COMPN 接 PGNDx、V3P3 分压设阈值（R1=56k/R2=10k ≈ 0.5V → 2.5A 触发，8.2.2 节）。数据手册全文无 "VREF" 引脚（参考电压由 V3P3 分压实现）。
- **电流能力**：**2.5A 峰值/半桥**（特性页 "2.5-A Peak"，简化示意图标 3.5A 瞬态）；OCP 保护 3~5A（typ 5A，min 3A，deglitch 5μs）；过温 150~180°C（回差 35°C）。数据手册未给"连续电流"单值（TI 另有 SLVA505《Understanding Motor Driver Current Ratings》讨论热限），典型设计例取 1.2A RMS。
- 桥控逻辑：ENx=0 → OUTx=Hi-Z；ENx=1 → OUTx=INx。

## B.6 LSM6DSR（ST，六轴 IMU）——简要

来源：ST 数据手册 DS12488 https://www.st.com/resource/en/datasheet/lsm6dsr.pdf （摘要经检索核对）

- **供电**：Vdd 1.71~3.6V（abs max 任意脚 4.8V）；Vdd_IO 1.08~3.6V。
- **SPI 模式引脚**：CS（SPI 使能，低有效；接 Vdd_IO 时进入 I2C 模式）、SDI（I2C SDA / SPI 4 线 MOSI / 3 线数据）、SDO/SA0（SPI 4 线 MISO；I2C 地址最低位）、SCL/SPC（I2C/SPI 时钟）。速率：I2C ≤400kHz，**SPI ≤10MHz**（固件 SPI2 配置 5.25MHz，合规）。
- 固件 WHO_AM_I 寄存器 0x0F 期望 0x6B（lsm6dsr.h），SPI Mode 3（CPOL=1/CPHA=1）与 ST 器件兼容。

## B.7 ESP32（经典款 WROOM-32/WROOM-32E）下载/启动 strapping 要求

来源：Espressif ESP32 数据手册（中文，Boot Configurations/Strapping Pins 章节）https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_cn.pdf ；ESP-IDF GPIO 文档 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gpio.html ；硬件设计指南摘录（kio4 镜像）

- **EN（CHIP_PU）**：复位脚，需 **10kΩ 上拉 + ≥1μF 对地电容**（RC 上电延时）；EN 拉低期间芯片复位。
- **GPIO0（IO0）**：boot 模式选择。**EN 上升沿时 IO0=1（弱内部上拉）→ 正常 SPI Flash 启动；IO0=0 → UART 下载模式**。外部建议 10k 上拉 + 按键拉地进下载。
- **GPIO2（IO2）**：下载模式下必须为低（或悬空靠下拉），正常启动时 don't-care；与 IO0 联合决定启动模式。
- **GPIO12（MTDI）**：选择 **VDD_SDIO/Flash 电压：0=3.3V（默认），1=1.8V**。⚠️ 外部误上拉 GPIO12 会让 3.3V Flash 板无法启动；无 1.8V Flash 需求时**不得上拉**（内置弱下拉）。
- **GPIO15（MTDO）**：控制启动时 U0TXD 调试日志输出（0=静默）。
- **TXD0（GPIO1）/RXD0（GPIO3）**：UART0 下载与日志串口。
- 本板 ESP32 仅通过 IO25/IO26（UART2 重映射）与 STM32 通信，不占用 UART0，与下载电路无冲突；需要审查的是板上对 IO0/IO2/IO12/EN 的上拉/下拉处理。

---

## 附：审查提示（固件 ↔ 数据手册交叉发现，供后续审查使用）

1. **PD0/1/2（M1/M2/M3 Enable）到底接 DRV8313 的 EN1/2/3、nSLEEP 还是 nRESET** —— 固件只推高/低，三者内置均为 100k 下拉；若接 nRESET 则无法复位故障（nRESET 需脉冲），若接 nSLEEP 则 EN 通道仍需另配。原理图审查必须确认。
2. **DRV8313 VM=12V（3S 满电 12.6V）在 8~60V 范围内 OK；若供电被 IP5306 的 5V 或 2S 7.4V 驱动则低于 8V UVLO**。固件 `voltage_power_supply=12.0f` 表明设计意图为 12V（DRV8313 注释一致）。
3. **nFAULT（DRV8313 开漏）与 PD7（输入，无上拉）疑似配对** —— PD7 若接 nFAULT，缺少外部上拉时固件读到恒低；PD7 无代码引用，功能待确认。
4. **死区**：timer.c 注释称 60ns，代码 DeadTime=0；DRV8313 内部固定 90ns 死区可兜底，但注释应修正。
5. **PB10/PB11 被初始化为推挽输出低**，若板上挂 MPU6050 软 I2C，LSM6DSR 主用模式下两线被钳低（见 A.4）。
6. **三路硬件 I2C 依赖上拉**：MSP 配置为 AF_OD+内部上拉（F4 内部上拉 ~40kΩ，400kHz 下偏弱），MT6701 参考电路要求 4.7kΩ 外部上拉 —— 原理图应有 4.7k（或类似）外部上拉。
7. **MT6701 与 TIM2/5/8 ABZ 解码互斥**（芯片引脚复用），固件默认走 I2C；若原理图同时引出 ABZ 与 I2C 到 MCU，I2C 与 ABZ 不能同时使能（MODE 脚决定）。
8. **IP5306 轻载关机（<45mA/32s）**：本板 ESP32+STM32 常态电流是否 >45mA 需评估，否则上电 32s 后 5V 输出被自动关断；原理图审查 KEY/LED 接法与是否需要常开版。
9. **SH367103 的 CHG/DSG 外部 MOS 拓扑与 CTL/VI 的接法**是保护板审查重点；VM/CHSE/DSD/CDC 逐脚功能需以原厂 PDF 复核（免费渠道未获取全文）。
10. **IP2326 是否真的参与 D+/D- 检测**：公开资料无法证实，原理图审查时按"升压充电 IC、无协议检测"假设并核对原厂 PDF。

（本文件由审查调研 agent 生成；固件路径：D:\Document\tempPrj\Cubli-F407ZGT6\无刷闭环联调\）
