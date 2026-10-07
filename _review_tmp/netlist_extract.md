# Cubli-F407ZGT6 主板网表连接关系完整报告

- 分析对象：`D:/Document/tempPrj/Cubli-F407ZGT6/Netlist_MB_2026-10-07.net`（嘉立创EDA 导出 PROTEL NETLIST 2.0，PCB 网表）
- 对照文件：`D:/Document/tempPrj/Cubli-F407ZGT6/无刷闭环联调/Netlist_MB_Schematic_2026-10-07.net`（同项目原理图网表，含符号引脚名，见第 12 节）
- 规模：229 个器件、252 个网络。所有网络名/位号/引脚号均**原样照抄**自网表。
- 引脚名来源说明：PCB 网表中 IC 引脚仅有引脚号；但原理图网表（第 12 节对照文件）第二列携带嘉立创器件库符号引脚名（如 `U14-69 STM32F407ZGT6-PB10`）。本报告"符号引脚名"列取自该文件，并已用联网获取的权威数据手册逐项交叉验证（来源 URL 见文末）。
- 说明：网表中大量网络仅含 1 个引脚（网络已命名、板上无其它连接），本文标注"单脚网络"；这类引脚物理上仍是 PCB 走线/焊盘，电气上等效悬空。

---

## 1. 器件清单（位号 → 型号/参数 → 封装）

### 1.1 关键器件

| 位号 | 型号/参数（PARTTYPE / Value） | 封装 | 备注 |
|---|---|---|---|
| U14 | STM32F407ZGT6（LCSC C19156，ST） | LQFP-144_L20.0-W20.0-P0.50-LS22.0-BL | 主控 MCU |
| U8 | ESP32-WROOM-32E-N4（Value=2.4GHz，LCSC C701341，乐鑫） | WIFI-SMD_ESP32-WROOM-32E | Wi-Fi/BT 模组 |
| U1, U2, U3 | DRV8313PWPR（LCSC C92482，TI，三相半桥驱动） | HTSSOP-28_L9.8-W4.5-P0.65-LS6.6-BL-EP（28 脚 + EP=29 脚） | 3 片电机驱动 |
| U4 | SH367103X/016XY-AAE00（LCSC C160668，sinowealth/中颖电子，"3/4串锂电池Pack保护芯片"） | TSSOP-16_L5.0-W4.4-P0.65-LS6.4-BL | 电池保护 AFE |
| U9, U10, U11 | HY2213-BB3A（LCSC C113632，HYCON/宏康，"单节锂电池充电平衡IC"） | SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BR | 3 片，逐节均衡 |
| U12 | IP2326（LCSC C2832094，INJOINIC/英集芯，"支持15W快充的2节/3节串联锂电池升压充电IC"） | VQFN-24_L4.0-W4.0-P0.50-BL-EP2.5 | USB 升压充电 |
| U7 | TPS565201DDCR（LCSC C327676，TI，4.5-17V 输入 5A 同步降压） | SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BR | 产生 3V3 |
| U16 | LSM6DSRTR（LCSC C784817，ST，3D 加速度+陀螺仪） | LGA-14_L3.0-W2.5-P0.50-BR | IMU（SPI） |
| U17 | MPU-6050（LCSC C24112，TDK InvenSense） | QFN-24_L4.0-W4.0-P0.50-BL-EP2.7 | IMU（I2C） |
| U15 | BAT54C（LCSC C22466350，双二极管共阴） | SOT-23-3 | MCU VBAT 供电切换 |
| U5, U6 | TPN2R703NL,L1Q(M（LCSC C5307666，东芝，单 N 沟 30V） | TSON-8_L3.1-W3.1-P0.65-LS3.3-BL-EP | 充放电通路 FET |
| Q1 | AON7403（LCSC C5310961，30V 50A，符号引脚 1-3=S、4=G、5-9=D） | DFN-8_L3.0-W3.0-P0.65-BL | 充电通路 P-FET |
| Q7 | HY19P03D（LCSC C122492，华羿微，P 沟 30V 90A，TO-252：1=G、2=D、3=S） | TO-252-2_L6.6-W6.1 | 总电源开关 P-FET |
| Q4, Q5, Q6 | AO3400（LCSC C2938367，N 沟 30V，1=G、2=S、3=D） | SOT-23 | 3 片均衡开关管 |
| CN1 | XT30PW-M30.G.Y（AMASS 航模插头 XT30 公头） | CONN-TH_XT30PW-M | 电池主接口 |
| CN2 | HX25003-4AWB（XH 型 1x4P 2.5mm 卧贴） | CONN-SMD_4P-P2.50 | 电池平衡线插座（4 线） |
| USB1 | TYPE-C16PIN（首韩 16P 卧贴母座） | USB-C-SMD_TYPE-C16PIN | USB-C（充电输入） |
| H2 | HDR-M_2.54_2x3（6P 2.54 排针） | HDR-TH_6P-P2.54 | SWD + 串口调试 |
| H3 | HDR-M_2.54_1x3P（3P 2.54 排针） | HDR-TH_3P-P2.54 | ESP32 下载口 |
| I2C-1, I2C-2, I2C-3 | ZX-SH1.0-4PWT（SH1.0 4P 卧贴座） | CONN-SMD_4P-P1.00 | 3 路外接 I2C 编码器座 |
| ABZ-1, ABZ-2, ABZ-3 | ZX-SH1.0-4PWT（SH1.0 4P 卧贴座） | CONN-SMD_4P-P1.00 | 3 路外接 ABZ 编码器座 |
| ST-KEY | TS-1088-AR02016（轻触按键，讯普 C720477） | SW-SMD_L3.9-W3.0-P4.45 | MCU 按键（接 U14-49/PF11） |
| ST-RST | TS-1088-AR02016 | SW-SMD_L3.9-W3.0-P4.45 | MCU 复位按键 |
| ESP-BOOT | TS-1088-AR02016 | SW-SMD_L3.9-W3.0-P4.45 | ESP32 IO0 BOOT 按键 |
| ESP-RST | TS-1088-AR02016 | SW-SMD_L3.9-W3.0-P4.45 | ESP32 EN 复位按键 |
| SW1 | SK12D07VG4（拨码/滑动开关，首韩 C393937） | SW-TH_SK12D07VG4 | DRV8313 SLEEP 选择 |
| SW3 | SS12D10G4 071（滑动开关，首韩 C2887259） | SW-TH_SHOU-HAN_SS12D10G4 | 电池总电源开关 |
| SW4 | SK12D07VG4 | SW-TH_SK12D07VG4 | IP2326 EN 选择 |
| X1 | XKXGI-SUA-32.768K（32.768kHz，3215 2P，YXC C5213671） | CRYSTAL-SMD_L3.2-W1.5-1 | RTC 晶振（PC14/PC15） |
| X2 | X322525MOB4SI（25MHz，3225 4P，YXC C9006） | CRYSTAL-SMD_4P-L3.2-W2.5-BL | 主晶振（PH0/PH1） |
| L1 | SWPA6045S2R2NT（2.2uH，7.4A，顺络 C36500） | IND-SMD_L6.0-W6.0_SWPA6045S | IP2326 升压电感 |
| L3 | GZ1608D601TF（600Ω@100MHz 磁珠，顺络 C1002） | L0603 | 3V3 → VDDA 滤波 |
| L4 | Value=2.2uH；PARTTYPE/符号=SDFL1608S100KTF（见下方注意） | IND-SMD_L2.0-W1.6 | 3V3 buck 功率电感 |
| D2 | PESD3V3L1BA（ESD 二极管） | SOD-323 | NRST 保护 |
| D1, D5 | 1N5819W（肖特基，1=C、2=A） | SOD-123 | D5 在充电通路，D1 在 SH367103 驱动供电 |
| D3 | BZT52C16（稳压管，1=K、2=A） | SOD-123 | 电源开关电路 |
| D4 | BZT52C15（稳压管，1=K、2=A） | SOD-123 | SH367103 驱动供电钳位 |
| B00 | Res 10K | R0402 | BOOT0 下拉（焊接） |
| B01 | Res NC（未贴） | R0402 | BOOT0 上拉位（接 3V3） |
| B10 | Res NC（未贴） | R0402 | BOOT1 下拉位（接 GND） |
| B11 | Res NC（未贴） | R0402 | BOOT1 上拉位（接 3V3） |
| R44, R45, R46 | 220Ω（PTFR0402B220RP9 精密薄膜） | R0402 | LSM6DSR SPI 串阻 |
| R36 | VG039NCHXTB503 50kΩ（HDK 电位器） | RES-ADJ-SMD_VG039NCH | IP2326 ISET 充电电流调节 |
| RNTC | Res 51K | R0402 | IP2326 VSET |
| RVST | Res 68K | R0402 | IP2326 NTC 脚 |
| AT103-NTC | Res 10K | R0402 | SH367103 TS 热敏 |
| R6 | RE1206F1R100，Value=4mΩ（2512 采样电阻；型号后缀 1R100 与 Value 不一致，建议核对 BOM） | R2512 | 电池负极电流采样 |
| R28 | Res 0.5 | R0402 | VBUS → IP2326 VIN 串阻 |
| LED 类位号 | LED4、LED5、LED1、LED2、LED3、LED16~LED21 为 LED_0603-R / LED_0402-R；注意 **"3V3"、"DRV8313-1"、"DRV8313-2"、"DRV8313-3"、"UNDEFINE" 也是 LED 的位号**（LED_0603-R / LED_0402-R） | LED_0402/0603 | LED 引脚符号名：1=C（阴极）、2=A（阳极） |

**重要说明（与任务预期不符的部分）**：
1. **网表中未找到任何 MT6701 器件**（全部 229 个器件的 PARTTYPE/Value/Device 字段中无 MT6701 字样，也没有其它编码器芯片）。板上有 ABZ-1/2/3、I2C-1/2/3 六个 SH1.0-4P 外接接口，应为外接编码器小板（如 MT6701 模块）的连接器，编码器芯片本身不在本板上。
2. **网表中未找到 IP5306**。电源部分实际由 IP2326（升压充电）+ TPS565201（3V3 buck）+ SH367103/HY2213（保护/均衡）构成。

### 1.2 阻容感归类统计（Value 原样照抄）

| 类别 | 值 | 数量 | 位号 |
|---|---|---|---|
| CAP_0603 | 100nF | 60 | C1 C2 C4 C6 C7 C9 C11 C12 C14 C28 C40 C41 C42 C43 C44 C45 C46 C47 C48 C49 C50 C51 C52 C53 C54 C55 C56 C57 C58 C61 C62 C63 C67 C78 C79 C73 C77 C80 C81 C72 C74 C75 C16 C17 C18 C19 C21 C22 C23 C24 C26 C27 C82 C84 C36 C107 C37 C38 C39 C86 |
| CAP_0603 | 10nF | 4 | C3 C8 C13 C20 |
| CAP_0603 | 10pF | 4 | C68 C69 C70 C71 |
| CAP_0603 | 10uF | 7 | C29 C30 C31 C32 C35 C33 C34 |
| CAP_0603 | 2.2nF | 3 | C64 C65 C66 |
| CAP_0603 | 2.2uF | 3 | C59 C60 C25 |
| CAP_0603 | 47uF | 2 | C76 C83 |
| C_Ele_SMD_3x5.4mm（电解） | 10uF | 3 | C5 C10 C15 |
| Res_0603/Res_0402 | 1K | 16 | R29 R30 R32 R43 R1 R2 R3 R4 R5 R7 R9 R10 R11 R21 R24 R27 |
| Res_0603/Res_0402 | 10K | 12 | R41 R47 R48 R49 R50 R70 R71 R72 R60 R38 R39 R53 |
| Res_0603 | 3.3K | 9 | R61 R62 R63 R64 R65 R66 R67 R68 R69 |
| Res_0603 | 100 | 8 | R51 R52 R19 R20 R22 R23 R25 R26 |
| Res_0603 | 220Ω | 3 | R44 R45 R46（PTFR0402B220RP9） |
| Res_0603 | 51K | 1 | R37 |
| Res_0603 | 68K | 1 | R31 |
| Res_0603 | 75K | 1 | R33 |
| Res_0603 | 49.9K | 1 | R18 |
| Res_0603 | 14.7K | 1 | R17 |
| Res_0603 | 4.7K | 1 | R16 |
| Res_0603 | 5.1K | 2 | R34 R35 |
| Res_0603 | 2K | 1 | R59 |
| Res_0603 | 0.5 | 1 | R28 |
| Res_0603 | 1M | 2 | R8 R14 |
| Res_0603 | 2M | 1 | R15 |
| Res_0603 | 5.1M | 1 | R13 |
| Res_0603 | 10M | 1 | R12 |
| Res_0603 | NC | 3（B01 B10 B11）＋R42 | 见上文 |
| RE1206F1R100 | 4mΩ | 1 | R6 |
| L_0603 | 2.2uH | 1 | L4 |
| VG039NCHXTB503 | 50kΩ 电位器 | 1 | R36 |
| AT103-NTC（Res_0603） | 10K | 1 | AT103-NTC |
| RNTC（Res_0603） | 51K | 1 | RNTC |
| RVST（Res_0603） | 68K | 1 | RVST |

注意：L4 的 Value=2.2uH，但其 PARTTYPE/符号为 SDFL1608S100KTF（该型号编码通常对应 10uH），Value 与型号编码不一致，建议核对 BOM。

---

## 2. STM32F407ZGT6（U14，LQFP144）逐脚连接表

符号引脚名列取自原理图网表中的嘉立创库符号名，已与 ST DS8626 Rev12（Table 7 LQFP144 列 + Figure 14）和 Olimex STM32-E407 用户手册中的 STM32F407ZGT6 引脚表交叉验证，二者与符号名 100% 一致。全部 144 脚均出现在网表中（无一缺失）。

| 引脚 | 符号引脚名 | 网络名 | 同网络其它引脚 |
|---|---|---|---|
| 1 | PE2 | PE2 | （单脚网络） |
| 2 | PE3 | PE3 | （单脚网络） |
| 3 | PE4 | PE4 | （单脚网络） |
| 4 | PE5 | PE5 | （单脚网络） |
| 5 | PE6 | PE6 | （单脚网络） |
| 6 | VBAT | VBAT | U15-3（BAT54C 阴极 K）、C58-2 |
| 7 | PC13 | PC13 | （单脚网络） |
| 8 | PC14 | PC14 | X1-1（32.768k 晶振 OSC1）、C68-2 |
| 9 | PC15 | PC15 | X1-2（晶振 OSC2）、C69-2 |
| 10 | PF0 | M2-SDA | I2C-2-1 |
| 11 | PF1 | M2-SCL | I2C-2-2 |
| 12 | PF2 | PF2 | （单脚网络） |
| 13 | PF3 | PF3 | （单脚网络） |
| 14 | PF4 | PF4 | （单脚网络） |
| 15 | PF5 | PF5 | （单脚网络） |
| 16 | VSS | GND | （GND 网络，167 脚） |
| 17 | VDD | 3V3 | （3V3 网络，73 脚） |
| 18 | PF6 | PF6 | （单脚网络） |
| 19 | PF7 | PF7 | （单脚网络） |
| 20 | PF8 | PF8 | （单脚网络） |
| 21 | PF9 | PF9 | （单脚网络） |
| 22 | PF10 | PF10 | （单脚网络） |
| 23 | PH0 | PH0 | X2-1（25MHz 晶振 OSC1）、C71-2 |
| 24 | PH1 | PH1 | X2-3（晶振 OSC2）、C70-1 |
| 25 | NRST | NRST | D2-1（ESD）、ST-RST-1（复位键）、C61-2（100nF→GND）、R43-1（R43 1K，另一端→3V3） |
| 26 | PC0 | $1N80 | （单脚网络，未接） |
| 27 | PC1 | PC1 | （单脚网络） |
| 28 | PC2 | PC2 | （单脚网络） |
| 29 | PC3 | PC3 | （单脚网络） |
| 30 | VDD | 3V3 | |
| 31 | VSSA | GND | |
| 32 | VREF+ | VDDA | L3-2（磁珠，另一端→3V3）、U14-33、C56-2、C57-1（100nF→GND） |
| 33 | VDDA | VDDA | 同上 |
| 34 | PA0 | M2-A | ABZ-2-1 |
| 35 | PA1 | M2-B | ABZ-2-2 |
| 36 | PA2 | PA2 | （单脚网络） |
| 37 | PA3 | PA3 | （单脚网络） |
| 38 | VSS | GND | |
| 39 | VDD | 3V3 | |
| 40 | PA4 | PA4 | （单脚网络） |
| 41 | PA5 | M1-A | ABZ-1-1 |
| 42 | PA6 | M2-PWM1 | U2-27（DRV8313 IN1） |
| 43 | PA7 | M2-PWM2 | U2-25（IN2） |
| 44 | PC4 | PC4 | （单脚网络） |
| 45 | PC5 | PC5 | （单脚网络） |
| 46 | PB0 | M2-PWM3 | U2-23（IN3） |
| 47 | PB1 | PB1 | （单脚网络） |
| 48 | PB2（BOOT1） | BOOT1 | B11-1（NC 电阻，另一端→3V3）、B10-2（NC 电阻，另一端→GND） |
| 49 | PF11 | ST-KEY | ST-KEY-1（按键）、C86-2（100nF→GND）、R53-2（R53 10K→3V3） |
| 50 | PF12 | PF12 | （单脚网络） |
| 51 | VSS | GND | |
| 52 | VDD | 3V3 | |
| 53 | PF13 | PF13 | （单脚网络） |
| 54 | PF14 | PF14 | （单脚网络） |
| 55 | PF15 | PF15 | （单脚网络） |
| 56 | PG0 | DRV1-CS | DRV8313-1-2（LED 阳极；LED 阴极→R69 3.3K→GND） |
| 57 | PG1 | DRV2-CS | DRV8313-2-2（LED 阳极；→R61 3.3K→GND） |
| 58 | PE7 | PE7 | （单脚网络） |
| 59 | PE8 | PE8 | （单脚网络） |
| 60 | PE9 | M1-PWM1 | U1-27（DRV8313 IN1） |
| 61 | VSS | GND | |
| 62 | VDD | 3V3 | |
| 63 | PE10 | PE10 | （单脚网络） |
| 64 | PE11 | M1-PWM2 | U1-25（IN2） |
| 65 | PE12 | PE12 | （单脚网络） |
| 66 | PE13 | M1-PWM3 | U1-23（IN3） |
| 67 | PE14 | PE14 | （单脚网络） |
| 68 | PE15 | PE15 | （单脚网络） |
| 69 | PB10 | $1N123 | （单脚网络，未接） |
| 70 | PB11 | $1N124 | （单脚网络，未接） |
| 71 | VCAP_1 | VCAP_1 | U14-106（VCAP_2，两脚并在同一网络）、C59-2、C60-2（各 2.2uF→GND） |
| 72 | VDD | 3V3 | |
| 73 | PB12 | LSM6DSRTR-CS | U16-12（IMU CS）、R49-2（R49 10K→3V3 上拉） |
| 74 | PB13 | LSM6DSRTR-SCK | R45-2（220Ω 串阻→U16-13）、R48-2（R48 10K→3V3 上拉） |
| 75 | PB14 | LSM6DSRTR-MISO | R46-1（220Ω 串阻→U16-1） |
| 76 | PB15 | LSM6DSRTR-MOSI | R44-2（220Ω 串阻→U16-14）、R47-2（R47 10K→3V3 上拉） |
| 77 | PD8 | PD8 | （单脚网络） |
| 78 | PD9 | PD9 | （单脚网络） |
| 79 | PD10 | PD10 | （单脚网络） |
| 80 | PD11 | PD11 | （单脚网络） |
| 81 | PD12 | M3-PWM1 | U3-27（DRV8313 IN1） |
| 82 | PD13 | M3-PWM2 | U3-25（IN2） |
| 83 | VSS | GND | |
| 84 | VDD | 3V3 | |
| 85 | PD14 | M3-PWM3 | U3-23（IN3） |
| 86 | PD15 | PD15 | （单脚网络） |
| 87 | PG2 | DRV3-CS | DRV8313-3-2（LED 阳极；→R62 3.3K→GND） |
| 88 | PG3 | $1N142 | （单脚网络，未接） |
| 89 | PG4 | LED5 | LED10-2（LED 阳极；阴极→R64 3.3K→GND） |
| 90 | PG5 | LED6 | LED11-2（→R65 3.3K→GND） |
| 91 | PG6 | LED7 | LED12-2（→R66 3.3K→GND） |
| 92 | PG7 | LED8 | LED13-2（→R67 3.3K→GND） |
| 93 | PG8 | LED9 | LED14-2（→R68 3.3K→GND） |
| 94 | VSS | GND | |
| 95 | VDD | 3V3 | |
| 96 | PC6 | M3-A | ABZ-3-1 |
| 97 | PC7 | M3-B | ABZ-3-2 |
| 98 | PC8 | WRK | UNDEFINE-2（LED 阳极；阴极→R63 3.3K→GND） |
| 99 | PC9 | M3-SDA | R52-1（100Ω 串阻→U17-24 MPU6050 SDA）、I2C-3-1 |
| 100 | PA8 | M3-SCL | R51-1（100Ω 串阻→U17-23 MPU6050 SCL）、I2C-3-2 |
| 101 | PA9 | UART1-TX | U8-11（ESP32 IO26） |
| 102 | PA10 | UART1-RX | U8-10（ESP32 IO25） |
| 103 | PA11 | PA11 | （单脚网络） |
| 104 | PA12 | PA12 | （单脚网络） |
| 105 | PA13 | SWDIO | H2-4 |
| 106 | VCAP_2 | VCAP_1 | 与 U14-71 同网络（见上） |
| 107 | VSS | GND | |
| 108 | VDD | 3V3 | |
| 109 | PA14 | SCLK | H2-6（SWCLK） |
| 110 | PA15 | PA15 | （单脚网络） |
| 111 | PC10 | PC10 | （单脚网络） |
| 112 | PC11 | PC11 | （单脚网络） |
| 113 | PC12 | PC12 | （单脚网络） |
| 114 | PD0 | M1-PWM-EN | U1-22（EN3）、U1-24（EN2）、U1-26（EN1） |
| 115 | PD1 | M2-PWM-EN | U2-22、U2-24、U2-26（EN3/2/1） |
| 116 | PD2 | M3-PWM-EN | U3-22、U3-24、U3-26（EN3/2/1） |
| 117 | PD3 | USART2-CTS | （单脚网络） |
| 118 | PD4 | USART2-RTS | （单脚网络） |
| 119 | PD5 | USART2-TX | H2-3 |
| 120 | VSS | GND | |
| 121 | VDD | 3V3 | |
| 122 | PD6 | USART2-RX | H2-5 |
| 123 | PD7 | USART2-CK | （单脚网络） |
| 124 | PG9 | FAULT1 | U1-18（DRV8313 nFAULT）、LED16-2（LED 阳极；阴极→GND） |
| 125 | PG10 | FAULT2 | U2-18（nFAULT）、LED17-2 |
| 126 | PG11 | FAULT3 | U3-18（nFAULT）、LED18-2 |
| 127 | PG12 | PG12 | （单脚网络） |
| 128 | PG13 | PG13 | （单脚网络） |
| 129 | PG14 | PG14 | （单脚网络） |
| 130 | VSS | GND | |
| 131 | VDD | 3V3 | |
| 132 | PG15 | PG15 | （单脚网络） |
| 133 | PB3 | M1-B | ABZ-1-2 |
| 134 | PB4 | PB4 | （单脚网络） |
| 135 | PB5 | PB5 | （单脚网络） |
| 136 | PB6 | M1-SCL | I2C-1-2 |
| 137 | PB7 | M1-SDA | I2C-1-1 |
| 138 | BOOT0 | BOOT0 | B01-1（NC 电阻，另一端→3V3）、B00-2（B00 10K，另一端→GND） |
| 139 | PB8 | $1N193 | （单脚网络，未接） |
| 140 | PB9 | PB9 | （单脚网络） |
| 141 | PE0 | PE0 | （单脚网络） |
| 142 | PE1 | PE1 | （单脚网络） |
| 143 | PDR_ON | PDR_ON | R41-1（R41 10K→3V3 上拉）、R42-2（R42=NC，另一端→GND） |
| 144 | VDD | 3V3 | |

VDD/VSS 核对：10 个 VSS（16/31/38/51/61/83/94/107/120/130）全部接 GND；12 个 VDD（17/30/39/52/62/72/84/95/108/121/131/144）全部接 3V3；VBAT(6)、NRST(25)、VCAP_1(71)、VCAP_2(106)、BOOT0(138)、PDR_ON(143) 均与数据手册一致。

### 2.1 网络名与真实端口名一致性核对结论

核对方法：网络名本身是端口名的（如网络"PE2"），检查该网络所接 U14 引脚号的真实端口名（经 DS8626 + Olimex + 嘉立创库符号名三重验证）是否与网络名一致。

**结论：全部一致，未发现原理图符号引脚号错位问题。** 逐项核对清单（网络名=端口名，全部 ✓）：

- PE2(脚1)✓ PE3(2)✓ PE4(3)✓ PE5(4)✓ PE6(5)✓
- PC13(7)✓ PC14(8)✓ PC15(9)✓
- PF2(12)✓ PF3(13)✓ PF4(14)✓ PF5(15)✓ PF6(18)✓ PF7(19)✓ PF8(20)✓ PF9(21)✓ PF10(22)✓
- PH0(23)✓ PH1(24)✓（与 25MHz 晶振 X2 相接，正确）
- NRST(25)✓
- PC1(27)✓ PC2(28)✓ PC3(29)✓
- PA2(36)✓ PA3(37)✓ PA4(40)✓
- PC4(44)✓ PC5(45)✓ PB1(47)✓
- PF12(50)✓ PF13(53)✓ PF14(54)✓ PF15(55)✓
- PE7(58)✓ PE8(59)✓ PE10(63)✓ PE12(65)✓ PE14(67)✓ PE15(68)✓
- PD8(77)✓ PD9(78)✓ PD10(79)✓ PD11(80)✓ PD15(86)✓
- PA11(103)✓ PA12(104)✓ PA15(110)✓ PC10(111)✓ PC11(112)✓ PC12(113)✓
- PG12(127)✓ PG13(128)✓ PG14(129)✓ PG15(132)✓
- PB4(134)✓ PB5(135)✓ PB9(140)✓ PE0(141)✓ PE1(142)✓

网络名是功能名（非端口名）的引脚，其真实端口如下（供数据手册核对 AF 复用）：

| 网络名 | 引脚 | 真实端口 | 功能符合性（据 ST 手册 AF 表） |
|---|---|---|---|
| M2-SDA | U14-10 | PF0 | I2C2_SDA(AF4) ✓ |
| M2-SCL | U14-11 | PF1 | I2C2_SCL(AF4) ✓ |
| M2-A | U14-34 | PA0 | TIM2_CH1/TIM5_CH1 ✓（编码器） |
| M2-B | U14-35 | PA1 | TIM2_CH2/TIM5_CH2 ✓ |
| M1-A | U14-41 | PA5 | TIM2_CH1_ETR ✓ |
| M2-PWM1 | U14-42 | PA6 | TIM3_CH1 ✓ |
| M2-PWM2 | U14-43 | PA7 | TIM3_CH2 ✓ |
| M2-PWM3 | U14-46 | PB0 | TIM3_CH3 ✓ |
| ST-KEY | U14-49 | PF11 | GPIO 按键输入 |
| DRV1-CS | U14-56 | PG0 | GPIO（实为 LED 驱动，见第 11 节） |
| DRV2-CS | U14-57 | PG1 | GPIO（LED 驱动） |
| DRV3-CS | U14-87 | PG2 | GPIO（LED 驱动） |
| M1-PWM1 | U14-60 | PE9 | TIM1_CH1 ✓ |
| M1-PWM2 | U14-64 | PE11 | TIM1_CH2 ✓ |
| M1-PWM3 | U14-66 | PE13 | TIM1_CH3 ✓ |
| LSM6DSRTR-CS | U14-73 | PB12 | SPI2_NSS ✓ |
| LSM6DSRTR-SCK | U14-74 | PB13 | SPI2_SCK ✓ |
| LSM6DSRTR-MISO | U14-75 | PB14 | SPI2_MISO ✓ |
| LSM6DSRTR-MOSI | U14-76 | PB15 | SPI2_MOSI ✓ |
| M3-PWM1 | U14-81 | PD12 | TIM4_CH1 ✓ |
| M3-PWM2 | U14-82 | PD13 | TIM4_CH2 ✓ |
| M3-PWM3 | U14-85 | PD14 | TIM4_CH3 ✓ |
| LED5~LED9 | U14-89~93 | PG4~PG8 | GPIO（LED 驱动） |
| M3-A | U14-96 | PC6 | TIM3_CH1/TIM8_CH1 ✓（编码器；TIM3 与 M2-PWM 同定时器，若同用需注意，可改用 TIM8） |
| M3-B | U14-97 | PC7 | TIM3_CH2/TIM8_CH2 ✓ |
| WRK | U14-98 | PC8 | GPIO（LED 驱动） |
| M3-SDA | U14-99 | PC9 | I2C3_SDA(AF4) ✓ |
| M3-SCL | U14-100 | PA8 | I2C3_SCL(AF4) ✓ |
| UART1-TX | U14-101 | PA9 | USART1_TX ✓（与 ESP32 IO26 相接） |
| UART1-RX | U14-102 | PA10 | USART1_RX ✓（与 ESP32 IO25 相接） |
| SWDIO | U14-105 | PA13 | SWDIO ✓ |
| SCLK | U14-109 | PA14 | SWCLK ✓ |
| M1-PWM-EN | U14-114 | PD0 | GPIO |
| M2-PWM-EN | U14-115 | PD1 | GPIO |
| M3-PWM-EN | U14-116 | PD2 | GPIO |
| USART2-CTS | U14-117 | PD3 | USART2_CTS ✓ |
| USART2-RTS | U14-118 | PD4 | USART2_RTS ✓ |
| USART2-TX | U14-119 | PD5 | USART2_TX ✓（接 H2 排针） |
| USART2-RX | U14-122 | PD6 | USART2_RX ✓ |
| USART2-CK | U14-123 | PD7 | USART2_CK ✓ |
| FAULT1/2/3 | U14-124/125/126 | PG9/PG10/PG11 | GPIO（FAULT 输入） |
| M1-B | U14-133 | PB3 | TIM2_CH2 ✓ |
| M1-SCL | U14-136 | PB6 | I2C1_SCL(AF4) ✓ |
| M1-SDA | U14-137 | PB7 | I2C1_SDA(AF4) ✓ |
| BOOT1 | U14-48 | PB2/BOOT1 ✓ | B10/B11 均 NC → BOOT1 悬空（仅靠 PB2 内部下拉），建议核对 |
| PDR_ON | U14-143 | PDR_ON ✓ | R41 10K 上拉至 3V3，R42(NC) 预留下拉位 |

其它核对要点：
- 网络名 **VCAP_1 同时包含 U14-71（VCAP_1）和 U14-106（VCAP_2）**：两脚并在同一网络，C59、C60（各 2.2uF→GND）并联挂在该网络上。两 VCAP 内部为同一 1.2V 内核电源节点，此接法常见可行，但网络名只写了 VCAP_1（命名小瑕疵）。
- U14-32（VREF+）与 U14-33（VDDA）接同一网络 VDDA（经磁珠 L3 来自 3V3）：数据手册允许 VREF+ 接 VDDA 的配置。
- U14-26（PC0）挂在自动命名单脚网络 $1N80 上，等效未使用。
- UART1-TX/RX 与 ESP32 的对接：STM32 PA9/PA10（USART1）接 ESP32 **IO26/IO25**（见第 3 节），ESP32 的 UART 可由 GPIO 矩阵重映射到 IO25/26，功能可行，但不是模组硬件 UART0（RXD0=34/TXD0=35 用于 H3 下载口）。

---

## 3. ESP32-WROOM-32E-N4（U8）逐脚连接表

模组型号确认：PARTTYPE=ESP32-WROOM-32E-N4、符号=ESP32-WROOM-32E(4MB)、Value=2.4GHz、封装 WIFI-SMD_ESP32-WROOM-32E（LCSC C701341，乐鑫）。引脚号与乐鑫官方 38 脚定义一致（经 Espressif 数据手册引脚表验证：pin10=IO25、pin11=IO26、pin14=IO12、pin29=IO5、pin32=NC 等），第 39 脚为模块底部接地焊盘（EP）。

| 引脚 | 符号引脚名 | 网络名 | 同网络其它引脚 |
|---|---|---|---|
| 1 | GND | GND | |
| 2 | 3V3 | 3V3 | |
| 3 | EN | EN | C36-2（100nF→GND）、ESP-RST-1（复位键，另一端→GND）、R39-2（R39 10K→3V3 上拉） |
| 4 | SENSOR_VP | SENSOR_VP | （单脚网络） |
| 5 | SENSOR_VN | SENSOR_VN | （单脚网络） |
| 6 | IO34 | IO34 | （单脚网络） |
| 7 | IO35 | IO35 | （单脚网络） |
| 8 | IO32 | IO32 | （单脚网络） |
| 9 | IO33 | IO33 | （单脚网络） |
| 10 | IO25 | UART1-RX | U14-102（STM32 PA10/USART1_RX） |
| 11 | IO26 | UART1-TX | U14-101（STM32 PA9/USART1_TX） |
| 12 | IO27 | IO27 | （单脚网络） |
| 13 | IO14 | IO14 | （单脚网络） |
| 14 | IO12 | —（未出现在任何网络） | NC |
| 15 | GND | GND | |
| 16 | IO13 | IO13 | （单脚网络） |
| 17~24 | SD2/SD3/CMD/CLK/SD0/SD1/IO15/IO2 | — | NC（未出现在任何网络） |
| 25 | IO0 | IO0 | C107-2（100nF→GND）、ESP-BOOT-1（BOOT 按键，另一端→GND）、R38-2（R38 10K→3V3 上拉） |
| 26 | IO4 | IO4 | （单脚网络） |
| 27 | IO16 | IO16 | （单脚网络） |
| 28 | IO17 | IO17 | （单脚网络） |
| 29 | IO5 | —（未出现在任何网络） | NC |
| 30 | IO18 | IO18 | （单脚网络） |
| 31 | IO19 | IO19 | （单脚网络） |
| 32 | NC | —（未出现在任何网络） | NC |
| 33 | IO21 | IO21 | （单脚网络） |
| 34 | RXD0 | ESP-RXD0 | H3-2（下载排针） |
| 35 | TXD0 | ESP-TXD0 | H3-1（下载排针） |
| 36 | IO22 | IO22 | （单脚网络） |
| 37 | IO23 | IO23 | （单脚网络） |
| 38 | GND | GND | |
| 39 | GND（底部 EP） | GND | |

注意：网表中 "UART1-RX/TX" 网络实际落在模组 **IO25/IO26** 上（网表引脚号与乐鑫手册一致，符号引脚名亦为 IO25/IO26）。若固件按"硬件 UART1 默认引脚"理解会出错，需在固件中将 UART 重映射到 GPIO25/26。

---

## 4. DRV8313（U1 / U2 / U3，HTSSOP-28 + EP=29 脚）逐脚连接表

三片型号均为 DRV8313PWPR（TI）。符号引脚名与 TI SLVSBA5D 数据手册 PWP 封装引脚表一致（引脚 21=NC，网表中确未连接）。

### U1（M1 电机）

| 引脚 | 符号引脚名 | 网络名 | 同网络其它引脚 |
|---|---|---|---|
| 1 | CP1 | $1N50 | C3-2（C3 10nF，另一端→U1-2） |
| 2 | CP2 | $1N47 | C3-1 |
| 3 | VCP | $1N43 | C2-2（C2 100nF，另一端 C2-1→P+，即接 VM，符合 TI 手册） |
| 4 | VM | P+ | （电源网络） |
| 5 | OUT1 | M1-OUT1 | （单脚网络——电机相线，板上无连接器） |
| 6 | PGND1 | GND | |
| 7 | PGND2 | GND | |
| 8 | OUT2 | M1-OUT2 | （单脚网络） |
| 9 | OUT3 | M1-OUT3 | （单脚网络） |
| 10 | PGND3 | GND | |
| 11 | VM | P+ | |
| 12 | COMPP | GND | |
| 13 | COMPN | GND | |
| 14 | GND | GND | |
| 15 | V3P3OUT | $1N328 | U1-16（仅此两脚：V3P3OUT 与 RESET# 直连，**无 0.47uF 旁路电容**） |
| 16 | RESET# | $1N328 | 同上 |
| 17 | SLEEP# | DRV8313-SLEEP | U2-17、U3-17、R70-1、R71-1、R72-1、SW1-2 |
| 18 | FAULT# | FAULT1 | U14-124（PG9）、LED16-2 |
| 19 | COMPO# | GND | |
| 20 | GND | GND | |
| 21 | NC | —（未出现在任何网络） | NC |
| 22 | EN3 | M1-PWM-EN | U1-24（EN2）、U1-26（EN1）、U14-114 |
| 23 | IN3 | M1-PWM3 | U14-66 |
| 24 | EN2 | M1-PWM-EN | 同 22 |
| 25 | IN2 | M1-PWM2 | U14-64 |
| 26 | EN1 | M1-PWM-EN | 同 22 |
| 27 | IN1 | M1-PWM1 | U14-60 |
| 28 | GND | GND | |
| 29 | EP | GND | |

### U2（M2 电机）

| 引脚 | 符号引脚名 | 网络名 | 同网络其它引脚 |
|---|---|---|---|
| 1 | CP1 | $1N51 | C8-2（C8 10nF→U2-2） |
| 2 | CP2 | $1N48 | C8-1 |
| 3 | VCP | $1N45 | C7-2（C7 100nF，另一端 C7-1→P+/VM） |
| 4 | VM | P+ | |
| 5 | OUT1 | M2-OUT1 | （单脚网络） |
| 6 | PGND1 | GND | |
| 7 | PGND2 | GND | |
| 8 | OUT2 | M2-OUT2 | （单脚网络） |
| 9 | OUT3 | M2-OUT3 | （单脚网络） |
| 10 | PGND3 | GND | |
| 11 | VM | P+ | |
| 12 | COMPP | GND | |
| 13 | COMPN | GND | |
| 14 | GND | GND | |
| 15 | V3P3OUT | $1N329 | U2-16（仅两脚，无 0.47uF 旁路） |
| 16 | RESET# | $1N329 | |
| 17 | SLEEP# | DRV8313-SLEEP | |
| 18 | FAULT# | FAULT2 | U14-125（PG10）、LED17-2 |
| 19 | COMPO# | GND | |
| 20 | GND | GND | |
| 21 | NC | —（未连接） | NC |
| 22 | EN3 | M2-PWM-EN | U2-24、U2-26、U14-115 |
| 23 | IN3 | M2-PWM3 | U14-46 |
| 24 | EN2 | M2-PWM-EN | |
| 25 | IN2 | M2-PWM2 | U14-43 |
| 26 | EN1 | M2-PWM-EN | |
| 27 | IN1 | M2-PWM1 | U14-42 |
| 28 | GND | GND | |
| 29 | EP | GND | |

### U3（M3 电机）

| 引脚 | 符号引脚名 | 网络名 | 同网络其它引脚 |
|---|---|---|---|
| 1 | CP1 | $1N49 | C13-2（C13 10nF→U3-2） |
| 2 | CP2 | $1N46 | C13-1 |
| 3 | VCP | $1N44 | C12-2（C12 100nF，另一端 C12-1→P+/VM） |
| 4 | VM | P+ | |
| 5 | OUT1 | M3-OUT1 | （单脚网络） |
| 6 | PGND1 | GND | |
| 7 | PGND2 | GND | |
| 8 | OUT2 | M3-OUT2 | （单脚网络） |
| 9 | OUT3 | M3-OUT3 | （单脚网络） |
| 10 | PGND3 | GND | |
| 11 | VM | P+ | |
| 12 | COMPP | GND | |
| 13 | COMPN | GND | |
| 14 | GND | GND | |
| 15 | V3P3OUT | $1N330 | U3-16（仅两脚，无 0.47uF 旁路） |
| 16 | RESET# | $1N330 | |
| 17 | SLEEP# | DRV8313-SLEEP | |
| 18 | FAULT# | FAULT3 | U14-126（PG11）、LED18-2 |
| 19 | COMPO# | GND | |
| 20 | GND | GND | |
| 21 | NC | —（未连接） | NC |
| 22 | EN3 | M3-PWM-EN | U3-24、U3-26、U14-116 |
| 23 | IN3 | M3-PWM3 | U14-85 |
| 24 | EN2 | M3-PWM-EN | |
| 25 | IN2 | M3-PWM2 | U14-82 |
| 26 | EN1 | M3-PWM-EN | |
| 27 | IN1 | M3-PWM1 | U14-81 |
| 28 | GND | GND | |
| 29 | EP | GND | |

### DRV8313 外围核对（对照 TI SLVSBA5D）

- CPH-CPL 飞跨电容：C3/C8/C13 = 10nF ✓（TI：VM 耐压 0.01uF，接 CPH-CPL 之间）。
- VCP 电容：C2/C7/C12 = 100nF，接 **VCP→VM(P+)** ✓（TI：Connect a 16-V, 0.1-µF ceramic capacitor **to VM**）。
- VM 去耦：每片 2×100nF（U1:C1,C4；U2:C6,C9；U3:C11,C14）+ 板级 C5/C10/C15（10uF 电解）+ C76（47uF）。
- **偏差 1：** 三片的 V3P3OUT（脚15）均只与 RESET#（脚16）互连（$1N328/$1N329/$1N330），**没有 TI 要求的 6.3V/0.47uF 旁路电容**。
- **偏差 2：** nFAULT 为开漏输出，TI 要求外接上拉；FAULT1/2/3 网络上**没有任何上拉电阻**（只有 LED16/17/18 阳极接在 fault 线上、阴极接 GND——该接法使 LED 仅在线为高时导通，而线本身无上拉源，LED 不会点亮，MCU 读到的电平也不确定）。
- nSLEEP 由拨码开关 SW1 硬件选择 3V3/GND，**MCU 无控制**；R70/R71/R72（10K）+LED19/20/21 为 SLEEP 指示（高电平时经 10K 限流点亮，电流约 0.13mA，偏暗）。
- 电机相线 M1/M2/M3-OUT1~3 在网表中均为单脚网络，板上无电机连接器（应为直接焊接）。

---

## 5. MT6701（三片）——**网表中未找到**

- 在两个网表文件的全部器件字段（DESIGNATOR/PARTTYPE/Value/Device/LCSC Part Name/Manufacturer Part）中检索 "MT6701"：**0 条记录，网表中未找到**。
- 也没有任何其它磁编码器芯片（AS5600/TLE5012 等亦未出现）。
- 与之相关的板载接口为 6 个 SH1.0-4P 连接器（应为外接编码器小板）：
  - ABZ-1：1=M1-A（U14-41/PA5）、2=M1-B（U14-133/PB3）、3=3V3、4/5/6=GND
  - ABZ-2：1=M2-A（U14-34/PA0）、2=M2-B（U14-35/PA1）、3=3V3、4/5/6=GND
  - ABZ-3：1=M3-A（U14-96/PC6）、2=M3-B（U14-97/PC7）、3=3V3、4/5/6=GND
  - I2C-1：1=M1-SDA（U14-137/PB7）、2=M1-SCL（U14-136/PB6）、3=3V3、4/5/6=GND
  - I2C-2：1=M2-SDA（U14-10/PF0）、2=M2-SCL（U14-11/PF1）、3=3V3、4/5/6=GND
  - I2C-3：1=M3-SDA（U14-99/PC9）、2=M3-SCL（U14-100/PA8）、3=3V3、4/5/6=GND（I2C-3 上还并接 MPU-6050）
- 注意：M1/M2/M3 三条 I2C 总线在网表中**均无上拉电阻**（依赖外接模块上的上拉）；M3-SDA/M3-SCL 上各有 100Ω 串阻（R52/R51）到 MPU-6050。

---

## 6. SH367103（U4，TSSOP-16）逐脚连接表

型号：SH367103X/016XY-AAE00，LCSC C160668，sinowealth（中颖电子），"3/4串锂电池Pack保护芯片"。公开渠道未能获取该型号完整逐脚数据手册 PDF；下表"符号引脚名"取自嘉立创库符号（原理图网表），功能解读仅供参考、以原厂手册为准。

| 引脚 | 符号引脚名 | 网络名 | 本脚外围（位号+值+另一端网络） |
|---|---|---|---|
| 1 | CTL | $2N229 | R5（1K）→ $2N220（=U4-16 VDD 节点） |
| 2 | CHSE | $2N231 | C21（100nF）→ $2N220；R8（1M）→ CHARGE+ |
| 3 | CHG | $2N235 | R9（1K）→ $2N243（=U6-4 栅极节点） |
| 4 | VM | $2N236 | R10（1K）→ GND |
| 5 | DSG | $2N237 | R11（1K）→ $2N241（=U5-4 栅极节点） |
| 6 | DSD | $2N239 | C23（100nF）→ VBAT- |
| 7 | CDC | $2N238 | C22（100nF）→ VBAT- |
| 8 | VI | $2N234 | R7（1K）→ $2N233（=U5 源极/采样电阻上端节点） |
| 9 | TS | $2N230 | AT103-NTC（10K）→ VBAT-；C20（10nF）→ VBAT- |
| 10 | SEL | VBAT- | （直接接电池负极，推测为串数/模式选择） |
| 11 | GND | VBAT- | |
| 12 | VC4 | VBAT- | （接最低电位，对应 3 串配置） |
| 13 | VC3 | $2N223 | R4（1K）→ VBATP |
| 14 | VC2 | $2N222 | R3（1K）→ VBATM |
| 15 | VC1 | $2N221 | R2（1K）→ VBAT+ |
| 16 | VDD | $2N220 | C21（100nF）→ $2N231；R1（1K）→ $2N224；R5（1K）→ $2N229 |

相关网络的完整器件挂接明细（位号+值+另一端网络，全部照网表）：

- $2N220（U4-16 VDD、U4-1 经 R5）：C21-1（100nF，另一端 $2N231）、R1-1（1K，另一端 $2N224）、R5-2（1K，另一端 $2N229=U4-1）
- $2N231（U4-2 CHSE）：C21-2（100nF，另一端 $2N220）、R8-2（1M，另一端 CHARGE+）
- $2N229（U4-1 CTL）：R5-1（1K，另一端 $2N220）
- $2N224（驱动供电节点）：D1-1（1N5819W 阴极 C；阳极 D1-2→VBAT+）、D4-1（BZT52C15 阴极 K；阳极 D4-2→VBAT-）、C25-2（2.2uF，另一端→VBAT-）、R1-2（1K，另一端→$2N220）。即：该节点由 VBAT+ 经 D1 供电、被 D4（15V）对 VBAT- 钳位。
- $2N243（U6 栅极）：C24-1（100nF，另一端 C24-2→VBAT-）、R13-1（5.1M，另一端 R13-2→VBAT-）、R9-1（1K，来自 U4-3 CHG）、U6-4（G）
- $2N241（U5 栅极）：R12-2（10M，另一端 R12-1→$2N233）、R11-1（1K，来自 U4-5 DSG）、U5-4（G）
- $2N233（U5 源极/采样节点）：R6-1（4mΩ 采样电阻，另一端 R6-2→VBAT-）、R7-1（1K，来自 U4-8 VI）、R12-1（10M，另一端→U5 栅极）、U5-1/2/3（S）
- U5（TPN2R703NL N-FET）：S=$2N233、G=$2N241、D（脚5~9）=GND —— 充放电主回路开关（GND 与 VBAT- 之间串 R6）
- U6（TPN2R703NL N-FET）：S（脚1~3）=VBAT-、G（脚4）=$2N243、D（脚5~9）=$2N247 —— Q1 栅极驱动级
- $2N247（U6 漏极）：R14-2（1M，另一端 R14-1→$2N251=Q1 栅极）
- $2N251（Q1 栅极）：Q1-4（G）、R14-1（1M）、R15-1（2M，另一端 R15-2→$2N252）
- $2N252（Q1 源极）：Q1-1/2/3（S）、D5-1（1N5819W 阴极 C；阳极 D5-2→CHARGE+）、R15-2（2M）
- Q1（AON7403 P-FET）：S=$2N252、G=$2N251、D（脚5~9）=VBAT+ —— 充电通路开关
- 电芯电压接入链：VBAT+ →R2(1K)→ U4-15(VC1)；VBATM →R3(1K)→ U4-14(VC2)；VBATP →R4(1K)→ U4-13(VC3)；VBAT- → U4-12(VC4)/U4-11(GND)/U4-10(SEL)
- 逐节均衡（与 HY2213 配合，见第 10 节）：VBAT+ →R19(100)→ U9-2(VDD)；VBATM →R22(100)→ U10-2(VDD)；VBATP →R25(100)→ U11-2(VDD)

---

## 7. IP5306 —— **网表中未找到**

在两个网表文件全部器件字段中检索 "IP5306"：**0 条记录，网表中未找到**。本板的充电管理由 IP2326（U12）完成、3V3 由 TPS565201（U7）产生，详见第 8 节与第 10 节。

---

## 8. IP2326（U12，VQFN-24 + EP=25 脚）逐脚连接表

型号：IP2326，LCSC C2832094，INJOINIC（英集芯），"支持15W快充的2节/3节串联锂电池升压充电IC"。官方逐脚数据手册未能在公开渠道直接获取；"符号引脚名"取自嘉立创库符号（原理图网表），与升压充电拓扑完全吻合。

| 引脚 | 符号引脚名 | 网络名 | 本脚外围（位号+值+另一端网络） |
|---|---|---|---|
| 1 | DM | IP2326-DM | USB1-A7、USB1-B7（USB-C D-） |
| 2 | DP | IP2326-DP | USB1-A6、USB1-B6（USB-C D+） |
| 3 | VSET | $2N281 | RNTC（51K）→ VBAT- |
| 4 | NTC | $2N280 | RVST（68K）→ VBAT- |
| 5 | BAT_STAT | $2N279 | R29（1K）→ $2N274（LED4-2 阳极；LED4-1 阴极→VBAT-） |
| 6 | LED | $2N278 | R30（1K）→ $2N273（LED5-2 阳极；LED5-1 阴极→VBAT-） |
| 7, 8 | —（库符号中无对应网络） | —（未出现在任何网络） | NC |
| 9 | VIN_OVSET | $2N277 | R31（68K）→ VBAT- |
| 10 | CON_SEL | $2N276 | R32（1K）→ VBAT- |
| 11 | ISET | $2N275 | R33（75K）→ $2N265（=R36-1，50kΩ 电位器；R36-2→VBAT-） |
| 12 | EN | $2N264 | SW4-2（滑动开关公共端）；SW4-1 → R37（51K）→ VBUS；SW4-3/4/5 → VBAT- |
| 13 | VIN | $2N266 | C35（10uF）→ VBAT-；R28（0.5）→ VBUS |
| 14 | BST | $2N267 | C28（100nF）→ $2N269（=LX 开关节点） |
| 15, 16, 17 | LX | $2N269 | L1-1（2.2uH；L1-2→VBUS）；C28-1（100nF→BST） |
| 18 | PGND | VBAT- | |
| 19, 20 | VSYS | $2N270 | C31-1、C32-1（各 10uF；另一端 C31-2/C32-2→VBAT-） |
| 21, 22 | VOUT | CHARGE+ | C29-2、C30-2（各 10uF→VBAT-）；D5-2（1N5819W 阳极；阴极→$2N252=Q1 源极）；R8-1（1M→U4-2 CHSE） |
| 23, 24 | — | —（未出现在任何网络） | NC |
| 25 | EP | VBAT- | |

充电通路链（网表连接）：USB-C VBUS → L1（2.2uH）→ LX（U12-15/16/17）→ 内部升压 → VOUT（U12-21/22）= CHARGE+ → D5 → Q1（P-FET，栅极由 U6/SH367103-CHG 控制）→ VBAT+（3S 电池正极）。Q1 栅极节点 $2N251 由 R14（1M，来自 U6 漏极）下拉、R15（2M）上拉至自身源极。

---

## 9. 人机接口与调试（按键/拨码/排针/USB/BOOT/SWD）

### 9.1 按键（TS-1088-AR02016，4 只，引脚符号名见原理图网表）

| 位号 | 引脚 | 网络 | 外围 |
|---|---|---|---|
| ST-KEY（MCU 按键） | 1 | ST-KEY | U14-49（PF11）、C86（100nF→GND）、R53（10K 上拉→3V3）；引脚 2 → GND |
| ST-RST（MCU 复位） | 1 | NRST | U14-25、D2-1（PESD3V3L1BA ESD，另一端→GND）、C61（100nF→GND）、R43（1K 上拉→3V3）；引脚 2 → GND |
| ESP-BOOT | 1 | IO0 | U8-25、C107（100nF→GND）、R38（10K 上拉→3V3）；引脚 2 → GND |
| ESP-RST | 1 | EN | U8-3、C36（100nF→GND）、R39（10K 上拉→3V3）；引脚 2 → GND |

### 9.2 拨码/滑动开关

| 位号 | 引脚 | 网络 | 说明 |
|---|---|---|---|
| SW1（SLEEP 选择，SK12D07VG4） | 1 | 3V3 | 公共端 2 → DRV8313-SLEEP；3/4/5 → GND |
| SW3（总电源开关，SS12D10G4 071） | 3 | VBAT+ | 公共端 2 → Q7-1（G）；1 → $2N215 = D3-2（BZT52C16 阳极 A；阴极 D3-1→VBAT+）+ R60（10K→VBAT-）。即：开关置 1 位时 Q7 栅极经 R60 拉到 VBAT-（P-FET 导通，VBAT+ → Q7 → P+），置 3 位时栅极=VBAT+（关断） |
| SW4（IP2326 EN 选择，SK12D07VG4） | 2 | $2N264 | → U12-12（EN）；1 → R37（51K）→ VBUS；3/4/5 → VBAT- |

### 9.3 调试/下载排针

- **H2（2x3，SWD+串口）**：1=GND、2=GND、3=USART2-TX（U14-119/PD5）、4=SWDIO（U14-105/PA13）、5=USART2-RX（U14-122/PD6）、6=SCLK（U14-109/PA14=SWCLK）。注意：H2 上**没有 3V3/VCC 引脚**（供电需另取）。
- **H3（1x3，ESP32 下载口）**：1=ESP-TXD0（U8-35）、2=ESP-RXD0（U8-34）、3=GND。无 VCC/IO0/EN 引脚（IO0/EN 由板上按键控制）。

### 9.4 USB 座（USB1，TYPE-C16PIN）

| 引脚 | 符号引脚名 | 网络 |
|---|---|---|
| 1/2/3/4 | SHELL | VBAT- |
| A1B12 / B1A12 | GND | VBAT- |
| A4B9 / B4A9 | VBUS | VBUS |
| A5 | CC1 | $2N188（R34 5.1K → VBAT-） |
| B5 | CC2 | $2N189（R35 5.1K → VBAT-） |
| A6 / B6 | DP1 / DP2 | IP2326-DP |
| A7 / B7 | DN1 / DN2 | IP2326-DM |

CC1/CC2 各 5.1K 下拉（默认 UFP 受电），D+/D- 接 IP2326。本座仅用于充电输入，**不接 MCU/ESP32**。

### 9.5 BOOT 电路完整明细

- **BOOT0（U14-138）**：网络 BOOT0。B00（10K，另一端→GND，已贴）下拉；B01（NC 未贴，另一端→3V3）为上拉预留位。默认 BOOT0=低（主闪存启动）；焊接 B01 可进系统 bootloader。
- **BOOT1（U14-48=PB2）**：网络 BOOT1。B10（NC，另一端→GND）与 B11（NC，另一端→3V3）**均未贴**，BOOT1 实际悬空（PB2 内部有弱下拉，但建议复核是否需贴 B10）。

---

## 10. 电源树

### 10.1 电源网络与产生器件（输入→输出链路）

1. **3S 电池组（约 9.0~12.6V）**
   - 主接口：CN1（XT30）：CN1-1/3/4=VBAT-（符号名"-"），CN1-2=VBAT+（符号名"+"）
   - 平衡插座：CN2（XH 4P）：CN2-1=VBAT+、CN2-2=VBATM、CN2-3=VBATP、CN2-4/5/6=VBAT- —— 4 线平衡口，两个电心中点
   - **3 串证据**：3 片 HY2213-BB3A 各跨一节（U9：VDD 经 R19 100Ω 接 VBAT+、VSS=VBATM；U10：VDD 经 R22 接 VBATM、VSS=VBATP；U11：VDD 经 R25 接 VBATP、VSS=VBAT-）；SH367103 VC1/VC2/VC3 分别经 R2/R3/R4 接 VBAT+/VBATM/VBATP、VC4/GND/SEL 接 VBAT-（3 串配置）；IP2326 为 2S/3S 升压充电 IC
   - 均衡回路（每节相同结构）：HY2213 OUT → N-FET（Q4/Q5/Q6 AO3400）栅极；漏极节点 → R20/R23/R26（100Ω，另一端接该节上端）为主均衡电流；同节点 → R21/R24/R27（1K）→ LED1/LED2/LED3（阳极），LED 阴极接该节下端（VBATM/VBATP/VBAT-）作均衡指示
2. **VBAT+ → P+（总电源开关）**：VBAT+ → Q7（HY19P03D P-FET，S=VBAT+、D=P+、G 由 SW3-2 选择 VBAT+/经 R60 10K 到 VBAT-）→ **P+**。P+ 上挂：3 片 DRV8313 的 VM（各 2 脚）、TPS565201 VIN、去耦 C5/C10/C15（10uF 电解）、C76（47uF）、C1/C4/C6/C9/C11/C14（100nF）、R16（4.7K 上拉至 U7 EN）
3. **P+ → 3V3**：TPS565201（U7，5A 同步 buck）：VIN=U7-3（P+）、EN=U7-5（经 R16 4.7K 上拉到 VIN，常开）、SW=U7-2 → **L4（2.2uH）** → 3V3；VBST=U7-6 经 C82（100nF）接 SW；FB=U7-4 由 R18（49.9K，上端→3V3）/R17（14.7K，下端→GND）分压，Vout=0.760×(1+49.9/14.7)≈3.34V；输出电容：C83（47uF）等
4. **3V3 → VDDA**：磁珠 L3（GZ1608D601TF 600Ω）→ VDDA；VDDA 上挂 U14-32（VREF+）、U14-33（VDDA）、C56/C57（100nF→GND）
5. **MCU 备用电源 VBAT（U14-6）**：由 U15（BAT54C 共阴双二极管）供给：阳极 A=U15-1（3V3）、U15-2（VBAT-IN，**单脚网络，未接其它器件**——预留位），阴极 K=U15-3 → VBAT 网络 + C58（100nF→GND）
6. **VBUS（USB-C 5V 输入）**：USB1-A4B9/B4A9 → VBUS → R28（0.5Ω）→ IP2326 VIN；VBUS → L1 → IP2326 LX；VBUS → R37（51K）→ SW4（EN 选择）
7. **CHARGE+（IP2326 升压充电输出，3S 约 12.6V）**：U12-21/22（VOUT）→ C29/C30（10uF→VBAT-）→ D5 → Q1（栅极受 SH367103-CHG 经 U6 控制）→ VBAT+（给 3S 电池充电）
8. **$2N269（LX 开关节点）**：L1-1、U12-15/16/17（LX）、C28-1（100nF，另一端→U12-14 BST）
9. **$2N270（VSYS）**：U12-19/20，C31/C32（各 10uF→VBAT-）
10. 其它电源相关：**P+ 上另有 SH367103 驱动供电**（VBAT+ → D1 → $2N224 节点，D4 15V 钳位、C25 2.2uF 滤波 → R1 → U4-16 VDD）

### 10.2 主要电源网络成员（原样照抄，摘录）

- **P+（22 脚）**：C5-1 C10-1 C15-1 U1-4 U1-11 U3-4 U3-11 C1-2 C2-1 C4-2 C6-2 C7-1 C9-2 C11-2 C12-1 C14-2 U2-4 U2-11 C76-1 R16-1 U7-3 Q7-2
- **VBAT+（15 脚）**：CN1-2 CN2-1 SW3-3 C17-2 D1-2 Q1-5 Q1-6 Q1-7 Q1-8 Q1-9 R2-2 R19-2 R20-2 D3-1 Q7-3
- **VBAT-（56 脚）**：CN1-1 CN1-3 CN1-4 R36-2 U12-18 U12-25 USB1-1 USB1-2 USB1-3 USB1-4 USB1-A1B12 USB1-B1A12 CN2-4 CN2-5 CN2-6 C29-1 C30-1 C31-2 C32-2 C35-1 LED4-1 LED5-1 R31-1 R32-1 R34-1 R35-1 RNTC-1 RVST-1 C33-1 C34-1 SW4-3 SW4-4 SW4-5 AT103-NTC-1 C16-1 C17-1 C19-1 C20-1 C22-1 C23-1 C24-2 C25-1 C27-1 D4-2 LED3-1 Q6-2 R6-2 R13-2 U4-10 U4-11 U4-12 U6-1 U6-2 U6-3 U11-3 R60-1
- **VBATM（9 脚）**：CN2-2 C16-2 C18-1 LED1-1 Q4-2 R3-2 R22-2 R23-2 U9-3
- **VBATP（9 脚）**：CN2-3 C19-2 C26-1 LED2-1 Q5-2 R4-2 R25-2 R26-2 U10-3
- **VBUS（7 脚）**：L1-2 USB1-A4B9 USB1-B4A9 R28-2 R37-2 C33-2 C34-2
- **CHARGE+（6 脚）**：U12-21 U12-22 C29-2 C30-2 D5-2 R8-1
- **3V3（73 脚）**：L3-1 U14-17 U14-30 U14-39 U14-52 U14-62 U14-72 U14-84 U14-95 U14-108 U14-121 U14-131 U14-144 U15-1 U16-5 U16-8 U17-8 U17-13 C40-2 C41-2 C42-2 C43-2 C44-2 C45-2 C46-2 C47-2 C48-2 C49-2 C50-2 C51-2 C52-2 C53-2 C54-2 C55-2 C62-2 C63-2 C65-1 C66-1 R41-2 R43-2 R47-1 R48-1 R49-1 B01-2 B11-2 R59-1 C78-2 C79-2 C73-2 C77-2 C80-2 C81-2 I2C-1-3 I2C-2-3 I2C-3-3 C72-2 C74-2 C75-2 C84-2 L4-2 R18-1 SW1-1 C83-2 U8-2 R38-1 R39-1 ABZ-1-3 ABZ-2-3 ABZ-3-3 C37-2 C38-2 C39-2 R53-1
- **VDDA（5 脚）**：L3-2 U14-32 U14-33 C56-2 C57-1
- **VBAT（3 脚）**：U14-6 U15-3 C58-2
- **VBAT-IN（1 脚）**：U15-2（预留，未接其它器件）
- **NRST（5 脚）**：D2-1 ST-RST-1 U14-25 C61-2 R43-1
- **PDR_ON（3 脚）**：U14-143 R41-1 R42-2
- **GND（167 脚）**：（数量过大不逐条列出，含 U14 全部 VSS、三片 DRV8313 PGND/GND/EP、U5 漏极、USB 壳、按键/开关地、去耦电容地等）

板上**没有 5V 电源网络**：VBUS（USB 5V）仅为充电输入；电机母线为 P+（电池电压），逻辑电源为 3V3。

---

## 11. 功能网络汇总（网络名含 PWM/EN/FAULT/SLEEP/SDA/SCL/-A/-B/CS/OUT/TX/RX/KEY/BOOT/SW/DM/DP）

以下为 PCB 网表原文逐行照抄（格式：位号-引脚号 封装-引脚 类型）：

```
(M1-PWM1
 U1-27 DRV8313PWPR-27 Passive
 U14-60 STM32F407ZGT6-60 Passive
)
(M1-PWM2
 U1-25 DRV8313PWPR-25 Passive
 U14-64 STM32F407ZGT6-64 Passive
)
(M1-PWM3
 U1-23 DRV8313PWPR-23 Passive
 U14-66 STM32F407ZGT6-66 Passive
)
(M2-PWM1
 U14-42 STM32F407ZGT6-42 Passive
 U2-27 DRV8313PWPR-27 Passive
)
(M2-PWM2
 U14-43 STM32F407ZGT6-43 Passive
 U2-25 DRV8313PWPR-25 Passive
)
(M2-PWM3
 U14-46 STM32F407ZGT6-46 Passive
 U2-23 DRV8313PWPR-23 Passive
)
(M3-PWM1
 U3-27 DRV8313PWPR-27 Passive
 U14-81 STM32F407ZGT6-81 Passive
)
(M3-PWM2
 U3-25 DRV8313PWPR-25 Passive
 U14-82 STM32F407ZGT6-82 Passive
)
(M3-PWM3
 U3-23 DRV8313PWPR-23 Passive
 U14-85 STM32F407ZGT6-85 Passive
)
(M1-PWM-EN
 U1-22 DRV8313PWPR-22 Passive
 U1-24 DRV8313PWPR-24 Passive
 U1-26 DRV8313PWPR-26 Passive
 U14-114 STM32F407ZGT6-114 Passive
)
(M2-PWM-EN
 U14-115 STM32F407ZGT6-115 Passive
 U2-22 DRV8313PWPR-22 Passive
 U2-24 DRV8313PWPR-24 Passive
 U2-26 DRV8313PWPR-26 Passive
)
(M3-PWM-EN
 U3-22 DRV8313PWPR-22 Passive
 U3-24 DRV8313PWPR-24 Passive
 U3-26 DRV8313PWPR-26 Passive
 U14-116 STM32F407ZGT6-116 Passive
)
(FAULT1
 U1-18 DRV8313PWPR-18 Passive
 U14-124 STM32F407ZGT6-124 Passive
 LED16-2 LED_0402-R-2 Passive
)
(FAULT2
 U14-125 STM32F407ZGT6-125 Passive
 U2-18 DRV8313PWPR-18 Passive
 LED17-2 LED_0402-R-2 Passive
)
(FAULT3
 U3-18 DRV8313PWPR-18 Passive
 U14-126 STM32F407ZGT6-126 Passive
 LED18-2 LED_0402-R-2 Passive
)
(DRV8313-SLEEP
 U1-17 DRV8313PWPR-17 Passive
 U3-17 DRV8313PWPR-17 Passive
 U2-17 DRV8313PWPR-17 Passive
 R70-1 Res_0402-1 Passive
 R71-1 Res_0402-1 Passive
 R72-1 Res_0402-1 Passive
 SW1-2 SK12D07VG4-2 Passive
)
(M1-SDA
 U14-137 STM32F407ZGT6-137 Passive
 I2C-1-1 ZX-SH1.0-4PWT-1 Passive
)
(M1-SCL
 U14-136 STM32F407ZGT6-136 Passive
 I2C-1-2 ZX-SH1.0-4PWT-2 Passive
)
(M2-SDA
 U14-10 STM32F407ZGT6-10 Passive
 I2C-2-1 ZX-SH1.0-4PWT-1 Passive
)
(M2-SCL
 U14-11 STM32F407ZGT6-11 Passive
 I2C-2-2 ZX-SH1.0-4PWT-2 Passive
)
(M3-SDA
 U14-99 STM32F407ZGT6-99 Passive
 R52-1 Res_0603-1 Passive
 I2C-3-1 ZX-SH1.0-4PWT-1 Passive
)
(M3-SCL
 U14-100 STM32F407ZGT6-100 Passive
 R51-1 Res_0603-1 Passive
 I2C-3-2 ZX-SH1.0-4PWT-2 Passive
)
(LSM6DSRTR-CS
 U14-73 STM32F407ZGT6-73 Passive
 U16-12 LSM6DSRTR-12 Passive
 R49-2 Res_0603-2 Passive
)
(LSM6DSRTR-SCK
 R45-2 PTFR0402B220RP9-2 Passive
 U14-74 STM32F407ZGT6-74 Passive
 R48-2 Res_0603-2 Passive
)
(LSM6DSRTR-MISO
 R46-1 PTFR0402B220RP9-1 Passive
 U14-75 STM32F407ZGT6-75 Passive
)
(LSM6DSRTR-MOSI
 R44-2 PTFR0402B220RP9-2 Passive
 U14-76 STM32F407ZGT6-76 Passive
 R47-2 Res_0603-2 Passive
)
(M1-A
 U14-41 STM32F407ZGT6-41 Passive
 ABZ-1-1 ZX-SH1.0-4PWT-1 Passive
)
(M1-B
 U14-133 STM32F407ZGT6-133 Passive
 ABZ-1-2 ZX-SH1.0-4PWT-2 Passive
)
(M2-A
 U14-34 STM32F407ZGT6-34 Passive
 ABZ-2-1 ZX-SH1.0-4PWT-1 Passive
)
(M2-B
 U14-35 STM32F407ZGT6-35 Passive
 ABZ-2-2 ZX-SH1.0-4PWT-2 Passive
)
(M3-A
 U14-96 STM32F407ZGT6-96 Passive
 ABZ-3-1 ZX-SH1.0-4PWT-1 Passive
)
(M3-B
 U14-97 STM32F407ZGT6-97 Passive
 ABZ-3-2 ZX-SH1.0-4PWT-2 Passive
)
(DRV1-CS
 U14-56 STM32F407ZGT6-56 Passive
 DRV8313-1-2 LED_0402-R-2 Passive
)
(DRV2-CS
 U14-57 STM32F407ZGT6-57 Passive
 DRV8313-2-2 LED_0402-R-2 Passive
)
(DRV3-CS
 U14-87 STM32F407ZGT6-87 Passive
 DRV8313-3-2 LED_0402-R-2 Passive
)
(M1-OUT1
 U1-5 DRV8313PWPR-5 Passive
)
(M1-OUT2
 U1-8 DRV8313PWPR-8 Passive
)
(M1-OUT3
 U1-9 DRV8313PWPR-9 Passive
)
(M2-OUT1
 U2-5 DRV8313PWPR-5 Passive
)
(M2-OUT2
 U2-8 DRV8313PWPR-8 Passive
)
(M2-OUT3
 U2-9 DRV8313PWPR-9 Passive
)
(M3-OUT1
 U3-5 DRV8313PWPR-5 Passive
)
(M3-OUT2
 U3-8 DRV8313PWPR-8 Passive
)
(M3-OUT3
 U3-9 DRV8313PWPR-9 Passive
)
(UART1-TX
 U14-101 STM32F407ZGT6-101 Passive
 U8-11 ESP32-WROOM-32E-N4-11 Passive
)
(UART1-RX
 U14-102 STM32F407ZGT6-102 Passive
 U8-10 ESP32-WROOM-32E-N4-10 Passive
)
(USART2-TX
 U14-119 STM32F407ZGT6-119 Passive
 H2-3 HDR-M_2.54_2x3-3 Passive
)
(USART2-RX
 U14-122 STM32F407ZGT6-122 Passive
 H2-5 HDR-M_2.54_2x3-5 Passive
)
(USART2-CK
 U14-123 STM32F407ZGT6-123 Passive
)
(USART2-CTS
 U14-117 STM32F407ZGT6-117 Passive
)
(USART2-RTS
 U14-118 STM32F407ZGT6-118 Passive
)
(ESP-RXD0
 U8-34 ESP32-WROOM-32E-N4-34 Passive
 H3-2 HDR-M_2.54_1x3P-2 Passive
)
(ESP-TXD0
 U8-35 ESP32-WROOM-32E-N4-35 Passive
 H3-1 HDR-M_2.54_1x3P-1 Passive
)
(ST-KEY
 U14-49 STM32F407ZGT6-49 Passive
 C86-2 CAP_0603-2 Passive
 ST-KEY-1 TS-1088-AR02016-1 Passive
 R53-2 Res_0402-2 Passive
)
(IO0
 U8-25 ESP32-WROOM-32E-N4-25 Passive
 C107-2 CAP_0603-2 Passive
 ESP-BOOT-1 TS-1088-AR02016-1 Passive
 R38-2 Res_0402-2 Passive
)
(EN
 U8-3 ESP32-WROOM-32E-N4-3 Passive
 C36-2 CAP_0603-2 Passive
 ESP-RST-1 TS-1088-AR02016-1 Passive
 R39-2 Res_0603-2 Passive
)
(BOOT0
 U14-138 STM32F407ZGT6-138 Passive
 B01-1 Res_0603-1 Passive
 B00-2 Res_0603-2 Passive
)
(BOOT1
 U14-48 STM32F407ZGT6-48 Passive
 B11-1 Res_0603-1 Passive
 B10-2 Res_0603-2 Passive
)
(SWDIO
 U14-105 STM32F407ZGT6-105 Passive
 H2-4 HDR-M_2.54_2x3-4 Passive
)
(SCLK
 U14-109 STM32F407ZGT6-109 Passive
 H2-6 HDR-M_2.54_2x3-6 Passive
)
(IP2326-DM
 U12-1 IP2326-1 Passive
 USB1-A7 TYPE-C16PIN-A7 Passive
 USB1-B7 TYPE-C16PIN-B7 Passive
)
(IP2326-DP
 U12-2 IP2326-2 Passive
 USB1-B6 TYPE-C16PIN-B6 Passive
 USB1-A6 TYPE-C16PIN-A6 Passive
)
```

补充说明：
- 命名含 **-Z** 的网络：**网表中未找到**（编码器仅 A/B 两相）。
- 命名含 **CS** 的网络（DRV1-CS/DRV2-CS/DRV3-CS）实际都只接 LED（阳极），**并没有接到 DRV8313**——DRV8313 无 CS 引脚，这些网络是 LED 指示灯驱动线；真正的 SPI 片选只有 LSM6DSRTR-CS。
- 命名含 **OUT** 的 M1/M2/M3-OUT1~3 全部为单脚网络（电机相线无连接器）。
- 命名含 **KEY** 的还有 $2N263/$2N264（SW4→U12-12 EN，见第 8 节）；含 **SW** 的为 SWDIO/SCLK（SWCLK）。

---

## 12. 两份网表文件对比

`diff`（规范化 CRLF 后）结果：两文件文本差异极大（约 19905 个 diff 行；主文件 13419 行，对照文件 14611 行），但**语义级完全等价**：

| 对比项 | 结果 |
|---|---|
| 器件集合 | 完全相同（229 个，无增减，FOOTPRINT/PARTTYPE 无一差异） |
| 网络集合 | 完全相同（252 个，无增减） |
| 网络成员（引脚连接） | **0 差异**（252 个网络的引脚集合逐一对齐） |
| 器件块顺序 | 不同（PCB 网表以 C5 开头、原理图网表以 U1 开头等） |
| 器件属性 | 原理图网表多出 Symbol/LCSC Part Name/Supplier Part/Manufacturer/Manufacturer Part/Datasheet/Value 等字段（PCB 网表只有 Footprint/Parttype/Value/Device） |
| 网络行第二列 | PCB 网表为"封装-**引脚号**"（如 `STM32F407ZGT6-69`）；原理图网表为"封装-**符号引脚名**"（如 `STM32F407ZGT6-PB10`），共 442 个引脚的第二列写法不同（IC 类引脚） |

主要差异点总结：**仅导出格式/属性详略与排序不同，电气连接（器件+网络+逐脚连接）100% 一致**。原理图网表的符号引脚名是本报告引脚名核对的直接依据。

---

## 13. 引脚表核对来源（URL）

1. **ST DS8626 Rev 12**《STM32F405xx/STM32F407xx datasheet》，Table 7（STM32F40x pin definitions，LQFP144 列）+ Figure 14（LQFP144 pinout）：https://www.st.com/resource/en/datasheet/dm00037051.pdf
2. **Olimex STM32-E407 用户手册（rev L，2021）** 第 27 页 STM32F407ZGT6 引脚表（完整 144 脚编号-端口名对照）：https://www.olimex.com/Products/ARM/ST/STM32-E407/resources/STM32-E407.pdf
3. **Espressif ESP32-WROOM-32E/32D/32U 数据手册** 引脚定义表（pin10=IO25、pin11=IO26、pin14=IO12、pin29=IO5、pin32=NC，与网表符号名一致）：https://documentation.espressif.com （镜像见 cdn.sparkfun.com / Mouser）
4. **TI DRV8313 数据手册 SLVSBA5D**（PWP 封装 1~29 脚定义、CPH/CPL 0.01uF、VCP 0.1uF 接 VM、V3P3OUT 0.47uF 旁路、nFAULT 需上拉）：https://www.ti.com/lit/ds/symlink/drv8313.pdf
5. **TI TPS565201 数据手册**（DDC/SOT-23-6：1=GND、2=SW、3=VIN、4=VFB、5=EN、6=VBST；VREF=0.760V）：https://www.ti.com/lit/ds/symlink/tps565201.pdf
6. **InvenSense PS-MPU-6000A**（QFN-24：1=CLKIN、9=AD0、10=REGOUT(0.1µF)、13=VDD、18=GND、20=CPOUT(2.2nF)、23=SCL、24=SDA）：https://cdn.sparkfun.com/datasheets/Components/General%20IC/PS-MPU-6000A.pdf
7. **HYCON HY2213 系列数据手册**（单节均衡 IC，SOT-23-6）：https://cdn.promelec.ru/upload/items/2024/03/04/hy2213-Series_.pdf
8. **嘉立创器件库符号引脚名与 LCSC 元数据**：内嵌于对照文件 `Netlist_MB_Schematic_2026-10-07.net`（如 U4=C160668 sinowealth 中颖"3/4串锂电池Pack保护芯片"、U12=C2832094 英集芯"2S/3S 15W 升压充电IC"）；SH367103 与 IP2326 的原厂完整逐脚数据手册未能在公开渠道获取，其引脚名以嘉立创库符号为准，功能解读需以原厂手册复核。

## 14. 网表中发现的设计疑点清单（供复核，非连接错误）

1. DRV8313 V3P3OUT（脚15）与 RESET#（脚16）直连且无 0.47uF 旁路电容（$1N328/$1N329/$1N330，三片相同）。
2. FAULT1/2/3 无上拉电阻；LED16/17/18 接于 FAULT 线与 GND 之间，正常情况下不点亮、FAULT 电平不确定。
3. BOOT1（U14-48/PB2）的 B10/B11 均为 NC → BOOT1 悬空。
4. R42（PDR_ON 下拉位）= NC，PDR_ON 由 R41 10K 上拉至 3V3（符合内部复位使能要求，仅提示）。
5. 网络名 VCAP_1 同时包含 U14-71（VCAP_1）与 U14-106（VCAP_2）两脚（C59/C60 并联同一网络）。
6. M1/M2/M3 三路 I2C 总线（接 I2C-1/2/3 座）无上拉电阻，依赖外接模块。
7. M3-A/M3-B（PC6/PC7）与 M2-PWM1/2（PA6/PA7）同属 TIM3 CH1/CH2，若同时使用需改用 TIM8（PC6/PC7 的 AF4）或重映射。
8. U8-10/U8-11（ESP32 IO25/IO26）承载网络名 UART1-RX/TX：非模组硬件 UART0 引脚，固件需重映射。
9. L4 的 Value=2.2uH 与其 PARTTYPE（SDFL1608S100KTF，编码为 10uH）不一致。
10. R6 的 Value=4mΩ 与其型号后缀（RE1206F1R100）不一致，建议核对 BOM。
11. H2（SWD/串口排针）无 VCC 引脚；H3（ESP 下载口）无 VCC/IO0 引脚。
12. "DRV8313-1/2/3"、"UNDEFINE"、"3V3" 是 LED 位号而非芯片/电源，易误读。
