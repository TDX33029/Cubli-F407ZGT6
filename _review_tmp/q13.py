import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
nets = d['nets']
targets = ['M1-PWM1','M1-PWM2','M1-PWM3','M2-PWM1','M2-PWM2','M2-PWM3','M3-PWM1','M3-PWM2','M3-PWM3',
'M1-PWM-EN','M2-PWM-EN','M3-PWM-EN','FAULT1','FAULT2','FAULT3','DRV8313-SLEEP',
'M1-SDA','M1-SCL','M2-SDA','M2-SCL','M3-SDA','M3-SCL','LSM6DSRTR-CS','LSM6DSRTR-SCK','LSM6DSRTR-MISO','LSM6DSRTR-MOSI',
'M1-A','M1-B','M2-A','M2-B','M3-A','M3-B','DRV1-CS','DRV2-CS','DRV3-CS',
'M1-OUT1','M1-OUT2','M1-OUT3','M2-OUT1','M2-OUT2','M2-OUT3','M3-OUT1','M3-OUT2','M3-OUT3',
'UART1-TX','UART1-RX','USART2-TX','USART2-RX','USART2-CK','USART2-CTS','USART2-RTS',
'ESP-RXD0','ESP-TXD0','ST-KEY','IO0','EN','BOOT0','BOOT1','SWDIO','SCLK','IP2326-DM','IP2326-DP']
for t in targets:
    print(f"({t}")
    for p in nets[t]:
        print(f" {p}")
    print(")")
