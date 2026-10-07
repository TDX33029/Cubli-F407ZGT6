import json, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
d = json.load(open(r'D:/Document/tempPrj/Cubli-F407ZGT6/_review_tmp/parsed_MAIN.json', encoding='utf-8'))
devs, nets = d['devices'], d['nets']
# key devices by designator guess
for des, v in devs.items():
    pt = (v.get('parttype','') + ' ' + v.get('value','')).upper()
    if any(k in pt for k in ['STM32','ESP32','DRV8313','MT6701','SH367103','IP5306','IP2326','LSM6DSR','IP2326','CH340','MPU','AMS1117','ME6211','XC6206','TPS','LDO','USB','TYPE-C','HROP','XTAL','Y_','CRYSTAL','16MHZ','8MHZ','32.768']):
        print(repr(des), '|', v.get('parttype'), '|', v.get('value'), '|', v.get('footprint'), '| dev:', v.get('device'))
