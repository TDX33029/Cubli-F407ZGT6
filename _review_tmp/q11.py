import re, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
txt = open(r'D:/Document/tempPrj/Cubli-F407ZGT6/无刷闭环联调/Netlist_MB_Schematic_2026-10-07.net', encoding='utf-8', errors='replace').read()
lines = txt.splitlines()
i, n = 0, len(lines)
keys = ['LCSC Part Name','Supplier Part','Manufacturer','Manufacturer Part','Datasheet','Symbol','Value']
want = {'U14','U8','U1','U2','U3','U4','U12','U7','U16','U17','U9','U10','U11','U5','U6','U15','Q1','Q4','Q7','USB1','X1','X2','L3','L4','L1','SW1','SW3','SW4','ST-KEY','ST-RST','ESP-BOOT','ESP-RST','H2','H3','CN1','CN2','D1','D3','D4','D5','D2','B00','B01','B10','B11','R36','RNTC','RVST','AT103-NTC','R44','R59'}
while i < n:
    if lines[i].strip() == '[':
        j = i+1; block = []
        while j < n and lines[j].strip() != ']':
            block.append(lines[j]); j += 1
        if len(block) >= 2:
            des = block[1].strip()
            if des in want:
                meta = {}
                k = 0
                while k < len(block)-1:
                    key = block[k].strip()
                    if key in keys and key not in meta:
                        meta[key] = block[k+1].strip()
                    k += 1
                print(f"{des}: " + " | ".join(f"{k}={v}" for k,v in meta.items() if v))
        i = j+1
    else:
        i += 1
