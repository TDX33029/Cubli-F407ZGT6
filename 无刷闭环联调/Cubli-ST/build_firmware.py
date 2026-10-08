# -*- coding: utf-8 -*-
import os
import sys
import time
import subprocess

def build():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)
    mdk_dir = "MDK-ARM"
    obj_dir = "MDK-ARM/CubeMX"

    armcc = "C:/Keil_v5/ARM/ARMCC/bin/armcc.exe"
    armasm = "C:/Keil_v5/ARM/ARMCC/bin/armasm.exe"
    armlink = "C:/Keil_v5/ARM/ARMCC/bin/armlink.exe"
    fromelf = "C:/Keil_v5/ARM/ARMCC/bin/fromelf.exe"

    if not os.path.exists(armcc):
        print(f"Error: ARMCC not found at {armcc}")
        return False

    os.makedirs(obj_dir, exist_ok=True)

    inc_paths = [
        "Core/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc",
        "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy",
        "Drivers/CMSIS/Device/ST/STM32F4xx/Include",
        "Drivers/CMSIS/Include",
        "Hardware",
        "SimpleFOC"
    ]

    inc_args = []
    for p in inc_paths:
        inc_args.extend(["-I", p])

    defines = ["-D__UVISION_VERSION=542", "-D_RTE_", "-DSTM32F407xx", "-DUSE_HAL_DRIVER"]
    cflags = ["--cpu=Cortex-M4.fp.sp", "-g", "-O2", "--apcs=interwork", "--split_sections", "--c99"] + defines + inc_args

    srcs = [
        ("c", "Core/Src/main.c", "MDK-ARM/CubeMX/main.o"),
        ("c", "Core/Src/gpio.c", "MDK-ARM/CubeMX/gpio.o"),
        ("c", "Core/Src/i2c.c", "MDK-ARM/CubeMX/i2c.o"),
        ("c", "Core/Src/tim.c", "MDK-ARM/CubeMX/tim.o"),
        ("c", "Core/Src/usart.c", "MDK-ARM/CubeMX/usart.o"),
        ("c", "Core/Src/stm32f4xx_it.c", "MDK-ARM/CubeMX/stm32f4xx_it.o"),
        ("c", "Core/Src/stm32f4xx_hal_msp.c", "MDK-ARM/CubeMX/stm32f4xx_hal_msp.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c", "MDK-ARM/CubeMX/stm32f4xx_hal_i2c.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_i2c_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c", "MDK-ARM/CubeMX/stm32f4xx_hal_rcc.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_rcc_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c", "MDK-ARM/CubeMX/stm32f4xx_hal_flash.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_flash_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c", "MDK-ARM/CubeMX/stm32f4xx_hal_flash_ramfunc.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c", "MDK-ARM/CubeMX/stm32f4xx_hal_gpio.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_dma_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c", "MDK-ARM/CubeMX/stm32f4xx_hal_dma.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c", "MDK-ARM/CubeMX/stm32f4xx_hal_pwr.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_pwr_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_cortex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c", "MDK-ARM/CubeMX/stm32f4xx_hal.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c", "MDK-ARM/CubeMX/stm32f4xx_hal_exti.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c", "MDK-ARM/CubeMX/stm32f4xx_hal_tim.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c", "MDK-ARM/CubeMX/stm32f4xx_hal_tim_ex.o"),
        ("c", "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c", "MDK-ARM/CubeMX/stm32f4xx_hal_uart.o"),
        ("c", "Core/Src/system_stm32f4xx.c", "MDK-ARM/CubeMX/system_stm32f4xx.o"),
        ("c", "SimpleFOC/foc_utils.c", "MDK-ARM/CubeMX/foc_utils.o"),
        ("c", "SimpleFOC/FOCMotor.c", "MDK-ARM/CubeMX/focmotor.o"),
        ("c", "SimpleFOC/BLDCMotor.c", "MDK-ARM/CubeMX/bldcmotor.o"),
        ("c", "SimpleFOC/CurrentSense.c", "MDK-ARM/CubeMX/currentsense.o"),
        ("c", "Hardware/timer.c", "MDK-ARM/CubeMX/timer.o"),
        ("c", "Hardware/delay.c", "MDK-ARM/CubeMX/delay.o"),
        ("c", "Hardware/spi.c", "MDK-ARM/CubeMX/spi.o"),
        ("c", "Hardware/lsm6dsr.c", "MDK-ARM/CubeMX/lsm6dsr.o"),
        ("c", "Hardware/mpu6050.c", "MDK-ARM/CubeMX/mpu6050.o"),
        ("c", "Hardware/lqr_balance.c", "MDK-ARM/CubeMX/lqr_balance.o"),
        ("c", "Hardware/enc_quad.c", "MDK-ARM/CubeMX/enc_quad.o"),
        ("s", "MDK-ARM/startup_stm32f407xx.s", "MDK-ARM/CubeMX/startup_stm32f407xx.o"),
    ]

    print(f"Building Cubli-F407ZGT6 firmware ({len(srcs)} files)...")
    errors = 0
    for stype, src, obj in srcs:
        src_path = os.path.normpath(src)
        obj_path = os.path.normpath(obj)
        if os.path.exists(obj_path):
            try: os.remove(obj_path)
            except Exception: pass
        if stype == "c":
            cmd = [armcc, "-c"] + cflags + [src_path, "-o", obj_path]
        else:
            cmd = [armasm, "--cpu=Cortex-M4.fp.sp", "-g", "--apcs=interwork", "--pd", "__MICROLIB SETA 1", src_path, "-o", obj_path]
        
        proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if proc.returncode != 0:
            time.sleep(0.05)
            if os.path.exists(obj_path):
                try: os.remove(obj_path)
                except Exception: pass
            proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if proc.returncode != 0:
            print(f"FAILED: {os.path.basename(src)}")
            print(proc.stderr.decode("gbk", errors="ignore") or proc.stdout.decode("gbk", errors="ignore"))
            errors += 1

    if errors > 0:
        print(f"Build failed with {errors} compilation error(s).")
        return False

    print("All files compiled cleanly. Linking...")
    orig_cwd = os.getcwd()
    os.chdir(mdk_dir)
    try:
        link_cmd = [armlink, "--via=CubeMX/CubeMX.lnp"]
        proc = subprocess.run(link_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out = proc.stdout.decode("gbk", errors="ignore") + proc.stderr.decode("gbk", errors="ignore")
        if proc.returncode != 0:
            print("Link failed:")
            print(out)
            return False
        
        print("Link Output:")
        print(out.strip())

        hex_cmd = [fromelf, "--i32", "--output=CubeMX/CubeMX.hex", "CubeMX/CubeMX.axf"]
        subprocess.run(hex_cmd)
        print("Successfully generated CubeMX/CubeMX.hex!")
        return True
    finally:
        os.chdir(orig_cwd)

if __name__ == "__main__":
    success = build()
    sys.exit(0 if success else 1)
