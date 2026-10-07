# -*- coding: utf-8 -*-
"""
Cubli-F407ZGT6 工程全自动命令行编译与固件打包工具
调用 Keil ARMCC 编译器与连接器完整构建 CubeMX.axf 与 CubeMX.hex
"""
import os
import sys
import subprocess

def build():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    proj_dir = script_dir
    mdk_dir = os.path.join(proj_dir, "MDK-ARM")
    obj_dir = os.path.join(mdk_dir, "CubeMX")

    armcc = r"C:\Keil_v5\ARM\ARMCC\bin\armcc.exe"
    armasm = r"C:\Keil_v5\ARM\ARMCC\bin\armasm.exe"
    armlink = r"C:\Keil_v5\ARM\ARMCC\bin\armlink.exe"
    fromelf = r"C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe"

    if not os.path.exists(armcc):
        print(f"Error: ARMCC not found at {armcc}")
        return False

    os.makedirs(obj_dir, exist_ok=True)

    inc_paths = [
        os.path.join(proj_dir, "Core/Inc"),
        os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Inc"),
        os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Inc/Legacy"),
        os.path.join(proj_dir, "Drivers/CMSIS/Device/ST/STM32F4xx/Include"),
        os.path.join(proj_dir, "Drivers/CMSIS/Include"),
        os.path.join(proj_dir, "Hardware"),
        os.path.join(proj_dir, "SimpleFOC")
    ]

    inc_args = []
    for p in inc_paths:
        inc_args.extend(["-I", p])

    defines = ["-D__UVISION_VERSION=542", "-D_RTE_", "-DSTM32F407xx", "-DUSE_HAL_DRIVER"]
    cflags = ["--cpu=Cortex-M4.fp.sp", "-g", "-O2", "--apcs=interwork", "--split_sections", "--c99"] + defines + inc_args

    srcs = [
        ("c", os.path.join(proj_dir, "Core/Src/main.c"), os.path.join(obj_dir, "main.o")),
        ("c", os.path.join(proj_dir, "Core/Src/gpio.c"), os.path.join(obj_dir, "gpio.o")),
        ("c", os.path.join(proj_dir, "Core/Src/i2c.c"), os.path.join(obj_dir, "i2c.o")),
        ("c", os.path.join(proj_dir, "Core/Src/tim.c"), os.path.join(obj_dir, "tim.o")),
        ("c", os.path.join(proj_dir, "Core/Src/usart.c"), os.path.join(obj_dir, "usart.o")),
        ("c", os.path.join(proj_dir, "Core/Src/stm32f4xx_it.c"), os.path.join(obj_dir, "stm32f4xx_it.o")),
        ("c", os.path.join(proj_dir, "Core/Src/stm32f4xx_hal_msp.c"), os.path.join(obj_dir, "stm32f4xx_hal_msp.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c"), os.path.join(obj_dir, "stm32f4xx_hal_i2c.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c"), os.path.join(obj_dir, "stm32f4xx_hal_i2c_ex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"), os.path.join(obj_dir, "stm32f4xx_hal_rcc.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"), os.path.join(obj_dir, "stm32f4xx_hal_rcc_ex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"), os.path.join(obj_dir, "stm32f4xx_hal_flash.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"), os.path.join(obj_dir, "stm32f4xx_hal_flash_ex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"), os.path.join(obj_dir, "stm32f4xx_hal_flash_ramfunc.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"), os.path.join(obj_dir, "stm32f4xx_hal_gpio.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"), os.path.join(obj_dir, "stm32f4xx_hal_dma_ex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"), os.path.join(obj_dir, "stm32f4xx_hal_dma.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"), os.path.join(obj_dir, "stm32f4xx_hal_pwr.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"), os.path.join(obj_dir, "stm32f4xx_hal_pwr_ex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"), os.path.join(obj_dir, "stm32f4xx_hal_cortex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"), os.path.join(obj_dir, "stm32f4xx_hal.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c"), os.path.join(obj_dir, "stm32f4xx_hal_exti.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"), os.path.join(obj_dir, "stm32f4xx_hal_tim.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"), os.path.join(obj_dir, "stm32f4xx_hal_tim_ex.o")),
        ("c", os.path.join(proj_dir, "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c"), os.path.join(obj_dir, "stm32f4xx_hal_uart.o")),
        ("c", os.path.join(proj_dir, "Core/Src/system_stm32f4xx.c"), os.path.join(obj_dir, "system_stm32f4xx.o")),
        ("c", os.path.join(proj_dir, "SimpleFOC/foc_utils.c"), os.path.join(obj_dir, "foc_utils.o")),
        ("c", os.path.join(proj_dir, "SimpleFOC/FOCMotor.c"), os.path.join(obj_dir, "focmotor.o")),
        ("c", os.path.join(proj_dir, "SimpleFOC/BLDCMotor.c"), os.path.join(obj_dir, "bldcmotor.o")),
        ("c", os.path.join(proj_dir, "SimpleFOC/CurrentSense.c"), os.path.join(obj_dir, "currentsense.o")),
        ("c", os.path.join(proj_dir, "Hardware/timer.c"), os.path.join(obj_dir, "timer.o")),
        ("c", os.path.join(proj_dir, "Hardware/delay.c"), os.path.join(obj_dir, "delay.o")),
        ("c", os.path.join(proj_dir, "Hardware/spi.c"), os.path.join(obj_dir, "spi.o")),
        ("c", os.path.join(proj_dir, "Hardware/lsm6dsr.c"), os.path.join(obj_dir, "lsm6dsr.o")),
        ("c", os.path.join(proj_dir, "Hardware/mpu6050.c"), os.path.join(obj_dir, "mpu6050.o")),
        ("c", os.path.join(proj_dir, "Hardware/lqr_balance.c"), os.path.join(obj_dir, "lqr_balance.o")),
        ("c", os.path.join(proj_dir, "Hardware/enc_quad.c"), os.path.join(obj_dir, "enc_quad.o")),
        ("s", os.path.join(mdk_dir, "startup_stm32f407xx.s"), os.path.join(obj_dir, "startup_stm32f407xx.o")),
    ]

    print(f"Building Cubli-F407ZGT6 firmware ({len(srcs)} files)...")
    errors = 0
    for stype, src, obj in srcs:
        if stype == "c":
            cmd = [armcc, "-c"] + cflags + [src, "-o", obj]
        else:
            cmd = [armasm, "--cpu=Cortex-M4.fp.sp", "-g", "--apcs=interwork", "--pd", "__MICROLIB SETA 1", src, "-o", obj]
        
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
