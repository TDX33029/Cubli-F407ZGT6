@echo off
REM === Post-CubeMX generation fix ===
REM Run from project root (where CubeMX.ioc is)

set "PROJ=%~dp0"
set "CONF=%PROJ%Core\Inc\stm32f4xx_hal_conf.h"
set "FW_USART_H=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.3\Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_usart.h"
set "FW_USART_C=%USERPROFILE%\STM32Cube\Repository\STM32Cube_FW_F4_V1.28.3\Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_usart.c"
set "DEST_H=%PROJ%Drivers\STM32F4xx_HAL_Driver\Inc\stm32f4xx_hal_usart.h"
set "DEST_C=%PROJ%Drivers\STM32F4xx_HAL_Driver\Src\stm32f4xx_hal_usart.c"

echo [fix_cubemx] Removing #include "main.h" from conf.h...
powershell -NoProfile -Command "(Get-Content '%CONF%' -Raw) -replace '#include .main\.h.\r?\n', '' | Set-Content '%CONF%' -NoNewline"

echo [fix_cubemx] Restoring USART HAL driver files...
copy /Y "%FW_USART_H%" "%DEST_H%" >nul 2>&1
copy /Y "%FW_USART_C%" "%DEST_C%" >nul 2>&1

echo [fix_cubemx] Done.
exit /b 0
