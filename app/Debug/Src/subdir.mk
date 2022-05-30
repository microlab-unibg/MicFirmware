################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/app.c \
../Src/bsp_driver_sd.c \
../Src/fatfs.c \
../Src/fatfs_platform.c \
../Src/log.c \
../Src/main.c \
../Src/sd_diskio.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/syscalls.c \
../Src/system_stm32l4xx.c 

OBJS += \
./Src/app.o \
./Src/bsp_driver_sd.o \
./Src/fatfs.o \
./Src/fatfs_platform.o \
./Src/log.o \
./Src/main.o \
./Src/sd_diskio.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/syscalls.o \
./Src/system_stm32l4xx.o 

C_DEPS += \
./Src/app.d \
./Src/bsp_driver_sd.d \
./Src/fatfs.d \
./Src/fatfs_platform.d \
./Src/log.d \
./Src/main.d \
./Src/sd_diskio.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/syscalls.d \
./Src/system_stm32l4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DBLUENRG_MS=1 -DSTM32L475xx -c -I../Inc -I../Src/modules/BlueNRG/includes -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/app.d ./Src/app.o ./Src/bsp_driver_sd.d ./Src/bsp_driver_sd.o ./Src/fatfs.d ./Src/fatfs.o ./Src/fatfs_platform.d ./Src/fatfs_platform.o ./Src/log.d ./Src/log.o ./Src/main.d ./Src/main.o ./Src/sd_diskio.d ./Src/sd_diskio.o ./Src/stm32l4xx_hal_msp.d ./Src/stm32l4xx_hal_msp.o ./Src/stm32l4xx_it.d ./Src/stm32l4xx_it.o ./Src/syscalls.d ./Src/syscalls.o ./Src/system_stm32l4xx.d ./Src/system_stm32l4xx.o

.PHONY: clean-Src

