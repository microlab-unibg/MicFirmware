################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/modules/RTC/rtc.c 

OBJS += \
./Src/modules/RTC/rtc.o 

C_DEPS += \
./Src/modules/RTC/rtc.d 


# Each subdirectory must supply rules for building sources it contributes
Src/modules/RTC/%.o: ../Src/modules/RTC/%.c Src/modules/RTC/subdir.mk
	arm-none-eabi-gcc -c "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DUSE_HAL_DRIVER -DBLUENRG_MS=1 -DSTM32L475xx -c -I../Inc -I../Src/modules/BlueNRG/includes -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Src-2f-modules-2f-RTC

clean-Src-2f-modules-2f-RTC:
	-$(RM) ./Src/modules/RTC/rtc.d ./Src/modules/RTC/rtc.o

.PHONY: clean-Src-2f-modules-2f-RTC

