################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/buzzer/buzzer.c 

OBJS += \
./Drivers/BSP/Components/buzzer/buzzer.o 

C_DEPS += \
./Drivers/BSP/Components/buzzer/buzzer.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/buzzer/%.o Drivers/BSP/Components/buzzer/%.su: ../Drivers/BSP/Components/buzzer/%.c Drivers/BSP/Components/buzzer/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L475xx -c -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-Components-2f-buzzer

clean-Drivers-2f-BSP-2f-Components-2f-buzzer:
	-$(RM) ./Drivers/BSP/Components/buzzer/buzzer.d ./Drivers/BSP/Components/buzzer/buzzer.o ./Drivers/BSP/Components/buzzer/buzzer.su

.PHONY: clean-Drivers-2f-BSP-2f-Components-2f-buzzer

