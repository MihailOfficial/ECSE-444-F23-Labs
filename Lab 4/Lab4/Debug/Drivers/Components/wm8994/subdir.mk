################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/wm8994/wm8994.c 

OBJS += \
./Drivers/Components/wm8994/wm8994.o 

C_DEPS += \
./Drivers/Components/wm8994/wm8994.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/wm8994/%.o Drivers/Components/wm8994/%.su Drivers/Components/wm8994/%.cyclo: ../Drivers/Components/wm8994/%.c Drivers/Components/wm8994/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I/Lab1/Core -I/Lab1/Drivers/CMSIS -I/Lab1/Drivers/Components -I/Lab1/Drivers/STM32L4xx_HAL_Driver -I/Users/mihailcalitoiu/STM32CubeIDE/workspace_1.13.1/Lab1/Drivers/B-L4S5I-IOT01 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-wm8994

clean-Drivers-2f-Components-2f-wm8994:
	-$(RM) ./Drivers/Components/wm8994/wm8994.cyclo ./Drivers/Components/wm8994/wm8994.d ./Drivers/Components/wm8994/wm8994.o ./Drivers/Components/wm8994/wm8994.su

.PHONY: clean-Drivers-2f-Components-2f-wm8994

