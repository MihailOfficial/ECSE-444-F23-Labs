################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/stmpe811/stmpe811.c 

OBJS += \
./Drivers/Components/stmpe811/stmpe811.o 

C_DEPS += \
./Drivers/Components/stmpe811/stmpe811.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/stmpe811/%.o Drivers/Components/stmpe811/%.su Drivers/Components/stmpe811/%.cyclo: ../Drivers/Components/stmpe811/%.c Drivers/Components/stmpe811/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I/Lab1/Core -I/Lab1/Drivers/CMSIS -I/Lab1/Drivers/Components -I/Lab1/Drivers/STM32L4xx_HAL_Driver -I/Users/mihailcalitoiu/STM32CubeIDE/workspace_1.13.1/Lab1/Drivers/B-L4S5I-IOT01 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-stmpe811

clean-Drivers-2f-Components-2f-stmpe811:
	-$(RM) ./Drivers/Components/stmpe811/stmpe811.cyclo ./Drivers/Components/stmpe811/stmpe811.d ./Drivers/Components/stmpe811/stmpe811.o ./Drivers/Components/stmpe811/stmpe811.su

.PHONY: clean-Drivers-2f-Components-2f-stmpe811

