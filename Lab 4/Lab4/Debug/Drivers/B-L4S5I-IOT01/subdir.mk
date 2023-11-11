################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.c \
../Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.c 

OBJS += \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.o \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.o 

C_DEPS += \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.d \
./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/B-L4S5I-IOT01/%.o Drivers/B-L4S5I-IOT01/%.su Drivers/B-L4S5I-IOT01/%.cyclo: ../Drivers/B-L4S5I-IOT01/%.c Drivers/B-L4S5I-IOT01/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I/Lab1/Core -I/Lab1/Drivers/CMSIS -I/Lab1/Drivers/Components -I/Lab1/Drivers/STM32L4xx_HAL_Driver -I/Users/mihailcalitoiu/STM32CubeIDE/workspace_1.13.1/Lab1/Drivers/B-L4S5I-IOT01 -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-B-2d-L4S5I-2d-IOT01

clean-Drivers-2f-B-2d-L4S5I-2d-IOT01:
	-$(RM) ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_qspi.su ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.cyclo ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.d ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.o ./Drivers/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.su

.PHONY: clean-Drivers-2f-B-2d-L4S5I-2d-IOT01

