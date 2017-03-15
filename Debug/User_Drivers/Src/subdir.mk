################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User_Drivers/Src/cc1101_routine.c \
../User_Drivers/Src/circular_queue.c \
../User_Drivers/Src/rs_work.c 

OBJS += \
./User_Drivers/Src/cc1101_routine.o \
./User_Drivers/Src/circular_queue.o \
./User_Drivers/Src/rs_work.o 

C_DEPS += \
./User_Drivers/Src/cc1101_routine.d \
./User_Drivers/Src/circular_queue.d \
./User_Drivers/Src/rs_work.d 


# Each subdirectory must supply rules for building sources it contributes
User_Drivers/Src/%.o: ../User_Drivers/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -U__printf_float -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Inc" -I/home/gs-ms/STMCube_Workspace/STM32_TEST/User_Drivers/Inc -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


