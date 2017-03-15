################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/dma.c \
../Src/gpio.c \
../Src/iwdg.c \
../Src/main.c \
../Src/rng.c \
../Src/spi.c \
../Src/stm32l4xx_hal_msp.c \
../Src/stm32l4xx_it.c \
../Src/system_stm32l4xx.c \
../Src/tim.c \
../Src/usart.c 

OBJS += \
./Src/dma.o \
./Src/gpio.o \
./Src/iwdg.o \
./Src/main.o \
./Src/rng.o \
./Src/spi.o \
./Src/stm32l4xx_hal_msp.o \
./Src/stm32l4xx_it.o \
./Src/system_stm32l4xx.o \
./Src/tim.o \
./Src/usart.o 

C_DEPS += \
./Src/dma.d \
./Src/gpio.d \
./Src/iwdg.d \
./Src/main.d \
./Src/rng.d \
./Src/spi.d \
./Src/stm32l4xx_hal_msp.d \
./Src/stm32l4xx_it.d \
./Src/system_stm32l4xx.d \
./Src/tim.d \
./Src/usart.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32L476xx -U__printf_float -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Inc" -I/home/gs-ms/STMCube_Workspace/STM32_TEST/User_Drivers/Inc -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/STM32L4xx_HAL_Driver/Inc" -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy" -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/CMSIS/Device/ST/STM32L4xx/Include" -I"/home/gs-ms/STMCube_Workspace/STM32_TEST/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


