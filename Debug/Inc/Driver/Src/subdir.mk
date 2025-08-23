################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Inc/Driver/Src/stm32f401re_gpio.c \
../Inc/Driver/Src/stm32f401re_i2c.c \
../Inc/Driver/Src/stm32f401re_rcc.c \
../Inc/Driver/Src/stm32f401re_spi.c \
../Inc/Driver/Src/stm32f401re_usart.c 

C_DEPS += \
./Inc/Driver/Src/stm32f401re_gpio.d \
./Inc/Driver/Src/stm32f401re_i2c.d \
./Inc/Driver/Src/stm32f401re_rcc.d \
./Inc/Driver/Src/stm32f401re_spi.d \
./Inc/Driver/Src/stm32f401re_usart.d 

OBJS += \
./Inc/Driver/Src/stm32f401re_gpio.o \
./Inc/Driver/Src/stm32f401re_i2c.o \
./Inc/Driver/Src/stm32f401re_rcc.o \
./Inc/Driver/Src/stm32f401re_spi.o \
./Inc/Driver/Src/stm32f401re_usart.o 


# Each subdirectory must supply rules for building sources it contributes
Inc/Driver/Src/%.o Inc/Driver/Src/%.su Inc/Driver/Src/%.cyclo: ../Inc/Driver/Src/%.c Inc/Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DNUCLEO_F401RE -DSTM32 -DSTM32F401RETx -DSTM32F4 -c -I../Inc -I"C:/Users/subha/Desktop/STMCUBE/Suvasis003/Inc/Driver/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Inc-2f-Driver-2f-Src

clean-Inc-2f-Driver-2f-Src:
	-$(RM) ./Inc/Driver/Src/stm32f401re_gpio.cyclo ./Inc/Driver/Src/stm32f401re_gpio.d ./Inc/Driver/Src/stm32f401re_gpio.o ./Inc/Driver/Src/stm32f401re_gpio.su ./Inc/Driver/Src/stm32f401re_i2c.cyclo ./Inc/Driver/Src/stm32f401re_i2c.d ./Inc/Driver/Src/stm32f401re_i2c.o ./Inc/Driver/Src/stm32f401re_i2c.su ./Inc/Driver/Src/stm32f401re_rcc.cyclo ./Inc/Driver/Src/stm32f401re_rcc.d ./Inc/Driver/Src/stm32f401re_rcc.o ./Inc/Driver/Src/stm32f401re_rcc.su ./Inc/Driver/Src/stm32f401re_spi.cyclo ./Inc/Driver/Src/stm32f401re_spi.d ./Inc/Driver/Src/stm32f401re_spi.o ./Inc/Driver/Src/stm32f401re_spi.su ./Inc/Driver/Src/stm32f401re_usart.cyclo ./Inc/Driver/Src/stm32f401re_usart.d ./Inc/Driver/Src/stm32f401re_usart.o ./Inc/Driver/Src/stm32f401re_usart.su

.PHONY: clean-Inc-2f-Driver-2f-Src

