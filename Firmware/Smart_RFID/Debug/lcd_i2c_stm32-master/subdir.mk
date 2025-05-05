################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Admin/Downloads/lcd_i2c_stm32-master/lcd_i2c_stm32-master/MFRC522.c \
C:/Users/Admin/Downloads/lcd_i2c_stm32-master/lcd_i2c_stm32-master/i2clcd.c 

OBJS += \
./lcd_i2c_stm32-master/MFRC522.o \
./lcd_i2c_stm32-master/i2clcd.o 

C_DEPS += \
./lcd_i2c_stm32-master/MFRC522.d \
./lcd_i2c_stm32-master/i2clcd.d 


# Each subdirectory must supply rules for building sources it contributes
lcd_i2c_stm32-master/MFRC522.o: C:/Users/Admin/Downloads/lcd_i2c_stm32-master/lcd_i2c_stm32-master/MFRC522.c lcd_i2c_stm32-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Downloads/lcd_i2c_stm32-master/lcd_i2c_stm32-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
lcd_i2c_stm32-master/i2clcd.o: C:/Users/Admin/Downloads/lcd_i2c_stm32-master/lcd_i2c_stm32-master/i2clcd.c lcd_i2c_stm32-master/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Downloads/lcd_i2c_stm32-master/lcd_i2c_stm32-master" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-lcd_i2c_stm32-2d-master

clean-lcd_i2c_stm32-2d-master:
	-$(RM) ./lcd_i2c_stm32-master/MFRC522.cyclo ./lcd_i2c_stm32-master/MFRC522.d ./lcd_i2c_stm32-master/MFRC522.o ./lcd_i2c_stm32-master/MFRC522.su ./lcd_i2c_stm32-master/i2clcd.cyclo ./lcd_i2c_stm32-master/i2clcd.d ./lcd_i2c_stm32-master/i2clcd.o ./lcd_i2c_stm32-master/i2clcd.su

.PHONY: clean-lcd_i2c_stm32-2d-master

