################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/MFRC522.c \
C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/gpio.c \
C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/i2clcd.c \
C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/keypad.c \
C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/rtc.c \
C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/systick.c 

OBJS += \
./comm/MFRC522.o \
./comm/gpio.o \
./comm/i2clcd.o \
./comm/keypad.o \
./comm/rtc.o \
./comm/systick.o 

C_DEPS += \
./comm/MFRC522.d \
./comm/gpio.d \
./comm/i2clcd.d \
./comm/keypad.d \
./comm/rtc.d \
./comm/systick.d 


# Each subdirectory must supply rules for building sources it contributes
comm/MFRC522.o: C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/MFRC522.c comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
comm/gpio.o: C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/gpio.c comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
comm/i2clcd.o: C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/i2clcd.c comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
comm/keypad.o: C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/keypad.c comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
comm/rtc.o: C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/rtc.c comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
comm/systick.o: C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm/systick.c comm/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Admin/Desktop/HK242/Embedded/RFID-Card-Reader/Firmware/comm" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-comm

clean-comm:
	-$(RM) ./comm/MFRC522.cyclo ./comm/MFRC522.d ./comm/MFRC522.o ./comm/MFRC522.su ./comm/gpio.cyclo ./comm/gpio.d ./comm/gpio.o ./comm/gpio.su ./comm/i2clcd.cyclo ./comm/i2clcd.d ./comm/i2clcd.o ./comm/i2clcd.su ./comm/keypad.cyclo ./comm/keypad.d ./comm/keypad.o ./comm/keypad.su ./comm/rtc.cyclo ./comm/rtc.d ./comm/rtc.o ./comm/rtc.su ./comm/systick.cyclo ./comm/systick.d ./comm/systick.o ./comm/systick.su

.PHONY: clean-comm

