################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../src/BarGraph.cpp \
../src/DigitalIoPin.cpp \
../src/Game_FreeRTOSv2.cpp \
../src/IntegerEdit.cpp \
../src/LiquidCrystal.cpp \
../src/MenuItem.cpp \
../src/SaveEdit.cpp \
../src/SimpleMenu.cpp \
../src/StateMachine.cpp \
../src/StatusEdit.cpp \
../src/cr_cpp_config.cpp \
../src/cr_startup_lpc15xx.cpp \
../src/eepromIO.cpp \
../src/lcd_port.cpp 

C_SRCS += \
../src/ITM_write.c \
../src/crp.c \
../src/sysinit.c 

OBJS += \
./src/BarGraph.o \
./src/DigitalIoPin.o \
./src/Game_FreeRTOSv2.o \
./src/ITM_write.o \
./src/IntegerEdit.o \
./src/LiquidCrystal.o \
./src/MenuItem.o \
./src/SaveEdit.o \
./src/SimpleMenu.o \
./src/StateMachine.o \
./src/StatusEdit.o \
./src/cr_cpp_config.o \
./src/cr_startup_lpc15xx.o \
./src/crp.o \
./src/eepromIO.o \
./src/lcd_port.o \
./src/sysinit.o 

CPP_DEPS += \
./src/BarGraph.d \
./src/DigitalIoPin.d \
./src/Game_FreeRTOSv2.d \
./src/IntegerEdit.d \
./src/LiquidCrystal.d \
./src/MenuItem.d \
./src/SaveEdit.d \
./src/SimpleMenu.d \
./src/StateMachine.d \
./src/StatusEdit.d \
./src/cr_cpp_config.d \
./src/cr_startup_lpc15xx.d \
./src/eepromIO.d \
./src/lcd_port.d 

C_DEPS += \
./src/ITM_write.d \
./src/crp.d \
./src/sysinit.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C++ Compiler'
	arm-none-eabi-c++ -D__NEWLIB__ -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -DCPP_USE_HEAP -D__LPC15XX__ -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/lpc_board_nxp_lpcxpresso_1549/inc" -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/lpc_chip_15xx/inc" -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/Game_FreeRTOSv2/freertos/inc" -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/Game_FreeRTOSv2/src" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__NEWLIB__ -DDEBUG -D__CODE_RED -DCORE_M3 -D__USE_LPCOPEN -DCPP_USE_HEAP -D__LPC15XX__ -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/lpc_board_nxp_lpcxpresso_1549/inc" -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/lpc_chip_15xx/inc" -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/Game_FreeRTOSv2/freertos/inc" -I"/Users/macos/Documents/LPCXpresso_8.2.2/Shooting_Gallery/Game_FreeRTOSv2/src" -O0 -fno-common -g3 -Wall -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -mcpu=cortex-m3 -mthumb -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


