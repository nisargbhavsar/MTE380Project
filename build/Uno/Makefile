ifeq ($(OS),Windows_NT)
SHELL = $(ComSpec)
RMDIR = rmdir /s /q
RM = del /q
mymkdir = if not exist "$1" mkdir "$1"
else
RMDIR = rm -fr
RM = rm -f
mymkdir = mkdir -p $1
endif

PROJECT_OBJS = \
	project/AFMotor.cpp.o \
	project/ArduinoTest.cpp.o \
	project/SensorIMU.cpp.o \
	project/SensorUltrasonic.cpp.o \

PLATFORM_CORE_OBJS = \
	core/IPAddress.cpp.o \
	core/WString.cpp.o \
	core/Tone.cpp.o \
	core/main.cpp.o \
	core/WMath.cpp.o \
	core/Print.cpp.o \
	core/wiring_pulse.S.o \
	core/wiring.c.o \
	core/wiring_digital.c.o \
	core/wiring_analog.c.o \
	core/HardwareSerial0.cpp.o \
	core/CDC.cpp.o \
	core/HardwareSerial.cpp.o \
	core/wiring_shift.c.o \
	core/USBCore.cpp.o \
	core/wiring_pulse.c.o \
	core/HardwareSerial3.cpp.o \
	core/new.cpp.o \
	core/HardwareSerial2.cpp.o \
	core/WInterrupts.c.o \
	core/hooks.c.o \
	core/HardwareSerial1.cpp.o \
	core/Stream.cpp.o \
	core/abi.cpp.o \
	core/PluggableUSB.cpp.o \

PLATFORM_VARIANT_OBJS = \

LIBRARIES_OBJS = \
	libraries/VL53L1X/1.0.1/VL53L1X.cpp.o \
	libraries/ST_HW_HC_SR04/2.0.3/src/ST_HW_HC_SR04.cpp.o \
	libraries/Wire/src/utility/twi.c.o \
	libraries/Wire/src/Wire.cpp.o \
	libraries/SPI/src/SPI.cpp.o \
	libraries/MPU6050_tockn/1.4.0/src/MPU6050_tockn.cpp.o \
	libraries/SoftwareSerial/src/SoftwareSerial.cpp.o \
	libraries/HID/src/HID.cpp.o \

TARGETS = \
	ArduinoTest.hex \

all: $(TARGETS)

ArduinoTest.hex: ArduinoTest.elf
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-objcopy" -O ihex -R .eeprom  "./ArduinoTest.elf" "./ArduinoTest.hex"

ArduinoTest.elf: $(PROJECT_OBJS) $(LIBRARIES_OBJS) core.a
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p  -o "./ArduinoTest.elf" $(PROJECT_OBJS) $(LIBRARIES_OBJS) "./core.a" "-L." -lm

core.a:	$(PLATFORM_CORE_OBJS) $(PLATFORM_VARIANT_OBJS)

clean:
	-$(RMDIR) project core variant libraries
	-$(RM) *.hex *.eep *.bin *.elf *.a *.ar *.d

size:
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-size" -A "./ArduinoTest.elf"

project/AFMotor.cpp.o: ../../AFMotor.cpp project/AFMotor.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

project/AFMotor.cpp.d: ;

-include project/AFMotor.cpp.d 

project/ArduinoTest.cpp.o: ../../ArduinoTest.cpp project/ArduinoTest.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

project/ArduinoTest.cpp.d: ;

-include project/ArduinoTest.cpp.d 

project/SensorIMU.cpp.o: ../../SensorIMU.cpp project/SensorIMU.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

project/SensorIMU.cpp.d: ;

-include project/SensorIMU.cpp.d 

project/SensorUltrasonic.cpp.o: ../../SensorUltrasonic.cpp project/SensorUltrasonic.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

project/SensorUltrasonic.cpp.d: ;

-include project/SensorUltrasonic.cpp.d 


core/IPAddress.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/IPAddress.cpp core/IPAddress.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/IPAddress.cpp.d: ;

-include core/IPAddress.cpp.d

core/WString.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/WString.cpp core/WString.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/WString.cpp.d: ;

-include core/WString.cpp.d

core/Tone.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/Tone.cpp core/Tone.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/Tone.cpp.d: ;

-include core/Tone.cpp.d

core/main.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/main.cpp core/main.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/main.cpp.d: ;

-include core/main.cpp.d

core/WMath.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/WMath.cpp core/WMath.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/WMath.cpp.d: ;

-include core/WMath.cpp.d

core/Print.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/Print.cpp core/Print.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/Print.cpp.d: ;

-include core/Print.cpp.d

core/wiring_pulse.S.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_pulse.S
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -x assembler-with-cpp -flto -MMD -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/wiring.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring.c core/wiring.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring.c.d: ;

-include core/wiring.c.d

core/wiring_digital.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_digital.c core/wiring_digital.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_digital.c.d: ;

-include core/wiring_digital.c.d

core/wiring_analog.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_analog.c core/wiring_analog.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_analog.c.d: ;

-include core/wiring_analog.c.d

core/HardwareSerial0.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial0.cpp core/HardwareSerial0.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial0.cpp.d: ;

-include core/HardwareSerial0.cpp.d

core/CDC.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/CDC.cpp core/CDC.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/CDC.cpp.d: ;

-include core/CDC.cpp.d

core/HardwareSerial.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial.cpp core/HardwareSerial.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial.cpp.d: ;

-include core/HardwareSerial.cpp.d

core/wiring_shift.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_shift.c core/wiring_shift.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_shift.c.d: ;

-include core/wiring_shift.c.d

core/USBCore.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/USBCore.cpp core/USBCore.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/USBCore.cpp.d: ;

-include core/USBCore.cpp.d

core/wiring_pulse.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/wiring_pulse.c core/wiring_pulse.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/wiring_pulse.c.d: ;

-include core/wiring_pulse.c.d

core/HardwareSerial3.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial3.cpp core/HardwareSerial3.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial3.cpp.d: ;

-include core/HardwareSerial3.cpp.d

core/new.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/new.cpp core/new.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/new.cpp.d: ;

-include core/new.cpp.d

core/HardwareSerial2.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial2.cpp core/HardwareSerial2.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial2.cpp.d: ;

-include core/HardwareSerial2.cpp.d

core/WInterrupts.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/WInterrupts.c core/WInterrupts.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/WInterrupts.c.d: ;

-include core/WInterrupts.c.d

core/hooks.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/hooks.c core/hooks.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"
	
core/hooks.c.d: ;

-include core/hooks.c.d

core/HardwareSerial1.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/HardwareSerial1.cpp core/HardwareSerial1.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/HardwareSerial1.cpp.d: ;

-include core/HardwareSerial1.cpp.d

core/Stream.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/Stream.cpp core/Stream.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/Stream.cpp.d: ;

-include core/Stream.cpp.d

core/abi.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/abi.cpp core/abi.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/abi.cpp.d: ;

-include core/abi.cpp.d

core/PluggableUSB.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino/PluggableUSB.cpp core/PluggableUSB.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc-ar" rcs  "./core.a" "$@"

core/PluggableUSB.cpp.d: ;

-include core/PluggableUSB.cpp.d



libraries/VL53L1X/1.0.1/VL53L1X.cpp.o: /home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1/VL53L1X.cpp libraries/VL53L1X/1.0.1/VL53L1X.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/VL53L1X/1.0.1/VL53L1X.cpp.d: ;

-include libraries/VL53L1X/1.0.1/VL53L1X.cpp.d

libraries/ST_HW_HC_SR04/2.0.3/src/ST_HW_HC_SR04.cpp.o: /home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src/ST_HW_HC_SR04.cpp libraries/ST_HW_HC_SR04/2.0.3/src/ST_HW_HC_SR04.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/ST_HW_HC_SR04/2.0.3/src/ST_HW_HC_SR04.cpp.d: ;

-include libraries/ST_HW_HC_SR04/2.0.3/src/ST_HW_HC_SR04.cpp.d

libraries/Wire/src/utility/twi.c.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src/utility/twi.c libraries/Wire/src/utility/twi.c.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-gcc" -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections -MMD -flto -fno-fat-lto-objects -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/Wire/src/utility/twi.c.d: ;

-include libraries/Wire/src/utility/twi.c.d

libraries/Wire/src/Wire.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src/Wire.cpp libraries/Wire/src/Wire.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/Wire/src/Wire.cpp.d: ;

-include libraries/Wire/src/Wire.cpp.d

libraries/SPI/src/SPI.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src/SPI.cpp libraries/SPI/src/SPI.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/SPI/src/SPI.cpp.d: ;

-include libraries/SPI/src/SPI.cpp.d

libraries/MPU6050_tockn/1.4.0/src/MPU6050_tockn.cpp.o: /home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src/MPU6050_tockn.cpp libraries/MPU6050_tockn/1.4.0/src/MPU6050_tockn.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/MPU6050_tockn/1.4.0/src/MPU6050_tockn.cpp.d: ;

-include libraries/MPU6050_tockn/1.4.0/src/MPU6050_tockn.cpp.d

libraries/SoftwareSerial/src/SoftwareSerial.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src/SoftwareSerial.cpp libraries/SoftwareSerial/src/SoftwareSerial.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/SoftwareSerial/src/SoftwareSerial.cpp.d: ;

-include libraries/SoftwareSerial/src/SoftwareSerial.cpp.d

libraries/HID/src/HID.cpp.o: /home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src/HID.cpp libraries/HID/src/HID.cpp.d
	@$(call mymkdir,$(dir $@))
	"/home/nisarg/.arduinocdt/packages/arduino/tools/avr-gcc/5.4.0-atmel3.6.1-arduino2/bin/avr-g++" -c -g -Os -w -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics -Wno-error=narrowing -MMD -flto -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10608 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR   -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/cores/arduino" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/variants/standard" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/EEPROM/src" -I"/home/nisarg/.arduinocdt/libraries/VL53L1X/1.0.1" -I"/home/nisarg/.arduinocdt/libraries/ST_HW_HC_SR04/2.0.3/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/Wire/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SPI/src" -I"/home/nisarg/.arduinocdt/libraries/MPU6050_tockn/1.4.0/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/SoftwareSerial/src" -I"/home/nisarg/.arduinocdt/packages/arduino/hardware/avr/1.6.23/libraries/HID/src" "$<" -o "$@"

libraries/HID/src/HID.cpp.d: ;

-include libraries/HID/src/HID.cpp.d

