##############################################################################################
# Makefile for STM32F030x6 with CMSIS directory structure extracated from STM32CubeIDE
# project.
#
# Mike Shegedin, 2023
# Modified by mztulip, 2024
##############################################################################################

TARGET    = output
SOURCE    = main
MCPU      = cortex-m0
STARTUP   = startup_stm32f030x6
LOADER    = STM32F030X6_FLASH.ld

CC = /home/tulip/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc
OBJCOPY = /home/tulip/gcc-arm-none-eabi-10.3-2021.10-x86_64-linux/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-objcopy

CFLAGS = -mcpu=$(MCPU) -g0 --specs=nano.specs -Os -mthumb -mfloat-abi=soft -Wall

INCLUDE1 =CMSIS/Device/ST/STM32F0xx/Include
INCLUDE2 =CMSIS/Include

$(TARGET).elf: $(SOURCE).o $(STARTUP).o $(LOADER) Makefile
	$(CC) -o $@ $(SOURCE).o $(STARTUP).o -mcpu=$(MCPU) --specs=nosys.specs -T"$(LOADER)" \
	-Wl,-Map=$(TARGET).map -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb \
	-Wl,--start-group -lc -lm -Wl,--end-group
	arm-none-eabi-size $(TARGET).elf

$(STARTUP).o: $(STARTUP).s Makefile
	$(CC) $(CFLAGS) -DDEBUG -c -x assembler-with-cpp -o $@ $<

$(SOURCE).o: $(SOURCE).c Makefile
	$(CC) $< $(CFLAGS) -I$(INCLUDE1) -I$(INCLUDE2) -std=gnu11 -DDEBUG  \
	-c -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -o $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex

.PHONY : all clean
all : $(TARGET).bin

clean:
	rm *.o *.elf *.map *.su *.bin *.hex -f
