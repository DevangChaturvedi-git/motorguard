TARGET = motorguard
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CPU = -mcpu=cortex-m3 -mthumb

SRCS = \
  Src/main.c \
  Src/system_stm32f1xx.c \
  Src/drivers.c \
  Src/gpio.c \
  Src/adc_driver.c \
  Src/pwm_driver.c \
  Src/uart_driver.c \
  Src/wwdg_driver.c \
  Src/flash_persist.c \
  Src/channels.c \
  Src/plant_model.c \
  Src/protection.c \
  Src/control.c \
  Src/limiter.c \
  Src/supervisor.c

ASM_SRCS = Startup/startup_stm32f103xb.s

INCLUDES = -IInc -Icmsis

DEFS = -DSTM32F103xB

CFLAGS = $(CPU) $(DEFS) $(INCLUDES) -Wall -Wextra -O2 -g -ffunction-sections -fdata-sections -std=c11
LDFLAGS = $(CPU) -TSTM32F103XB_FLASH.ld -Wl,--gc-sections -Wl,-Map=$(TARGET).map -specs=nano.specs -specs=nosys.specs

OBJS = $(SRCS:.c=.o) $(ASM_SRCS:.s=.o)

all: $(TARGET).elf

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CPU) -c $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) $(OBJS) -o $@
	$(SIZE) $@
	$(OBJCOPY) -O binary $@ $(TARGET).bin
	$(OBJCOPY) -O ihex $@ $(TARGET).hex

clean:
	rm -f $(OBJS) $(TARGET).elf $(TARGET).bin $(TARGET).hex $(TARGET).map

.PHONY: all clean
