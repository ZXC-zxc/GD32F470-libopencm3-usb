#工程的名称及最后生成文件的名字
TARGET = gd32f4xx_libopencm3usbxxx

#设定临时性环境变量
export CC             = arm-none-eabi-gcc           
export AS             = arm-none-eabi-as
export LD             = arm-none-eabi-ld
export OBJCOPY        = arm-none-eabi-objcopy

#读取当前工作目录
TOP=$(shell pwd)

#设定包含文件目录
INC_FLAGS= -I $(TOP)/libopencm3/lib/gd32/f4xx/GD32F4xx_standard_peripheral/Include  \
		-I $(TOP)/libopencm3/lib/gd32/f4xx/CMSIS  \
		-I $(TOP)/libopencm3/lib/gd32/f4xx/CMSIS/GD/GD32F4xx/Include  \
		-I $(TOP)/bixin_usb/common \
		-I $(TOP)/bixin_usb/firmware  \
		-I $(TOP)/bixin_i2c/mi2c  \
		-I $(TOP)/bixin_i2c/si2c  \
		-I $(TOP)/bixin_layout  \
		-I $(TOP)/bixin_usart  \
		-I $(TOP)/inc  \
		-I $(TOP)/libopencm3/include \

#    -I $(TOP)/bixin_usb/firmware  bootlader

CFLAGS +=  -W -Wall -mcpu=cortex-m4 -mthumb
CFLAGS +=  -ffunction-sections -fdata-sections
CFLAGS +=  -D GD32F470 -D USE_STDPERIPH_DRIVER -D STM32F4 -D USB_FS_CORE -D USE_USB_FS
CFLAGS +=  -D U2F_ENABLED
CFLAGS +=   $(INC_FLAGS) -Os -g -std=gnu11

ASMFLAGS = -mthumb -mcpu=cortex-m4 -g -Wa,--warn 
LDFLAGS += -mthumb -mcpu=cortex-m4
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=Project.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80
LDFLAGS += -L$(TOP)/libopencm3/lib  -lopencm3_gd32f4xx


LD_PATH = -T $(TOP)/ldscripts/gd32f470xK_flash.ld

C_SRC=$(shell find ./src -name '*.c')  

# usb related
C_SRC+=$(TOP)/libopencm3/lib/usb/usb.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_f107.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_gd32f470.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_standard.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_control.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_hid.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_msc.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_dwc_common.c
# bixin usb related
C_SRC+=$(TOP)/bixin_usb/common/rng.c
C_SRC+=$(TOP)/bixin_usb/common/random_delays.c
# C_SRC+=$(TOP)/bixin_usb/common/usb_standard.c
C_SRC+=$(TOP)/bixin_usb/common/util.c
C_SRC+=$(TOP)/bixin_usb/common/usb21_standard.c
C_SRC+=$(TOP)/bixin_usb/common/webusb.c
C_SRC+=$(TOP)/bixin_usb/common/winusb.c
# C_SRC+=$(TOP)/bixin_usb/bootloader/usb.c
C_SRC+=$(TOP)/bixin_usb/firmware/usb.c
# C_SRC+=$(TOP)/bixin_usb/firmware/u2f.c
C_SRC+=$(TOP)/bixin_layout/oled.c
C_SRC+=$(TOP)/bixin_layout/memzero.c
# C_SRC+=$(TOP)/bixin_layout/timer.c
C_SRC+=$(TOP)/bixin_i2c/mi2c/mi2c.c
C_SRC+=$(TOP)/bixin_i2c/si2c/si2c.c
C_SRC+=$(TOP)/bixin_usart/usart.c

C_OBJ=$(C_SRC:%.c=%.o)          

# ASM_SRC=$(shell find ./ -name '*.s')  
ASM_SRC = $(TOP)/libopencm3/lib/gd32/f4xx/CMSIS/GD/GD32F4xx/Source/GCC/startup_gd32f450_470.S
ASM_OBJ=$(ASM_SRC:%.s=%.o)  

.PHONY: all clean update      

all:$(C_OBJ) $(ASM_OBJ)
	echo $(C_SRC)
	$(CC) $(C_OBJ) $(ASM_OBJ) $(LD_PATH) -o $(TARGET).elf $(LDFLAGS) 
	$(OBJCOPY) $(TARGET).elf  $(TARGET).bin -Obinary 
	$(OBJCOPY) $(TARGET).elf  $(TARGET).hex -Oihex

$(C_OBJ):%.o:%.c
	$(CC) -c $(CFLAGS) -o $@ $<
$(ASM_OBJ):%.o:%.s
	$(CC) -c $(ASMFLAGS) -o $@ $<

clean:
	rm -f $(shell find ./ -name '*.o')
	rm -f $(shell find ./ -name '*.d')
	rm -f $(shell find ./ -name '*.map')
	rm -f $(shell find ./ -name '*.elf')
	rm -f $(shell find ./ -name '*.bin')
	rm -f $(shell find ./ -name '*.hex')

 


 










