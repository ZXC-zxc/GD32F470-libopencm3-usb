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
INC_FLAGS= -I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_standard_peripheral/Include  \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/CMSIS  \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/device/core/Include  \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/driver/Include \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/device/class/msc/Include  \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/ustd/common   \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/device/class/msc/Include  \
		-I $(TOP)/gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/ustd/class/msc \
		-I $(TOP)/bixin_usb/common \
		-I $(TOP)/bixin_usb/firmware  \
		-I $(TOP)/inc  \
		-I $(TOP)/libopencm3/include \

#    -I $(TOP)/bixin_usb/firmware  bootlader

CFLAGS +=  -W -Wall -mcpu=cortex-m4 -mthumb
CFLAGS +=  -ffunction-sections -fdata-sections
CFLAGS +=  -D GD32F427 -D USE_STDPERIPH_DRIVER -D STM32F4 -D USB_FS_CORE -D USE_USB_FS
CFLAGS +=  -D U2F_ENABLED
CFLAGS +=   $(INC_FLAGS) -Os -g -std=gnu11

ASMFLAGS = -mthumb -mcpu=cortex-m4 -g -Wa,--warn 
LDFLAGS += -mthumb -mcpu=cortex-m4
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=Project.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80

LD_PATH = -T $(TOP)/ldscripts/gd32f425_427_xK_flash.ld

C_SRC=$(shell find ./src -name '*.c')  
C_SRC+=$(shell find ./gd_libs/GD32F4xx/Firmware/CMSIS -name '*.c')  
C_SRC+=$(shell find ./gd_libs/GD32F4xx/Firmware/GD32F4xx_standard_peripheral -name '*.c')  
# C_SRC+=$(shell find ./gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/device/core/Source -name '*.c')  
# C_SRC+=$(shell find ./gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/driver/Source -name '*.c')  
# C_SRC+=$(shell find ./gd_libs/GD32F4xx/Firmware/GD32F4xx_usb_library/device/class/msc/Source -name '*.c')  
# libopencm3 related
C_SRC+=$(TOP)/libopencm3/lib/stm32/f4/rcc.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/f4/pwr.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/f4/flash.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/gpio_common_f0234.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/gpio_common_all.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/flash_common_idcache.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/flash_common_all.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/flash_common_f24.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/flash_common_f.c
C_SRC+=$(TOP)/libopencm3/lib/stm32/common/rcc_common_all.c
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

C_OBJ=$(C_SRC:%.c=%.o)          

ASM_SRC=$(shell find ./ -name '*.s')  
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

 


 










