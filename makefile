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
           -I $(TOP)/inc  \
		   -I $(TOP)/libopencm3/include

CFLAGS +=  -W -Wall -mcpu=cortex-m4 -mthumb
CFLAGS +=  -ffunction-sections -fdata-sections
CFLAGS +=  -D GD32F427 -D USE_STDPERIPH_DRIVER -D STM32F4
CFLAGS +=   $(INC_FLAGS) -Os -g -std=gnu11

ASMFLAGS = -mthumb -mcpu=cortex-m4 -g -Wa,--warn 
LDFLAGS += -mthumb -mcpu=cortex-m4
LDFLAGS += -Wl,--start-group -lc -lm -Wl,--end-group -specs=nosys.specs -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=Project.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x80

LD_PATH = -T $(TOP)/ldscripts/gd32f425_427_xK_flash.ld

# OPENCM3_DIR ?= $(realpath libopencm3)
# lib:
# 	$(Q)if [ ! "`ls -A $(OPENCM3_DIR)`" ] ; then \
# 		printf "######## ERROR ########\n"; \
# 		printf "\tlibopencm3 is not initialized.\n"; \
# 		printf "\tPlease run:\n"; \
# 		printf "\t$$ git submodule init\n"; \
# 		printf "\t$$ git submodule update\n"; \
# 		printf "\tbefore running make.\n"; \
# 		printf "######## ERROR ########\n"; \
# 		exit 1; \
# 		fi
# 	$(Q)$(MAKE) -C $(OPENCM3_DIR)

C_SRC=$(shell find ./src -name '*.c')  
C_SRC+=$(shell find ./gd_libs -name '*.c')  
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
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_f207.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_standard.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_control.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_hid.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_msc.c
C_SRC+=$(TOP)/libopencm3/lib/usb/usb_dwc_common.c

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

 


 










