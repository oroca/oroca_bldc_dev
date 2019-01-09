##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -O2 -ggdb -fomit-frame-pointer -falign-functions=16 -std=gnu99
  USE_OPT += -DBOARD_OTG_NOVBUSSENS
  USE_OPT += -fsingle-precision-constant -Wdouble-promotion
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT = 
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti -fno-exceptions
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT = 
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = no
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Enable this if you want to see the full log while compiling.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = yes
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = no
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 0x400
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU on Cortex-M4 (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

# Enable this if you really want to use the STM FWLib.
ifeq ($(USE_FWLIB),)
  USE_FWLIB = yes
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

CONFSRC  = src
CORESRC = src/core
MCSRC = src/mc
APPSRC = src/app

MAVSRC  =src/mavlink
#MAVSRC += src/mavlink/common
MAVSRC += src/mavlink/oroca_bldc

# Define project name here
PROJECT = oroca_bldc

# Imported source files and paths
CHIBIOS = src/ChibiOS_3.0.2

# Startup files.
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/startup_stm32f4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32F4xx/platform.mk
include $(CHIBIOS)/os/hal/boards/ST_STM32F4_DISCOVERY/board.mk
include $(CHIBIOS)/os/hal/osal/rt/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/rt/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk

# Define linker script file here
LDSCRIPT= ld_eeprom_emu.ld
#LDSCRIPT= $(PORTLD)/STM32F407xG.ld
#LDSCRIPT= $(PORTLD)/STM32F407xG_CCM.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(STARTUPSRC) \
		$(KERNSRC) \
		$(PORTSRC) \
		$(OSALSRC) \
		$(HALSRC) \
		$(PLATFORMSRC) \
		$(BOARDSRC) \
		$(CHIBIOS)/os/various/syscalls.c \
		$(CHIBIOS)/os/hal/lib/streams/memstreams.c \
		$(CHIBIOS)/os/hal/lib/streams/chprintf.c \
			$(CONFSRC)/hw_oroca.c\
			$(CONFSRC)/conf_general.c\
				$(MCSRC)/mc_pwm.c \
				$(MCSRC)/mc_interface.c \
				$(MCSRC)/mc_control.c \
				$(MCSRC)/mc_sensor.c \
				$(MCSRC)/mc_encoder.c \
					$(CORESRC)/mavlink_proc.c \
					$(CORESRC)/comm_can.c \
					$(CORESRC)/comm_usb.c \
					$(CORESRC)/comm_usb_serial.c \
					 $(CORESRC)/servo_dec.c \
					 $(CORESRC)/servo_simple.c \
					 $(CORESRC)/servo.c \
					  $(CORESRC)/ledpwm.c \
					  $(CORESRC)/led_external.c\
					  $(CORESRC)/ws2811.c \
					   $(CORESRC)/eeprom.c \
					   $(CORESRC)/flash_helper.c \
					$(CORESRC)/irq_handlers.c \
					$(CORESRC)/utils.c	\
					$(CORESRC)/timeout.c	\
					$(CORESRC)/buffer.c	\
					$(CORESRC)/crc.c \
				$(CORESRC)/spi_sw.c \
				$(CORESRC)/rf.c \
				$(CORESRC)/rfhelp.c \
				$(CORESRC)/nrf_driver.c\
		$(APPSRC)/app.c \
		$(APPSRC)/app_ppm.c \
		$(APPSRC)/app_adc.c \
		$(APPSRC)/app_sten.c \
		$(APPSRC)/app_uartcomm.c \
		$(APPSRC)/app_nunchuk.c\
		usart1_print.c\
       main.c

       

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC =


# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(STARTUPASM) $(PORTASM) $(OSALASM)

INCDIR = $(STARTUPINC) $(KERNINC) $(PORTINC) $(OSALINC) \
         $(HALINC) $(PLATFORMINC) $(BOARDINC) \
         $(CHIBIOS)/os/various \
         $(CHIBIOS)/os/hal/lib/ports/STM32/LLD/TIMv1 \
         $(CHIBIOS)/os/hal/lib/streams \
		 $(CONFSRC)\
         $(HWSRC)\
         $(CORESRC)\
         $(MCSRC)\
         $(APPSRC)\
         $(MAVSRC)
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
#TRGT = ~/sat/bin/arm-none-eabi-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
#LD   = $(TRGT)gcc
LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
OD   = $(TRGT)objdump
SIZE   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra

#
# Compiler settings
##############################################################################

##############################################################################
# Start of default section
#

# List all default C defines here, like -D_DEBUG=1
#jsyoon test
DDEFS =
#DDEFS = -D_TEST_STM32F4_DISCOVERY

# List all default ASM defines here, like -D_DEBUG=1
DADEFS =

# List all default directories to look for include files here
DINCDIR =

# List the default directory to look for the libraries here
DLIBDIR =

# List all default libraries here
DLIBS =

#
# End of default section
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm

#
# End of user defines
##############################################################################

ifeq ($(USE_FWLIB),yes)
  include $(CHIBIOS)/ext/stdperiph_stm32f4/stm32lib.mk
  CSRC += $(STM32SRC)
  INCDIR += $(STM32INC)
  USE_OPT += -DUSE_STDPERIPH_DRIVER
endif



build/$(PROJECT).bin: build/$(PROJECT).elf show_size
	$(BIN) build/$(PROJECT).elf build/$(PROJECT).bin

	
# Print size
show_size:
	$(SIZE) build/$(PROJECT).elf

	
# Program
upload: build/$(PROJECT).bin
	#qstlink2 --cli --erase --write build/$(PROJECT).bin
	openocd -f interface/stlink-v2.cfg -c "set WORKAREASIZE 0x2000" -f target/stm32f4x_stlink.cfg -c "program build/$(PROJECT).elf verify reset"
	#openocd -f board/stm32f4discovery.cfg -c "reset_config trst_only combined" -c "program build/$(PROJECT).elf verify reset" # For openocd 0.9

debug-start:
	openocd -f stm32-bv_openocd.cfg


RULESPATH = $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC
include $(RULESPATH)/rules.mk