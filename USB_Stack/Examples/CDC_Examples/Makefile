# Makefile for the USB-Stack CDC examples.
#
# Mirrors ../MIDI_Examples/Makefile: portable across XC8 v2, XC8 v1 and SDCC,
# and supports CODEOFFSET for building against the USB uC bootloader. Unlike
# the MIDI examples, these use the stack's CDC-ACM class driver
# (usb_cdc_acm.c) plus the shared usb_app.c/usb_descriptors.c pair - every
# example here is class driver + shared files + example-specific sources,
# not a single self-contained translation unit.
#
# ---------------------------------------------------------------------------
# USAGE
# ---------------------------------------------------------------------------
#
#   make                             both examples, XC8 v2, PIC18F25K50
#   make COMPILER=xc8v1              XC8 v1.4x
#   make COMPILER=sdcc               SDCC pic16 port
#   make DEVICE=18F47J53             a different target
#   make CDC_Serial_Example          just one example
#   make CODEOFFSET=0x2000           build for the USB uC bootloader
#   make clean
#
# ---------------------------------------------------------------------------

COMPILER ?= xc8
DEVICE   ?= 18F25K50

EXAMPLES     := CDC_Serial_Example CDC_Serial_UART_Example
STACK_DIR    := ../../USB
CORE_SRC     := $(STACK_DIR)/usb.c $(STACK_DIR)/usb_cdc_acm.c

# Each example's main.c/Shared_Files use explicit "../../../..." relative
# includes anchored on their own file location, not -I search paths, so no
# INCLUDES variable is needed here (matches how MPLAB X built them).
SRC_CDC_Serial_Example      := CDC_Serial_Example.X/main.c CDC_Serial_Example.X/morse.c \
                                Shared_Files/usb_app.c Shared_Files/usb_descriptors.c
SRC_CDC_Serial_UART_Example := CDC_Serial_UART_Example.X/main.c \
                                CDC_Serial_UART_Example.X/UART.c \
                                Shared_Files/usb_app.c Shared_Files/usb_descriptors.c

TCTAG    ?= $(COMPILER)
BUILD_DIR := build/$(TCTAG)/$(DEVICE)

DEVICE_LC := $(shell echo $(DEVICE) | tr 'A-Z' 'a-z')

# ---------------------------------------------------------------------------
# Toolchains
# ---------------------------------------------------------------------------

which = $(shell sh -c 'command -v $(1) 2>/dev/null')

XC8     ?= $(firstword $(call which,xc8-cc) $(lastword $(wildcard /opt/microchip/xc8/v2.*/bin/xc8-cc)))
XC8V1   ?= $(firstword $(call which,xc8)    $(lastword $(wildcard /opt/microchip/xc8/v1.*/bin/xc8)))
SDCC    ?= $(firstword $(call which,sdcc) sdcc)

ifeq ($(origin DFP), undefined)
DFP := $(patsubst %/pic/dat/ini/$(DEVICE_LC).ini,%,$(firstword \
         $(wildcard /opt/microchip/mplabx/*/packs/Microchip/PIC18F*_DFP/*/xc8/pic/dat/ini/$(DEVICE_LC).ini) \
         $(wildcard $(HOME)/.mchp_packs/Microchip/PIC18F*_DFP/*/xc8/pic/dat/ini/$(DEVICE_LC).ini)))
endif
MDFP := $(if $(DFP),-mdfp=$(DFP),)

ifdef CODEOFFSET
XC8_OFFSET   := -mcodeoffset=$(CODEOFFSET)
XC8V1_OFFSET := --codeoffset=$(CODEOFFSET)
endif

# ---------------------------------------------------------------------------
# Per-compiler command lines
# ---------------------------------------------------------------------------
# $(1) = example name

ifeq ($(COMPILER),xc8)
define BUILD_CMD
$(XC8) -mcpu=$(DEVICE) $(MDFP) -std=c90 -O2 $(XC8_OFFSET) \
	-o $(BUILD_DIR)/$(1).hex $(SRC_$(1)) $(CORE_SRC)
endef

else ifeq ($(COMPILER),xc8v1)
define BUILD_CMD
$(XC8V1) --chip=$(DEVICE) --outdir=$(BUILD_DIR) $(XC8V1_OFFSET) \
	$(SRC_$(1)) $(CORE_SRC)
endef

else ifeq ($(COMPILER),sdcc)
# SDCC compiles one source per invocation, so each example is compiled
# object-by-object then linked. Built via make functions (not a shell loop)
# because a shell loop's $$var references get double-expanded once through
# the $(eval $(call EXAMPLE_RULE,...)) below, corrupting the variable names.
SDCCFLAGS = -mpic16 -p$(DEVICE_LC) --use-non-free
objname = $(BUILD_DIR)/$(notdir $(basename $(1))).o
define BUILD_CMD
$(foreach s,$(SRC_$(1)) $(CORE_SRC),$(SDCC) $(SDCCFLAGS) -c -o $(call objname,$(s)) $(s) &&) \
$(SDCC) $(SDCCFLAGS) -o $(BUILD_DIR)/$(1).hex $(foreach s,$(SRC_$(1)) $(CORE_SRC),$(call objname,$(s)))
endef

else
$(error Unknown COMPILER "$(COMPILER)". Use xc8, xc8v1 or sdcc)
endif

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------

.PHONY: all clean list $(EXAMPLES)

all: $(EXAMPLES)

define EXAMPLE_RULE
$(1):
	@mkdir -p $(BUILD_DIR)
	$(BUILD_CMD)
endef
$(foreach e,$(EXAMPLES),$(eval $(call EXAMPLE_RULE,$(e))))

list:
	@echo "COMPILER          $(COMPILER)"
	@echo "DEVICE            $(DEVICE)"
	@echo "BUILD_DIR         $(BUILD_DIR)"
	@echo "XC8 (v2)          $(XC8)$(if $(wildcard $(XC8)),, [not found])"
	@echo "XC8V1             $(XC8V1)$(if $(wildcard $(XC8V1)),, [not found])"
	@echo "SDCC              $(SDCC)$(if $(call which,$(SDCC)),, [not found])"
	@echo "DFP               $(if $(DFP),$(DFP),<none found - assuming built-in support>)"

clean:
	rm -rf build
