# Makefile for the USB-Stack MIDI examples.
#
# Builds MIDI_Controller and MIDI_Synth. Each example is a single translation
# unit that replaces main.c/usb_app.c/usb_descriptors.c, so a build is just
# the example plus the stack core (usb.c) - no class driver is involved.
#
# ---------------------------------------------------------------------------
# USAGE
# ---------------------------------------------------------------------------
#
#   make                             both examples, XC8 v2, PIC18F25K50
#   make COMPILER=xc8v1              XC8 v1.4x - does not work, see matrix note
#   make COMPILER=sdcc               SDCC pic16 port
#   make DEVICE=18F47J53             a different target
#   make MIDI_Synth                  just one example
#   make matrix                      every compiler x every device, reported
#   make list                        show what was detected on this machine
#   make clean
#
# COMPILER  xc8 (default) | xc8v1 | sdcc
#
# Verified: XC8 v2.31/v2.36/v4.00 and SDCC 4.0.0/4.1.0/4.3.0rc1, each against
# PIC18F25K50, PIC18F14K50 and PIC18F47J53. All six emit identical config
# words. `make matrix` re-runs the whole sweep.
# DEVICE    18F25K50 (default), or any PIC18 part the stack supports
# CODEOFFSET  set to 0x2000 to build for the USB uC bootloader
#
# Everything below can be overridden on the command line, e.g.
#
#   make XC8=/opt/microchip/xc8/v2.36/bin/xc8-cc
#   make DFP=/path/to/PIC18F-K_DFP/1.3.84/xc8
#   make MATRIX_DEVICES="18F25K50 18F14K50"
#
# ---------------------------------------------------------------------------

COMPILER ?= xc8
DEVICE   ?= 18F25K50

EXAMPLES  := MIDI_Controller MIDI_Synth
STACK_DIR := ../../USB
HW_DIR    := ../../Hardware
CORE_SRC  := $(STACK_DIR)/usb.c
INCLUDES  := -I$(STACK_DIR) -I$(HW_DIR)

# usb_config.h keys the endpoint/interface counts off these.
DEFINE_MIDI_Controller := MIDI_CONTROLLER_EXAMPLE
DEFINE_MIDI_Synth      := MIDI_SYNTH_EXAMPLE

# TCTAG separates output per toolchain, so a matrix sweep does not leave one
# compiler's hex sitting where another's was.
TCTAG    ?= $(COMPILER)
BUILD_DIR := build/$(TCTAG)/$(DEVICE)

# Lower case device name, used by SDCC and by the DFP probe.
DEVICE_LC := $(shell echo $(DEVICE) | tr 'A-Z' 'a-z')

# ---------------------------------------------------------------------------
# Toolchains
# ---------------------------------------------------------------------------

# Prefer whatever is on PATH, then fall back to the newest install under
# /opt/microchip. `which` is wrapped in sh because make execs simple
# $(shell ...) commands directly, where shell builtins are not available.
which = $(shell sh -c 'command -v $(1) 2>/dev/null')

XC8     ?= $(firstword $(call which,xc8-cc) $(lastword $(wildcard /opt/microchip/xc8/v2.*/bin/xc8-cc)))
XC8V1   ?= $(firstword $(call which,xc8)    $(lastword $(wildcard /opt/microchip/xc8/v1.*/bin/xc8)))
SDCC    ?= $(firstword $(call which,sdcc) sdcc)

# XC8 2.4x and later moved the older PIC18 parts out of the compiler and into
# device family packs. Probe the installed packs for one that knows this
# device; if none is found we leave -mdfp off, which is correct for XC8
# versions that still have built-in support.
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
	-D$(DEFINE_$(1)) $(INCLUDES) \
	-o $(BUILD_DIR)/$(1).hex $(1).c $(CORE_SRC)
endef

else ifeq ($(COMPILER),xc8v1)
define BUILD_CMD
$(XC8V1) --chip=$(DEVICE) --outdir=$(BUILD_DIR) $(XC8V1_OFFSET) \
	-D$(DEFINE_$(1)) $(INCLUDES) \
	$(1).c $(CORE_SRC)
endef

else ifeq ($(COMPILER),sdcc)
# SDCC compiles one source per invocation ("cannot compile more than one
# source file"), so each example is built as compile-each-then-link.
SDCCFLAGS = -mpic16 -p$(DEVICE_LC) --use-non-free $(INCLUDES)
define BUILD_CMD
$(SDCC) $(SDCCFLAGS) -D$(DEFINE_$(1)) -c -o $(BUILD_DIR)/$(1).o $(1).c && \
$(SDCC) $(SDCCFLAGS) -D$(DEFINE_$(1)) -c -o $(BUILD_DIR)/usb_$(1).o $(CORE_SRC) && \
$(SDCC) $(SDCCFLAGS) -o $(BUILD_DIR)/$(1).hex \
	$(BUILD_DIR)/$(1).o $(BUILD_DIR)/usb_$(1).o
endef

else
$(error Unknown COMPILER "$(COMPILER)". Use xc8, xc8v1 or sdcc)
endif

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------

.PHONY: all clean list matrix help $(EXAMPLES)

all: $(EXAMPLES)

# XC8 leaves a pile of intermediates next to the output; keep them in
# BUILD_DIR by building from there where the compiler insists on cwd.
define EXAMPLE_RULE
$(1):
	@mkdir -p $(BUILD_DIR)
	$(BUILD_CMD)
endef
$(foreach e,$(EXAMPLES),$(eval $(call EXAMPLE_RULE,$(e))))

# ---------------------------------------------------------------------------
# matrix - every installed toolchain x every device x both examples
# ---------------------------------------------------------------------------
#
# Toolchains are discovered rather than assumed, so this reports on whatever
# is installed on the machine. Override any of the three lists to narrow it:
#
#   make matrix MATRIX_DEVICES=18F25K50
#   make matrix MATRIX_XC8S=/opt/microchip/xc8/v2.36/bin/xc8-cc
#   make matrix MATRIX_SDCCS=
#   make matrix MATRIX_EXCLUDE=          (sweep the skipped toolchains too)
#
# Known results on the reference machine:
#   XC8 v2.31, v2.36, v4.00      all pass
#   SDCC 4.0.0, 4.1.0, 4.3.0rc1  all pass
#   XC8 v1.43, v1.45             fail, and cannot work - v1's __at() accepts
#                                only a literal address, while every address
#                                in this stack is computed from EP0_SIZE and
#                                PINGPONG_MODE. Pre-existing: the shipped
#                                HID_Mouse_Example fails on v1 the same way.
#   XC8 v2.46                    fails, but for an environment reason: that
#                                install's bin/xclm is a 19KB stub where the
#                                genuine licence manager is ~2.4MB, and the
#                                compiler's own integrity check rejects it
#                                with "(2057) ... installation appears to be
#                                corrupted". Nothing to do with these sources
#                                - reinstalling XC8 v2.46 fixes it.
MATRIX_DEVICES ?= 18F25K50 18F14K50 18F47J53

# Toolchain directory names skipped by the default sweep, so that a red
# matrix means a real problem with the code. Clear it to sweep everything.
MATRIX_EXCLUDE ?= v2.46

# Map a toolchain binary to the install directory name the sweep labels it
# with (/opt/microchip/xc8/v2.36/bin/xc8-cc -> v2.36), then drop the excluded.
tc_name  = $(notdir $(patsubst %/bin/,%,$(dir $(1))))
tc_keep  = $(foreach t,$(1),$(if $(filter $(MATRIX_EXCLUDE),$(call tc_name,$(t))),,$(t)))

MATRIX_XC8S_ALL  ?= $(sort $(wildcard /opt/microchip/xc8/v[2-9].*/bin/xc8-cc)) \
                    $(filter-out $(wildcard /opt/microchip/xc8/*/bin/xc8-cc),$(call which,xc8-cc))
MATRIX_SDCCS_ALL ?= $(sort $(wildcard /opt/sdcc*/bin/sdcc)) \
                    $(filter-out $(wildcard /opt/sdcc*/bin/sdcc),$(call which,sdcc))

MATRIX_XC8S    ?= $(call tc_keep,$(MATRIX_XC8S_ALL))
MATRIX_SDCCS   ?= $(call tc_keep,$(MATRIX_SDCCS_ALL))

# $(1)=COMPILER $(2)=variable to set $(3)=toolchain path
define MATRIX_RUN
	@for d in $(MATRIX_DEVICES); do \
	  for e in $(EXAMPLES); do \
	    printf '%-34s %-10s %-16s ' "$$(echo $(3) | sed 's|/bin/.*||; s|.*/||')" "$$d" "$$e"; \
	    tag=$$(echo $(3) | sed 's|/bin/.*||; s|.*/||'); \
	    if $(MAKE) --no-print-directory COMPILER=$(1) $(2)=$(3) TCTAG=$$tag DEVICE=$$d $$e \
	         > $(BUILD_ROOT)/matrix.log 2>&1 \
	       && [ -s build/$$tag/$$d/$$e.hex ]; then \
	      echo "ok"; \
	    else \
	      echo "FAIL"; \
	      grep -oE '\([0-9]{3,4}\)[^|]*|error[^|]*' $(BUILD_ROOT)/matrix.log \
	        | head -2 | sed 's/^/    /'; \
	      echo fail >> $(BUILD_ROOT)/matrix.fail; \
	    fi; \
	  done; \
	done

endef

BUILD_ROOT := build

matrix:
	@mkdir -p $(BUILD_ROOT); rm -f $(BUILD_ROOT)/matrix.fail
	@echo "--- XC8 ---"
	$(foreach t,$(MATRIX_XC8S),$(call MATRIX_RUN,xc8,XC8,$(t)))
	@echo "--- SDCC ---"
	$(foreach t,$(MATRIX_SDCCS),$(call MATRIX_RUN,sdcc,SDCC,$(t)))
	@if [ -f $(BUILD_ROOT)/matrix.fail ]; then echo; echo "matrix: failures above"; exit 1; \
	 else echo; echo "matrix: all combinations ok"; fi

list:
	@echo "COMPILER          $(COMPILER)"
	@echo "DEVICE            $(DEVICE)"
	@echo "BUILD_DIR         $(BUILD_DIR)"
	@echo "XC8 (v2)          $(XC8)$(if $(wildcard $(XC8)),, [not found])"
	@echo "XC8V1             $(XC8V1)$(if $(wildcard $(XC8V1)),, [not found])"
	@echo "SDCC              $(SDCC)$(if $(call which,$(SDCC)),, [not found])"
	@echo "DFP               $(if $(DFP),$(DFP),<none found - assuming built-in support>)"
	@echo "matrix devices    $(MATRIX_DEVICES)"
	@echo "matrix XC8s"
	@for t in $(MATRIX_XC8S); do echo "                  $$t"; done
	@echo "matrix SDCCs"
	@for t in $(MATRIX_SDCCS); do echo "                  $$t"; done
	@echo "matrix excluded   $(if $(MATRIX_EXCLUDE),$(MATRIX_EXCLUDE),<none>)"
	@for t in $(filter-out $(MATRIX_XC8S) $(MATRIX_SDCCS),$(MATRIX_XC8S_ALL) $(MATRIX_SDCCS_ALL)); do \
	    echo "                  $$t"; done

clean:
	rm -rf build

help:
	@sed -n '3,30p' Makefile | sed 's/^# \{0,1\}//'
