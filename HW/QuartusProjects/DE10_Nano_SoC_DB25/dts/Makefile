all: socfpga_cyclone5_de10_nano_machinekit.dtb

ARCH ?= arm
KERNEL_SRC_DIR := "/home/src/de10-cnc/kernel/linux-4.13.10"
DTC := dtc
DTC_FLAGS :=

ifeq (,$(CROSS_COMPILE))
CPP := gcc
else
CPP := $(CROSS_COMPILE)gcc
endif

# snatched from kernel/scripts/Makefile.{include,lib}
comma := ,
dtc-tmp = $(subst $(comma),_,$(dot-target).dts.tmp)
dot-target = $(dir $@).$(notdir $@)
depfile = $(subst $(comma),_,$(dot-target).d)
dtc_cpp_flags  = -E -Wp,-MD,$(depfile).pre.tmp -nostdinc \
	-Iinclude -undef -D__DTS__

cmd_dtc = $(CPP) $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ; \
	$(DTC) -O dtb -o $@ -b 0 \
		-i $(dir $<) $(DTC_FLAGS) \
		-d $(depfile).dtc.tmp $(dtc-tmp) ; \
	cat $(depfile).pre.tmp $(depfile).dtc.tmp > $(depfile)

%.dtb: %.dts
	$(call cmd_dtc)
