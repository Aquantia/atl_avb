# SPDX-License-Identifier: GPL-2.0-only
# Atlantic Network Driver
#
# Copyright (C) 2014-2019 aQuantia Corporation
# Copyright (C) 2019-2020 Marvell International Ltd.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
################################################################################

#
# Makefile for the AQtion(tm) Ethernet driver specified for TSN
#


TARGET := atl_tsn

ifndef CC
CC = gcc
endif

export DEBIAN=`/usr/bin/dpkg --search /usr/bin/dpkg >/dev/null 2>&1 && echo 1 || echo 0`

export KO_EXISTS=`cat /etc/modules 2>/dev/null | grep atlantic && echo 1 || echo 0`

CFLAGS_EXT:=-DPCI_DEBUG -DPTM_SUPPORT -DTSN_SUPPORT
CFLAGS_GCC+=-Wall -ggdb $(CFLAGS_EXT)
# -DCONFIG_CRC_ITU_T

ifeq "$(CC)" "gcc"
	ccflags-y := $(CFLAGS_GCC)
endif

ifeq "$(CC)" "cc"
	ccflags-y := $(CFLAGS_GCC)
endif

ifeq "$(CC)" "clang-3.5"
	ccflags-y := -Wno-date-time -W --system-header-prefix=linux/ \
	-fsanitize=integer,address,undefined,alignment,bool,bounds,null,\
	enum,integer-divide-by-zero,shift,unreachable,unsigned-integer-overflow
endif

ccflags-y:=$(CFLAGS_EXT) -I$(src)

ifneq ($(KERNELRELEASE),)
	$(TARGET)-objs:=aq_main.o aq_nic.o aq_pci_func.o aq_nic.o aq_vec.o aq_ring.o aq_ptp.o aq_filters.o \
	aq_hw_utils.o aq_ethtool.o aq_drvinfo.o \
	aq_trace.o \
	aq_phy.o \
	aq_compat.o \
	aq_tsn.o \
	aq_sysfs.o \
	hw_atl/hw_atl_a0.o \
	hw_atl/hw_atl_b0.o \
	hw_atl/hw_atl_utils.o \
	hw_atl/hw_atl_utils_fw2x.o \
	hw_atl/hw_atl_llh.o \
	hw_atl/hw_atl_fw_hostboot.o \
	hw_atl/hw_atl_fw_image.o \
	hw_atl2/hw_atl2_llh.o \
	hw_atl2/hw_atl2.o \
	hw_atl2/hw_atl2_utils.o \
	hw_atl2/hw_atl2_utils_fw.o \
	macsec/macsec_api.o \

	$(TARGET)-$(CONFIG_MACSEC) += aq_macsec.o

	obj-m:=$(TARGET).o
else
	ifndef KDIR
		BUILD_DIR:=/lib/modules/$(shell uname -r)/build
	else
		BUILD_DIR:=$(KDIR)
	endif

	PWD:=$(shell pwd)


all:
	$(MAKE) -j4 -C $(BUILD_DIR) M=$(PWD) modules

dox:	.doxygen
	@doxygen $<

clean:
	$(MAKE) -j4 -C $(BUILD_DIR) M=$(PWD) clean
	@-rm -rdf doc/html 2 > /dev/null

load:
	-rmmod $(TARGET)
	-rmmod aqdiag
	-rmmod atlantic
	modprobe ptp
	modprobe crc_itu_t
	modprobe -q macsec || true
	insmod ./$(TARGET).ko

unload:
	rmmod ./$(TARGET).ko

install:
	@install -D -m 644 ${TARGET}.ko /lib/modules/$(shell uname -r)/updates/drivers/net/ethernet/aquantia/atlantic/${TARGET}.ko
	@depmod -a $(shell uname -r)
	@if [ "${DEBIAN}" = "1" ]; then \
		update-initramfs -u ; \
		if [ "${KO_EXISTS}" = "0" ]; then echo atlantic >> /etc/modules ; fi; \
	else \
		dracut --force ; \
	fi
	
uninstall:
	@if [ "${KO_EXISTS}" != "0" ]; then sed -in '/$TARGET/d' /etc/modules ; fi
	@rm -f /lib/modules/$(shell uname -r)/updates/drivers/net/ethernet/aquantia/atlantic/${TARGET}.ko
	@depmod -a $(shell uname -r)

endif

