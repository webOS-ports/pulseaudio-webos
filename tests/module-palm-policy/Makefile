# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
# SPDX-License-Identifier: LGPL-2.1-or-later
#
# Unit tests for module-palm-policy, built the way the LuneOS recipe builds the
# module itself (meta-luneos pulseaudio-module-palm-policy's Makefile): against
# an upstream PulseAudio 17 sysroot, with its pulsecore/config.h forced in.
#
#   make -C tests/module-palm-policy CC=<cross gcc> PKG_CONFIG=<pkg-config>
#
# produces palm-policy-tests for the target; run it on the device, or on the
# build host through the x86-64 recipe sysroot's loader (see README.md).

PKG_CONFIG ?= pkg-config
PA_MAJORMINOR ?= $(shell $(PKG_CONFIG) --modversion libpulse | cut -d. -f1,2)
PKGS := pulsecore libpulse alsa
MODULES := ../../src/modules

ALL_CFLAGS := -std=gnu11 -D_GNU_SOURCE -include pulsecore/config.h \
              -DPULSEAUDIO_VERSION=\"$(PA_MAJORMINOR)\" -I$(MODULES) \
              $(shell $(PKG_CONFIG) --cflags $(PKGS)) -g $(CFLAGS)
LIBS := $(shell $(PKG_CONFIG) --libs $(PKGS))

palm-policy-tests: unit/check.c unit/test_palm_policy.c $(MODULES)/module-palm-policy-util.c \
                   $(MODULES)/module-palm-policy.c
	$(CC) $(ALL_CFLAGS) -o $@ unit/check.c unit/test_palm_policy.c $(MODULES)/module-palm-policy-util.c \
	    $(LDFLAGS) $(LIBS)

clean:
	rm -f palm-policy-tests

.PHONY: clean
