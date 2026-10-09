#!/bin/sh
# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
# SPDX-License-Identifier: LGPL-2.1-or-later
#
# Run the unit tests under valgrind memcheck with an OE recipe workdir's x86-64
# sysroot, so the exact libraries from the image are what gets exercised:
#
#   tests/module-palm-policy/hardening/run-valgrind.sh <palm-policy-tests> <recipe-sysroot>
#
# Fails on any memory error or definite leak.

set -eu
BIN="${1:?unit test binary}"
SYSROOT="${2:?recipe sysroot}"

exec valgrind \
    --error-exitcode=42 \
    --leak-check=full \
    --show-leak-kinds=definite \
    --errors-for-leak-kinds=definite \
    "$SYSROOT/usr/lib/ld-linux-x86-64.so.2" \
    --library-path "$SYSROOT/usr/lib:$SYSROOT/usr/lib/pulseaudio" \
    "$BIN"
