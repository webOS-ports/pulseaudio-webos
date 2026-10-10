#!/bin/sh
# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
# SPDX-License-Identifier: LGPL-2.1-or-later
#
# On a LuneOS device, as root, with this directory, ../integration/client.py and
# the device's palm-policy-tests binary copied next to each other:
#
#   ./run-on-device.sh [fuzz-records]
#
# 1. runs the unit tests against the device's own libpulsecore;
# 2. stops audiod - the module serves one connection - and drives the live
#    module with the protocol cases, a fuzz run and silent streams (/dev/zero),
#    volume going to the pdefaultapp null sink only;
# 3. fails if pulseaudio crashed, restarted, or logged an assertion meanwhile;
# 4. starts audiod again, which re-sends its routing and volumes.

set -u
HERE=$(cd "$(dirname "$0")" && pwd)
FUZZ=${1:-3000}
fail=0

echo "== unit tests"
LD_LIBRARY_PATH=/usr/lib/pulseaudio "$HERE/palm-policy-tests" 2>/dev/null | tail -1
LD_LIBRARY_PATH=/usr/lib/pulseaudio "$HERE/palm-policy-tests" >/dev/null 2>&1 || { echo "FAIL unit tests"; fail=1; }

echo "== live module"
since=$(date '+%Y-%m-%d %H:%M:%S')
pid0=$(systemctl show -p MainPID --value pulseaudio)
restarts0=$(systemctl show -p NRestarts --value pulseaudio)
systemctl stop audiod
sleep 1
export PULSE_SERVER=unix:/run/pulse/native
for p in "media.type=pmedia" "media.role=phone" ""; do
    PULSE_PROP="$p application.name=palm-policy-test" pacat --playback /dev/zero &
done
PULSE_PROP="media.type=record application.name=palm-policy-test" pacat --record > /dev/null &
python3 "$HERE/client.py" 11 "$FUZZ" || { echo "FAIL client"; fail=1; }
kill $(jobs -p) 2>/dev/null; wait 2>/dev/null
sleep 1
pid1=$(systemctl show -p MainPID --value pulseaudio)
restarts1=$(systemctl show -p NRestarts --value pulseaudio)
[ "$pid0" = "$pid1" ] && [ "$restarts0" = "$restarts1" ] && systemctl -q is-active pulseaudio \
    || { echo "FAIL pulseaudio pid $pid0 -> $pid1, restarts $restarts0 -> $restarts1"; fail=1; }
if journalctl -u pulseaudio --since "$since" --no-pager -o cat | grep -E "Assertion|Aborting|Segmentation|core-dump"; then
    echo "FAIL pulseaudio logged a crash"; fail=1
fi

echo "== restoring audiod"
systemctl start audiod
sleep 2
systemctl -q is-active audiod || { echo "FAIL audiod did not come back"; fail=1; }

[ $fail = 0 ] && echo "PASS" || echo "FAILED"
exit $fail
