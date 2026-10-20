#!/bin/bash
# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
# SPDX-License-Identifier: LGPL-2.1-or-later
#
# Run a LuneOS x86-64 PulseAudio 17 (from a qemux86-64 rootfs) with a given
# module-palm-policy, drive it with client.py and real pacat streams, stop it so
# pa__done() runs, and keep the log:
#
#   ROOTFS=<extracted rootfs> integration/run.sh <label> <module.so> [preload] [wrapper...]
#
# <preload> is a colon-separated list for the loader's --preload (a sanitizer
# runtime), [wrapper...] runs the daemon under e.g. valgrind or setarch -R.
# Everything runs in private user, network and mount namespaces: the policy
# socket is abstract and so private to the netns, /dev/snd and the host's D-Bus
# are hidden, and only null sinks exist - nothing reaches real sound hardware or
# host services. Log: ${OUT:-./integration-out}/<label>.log; FUZZ sets the
# number of fuzz records (3000).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
LABEL=${1:?label}; MOD=$(readlink -f "${2:?module}"); PRELOAD=${3:-}; shift 3 2>/dev/null; WRAP=("$@")
ROOTFS=$(readlink -f "${ROOTFS:?set ROOTFS to an extracted qemux86-64 rootfs}")
OUT=$(mkdir -p "${OUT:-./integration-out}" && cd "${OUT:-./integration-out}" && pwd)
export HERE MOD PRELOAD ROOTFS OUT LABEL FUZZ=${FUZZ:-3000}
exec unshare --user --map-root-user --net --mount --fork bash -s -- "${WRAP[@]}" > "$OUT/$LABEL.log" 2>&1 <<'INNER'
set -u
WRAP=("$@")
mount -t tmpfs none /dev/snd 2>/dev/null || mount -t tmpfs none /dev
mount -t tmpfs none /run/dbus 2>/dev/null
export DBUS_SYSTEM_BUS_ADDRESS=unix:path=/nonexistent DBUS_SESSION_BUS_ADDRESS=unix:path=/nonexistent
ip link set lo up 2>/dev/null
T=$OUT/tmp-$LABEL; rm -rf "$T"; mkdir -p "$T/mods" "$T/run" "$T/home"
for m in "$ROOTFS"/usr/lib/pulseaudio/modules/*.so; do ln -s "$m" "$T/mods/"; done
rm -f "$T/mods/module-palm-policy.so"; cp "$MOD" "$T/mods/module-palm-policy.so"
grep -E "^load-module" "$ROOTFS/etc/pulse/webos-virtual-devices.pa" > "$T/test.pa"
sed "s|@NATIVE@|$T/run/native|" "$HERE/test.pa.in" >> "$T/test.pa"
LD=$ROOTFS/usr/lib/ld-linux-x86-64.so.2
LP=$ROOTFS/usr/lib:$ROOTFS/usr/lib/pulseaudio:$ROOTFS/usr/lib/pulseaudio/modules
export HOME=$T/home XDG_RUNTIME_DIR=$T/run PULSE_RUNTIME_PATH=$T/run
PA=("${WRAP[@]}" "$LD" --library-path "$LP" ${PRELOAD:+--preload "$PRELOAD"} "$ROOTFS/usr/bin/pulseaudio"
    -n --daemonize=no --exit-idle-time=-1 --use-pid-file=no --system=no --disallow-exit=no
    --dl-search-path="$T/mods" --file="$T/test.pa" --log-target=stderr --log-level=info)
"${PA[@]}" > "$T/pa.log" 2>&1 &
PAPID=$!
for i in $(seq 300); do [ -S "$T/run/native" ] && break; kill -0 $PAPID 2>/dev/null || break; sleep 0.2; done
echo "### daemon up: $([ -S "$T/run/native" ] && echo yes || echo NO)"
export PULSE_SERVER=unix:$T/run/native
PC=("$LD" --library-path "$LP" "$ROOTFS/usr/bin/pacat")
STREAMS=()
PULSE_PROP='media.type=pmedia application.name=t1' timeout 3 "${PC[@]}" --playback /dev/zero & STREAMS+=($!)
PULSE_PROP='application.name=t2 test.stream=t2' timeout 3 "${PC[@]}" --playback /dev/zero & STREAMS+=($!)
PULSE_PROP='application.name=t3 test.stream=t3' timeout 3 "${PC[@]}" --playback -d pcm_output /dev/zero & STREAMS+=($!)
PULSE_PROP='media.role=phone application.name=t4' timeout 3 "${PC[@]}" --playback /dev/zero & STREAMS+=($!)
PULSE_PROP='media.type=record application.name=r1' timeout 3 "${PC[@]}" --record > /dev/null & STREAMS+=($!)
PULSE_PROP='media.type=nonsense application.name=r2' timeout 3 "${PC[@]}" --record -d record > /dev/null & STREAMS+=($!)
# t3 names the default sink, as QtMultimedia does: it must be routed like t2,
# which names none - on pdefaultapp, with no preferred device of its own -
# or it is never moved off the loudspeaker.
sleep 1
"$LD" --library-path "$LP" "$ROOTFS/usr/bin/pactl" list sink-inputs > "$T/sink-inputs.txt" 2>&1
for app in t2 t3; do
    # pacat puts its own application.name over PULSE_PROP's
    props=$(awk -v app="test.stream = \"$app\"" '
        function flush() { if (hit) print block }
        /^Sink Input #/ { flush(); block = ""; hit = 0 }
        { block = block "\n" $0; if (index($0, app)) hit = 1 }
        END { flush() }' "$T/sink-inputs.txt")
    type=$(echo "$props" | sed -n 's/.*media.type = "\(.*\)"/\1/p' | head -n 1)
    pref=$(echo "$props" | grep -c "preferred.device")
    if [ "$type" = pdefaultapp ] && [ "$pref" = 0 ]; then
        echo "### routing $app: ok (media.type $type)"
    else
        echo "### routing $app: FAIL (media.type '$type', preferred.device $pref)"
    fi
done
python3 "$HERE/client.py" 7 "$FUZZ"; echo "### client rc=$?"
for p in "${STREAMS[@]}"; do wait $p; done
kill -TERM $PAPID; for i in $(seq 300); do kill -0 $PAPID 2>/dev/null || break; sleep 0.2; done
kill -0 $PAPID 2>/dev/null && { echo "### daemon did not exit, killing"; kill -KILL $PAPID; }
wait $PAPID; echo "### daemon rc=$?"
cat "$T/pa.log"
INNER
