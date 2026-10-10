# module-palm-policy test and hardening harness

Four layers, smallest first. LuneOS builds only this module out of the
repository, against an upstream PulseAudio 17 (meta-luneos
`pulseaudio-module-palm-policy`), so the tests are built the same way and do not
need the rest of the tree.

## Unit tests (`unit/`)

The module is included whole, so its static functions can be called. Each test
builds the module state with `userdata_new()` around a `pa_core` of its own - no
hooks, no listening socket - and stands in for audiod on a socketpair. They pin
down what the `herrie/palm-policy-fixes` audit corrected: replies are whole
zero-padded records, requests are framed and terminated, the volume and table
bounds hold, the RTP connection type is kept to its buffer, device lists are
bounded and re-initialised cleanly, and routing refuses a device that does not
exist or an id outside the table.

Build with a recipe workdir's environment (`CC`, `PKG_CONFIG_*`, `--sysroot`):

    make -C tests/module-palm-policy CC="$CC" PKG_CONFIG=pkg-config \
        LDFLAGS="-L$SYSROOT/usr/lib/pulseaudio -Wl,-rpath-link,$SYSROOT/usr/lib/pulseaudio"

and run on the build host through the x86-64 recipe sysroot's loader:

    $SYSROOT/usr/lib/ld-linux-x86-64.so.2 \
        --library-path $SYSROOT/usr/lib:$SYSROOT/usr/lib/pulseaudio \
        tests/module-palm-policy/palm-policy-tests [suite]

The runner in `unit/check.h` is the C twin of the one in notificationmgr's
tests; gtest is not in the LuneOS sysroot. Every test was mutation-checked:
each corrected bug, put back into the module, fails at least one of them.

## Hardening (`hardening/`)

`run-valgrind.sh <palm-policy-tests> <recipe-sysroot>` runs the x86-64 unit test
binary under memcheck and fails on any memory error or definite leak.

## Integration (`integration/`)

`run.sh` runs LuneOS's own x86-64 PulseAudio 17, taken from an extracted
qemux86-64 rootfs (`ROOTFS=`), with a given build of the module, inside private
user, network and mount namespaces: the policy socket is an abstract socket and
so private to the namespace, `/dev/snd` and the host's D-Bus are hidden, and
only null sinks exist. `client.py` drives the module with protocol edge cases
and a seeded fuzz run while pacat plays and records silence, then the daemon is
stopped so `pa__done()` runs too.

The module can be built with AddressSanitizer and UndefinedBehaviorSanitizer
and the runtime preloaded through the third argument, or the daemon run under
valgrind (`--keep-debuginfo=yes`, or the unloaded module's frames are lost) or
`setarch -R` for ThreadSanitizer through the remaining arguments.

## On the device (`device/`)

Copy `device/run-on-device.sh`, `integration/client.py` and the device's
`palm-policy-tests` into one directory and run the script as root. It runs the
unit tests, stops audiod (the module serves a single connection), drives the
live module with the same cases and a fuzz run while pacat streams silence,
fails if pulseaudio crashed, restarted or logged an assertion, and starts
audiod again. Volume and mute only ever go to the `pdefaultapp` null sink.
