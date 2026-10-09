#!/usr/bin/env python3
# Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>
# SPDX-License-Identifier: LGPL-2.1-or-later
"""Drive module-palm-policy's audiod socket: protocol edge cases, then a seeded fuzz run.
Records are SIZE_MESG_TO_PULSE (150) bytes; replies are 250-byte records.

Volume and mute go to the pdefaultapp null sink, never to a sink that drives
hardware: the same code runs, and nothing on a real device gets set loud.

  client.py [seed] [fuzz-records]"""
import socket, struct, random, sys, time, select

REC = 150
def hdr(msgtype, msgid=1):
    return struct.pack("<IBBII", msgtype, 0x01, 1, 14, msgid)

def rec(msgtype, payload, msgid=1):
    b = hdr(msgtype, msgid) + payload
    return b[:REC].ljust(REC, b"\0")

ROUTING, VOLUME, DEVICE, MODULE, SETPARAM, EFFECT = 1, 2, 3, 4, 5, 6
def routing(t, start=0, end=0, ident=0, dev=b"pcm_output"):
    return rec(ROUTING, struct.pack("<IIII50s", t, start, end, ident, dev))
def volume(t, ident=0, vol=0, table=0, ramp=0, mute=0, p1=0, p2=0, p3=0, index=0, dev=b"pdefaultapp"):
    return rec(VOLUME, struct.pack("<IIIIIIIIII50s", t, ident, vol, table, ramp, mute, p1, p2, p3, index, dev))
def device(t, card=0, dev=0, status=0, out=1, maxdev=1, name=b"usb"):
    return rec(DEVICE, struct.pack("<IIIBBBIIII50s", t, card, dev, 0, 0, 0, 0, status, out, maxdev, name))
def module(t, port=0, ip=b"", dev=b""):
    return rec(MODULE, struct.pack("<HIIII28s50s18s5s", t, 0, 0, 0, port, ip, dev, b"", b""))
def setparam(t, ident=0, p1=0, p2=0, p3=0):
    return rec(SETPARAM, struct.pack("<IIIII", t, ident, p1, p2, p3))
def effect(t, ident=0, a=0, b=0, c=0):
    return rec(EFFECT, struct.pack("<II3I", t, ident, a, b, c))

def connect():
    for _ in range(100):
        try:
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.connect(b"\0palmaudio")
            return s
        except OSError:
            time.sleep(0.1)
    sys.exit("cannot connect to palmaudio")

replies = 0
def drain(s, wait=0.05):
    global replies
    while True:
        r, _, _ = select.select([s], [], [], wait)
        if not r:
            return
        d = s.recv(65536)
        if not d:
            return
        replies += len(d) // 250

M = 0xFFFFFFFF
LONG = b"A" * 50                 # fills a 50-byte field with no terminator
cases = [
    ("volume in range", volume(0x0001, 0, 50, p1=50, dev=b"pdefaultapp")),
    ("sink master volume 100", volume(0x0001, 0, 100, dev=b"pdefaultapp")),
    ("sink master volume 101", volume(0x0001, 0, 101, dev=b"pdefaultapp")),
    ("sink master volume -1", volume(0x0001, 0, M, dev=b"pdefaultapp")),
    ("sink input volume huge", volume(0x0010, 0, 0, p1=M, p2=M)),
    ("ramp volume huge", volume(0x0040, 0, 0, p1=1000, p2=7)),
    ("index volume huge", volume(0x0030, 3, 0, p1=M, p2=M, index=5)),
    ("source mic volume -1", volume(0x0200, 0, M, dev=b"pcm_input")),
    ("source output volume huge", volume(0x1000, 0, 0, p1=M, p2=1)),
    ("sink mute bad id", volume(0x0020, M, mute=1)),
    ("sink master mute unterminated", volume(0x0002, 0, mute=1, dev=LONG)),
    ("routing move unterminated", routing(0x0010, ident=0, dev=LONG)),
    ("routing range reversed", routing(0x0020, start=20, end=1, dev=b"pcm_output")),
    ("routing range huge", routing(0x0020, start=M, end=M, dev=b"pcm_output")),
    ("routing default huge", routing(0x0030, start=0, end=M)),
    ("source routing range", routing(0x2000, start=0, end=8, dev=b"pcm_input")),
    ("internal card init", device(0x0002, maxdev=4, out=1)),
    ("internal card init again", device(0x0002, maxdev=4, out=1)),
    ("internal card init huge", device(0x0002, maxdev=M, out=0)),
    ("usb init long base name", device(0x0020, maxdev=4, name=LONG)),
    ("usb init", device(0x0020, maxdev=2, name=b"usb_out")),
    ("usb init huge count", device(0x0020, maxdev=1 << 30, name=b"u")),
    ("usb remove unknown", device(0x0010, card=9, dev=9, status=0)),
    ("rtp set unterminated type", module(0x0002, port=5004, dev=LONG)),
    ("rtp set unicast", module(0x0002, port=70000, dev=b"unicast")),
    ("a2dp source", module(0x0004)),
    ("suspend", setparam(0x0001)),
    ("sample rate", setparam(0x0002, p1=48000)),
    ("close playback unknown", setparam(0x0003, ident=12345)),
    ("unknown setparam", setparam(0x7777)),
    ("effect speech", effect(0x0001, a=1)),
    ("effect speech off", effect(0x0001, a=0)),
    ("effect eq param", effect(0x0006, a=M, b=M, c=M)),
    ("unknown type", rec(0x1001, b"\xff" * 136)),
    ("garbage header", b"\xff" * REC),
]

s = connect()
for name, r in cases:
    s.sendall(r)
    drain(s)
    print("ok:", name, flush=True)

# a record split over three writes, then one cut short by a reconnect
r = volume(0x0001, 0, 30, dev=b"pdefaultapp")
for part in (r[:7], r[7:100], r[100:]):
    s.sendall(part); time.sleep(0.02)
drain(s); print("ok: split record", flush=True)
s.sendall(r[:60]); s.close(); time.sleep(0.2)
s = connect(); drain(s, 0.2); print("ok: truncated record then reconnect", flush=True)

# seeded fuzz over everything but the bluetooth module commands
rng = random.Random(int(sys.argv[1]) if len(sys.argv) > 1 else 1)
types = {ROUTING: [0x10, 0x20, 0x30, 0x40, 0x1000, 0x2000, 0x3000, 0x4000],
         VOLUME: [1, 2, 0x10, 0x20, 0x30, 0x40, 0x50, 0x100, 0x200, 0x1000, 0x2000],
         DEVICE: [2, 0x20],
         SETPARAM: [1, 2, 3, 9],
         EFFECT: [1, 2, 3, 4, 5, 6, 7, 99]}
N = int(sys.argv[2]) if len(sys.argv) > 2 else 3000
for i in range(N):
    mt = rng.choice(list(types))
    t = rng.choice(types[mt])
    body = struct.pack("<H" if mt == MODULE else "<I", t) + bytes(rng.getrandbits(8) for _ in range(REC - 18))
    if rng.random() < 0.5:   # keep ids and volumes small often, to reach deeper paths
        body = body[:4] + bytes(rng.choice([0, 1, 2, 5, 20, 100, 101, 255]) if j % 4 == 0 else 0 for j in range(32)) + body[36:]
    s.sendall(rec(mt, body, msgid=i))
    if i % 50 == 0:
        drain(s, 0.01)
drain(s, 0.5)
print("ok: fuzz", N, "records, replies", replies, flush=True)
s.close()
