/***
  This file is part of PulseAudio.
  Copyright (c) 2026 Herman van Hazendonk <github.com@herrie.org>

  PulseAudio is free software; you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published
  by the Free Software Foundation; either version 2.1 of the License,
  or (at your option) any later version.

  PulseAudio is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with PulseAudio; if not, see <http://www.gnu.org/licenses/>.
***/

/*
 * Unit tests for module-palm-policy. The module is included whole so that its
 * static functions can be called; each test builds the module state with
 * userdata_new() around a real pa_core of its own - no hooks, no listening
 * socket - and talks to it over a socketpair standing in for audiod.
 *
 * What they pin down is what the herrie/palm-policy-fixes audit corrected:
 * replies are whole zero-padded records, requests are framed and bounded, the
 * volume and table bounds hold, wire strings are terminated, the RTP type is
 * kept to its buffer, device lists are bounded and re-initialised cleanly, and
 * routing refuses a device that does not exist instead of taking the daemon
 * down. Leaks and memory errors are left to tests/hardening/run-valgrind.sh.
 */

#include "../../../src/modules/module-palm-policy.c"

#include "check.h"

#include <pulse/mainloop.h>
#include <pulsecore/core.h>

/* --- fixture -------------------------------------------------------------- */

struct fixture {
    pa_mainloop *ml;
    pa_core *core;
    struct userdata *u;
    int audiod; /* our end of the socketpair; the module has the other */
};

static void fixture_up(struct fixture *f)
{
    int sv[2];

    f->ml = pa_mainloop_new();
    f->core = pa_core_new(pa_mainloop_get_api(f->ml), false, false, 0);
    f->u = userdata_new(f->core, NULL);
    if (socketpair(AF_UNIX, SOCK_STREAM, 0, sv) != 0) {
        perror("socketpair");
        abort();
    }
    f->u->newsockfd = sv[0];
    f->u->connectionactive = true;
    f->audiod = sv[1];
}

static void fixture_down(struct fixture *f)
{
    close_connection(f->u);
    userdata_free(f->u);
    close(f->audiod);
    pa_core_unref(f->core);
    pa_mainloop_free(f->ml);
}

/* A request as audiod frames it: header, payload, zeroes to the record size. */
static void make_record(char rec[SIZE_MESG_TO_PULSE], uint32_t type, uint32_t id, const void *payload, size_t len)
{
    struct paudiodMsgHdr hdr;

    memset(rec, 0, SIZE_MESG_TO_PULSE);
    memset(&hdr, 0, sizeof(hdr));
    hdr.msgType = type;
    hdr.msgTmp = 0x01;
    hdr.msgVer = 1;
    hdr.msgLen = sizeof(hdr);
    hdr.msgID = id;
    memcpy(rec, &hdr, sizeof(hdr));
    memcpy(rec + sizeof(hdr), payload, len);
}

/* Hand the module whatever is waiting on its end, as its IO event would. */
static void deliver(struct fixture *f)
{
    handle_io_event_connection(pa_mainloop_get_api(f->ml), NULL, f->u->newsockfd, PA_IO_EVENT_INPUT, f->u);
}

/* Read one reply record without blocking; returns its length (0: none). */
static ssize_t read_reply(struct fixture *f, char buf[SIZE_MESG_TO_AUDIOD])
{
    return recv(f->audiod, buf, SIZE_MESG_TO_AUDIOD, MSG_DONTWAIT);
}

static int all_zero(const char *p, size_t n)
{
    for (size_t i = 0; i < n; i++)
        if (p[i])
            return 0;
    return 1;
}

/* --- bounds --------------------------------------------------------------- */

TEST(VolumeRange, acceptsTheTable)
{
    CHECK(volume_in_range(0, MIN_VOLUME));
    CHECK(volume_in_range(0, MAX_VOLUME));
    CHECK(volume_in_range(1, 50));
}

/* The old check, "MIN_VOLUME <= volume <= MAX_VOLUME", was always true. */
TEST(VolumeRange, rejectsEverythingOutsideIt)
{
    CHECK(!volume_in_range(0, MAX_VOLUME + 1));
    CHECK(!volume_in_range(0, -1));
    CHECK(!volume_in_range(0, INT32_MAX));
    CHECK(!volume_in_range(-1, 0));
    CHECK(!volume_in_range((int)PA_ELEMENTSOF(_mapPercentToPulseRamp), 0));
}

TEST(CopyField, terminatesAndTruncates)
{
    char f[8];

    memset(f, 'x', sizeof(f));
    copy_field(f, sizeof(f), "0123456789");
    CHECK_EQ(strlen(f), 7);
    CHECK(memcmp(f, "0123456", 8) == 0);
}

TEST(CopyField, copiesNullAsEmpty)
{
    char f[4] = "abc";

    copy_field(f, sizeof(f), NULL);
    CHECK_EQ(f[0], 0);
}

/* --- replies -------------------------------------------------------------- */

TEST(Reply, isOneZeroPaddedRecord)
{
    struct fixture f;
    char buf[SIZE_MESG_TO_AUDIOD + 16];
    struct paudiodMsgHdr hdr;
    struct paReplyToAudiod ack;

    fixture_up(&f);
    send_callback_to_audiod(42, 1, f.u);
    CHECK_EQ(recv(f.audiod, buf, sizeof(buf), MSG_DONTWAIT), SIZE_MESG_TO_AUDIOD);
    memcpy(&hdr, buf, sizeof(hdr));
    memcpy(&ack, buf + sizeof(hdr), sizeof(ack));
    CHECK_EQ(hdr.msgType, PAUDIOD_REPLY_MSGTYPE_CALLBACK);
    CHECK_EQ(hdr.msgTmp, 0x01);
    CHECK_EQ(hdr.msgVer, 1);
    CHECK_EQ(hdr.msgLen, sizeof(hdr));
    CHECK_EQ(ack.id, 42);
    CHECK_EQ(ack.returnValue, 1);
    /* nothing of the heap or stack after the payload */
    CHECK(all_zero(buf + sizeof(hdr) + sizeof(ack), SIZE_MESG_TO_AUDIOD - sizeof(hdr) - sizeof(ack)));
    fixture_down(&f);
}

TEST(Reply, needsAConnection)
{
    struct fixture f;
    struct paReplyToAudiod ack = { 1, true };

    fixture_up(&f);
    f.u->connectionactive = false;
    CHECK(!send_to_audiod(f.u, PAUDIOD_REPLY_MSGTYPE_CALLBACK, 0, &ack, sizeof(ack)));
    fixture_down(&f);
}

/* --- receiving ------------------------------------------------------------ */

/* A sink master volume of 101 is refused; the acknowledgement says false. */
static void volume_101_record(char rec[SIZE_MESG_TO_PULSE], uint32_t id)
{
    struct paVolumeSet v;

    memset(&v, 0, sizeof(v));
    v.Type = PAUDIOD_VOLUME_SINK_VOLUME;
    v.volume = MAX_VOLUME + 1;
    copy_field(v.device, sizeof(v.device), "pcm_output");
    make_record(rec, PAUDIOD_MSGTYPE_VOLUME, id, &v, sizeof(v));
}

TEST(Receive, actsOnlyOnAWholeRecord)
{
    struct fixture f;
    char rec[SIZE_MESG_TO_PULSE], reply[SIZE_MESG_TO_AUDIOD];
    struct paReplyToAudiod ack;

    fixture_up(&f);
    volume_101_record(rec, 7);
    CHECK_EQ(write(f.audiod, rec, 60), 60);
    deliver(&f);
    CHECK(read_reply(&f, reply) < 0); /* nothing yet: the record is not whole */
    CHECK_EQ(write(f.audiod, rec + 60, sizeof(rec) - 60), sizeof(rec) - 60);
    deliver(&f);
    CHECK_EQ(read_reply(&f, reply), SIZE_MESG_TO_AUDIOD);
    memcpy(&ack, reply + sizeof(struct paudiodMsgHdr), sizeof(ack));
    CHECK_EQ(ack.id, 7);
    CHECK_EQ(ack.returnValue, 0);
    fixture_down(&f);
}

TEST(Receive, endOfFileClosesTheConnection)
{
    struct fixture f;

    fixture_up(&f);
    shutdown(f.audiod, SHUT_WR);
    deliver(&f);
    CHECK_EQ(f.u->newsockfd, -1);
    CHECK(!f.u->connectionactive);
    fixture_down(&f);
}

TEST(Receive, ignoresARecordOfTheWrongSize)
{
    struct fixture f;
    char rec[SIZE_MESG_TO_PULSE], reply[SIZE_MESG_TO_AUDIOD];

    fixture_up(&f);
    volume_101_record(rec, 9);
    parse_message(rec, SIZE_MESG_TO_PULSE - 1, f.u);
    CHECK(read_reply(&f, reply) < 0);
    fixture_down(&f);
}

/* Strings in the record are terminated within their fields before use. */
TEST(Receive, terminatesWireStrings)
{
    struct fixture f;
    char rec[SIZE_MESG_TO_PULSE];
    struct paVolumeSet v;
    size_t at = sizeof(struct paudiodMsgHdr) + offsetof(struct paVolumeSet, device);

    fixture_up(&f);
    memset(&v, 0, sizeof(v));
    v.Type = PAUDIOD_VOLUME_SINK_MUTE;
    memset(v.device, 'A', sizeof(v.device));
    make_record(rec, PAUDIOD_MSGTYPE_VOLUME, 1, &v, sizeof(v));
    memcpy(f.u->rxbuf, rec, sizeof(rec));
    parse_message(f.u->rxbuf, sizeof(f.u->rxbuf), f.u);
    CHECK_EQ(f.u->rxbuf[at + sizeof(v.device) - 1], 0);
    fixture_down(&f);
}

/* The connection type used to be copied with a bound of 50 into 12 bytes. */
TEST(Receive, keepsTheRtpTypeToItsBuffer)
{
    struct fixture f;
    char rec[SIZE_MESG_TO_PULSE];
    struct paModuleSet m;

    fixture_up(&f);
    memset(&m, 0, sizeof(m));
    m.Type = PAUDIOD_MODULE_RTP_SET;
    memset(m.device, 'B', sizeof(m.device) - 1);
    make_record(rec, PAUDIOD_MSGTYPE_MODULE, 1, &m, sizeof(m));
    parse_message(rec, sizeof(rec), f.u);
    CHECK_EQ(strlen(f.u->connectionType), RTP_CONNECTION_TYPE_STRING_SIZE - 1);
    fixture_down(&f);
}

/* --- device lists --------------------------------------------------------- */

TEST(Devices, boundTheDeviceCount)
{
    struct fixture f;

    fixture_up(&f);
    CHECK(!initialise_internal_card(f.u, 0, 1));
    CHECK(!initialise_internal_card(f.u, MAX_DEVICE_COUNT + 1, 1));
    CHECK(!initialise_internal_card(f.u, -1, 0));
    CHECK(initialise_internal_card(f.u, MAX_DEVICE_COUNT, 1));
    fixture_down(&f);
}

/* audiod sends this again whenever it restarts; valgrind checks for the leak. */
TEST(Devices, reinitialiseTheInternalList)
{
    struct fixture f;

    fixture_up(&f);
    CHECK(initialise_internal_card(f.u, 4, 1));
    CHECK(initialise_internal_card(f.u, 2, 1));
    CHECK_EQ(f.u->internalOutputDeviceInfo->maxDeviceCount, 2);
    CHECK_EQ(f.u->internalOutputDeviceInfo->deviceList[1].cardNumber, -1);
    fixture_down(&f);
}

/* Device names are <base><index> in a 16-byte field: a base that cannot fit
 * the highest index is refused, where it used to overflow the field. */
TEST(Devices, refuseABaseNameThatCannotFit)
{
    struct fixture f;
    char longname[DEVICE_NAME_LENGTH];

    fixture_up(&f);
    memset(longname, 'C', sizeof(longname) - 1);
    longname[sizeof(longname) - 1] = '\0';
    CHECK(!init_multiple_usb_device_info(f.u, true, 4, longname));
    /* 10 devices: indexes up to 9, so 15 + 1 characters do not fit, 14 + 1 do */
    CHECK(!init_multiple_usb_device_info(f.u, true, 10, "abcdefghijklmno"));
    CHECK(init_multiple_usb_device_info(f.u, true, 10, "abcdefghijklmn"));
    fixture_down(&f);
}

/* --- routing -------------------------------------------------------------- */

/* Moving streams to a missing sink asserted and took the daemon down. */
TEST(Routing, refusesADeviceThatIsNotThere)
{
    struct fixture f;
    char name[] = "no-such-sink";

    fixture_up(&f);
    CHECK(!set_sink_outputdevice(f.u, name, edefaultapp));
    CHECK(!set_source_inputdevice(f.u, name, erecord));
    CHECK(!virtual_sink_input_move_outputdevice(edefaultapp, name, f.u));
    CHECK(!virtual_source_output_move_inputdevice(erecord, name, f.u));
    CHECK(!set_sink_outputdevice(f.u, name, eVirtualSink_Count));
    fixture_down(&f);
}

/* --- life cycle ----------------------------------------------------------- */

TEST(Lifecycle, newAndFree)
{
    struct fixture f;

    fixture_up(&f);
    CHECK(f.u->sink_mapping_table[edefaultapp].virtualsinkname != NULL);
    CHECK_EQ(f.u->sockfd, -1);
    fixture_down(&f);
}
