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

/* What QtMultimedia does: it names the default sink for its default output.
 * That is no choice of sink, and the stream has to be routed - or it stays on
 * the loudspeaker when a headset connects. Only these addresses are compared. */
static pa_sink default_sink_stub, other_sink_stub;

TEST(Routing, namingTheDefaultSinkIsNoChoice)
{
    CHECK(!stream_chose_its_sink(NULL, &default_sink_stub, NULL));
    CHECK(!stream_chose_its_sink(&default_sink_stub, &default_sink_stub, NULL));
}

TEST(Routing, anotherSinkOrAPreferredDeviceIsAChoice)
{
    CHECK(stream_chose_its_sink(&other_sink_stub, &default_sink_stub, NULL));
    CHECK(stream_chose_its_sink(&default_sink_stub, &default_sink_stub, "bluez_sink.x"));
    CHECK(stream_chose_its_sink(NULL, &default_sink_stub, "pcm_output"));
    /* no default sink at all: a named sink is all there is to go on */
    CHECK(stream_chose_its_sink(&other_sink_stub, NULL, NULL));
}

/* --- phone-era sinks ------------------------------------------------------ */

/* palarm, pcalendar and the rest exist as null sinks but have no class of
 * their own; a stream on one was left there, silent. They are routed as the
 * class they are an alias of. */
TEST(SinkClass, aVirtualSinkIsItsOwnClass)
{
    for (int i = eVirtualSink_First; i < eVirtualSink_Count; i++)
        CHECK_EQ(virtual_sink_class(virtualsinkmap[i].virtualsinkname), i);
}

TEST(SinkClass, aPhoneEraSinkIsRoutedAsItsAlias)
{
    CHECK_EQ(virtual_sink_class("palarm"), eringtones);
    CHECK_EQ(virtual_sink_class("pcalendar"), ealerts);
    CHECK_EQ(virtual_sink_class("pnotifications"), ealerts);
    CHECK_EQ(virtual_sink_class("pDTMF"), efeedback);
    CHECK_EQ(virtual_sink_class("pvoip"), evoipcall);
    CHECK_EQ(virtual_sink_class("pvvm"), emedia);
    CHECK_EQ(virtual_sink_class("pnavigation"), etts);
    CHECK_EQ(virtual_sink_class("pvoicedial"), evoicerecognition);
    CHECK_EQ(virtual_sink_class("pflash"), edefaultapp);
}

TEST(SinkClass, everyAliasIsARoutedClassAndNotAVirtualSink)
{
    for (int a = 0; virtualsinkaliases[a].virtualsinkname; a++) {
        int routed = (int)virtualsinkaliases[a].routedas;

        CHECK(routed >= eVirtualSink_First && routed < eVirtualSink_Count);
        for (int i = eVirtualSink_First; i < eVirtualSink_Count; i++)
            CHECK(strcmp(virtualsinkaliases[a].virtualsinkname, virtualsinkmap[i].virtualsinkname) != 0);
    }
}

TEST(SinkClass, anythingElseIsNotRouted)
{
    CHECK_EQ(virtual_sink_class(NULL), -1);
    CHECK_EQ(virtual_sink_class(""), -1);
    CHECK_EQ(virtual_sink_class("pcm_output"), -1);
    CHECK_EQ(virtual_sink_class("palarm.monitor"), -1);
    CHECK_EQ(virtual_sink_class("palar"), -1);
    CHECK_EQ(virtual_sink_class("PALARM"), -1);
}

/* The new-stream hook: a stream opened on the palarm null sink is routed as a
 * ringtone and tagged with its own name. No sink is registered in the test
 * core, so where it would be moved to cannot be looked up and it stays put;
 * the class it was given is what the routing goes by. */
TEST(SinkClass, aStreamOpenedOnAPhoneEraSinkIsRoutedAsItsClass)
{
    struct fixture f;
    pa_sink_input_new_data d;
    pa_sink palarm;

    fixture_up(&f);
    memset(&palarm, 0, sizeof(palarm));
    palarm.name = (char *)"palarm";
    palarm.driver = (char *)MODULE_NULL_SINK;
    f.u->media_type = edefaultapp;

    pa_sink_input_new_data_init(&d);
    d.sink = &palarm;
    route_sink_input_new_hook_callback(f.core, &d, f.u);
    CHECK_EQ(f.u->media_type, eringtones);
    CHECK(pa_safe_streq(pa_proplist_gets(d.proplist, "media.type"), "palarm"));
    CHECK(d.sink == &palarm);
    pa_sink_input_new_data_done(&d);

    /* a null sink that is no class at all is still left alone */
    f.u->media_type = edefaultapp;
    palarm.name = (char *)"pcm_output_null";
    pa_sink_input_new_data_init(&d);
    d.sink = &palarm;
    route_sink_input_new_hook_callback(f.core, &d, f.u);
    CHECK_EQ(f.u->media_type, edefaultapp);
    pa_sink_input_new_data_done(&d);

    fixture_down(&f);
}

/* A stream opened on palarm, as the new-stream hook leaves it: tagged with its
 * own name. */
static void alarm_stream_data(pa_sink_input_new_data *d, const char *media_type)
{
    pa_sink_input_new_data_init(d);
    pa_proplist_sets(d->proplist, "media.type", media_type);
    pa_proplist_sets(d->proplist, PA_PROP_APPLICATION_NAME, "clock");
    pa_channel_map_init_stereo(&d->channel_map);
    d->volume_writable = true;
}

TEST(SinkClass, aPhoneEraStreamTakesItsClassVolumeAndMute)
{
    struct fixture f;
    pa_sink_input_new_data d;

    fixture_up(&f);
    f.u->sink_mapping_table[eringtones].volume = 80;
    f.u->sink_mapping_table[eringtones].ismuted = true;
    f.u->sink_mapping_table[ealerts].volume = 20;
    f.u->sink_mapping_table[ealerts].ismuted = false;

    alarm_stream_data(&d, "palarm");
    route_sink_input_fixate_hook_callback(f.core, &d, f.u);
    CHECK(d.muted_is_set);
    CHECK(d.muted);
    CHECK(d.volume_is_set);
    CHECK_EQ(d.volume.values[0],
             pa_sw_volume_from_dB(_mapPercentToPulseRamp[f.u->sink_mapping_table[eringtones].volumetable][80]));
    /* the tag stays the stream's own name, which audiod's ring state wants */
    CHECK(pa_safe_streq(pa_proplist_gets(d.proplist, "media.type"), "palarm"));
    pa_sink_input_new_data_done(&d);

    alarm_stream_data(&d, "pcalendar");
    route_sink_input_fixate_hook_callback(f.core, &d, f.u);
    CHECK(d.muted_is_set);
    CHECK(!d.muted);
    CHECK_EQ(d.volume.values[0],
             pa_sw_volume_from_dB(_mapPercentToPulseRamp[f.u->sink_mapping_table[ealerts].volumetable][20]));
    pa_sink_input_new_data_done(&d);

    /* not a class: volume and mute are left alone */
    alarm_stream_data(&d, "pcm_output");
    route_sink_input_fixate_hook_callback(f.core, &d, f.u);
    CHECK(!d.muted_is_set);
    CHECK(!d.volume_is_set);
    pa_sink_input_new_data_done(&d);

    fixture_down(&f);
}

/* The put hook is what puts a stream in the list the routing moves, and what
 * audiod is told about: a palarm stream has to be in it as a ringtone. */
TEST(SinkClass, aPhoneEraStreamIsTrackedAsItsClass)
{
    struct fixture f;
    pa_sink_input si;

    fixture_up(&f);
    memset(&si, 0, sizeof(si));
    si.proplist = pa_proplist_new();
    si.index = 42;
    si.state = PA_SINK_INPUT_RUNNING;

    pa_proplist_sets(si.proplist, "media.type", "palarm");
    route_sink_input_put_hook_callback(f.core, &si, f.u);
    CHECK(f.u->sinkinputnodelist != NULL);
    if (f.u->sinkinputnodelist) {
        CHECK_EQ(f.u->sinkinputnodelist->virtualsinkid, eringtones);
        CHECK_EQ(f.u->sinkinputnodelist->sinkinputidx, 42);
    }
    CHECK_EQ(f.u->n_sink_input_opened, 1);

    /* a stream the policy does not route is not tracked */
    si.index = 43;
    pa_proplist_sets(si.proplist, "media.type", "pcm_output");
    route_sink_input_put_hook_callback(f.core, &si, f.u);
    CHECK_EQ(f.u->n_sink_input_opened, 1);
    CHECK(f.u->sinkinputnodelist == NULL || f.u->sinkinputnodelist->sinkinputidx == 42);

    pa_proplist_free(si.proplist);
    fixture_down(&f);
}

/* The sink module-bluez5-device makes for a headset, which the policy looks
 * up to tell audiod of a headset already there (audiod restarted, or missed
 * it while bluetooth2 was away). */
TEST(Bluetooth, sinkNameOfAnAddress)
{
    char name[BLUETOOTH_SINK_NAME_SIZE];

    bluetooth_sink_name("74:1a:e0:20:8f:18", name, sizeof(name));
    CHECK(strcmp(name, "bluez_sink.74_1A_E0_20_8F_18") == 0);
    bluetooth_sink_name("", name, sizeof(name));
    CHECK(strcmp(name, "bluez_sink.") == 0);
    bluetooth_sink_name(NULL, name, sizeof(name));
    CHECK(strcmp(name, "bluez_sink.") == 0);
}

TEST(Bluetooth, sinkNameIsBoundedAndTerminated)
{
    char small[16];

    memset(small, 'x', sizeof(small));
    bluetooth_sink_name("74:1a:e0:20:8f:18", small, sizeof(small));
    CHECK_EQ(strlen(small), sizeof(small) - 1);
    CHECK(strncmp(small, "bluez_sink.74_1", sizeof(small) - 1) == 0);

    char tiny[4];
    memset(tiny, 'x', sizeof(tiny));
    bluetooth_sink_name("74:1a", tiny, sizeof(tiny));
    CHECK_EQ(tiny[0], '\0');
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
