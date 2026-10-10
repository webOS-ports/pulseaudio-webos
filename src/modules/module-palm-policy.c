/***
  This file is part of PulseAudio.
  Copyright (c) 2002-2025 LG Electronics, Inc.
  All rights reserved.

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

/***********************************************************************
 *
 * module-palm-policy.c - implements policy requests received via an SHM section
 * as determined by audioD.
 **********************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <pulse/xmalloc.h>
#include <pulse/mainloop.h>
#include <pulse/context.h>
#include <pulse/operation.h>
#include <pulsecore/thread.h>
#include <pulsecore/thread-mq.h>
#include <pulsecore/namereg.h>
#include <pulsecore/idxset.h>
#include <pulsecore/sink-input.h>
#include <pulsecore/log.h>
#include <pulsecore/module.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <pulsecore/modargs.h>

#include <alsa/asoundlib.h>
#include <alsa/control.h>

#include "module-palm-policy.h"
#include "module-palm-policy-tables.h"
#include "module-palm-policy-util.h"

/* Upstream PulseAudio has no such proplist key -- it is an LG addition carried in
 * this fork's own src/pulse/proplist.h. Define it locally so the module can be
 * built out-of-tree against an unmodified PulseAudio. Only this module reads and
 * writes the key, so a local definition is sufficient. */
#ifndef PA_PROP_PREFERRED_DEVICE
#define PA_PROP_PREFERRED_DEVICE "preferred.device"
#endif

#define _MEM_ZERO(object) (memset(&(object), '\0', sizeof((object))))

#ifndef PALM_UP_RAMP_MSEC
#define PALM_UP_RAMP_MSEC 600
#endif

#ifndef PALM_DOWN_RAMP_MSEC
#define PALM_DOWN_RAMP_MSEC 400
#endif

#ifndef CLAMP_VOLUME_TABLE
#define CLAMP_VOLUME_TABLE(a) (((a) < (1)) ? (a) : (1))
#endif

#define RAMP_DURATION_MSEC 1000
#define PCM_SINK_NAME "pcm_output"
#define PCM_SOURCE_NAME "pcm_input"
#define PCM_HEADPHONE_SINK "pcm_headphone"
#define RTP_SINK_NAME "rtp"
#define SCENARIO_STRING_SIZE 28
#define RTP_IP_ADDRESS_STRING_SIZE 28
#define RTP_CONNECTION_TYPE_STRING_SIZE 12
#define ROUTE_AUTO 0
#define ROUTE_HEADPHONES 1
#define BLUETOOTH_MAC_ADDRESS_SIZE 18
#define BLUETOOTH_SINK_NAME_SIZE 30
#define BLUETOOTH_PROFILE_SIZE 5
#define BLUETOOTH_SINK_INIT_SIZE 12
#define DISPLAY_ONE 1
#define DISPLAY_TWO 2
#define DISPLAY_SINK_COUNT 3
#define DISPLAY_ONE_CARD_NUMBER 1
#define DISPLAY_TWO_CARD_NUMBER 2
#define VOLUMETABLE 0
#define MIN_VOLUME 0
#define MAX_VOLUME 100
#define MUTE 1
#define UNMUTE 0
#define SAVE 0
#define SOURCE_NAME_LENGTH 18
#define SINK_NAME_LENGTH 16
#define BLUETOOTH_SINK_NAME_LENGTH 20
#define DEVICE_NAME_LENGTH 50

#define DEFAULT_SOURCE_0 "/dev/snd/pcmC0D0c"
#define DEFAULT_SOURCE_1 "/dev/snd/pcmC1D0c"

#define ALSA_CARD_NAME "alsa.card_name"
#define DEVICE_ICON_NAME "device.icon_name"
#define BLUEZ_DEVICE_NAME "device.description"
#define MODULE_ALSA_SINK_NAME "module-alsa-sink"
#define MODULE_ALSA_SOURCE_NAME "module-alsa-source"
#define MODULE_NULL_SINK "module-null-sink.c"
#define MODULE_NULL_SOURCE "module-null-source.c"

/* use this to tie an individual sink_input to the
 * virtual sink it was created against */

struct sinkinputnode
{
    int32_t sinkinputidx;     /* index of this sink-input */
    int32_t virtualsinkid;    /* index of virtual sink it was created against, this is our index from
                               * _enum_virtualsinkmap rather than a pulseaudio sink idx */
    pa_sink_input *sinkinput; /* reference to sink input with this index */

    bool paused;

    bool bypassRouting;

    char appname[APP_NAME_LENGTH];

    PA_LLIST_FIELDS(struct sinkinputnode); /* fields that use a pulse defined linked list */
};

struct sourceoutputnode
{
    int32_t sourceoutputidx;        /* index of this sink-input */
    int32_t virtualsourceid;        /* index of virtual sink it was created against, this is our index from
                                     * _enum_virtualsourcemap rather than a pulseaudio sink idx */
    pa_source_output *sourceoutput; /* reference to sink input with this index */
    bool paused;

    bool bypassRouting;

    char appname[APP_NAME_LENGTH];

    PA_LLIST_FIELDS(struct sourceoutputnode); /* fields that use a pulse defined linked list */
};

typedef struct deviceInfo
{
    int index;
    int cardNumber;
    int deviceNumber;
    char cardName[SINK_NAME_LENGTH];
    char *cardNameDetail;
    pa_module *alsaModule;
} deviceInfo;

typedef struct multipleDeviceInfo
{
    char *baseName;
    int maxDeviceCount;
    deviceInfo *deviceList;
} multipleDeviceInfo;

/* user data for the pulseaudio module, store this in init so that
 * stuff we need can be accessed when we get callbacks
 */

struct userdata
{
    /* cached references to pulse internals */
    pa_core *core;
    pa_module *module;

    /* slots for hook functions, these get called by pulse */
    pa_hook_slot *sink_input_new_hook_slot; /* called prior to creation of new sink-input */
    pa_hook_slot *sink_input_fixate_hook_slot;
    pa_hook_slot *sink_input_put_hook_slot;
    pa_hook_slot *sink_input_state_changed_hook_slot; /* called on state change, play/pause */
    pa_hook_slot *sink_input_unlink_hook_slot;        /* called prior to destruction of a sink-input */
    pa_hook_slot *sink_state_changed_hook_slot;       /* for BT sink open-close */

    pa_hook_slot *source_output_new_hook_slot; /* called prior to creation of new source-output */
    pa_hook_slot *source_output_fixate_hook_slot;
    pa_hook_slot *source_output_put_hook_slot;
    pa_hook_slot *source_output_state_changed_hook_slot;
    pa_hook_slot *source_output_unlink_hook_slot; /* called prior to destruction of a source-output */
    pa_hook_slot *source_output_move_finish;
    pa_hook_slot *sink_state_changed_hook;
    pa_hook_slot *source_state_changed_hook_slot;
    pa_hook_slot *module_unload_hook_slot;
    pa_hook_slot *module_load_hook_slot;
    pa_hook_slot *sink_load_hook_slot;
    pa_hook_slot *source_load_hook_slot;
    pa_hook_slot *sink_input_move_finish;
    pa_hook_slot *sink_new;
    pa_hook_slot *sink_unlink;
    pa_hook_slot *sink_unlink_post;
    pa_hook_slot *source_unlink_post;

    /* make sure sink_mapping_table is the same size as
     * defaulmappingtable, since we'll copy that to this */
    struct _virtualsinkmap sink_mapping_table[eVirtualSink_Count];
    struct _virtualsourcemap source_mapping_table[eVirtualSource_Count];

    /* fields for socket - ipc support for audiod */

    int sockfd;    /* descriptor for socket */
    int newsockfd; /* descriptor for connections on socket */

    struct sockaddr_un name;    /* filename for socket */
    pa_io_event *sockev;        /* socket event handler */
    pa_io_event *connev;        /* connection event handler */
    bool connectionactive; /* do we have an active connection on the socket */
    char rxbuf[SIZE_MESG_TO_PULSE]; /* the message from audiod being received */
    size_t rxlen;                   /* how much of it has arrived */

    // Maintain count of sinks opened, as sent to audiod, so that we can re-send data on reconnect for audiod to resync with us
    int32_t audiod_sink_input_opened[eVirtualSink_Count];
    int32_t audiod_source_output_opened[eVirtualSource_Count];
    int32_t n_sink_input_opened;
    int32_t n_source_output_opened;

    /* list of nodes describing sink-input and source-output vs virtual device created against */
    PA_LLIST_HEAD(struct sinkinputnode, sinkinputnodelist);
    PA_LLIST_HEAD(struct sourceoutputnode, sourceoutputnodelist);

    int32_t media_type; /* store stream type for combined sink */

    pa_module *rtp_module;
    pa_module *alsa_source;
    pa_module *alsa_sink1;
    pa_module *alsa_sink2;
    pa_module *default1_alsa_sink;
    pa_module *default2_alsa_sink;
    pa_module *headphone_sink;
    pa_module *preprocess_module;
    pa_module *postprocess_module;

    char *destAddress;
    int connectionPort;
    char *connectionType;
    char *deviceName;
    const char *callback_deviceName;

    int external_soundcard_number;
    int external_device_number;
    int a2dpSource;

    pa_module *combined;
    char *scenario;
    pa_module *btDiscoverModule;
    bool IsBluetoothEnabled;
    bool IsHeadphoneConnected;
    int externalSoundCardNumber[DISPLAY_SINK_COUNT];
    char address[BLUETOOTH_MAC_ADDRESS_SIZE];
    char physicalSinkBT[BLUETOOTH_SINK_NAME_SIZE];
    char btProfile[BLUETOOTH_PROFILE_SIZE];

    int PreprocessSourceId;
    int PreprocessSinkId;

    bool IsEqualizerEnabled;
    bool IsBassBoostEnabled;

    bool IsDRCEnabled;
    int enabledEffectsCount;

    bool isPcmOutputConnected;
    bool isPcmHeadphoneConnected;

    multipleDeviceInfo *usbOutputDeviceInfo;
    multipleDeviceInfo *usbInputDeviceInfo;

    multipleDeviceInfo *internalOutputDeviceInfo;
    multipleDeviceInfo *internalInputDeviceInfo;

    pa_palm_policy *palm_policy;
};

/* Every message to audiod is exactly SIZE_MESG_TO_AUDIOD bytes: audiod reads
 * fixed-size records. The largest reply has to fit in one. */
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paReplyToRoutingSet) <= SIZE_MESG_TO_AUDIOD,
               "routing reply does not fit in a message to audiod");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paReplyToModuleSet) <= SIZE_MESG_TO_AUDIOD,
               "module reply does not fit in a message to audiod");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paReplyToPolicySet) <= SIZE_MESG_TO_AUDIOD,
               "policy reply does not fit in a message to audiod");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paReplyToAudiod) <= SIZE_MESG_TO_AUDIOD,
               "callback reply does not fit in a message to audiod");

/* Send one message to audiod: the header, the payload, and zeroes up to the
 * fixed record size, so that nothing beyond the payload - heap or stack - goes
 * out with it. Returns false if there is no connection or the send failed. */
static bool send_to_audiod(struct userdata *u, uint32_t msgType, uint32_t msgID, const void *payload, size_t len)
{
    char buf[SIZE_MESG_TO_AUDIOD];
    struct paudiodMsgHdr hdr;

    pa_assert(u);
    pa_assert(sizeof(hdr) + len <= sizeof(buf));

    if (!u->connectionactive || u->newsockfd == -1)
        return false;

    memset(buf, 0, sizeof(buf));
    memset(&hdr, 0, sizeof(hdr));
    hdr.msgType = msgType;
    hdr.msgTmp = 0x01;
    hdr.msgVer = 1;
    hdr.msgLen = sizeof(hdr);
    hdr.msgID = msgID;
    memcpy(buf, &hdr, sizeof(hdr));
    if (len)
        memcpy(buf + sizeof(hdr), payload, len);

    if (send(u->newsockfd, buf, sizeof(buf), MSG_NOSIGNAL) != (ssize_t)sizeof(buf))
    {
        pa_log("send to audiod failed: %s", strerror(errno));
        return false;
    }
    return true;
}

/* Copy a string into a fixed-size field: always terminated, truncated if it
 * has to be, and a missing string copies as an empty one. */
static void copy_field(char *dst, size_t size, const char *src)
{
    pa_assert(dst);
    pa_assert(size > 0);
    pa_strlcpy(dst, src ? src : "", size);
}

/* _mapPercentToPulseRamp is indexed by a volume table and a volume in
 * percent, both of which come from audiod. */
static bool volume_in_range(int volumetable, int volume)
{
    if (volumetable < 0 || volumetable >= (int)PA_ELEMENTSOF(_mapPercentToPulseRamp) ||
        volume < MIN_VOLUME || volume > MAX_VOLUME)
    {
        pa_log_warn("volume %d in table %d is out of range", volume, volumetable);
        return false;
    }
    return true;
}

static bool virtual_source_output_move_inputdevice(int virtualsourceid, char *inputdevice, struct userdata *u);

static bool virtual_source_set_mute(int sourceid, int mute, struct userdata *u);

static bool virtual_source_input_set_volume(int sourceId, int volumetoset, int volumetable, struct userdata *u);


static void virtual_source_input_index_set_volume(int sourceId, int index, int volumetoset, int volumetable, struct userdata *u);

static bool virtual_sink_input_move_outputdevice(int virtualsinkid, char *outputdevice, struct userdata *u);

static bool virtual_sink_input_set_volume(int sinkid, int volumetoset, int volumetable, struct userdata *u);

static bool virtual_sink_input_index_set_volume(int sinkid, int index, int volumetoset, int volumetable, struct userdata *u);

static bool virtual_sink_input_set_ramp_volume(int sinkid, int volumetoset, int volumetable, struct userdata *u);

static bool virtual_sink_input_set_mute(int sinkid, bool mute, struct userdata *u);

static bool sink_set_master_mute(const char *outputdevice, bool mute, struct userdata *u);

static bool sink_set_master_volume(const char *outputdevice, int volume, struct userdata *u);

static int sink_suspend_request(struct userdata *u);

static int update_sample_spec(struct userdata *u, int rate);

static void parse_message(char *msgbuf, int bufsize, struct userdata *u);

static void handle_io_event_socket(pa_mainloop_api *ea, pa_io_event *e,
                                   int fd, pa_io_event_flags_t events, void *userdata);

static void handle_io_event_connection(pa_mainloop_api *ea, pa_io_event *e,
                                       int fd, pa_io_event_flags_t events, void *userdata);

static pa_hook_result_t route_sink_input_new_hook_callback(pa_core *c, pa_sink_input_new_data *data,
                                                           struct userdata *u);

static pa_hook_result_t route_sink_input_fixate_hook_callback(pa_core *c, pa_sink_input_new_data *data,
                                                              struct userdata *u);

static pa_hook_result_t route_sink_input_put_hook_callback(pa_core *c, pa_sink_input *data,
                                                           struct userdata *u);

static pa_hook_result_t route_source_output_new_hook_callback(pa_core *c, pa_source_output_new_data *data,
                                                              struct userdata *u);

static pa_hook_result_t route_source_output_fixate_hook_callback(pa_core *c, pa_source_output_new_data *data,
                                                                 struct userdata *u);

static pa_hook_result_t route_source_output_put_hook_callback(pa_core *c, pa_source_output *so,
                                                              struct userdata *u);

static pa_hook_result_t route_sink_input_unlink_hook_callback(pa_core *c, pa_sink_input *data,
                                                              struct userdata *u);

static pa_hook_result_t route_sink_input_state_changed_hook_callback(pa_core *c, pa_sink_input *data,
                                                                     struct userdata *u);
static pa_hook_result_t module_unload_subscription_callback(pa_core *c, pa_module *m, struct userdata *u);

// hook call back to inform output and input device module loading status to audiod
static pa_hook_result_t module_load_subscription_callback(pa_core *c, pa_module *m, struct userdata *u);

/* Hook callback for combined sink routing(sink input move,sink put & unlink) */
static pa_hook_result_t route_sink_input_move_finish_cb(pa_core *c, pa_sink_input *data, struct userdata *u);

static pa_hook_result_t route_source_output_move_finish_cb(pa_core *c, pa_source_output *data, struct userdata *u);

static pa_hook_result_t route_sink_unlink_cb(pa_core *c, pa_sink *sink, struct userdata *u);

static pa_hook_result_t sink_load_subscription_callback(pa_core *c, pa_sink_new_data *data, struct userdata *u);

static pa_hook_result_t source_load_subscription_callback(pa_core *c, pa_source_new_data *data, struct userdata *u);

static pa_hook_result_t route_sink_unlink_post_cb(pa_core *c, pa_sink *sink, struct userdata *u);

static pa_hook_result_t route_source_unlink_post_cb(pa_core *c, pa_source *source, struct userdata *u);

static pa_hook_result_t route_source_output_state_changed_hook_callback(pa_core *c, pa_source_output *so,
                                                                        struct userdata *u);

static pa_hook_result_t route_source_output_unlink_hook_callback(pa_core *c, pa_source_output *data,
                                                                 struct userdata *u);





static bool set_source_inputdevice(struct userdata *u, char *inputdevice, int sourceId);

static bool set_sink_outputdevice(struct userdata *u, char *outputdevice, int sinkid);

static bool set_sink_outputdevice_on_range(struct userdata *u, char *outputdevice, int startsinkid, int endsinkid);

static bool set_source_inputdevice_on_range(struct userdata *u, char *inputdevice, int startsourceid, int endsourceid);

static bool set_default_sink_routing(struct userdata *u, int startsinkid, int endsinkid);

static bool set_default_source_routing(struct userdata *u, int startsourceid, int endsourceid);

static bool detect_usb_device(struct userdata *u, bool isOutput, int cardNumber, int deviceNumber, bool status);

/* The module's entry points. An in-tree build declares them in a generated
 * symdef header; this one is built on its own. */
const char *pa__get_author(void);
const char *pa__get_description(void);
const char *pa__get_version(void);
bool pa__load_once(void);
const char *pa__get_usage(void);
int pa__init(pa_module *m);
void pa__done(pa_module *m);

PA_MODULE_AUTHOR("Palm, Inc.");
PA_MODULE_DESCRIPTION("Implements policy, communication with external app is a socket at /tmp/palmaudio");
PA_MODULE_VERSION(PULSEAUDIO_VERSION);
PA_MODULE_LOAD_ONCE(true);
PA_MODULE_USAGE("No parameters for this module");

/* audiod says how many devices of a kind to keep slots for. Far above anything
 * real, low enough that a bad record cannot make pa_xnew() abort the daemon. */
#define MAX_DEVICE_COUNT 64

/* Empty a device slot. Its ALSA module, if any, is the caller's to unload. */
static void device_slot_reset(deviceInfo *d)
{
    d->cardNumber = -1;
    d->deviceNumber = -1;
    d->alsaModule = NULL;
    free(d->cardNameDetail); /* from snd_card_get_name(), which mallocs it */
    d->cardNameDetail = NULL;
}

/* Free a device list and everything it owns, at module unload. The ALSA
 * modules are the core's to unload by then, so they are only forgotten. */
static void multiple_device_info_free(multipleDeviceInfo **pmdi)
{
    multipleDeviceInfo *mdi = *pmdi;

    if (!mdi)
        return;
    for (int i = 0; mdi->deviceList && i < mdi->maxDeviceCount; i++)
        device_slot_reset(mdi->deviceList + i);
    pa_xfree(mdi->deviceList);
    pa_xfree(mdi->baseName);
    pa_xfree(mdi);
    *pmdi = NULL;
}

/* Unload what the slots of a device list hold and free the list. */
static void device_list_free(multipleDeviceInfo *mdi)
{
    if (!mdi->deviceList)
        return;
    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *d = mdi->deviceList + i;
        if (d->alsaModule)
            pa_module_unload(d->alsaModule, true);
        device_slot_reset(d);
    }
    pa_xfree(mdi->deviceList);
    mdi->deviceList = NULL;
    mdi->maxDeviceCount = 0;
}

static bool initialise_internal_card(struct userdata *u, int maxDeviceCount, int isOutput)
{
    pa_assert(u);
    pa_assert(u->internalInputDeviceInfo);
    pa_assert(u->internalOutputDeviceInfo);

    pa_log_info("%s: is output:%d, max device count:%d", __FUNCTION__, isOutput, maxDeviceCount);

    multipleDeviceInfo *mdi = isOutput ? u->internalOutputDeviceInfo : u->internalInputDeviceInfo;

    if (maxDeviceCount <= 0 || maxDeviceCount > MAX_DEVICE_COUNT)
    {
        pa_log_warn("Invalid max device count(%d)", maxDeviceCount);
        return false;
    }

    /* audiod sends this again whenever it restarts: start from a clean list
     * rather than leaking the old one along with the modules it holds */
    device_list_free(mdi);
    mdi->maxDeviceCount = maxDeviceCount;
    mdi->deviceList = pa_xnew0(deviceInfo, mdi->maxDeviceCount);
    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        deviceList->index = i;
        device_slot_reset(deviceList);
    }
    return true;
}

static bool init_multiple_usb_device_info(struct userdata *u, bool isOutput, int maxDeviceCount, char *baseName)
{
    pa_assert(u);
    pa_assert(u->usbOutputDeviceInfo);
    pa_assert(u->usbInputDeviceInfo);
    pa_assert(baseName);
    pa_log_info("%s: is output:%d, max device count:%d base name:%s", __FUNCTION__, isOutput, maxDeviceCount, baseName);

    multipleDeviceInfo *mdi = isOutput ? u->usbOutputDeviceInfo : u->usbInputDeviceInfo;

    if (maxDeviceCount <= 0 || maxDeviceCount > MAX_DEVICE_COUNT)
    {
        pa_log_warn("Invalid max device count(%d)", maxDeviceCount);
        return false;
    }
    /* each device is named <baseName><index> in a SINK_NAME_LENGTH field */
    if (snprintf(NULL, 0, "%s%d", baseName, maxDeviceCount - 1) >= SINK_NAME_LENGTH)
    {
        pa_log_warn("Device base name %s is too long", baseName);
        return false;
    }

    // if already memory is allocated, free memory
    if (mdi->deviceList)
    {
        for (int i = 0; i < mdi->maxDeviceCount; i++)
        {
            // Unload the previously connected device.
            deviceInfo *deviceList = (mdi->deviceList + i);
            pa_log_debug("%s, index:%d, cardNumber:%d, deviceNumber:%d, alsaModule:%d", __FUNCTION__, deviceList->index, deviceList->cardNumber, deviceList->deviceNumber, deviceList->alsaModule ? 1 : 0);
            if (deviceList->alsaModule)
            {
                detect_usb_device(u, isOutput, deviceList->cardNumber, deviceList->deviceNumber, false);
            }
        }
        device_list_free(mdi);
    }
    if (mdi->baseName)
    {
        pa_xfree(mdi->baseName);
        mdi->baseName = NULL;
    }
    mdi->maxDeviceCount = maxDeviceCount;

    // calculate memory size
    int baseNameSize = strlen(baseName) * sizeof(char) + 1;
    pa_log_debug("%s, memory size for base name:%d, memory size for device list:%zu", __FUNCTION__, baseNameSize, (sizeof(deviceInfo) * (size_t)mdi->maxDeviceCount));

    // allocate memory
    mdi->baseName = (char *)pa_xmalloc0(baseNameSize);
    mdi->deviceList = pa_xnew0(deviceInfo, mdi->maxDeviceCount);

    // initialize member variable
    memcpy(mdi->baseName, baseName, (size_t)baseNameSize);
    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        deviceList->index = i;
        deviceList->cardNumber = -1;
        deviceList->deviceNumber = -1;
        deviceList->alsaModule = NULL;
    }
    return true;
}

static int get_usb_device_index(multipleDeviceInfo *mdi, int cardNumber, int deviceNumber)
{
    pa_assert(mdi);
    pa_assert(mdi->deviceList);

    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        if (deviceList->alsaModule != NULL &&
            deviceList->cardNumber == cardNumber && deviceList->deviceNumber == deviceNumber)
        {
            pa_log_debug("%s, return index(%d) for cardNumber(%d), deviceNumber(%d)", __FUNCTION__, deviceList->index, cardNumber, deviceNumber);
            return deviceList->index;
        }
    }

    pa_log_debug("%s, There is no usb device index for cardNumber(%d), deviceNumber(%d)", __FUNCTION__, cardNumber, deviceNumber);
    return -1;
}

static int get_new_usb_device_index(multipleDeviceInfo *mdi, int cardNumber, int deviceNumber)
{
    pa_assert(mdi);
    pa_assert(mdi->deviceList);

    if (get_usb_device_index(mdi, cardNumber, deviceNumber) != -1)
    {
        pa_log_warn("%s, device is already loaded for cardNumber(%d), deviceNumber(%d)", __FUNCTION__, cardNumber, deviceNumber);
        return -1;
    }

    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        if (deviceList->alsaModule == NULL)
        {
            pa_log_debug("%s, return index(%d) for cardNumber(%d), deviceNumber(%d)", __FUNCTION__, deviceList->index, cardNumber, deviceNumber);
            return deviceList->index;
        }
    }

    pa_log_warn("%s, could not find available usb device index", __FUNCTION__);
    return -1;
}

static bool check_multiple_usb_device_info_initialization(multipleDeviceInfo *mdi)
{
    pa_assert(mdi);

    if (mdi->baseName == NULL || mdi->deviceList == NULL || mdi->maxDeviceCount <= 0)
        return false;
    else
        return true;
    return true;
}

static void print_device_info(bool isOutput, multipleDeviceInfo *mdi)
{
    pa_assert(mdi);

    if (!check_multiple_usb_device_info_initialization(mdi))
    {
        pa_log_warn("%s, Haven't initialized the usb device yet", __FUNCTION__);
        return;
    }

    pa_log_debug("%s, usb %s device, baseName:%s", __FUNCTION__, isOutput ? "output" : "input", mdi->baseName);
    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        pa_log_debug("%s, index:%d, cardNumber:%d, deviceNumber:%d, alsaModule:%d", __FUNCTION__, deviceList->index, deviceList->cardNumber, deviceList->deviceNumber, deviceList->alsaModule ? 1 : 0);

        if (deviceList->cardNumber != -1)
            pa_log_debug("cardName : %s card details : %s", deviceList->cardName, deviceList->cardNameDetail);
    }
}

static bool detect_usb_device(struct userdata *u, bool isOutput, int cardNumber, int deviceNumber, bool status)
{
    pa_assert(u);
    pa_assert(u->usbOutputDeviceInfo);
    pa_assert(u->usbInputDeviceInfo);

    multipleDeviceInfo *mdi = isOutput ? u->usbOutputDeviceInfo : u->usbInputDeviceInfo;

    if (!check_multiple_usb_device_info_initialization(mdi))
    {
        pa_log_warn("%s, Haven't initialized the usb device yet", __FUNCTION__);
        return false;
    }

    int index;
    if (status)
    {
        index = get_new_usb_device_index(mdi, cardNumber, deviceNumber);
        if (index == -1 || index >= mdi->maxDeviceCount)
        {
            pa_log_warn("%s, There is no avaliable usb device index", __FUNCTION__);
            return false;
        }

        char *args = NULL;
        args = pa_sprintf_malloc("device=hw:%d,%d mmap=0 %s=%s%d fragment_size=4096 tsched=0",
                                 cardNumber, deviceNumber, isOutput ? "sink_name" : "source_name", mdi->baseName, (index));

        if (args)
        {
            pa_log_debug("%s, args:%s", __FUNCTION__, args);
            deviceInfo *deviceList = mdi->deviceList + index;
            // load module
            pa_module_load(&deviceList->alsaModule, u->core, isOutput ? MODULE_ALSA_SINK_NAME : MODULE_ALSA_SOURCE_NAME, args);
            if (deviceList->alsaModule)
            {
                deviceList->cardNumber = cardNumber;
                deviceList->deviceNumber = deviceNumber;
                snprintf(deviceList->cardName, sizeof(deviceList->cardName), "%s%d", mdi->baseName, (index));
                free(deviceList->cardNameDetail);
                deviceList->cardNameDetail = NULL;
                snd_card_get_name(cardNumber, &(deviceList->cardNameDetail));
                pa_log_info("USB %s:%s", deviceList->cardName, deviceList->cardNameDetail);
                pa_log_info("%s, usb device module is loaded with index %u", __FUNCTION__, deviceList->alsaModule->index);
            }
            pa_xfree(args);
        }
        else
        {
            pa_log_warn("%s, Failed to load the device due to an internal error", __FUNCTION__);
        }
    }
    else
    {
        index = get_usb_device_index(mdi, cardNumber, deviceNumber);
        if (index == -1 || index >= mdi->maxDeviceCount)
        {
            pa_log_warn("%s, There is no connected usb device for cardNumber(%d), deviceNumber(%d)", __FUNCTION__, cardNumber, deviceNumber);
            return false;
        }

        deviceInfo *deviceList = mdi->deviceList + index;
        if (deviceList->alsaModule)
        {
            // unload module
            pa_module_unload(deviceList->alsaModule, TRUE);
            device_slot_reset(deviceList);
            pa_log_info("%s, usb device module is unloaded", __FUNCTION__);
        }
    }
    print_device_info(isOutput, mdi);
    return true;
}

static const char *get_device_name_from_detail(const char *deviceDetail, struct userdata *u, bool isOutput)
{
    pa_log_debug("%s, deviceDetail:%s stream", __FUNCTION__, deviceDetail);
    if (isOutput)
    {
        for (int i = 0; i < u->internalOutputDeviceInfo->maxDeviceCount; i++)
        {
            if (u->internalOutputDeviceInfo->deviceList[i].cardNumber != -1)
            {
                pa_log_info("logging %s %s", u->internalOutputDeviceInfo->deviceList[i].cardName, u->internalOutputDeviceInfo->deviceList[i].cardNameDetail);
                if (!strncmp(u->internalOutputDeviceInfo->deviceList[i].cardNameDetail, deviceDetail, strlen(deviceDetail)))
                {
                    pa_log_info("Match found for device in internal : %s : %s", deviceDetail, u->internalOutputDeviceInfo->deviceList[i].cardName);
                    return u->internalOutputDeviceInfo->deviceList[i].cardName;
                }
            }
        }
        for (int i = 0; i < u->usbOutputDeviceInfo->maxDeviceCount; i++)
        {
            if (u->usbOutputDeviceInfo->deviceList[i].cardNumber != -1)
            {
                if (!strncmp(u->usbOutputDeviceInfo->deviceList[i].cardNameDetail, deviceDetail, strlen(deviceDetail)))
                {
                    pa_log_info("Match found for device in USB: %s : %s", deviceDetail, u->usbOutputDeviceInfo->deviceList[i].cardName);
                    return u->usbOutputDeviceInfo->deviceList[i].cardName;
                }
            }
        }
    }
    else
    {
        for (int i = 0; i < u->internalInputDeviceInfo->maxDeviceCount; i++)
        {
            pa_log_info("logging %s %s", u->internalInputDeviceInfo->deviceList[i].cardName, u->internalInputDeviceInfo->deviceList[i].cardNameDetail);
            if (u->internalInputDeviceInfo->deviceList[i].cardNumber != -1)
            {
                if (!strncmp(u->internalInputDeviceInfo->deviceList[i].cardNameDetail, deviceDetail, strlen(deviceDetail)))
                {
                    pa_log_info("Match found for device in internal : %s : %s", deviceDetail, u->internalInputDeviceInfo->deviceList[i].cardName);
                    return u->internalInputDeviceInfo->deviceList[i].cardName;
                }
            }
        }
        for (int i = 0; i < u->usbInputDeviceInfo->maxDeviceCount; i++)
        {
            pa_log_info("logging %s %s", u->usbInputDeviceInfo->deviceList[i].cardName, u->usbInputDeviceInfo->deviceList[i].cardNameDetail);
            if (u->usbInputDeviceInfo->deviceList[i].cardNumber != -1)
            {
                if (!strncmp(u->usbInputDeviceInfo->deviceList[i].cardNameDetail, deviceDetail, strlen(deviceDetail)))
                {
                    pa_log_info("Match found for device in USB: %s : %s", deviceDetail, u->usbInputDeviceInfo->deviceList[i].cardName);
                    return u->usbInputDeviceInfo->deviceList[i].cardName;
                }
            }
        }
    }
    pa_log_info("couldnt find matching devices");
    return deviceDetail;
}

static void find_and_load_usb_devices(struct userdata *u, const char *deviceName, snd_pcm_stream_t stream)
{
    pa_assert(u);
    pa_assert(deviceName);
    pa_log_debug("%s, deviceName:%s stream:%d", __FUNCTION__, deviceName, (int)stream);

    snd_ctl_t *handle;
    int card, err, dev;
    snd_ctl_card_info_t *info;
    snd_pcm_info_t *pcminfo;
    snd_ctl_card_info_alloca(&info);
    snd_pcm_info_alloca(&pcminfo);

    card = -1;
    if (snd_card_next(&card) < 0 || card < 0)
    {
        pa_log_warn("no soundcards found...");
        return;
    }

    while (card >= 0)
    {
        char name[32];
        snprintf(name, sizeof(name), "hw:%d", card);
        if ((err = snd_ctl_open(&handle, name, 0)) < 0)
        {
            pa_log_warn("control open (%i): %s", card, snd_strerror(err));
            goto next_card;
        }
        if ((err = snd_ctl_card_info(handle, info)) < 0)
        {
            pa_log_warn("control hardware info (%i): %s", card, snd_strerror(err));
            snd_ctl_close(handle);
            goto next_card;
        }
        dev = -1;
        while (1)
        {
            /* on failure dev is left as it was: stop rather than ask for the
             * same device again forever */
            if (snd_ctl_pcm_next_device(handle, &dev) < 0)
            {
                pa_log_warn("snd_ctl_pcm_next_device failed on card %d", card);
                break;
            }
            if (dev < 0)
                break;
            snd_pcm_info_set_device(pcminfo, dev);
            snd_pcm_info_set_subdevice(pcminfo, 0);
            snd_pcm_info_set_stream(pcminfo, stream);
            if ((err = snd_ctl_pcm_info(handle, pcminfo)) < 0)
            {
                if (err != -ENOENT)
                    pa_log_warn("control digital audio info (%i): %s", card, snd_strerror(err));
                continue;
            }
            pa_log_debug("%s, card %i: %s [%s], device %i: %s [%s]",
                         __FUNCTION__,
                         card, snd_ctl_card_info_get_id(info), snd_ctl_card_info_get_name(info),
                         dev,
                         snd_pcm_info_get_id(pcminfo),
                         snd_pcm_info_get_name(pcminfo));

            // check if device name and load device
            if (!strcmp(deviceName, snd_pcm_info_get_name(pcminfo)))
            {
                pa_log_debug("%s, found %s pcm device(%s), carNumber:%d, deviceNumber:%d", __FUNCTION__, snd_pcm_stream_name(stream), deviceName, card, dev);
                if (stream == SND_PCM_STREAM_PLAYBACK)
                {
                    detect_usb_device(u, true, card, dev, true);
                }
                else if (stream == SND_PCM_STREAM_CAPTURE)
                {
                    detect_usb_device(u, false, card, dev, true);
                }
            }
        }
        snd_ctl_close(handle);
    next_card:
        if (snd_card_next(&card) < 0)
        {
            pa_log_warn("snd_card_next");
            break;
        }
    }
}

static void initialize_usb_devices(struct userdata *u, bool isOutput)
{
    pa_assert(u);
    pa_log_info("%s, is output:%d", __FUNCTION__, isOutput);

    find_and_load_usb_devices(u, "USB Audio", isOutput ? SND_PCM_STREAM_PLAYBACK : SND_PCM_STREAM_CAPTURE);
}

static void check_and_remove_usb_device_module(struct userdata *u, bool isOutput, pa_module *m)
{
    pa_assert(u);
    pa_assert(u->usbOutputDeviceInfo);
    pa_assert(u->usbInputDeviceInfo);
    pa_log_info("%s, module name:%s, is output:%d", __FUNCTION__, m->name, isOutput);

    multipleDeviceInfo *mdi = isOutput ? u->usbOutputDeviceInfo : u->usbInputDeviceInfo;
    if (!check_multiple_usb_device_info_initialization(mdi))
    {
        pa_log_warn("%s, Haven't initialized the usb device yet", __FUNCTION__);
        return;
    }

    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        if (deviceList->alsaModule == m)
        {
            pa_log_info("%s, found unloaded module, remove unloaded module in usb %s device list", __FUNCTION__, isOutput ? "output" : "input");
            deviceList->alsaModule = NULL;
            deviceList->cardNumber = -1;
            deviceList->deviceNumber = -1;
            print_device_info(isOutput, mdi);
        }
    }
}

/* When headset is connected to Rpi,audio will be routed to headset.
Once it is removed, audio will be routed to HDMI.
*/

// To set the physical source(input device) for single virtual source
static bool set_source_inputdevice(struct userdata *u, char *inputdevice, int sourceId)
{
    pa_log("set_source_inputdevice: inputdevice:%s sourceId:%d", inputdevice, sourceId);
    pa_source *destsource = NULL;
    struct sourceoutputnode *thelistitem = NULL;
    if (sourceId >= 0 && sourceId < eVirtualSource_Count)
    {
        copy_field(u->source_mapping_table[sourceId].inputdevice, sizeof(u->source_mapping_table[sourceId].inputdevice), inputdevice);
        pa_log_info("set_source_inputdevice setting inputdevice:%s for source:%s",
                    u->source_mapping_table[sourceId].inputdevice, u->source_mapping_table[sourceId].virtualsourcename);
        destsource =
            pa_namereg_get(u->core, u->source_mapping_table[sourceId].inputdevice, PA_NAMEREG_SOURCE);
        if (destsource == NULL)
        {
            /* moving to no source asserts and takes the daemon down */
            pa_log_info("set_source_inputdevice destsource is null");
            return false;
        }
        /* walk the list of siource-inputs we know about and update their sources */
        struct sourceoutputnode *thelistitem_next;
        PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sourceoutputnodelist)
        {
            if ((int)thelistitem->virtualsourceid == sourceId && !pa_source_output_is_passthrough(thelistitem->sourceoutput))
            {
                pa_log_info("moving the virtual source%d to physical source%s:", sourceId, u->source_mapping_table[sourceId].inputdevice);
                pa_source_output_move_to(thelistitem->sourceoutput, destsource, true);
            }
        }
    }
    else
    {
        pa_log_warn("set_source_inputdevice: sourceId is not valid");
        return false;
    }
    return true;
}

// To set the physical sink(output device) for single sink
static bool set_sink_outputdevice(struct userdata *u, char *outputdevice, int sinkid)
{
    pa_log("set_sink_outputdevice: outputdevice:%s sinkid:%d", outputdevice, sinkid);
    pa_sink *destsink = NULL;
    struct sinkinputnode *thelistitem = NULL;
    if (sinkid >= 0 && sinkid < eVirtualSink_Count)
    {
        copy_field(u->sink_mapping_table[sinkid].outputdevice, sizeof(u->sink_mapping_table[sinkid].outputdevice), outputdevice);
        pa_log_info("set_sink_outputdevice setting outputdevice:%s for sink:%s",
                    u->sink_mapping_table[sinkid].outputdevice, u->sink_mapping_table[sinkid].virtualsinkname);
        destsink =
            pa_namereg_get(u->core, u->sink_mapping_table[sinkid].outputdevice, PA_NAMEREG_SINK);
        if (destsink == NULL)
        {
            /* moving to no sink asserts and takes the daemon down */
            pa_log_info("set_sink_outputdevice destsink is null");
            return false;
        }
        /* walk the list of sink-inputs we know about and update their sinks */
        struct sinkinputnode *thelistitem_next;
        PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sinkinputnodelist)
        {
            if ((int)thelistitem->virtualsinkid == sinkid && !pa_sink_input_is_passthrough(thelistitem->sinkinput))
            {
                pa_log_info("moving the virtual sink%d to physical sink%s:", sinkid, u->sink_mapping_table[sinkid].outputdevice);
                pa_sink_input_move_to(thelistitem->sinkinput, destsink, true);
            }
        }
    }
    else
    {
        pa_log_warn("set_sink_outputdevice: sinkid is not valid");
        return false;
    }
    return true;
}

// To set the physical sink(output device) for the streams with range based, consider this while adding any new virtual sink to the list
static bool set_sink_outputdevice_on_range(struct userdata *u, char *outputdevice, int startsinkid, int endsinkid)
{
    pa_log("set_sink_outputdevice_on_range: outputdevice:%s startsinkid:%d, endsinkid:%d", outputdevice, startsinkid, endsinkid);
    pa_sink *destsink = NULL;
    struct sinkinputnode *thelistitem = NULL;
    if (startsinkid >= 0 && endsinkid < eVirtualSink_Count)
    {
        for (int i = startsinkid; i <= endsinkid; i++)
        {
            copy_field(u->sink_mapping_table[i].outputdevice, sizeof(u->sink_mapping_table[i].outputdevice), outputdevice);
            destsink = pa_namereg_get(u->core, u->sink_mapping_table[i].outputdevice, PA_NAMEREG_SINK);

            if (destsink == NULL)
            {
                pa_log_info("set_sink_outputdevice_on_range destsink is null");
                return false;
            }
            /* walk the list of sink-inputs we know about and update their sinks */
            struct sinkinputnode *thelistitem_next;
            PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sinkinputnodelist)
            {
                if ((int)thelistitem->virtualsinkid == i && !pa_sink_input_is_passthrough(thelistitem->sinkinput) && !(thelistitem->bypassRouting))
                {
                    pa_log_info("moving the virtual sink%d to physical sink%s:", i, u->sink_mapping_table[i].outputdevice);
                    pa_sink_input_move_to(thelistitem->sinkinput, destsink, true);
                }
            }
        }
    }
    else
    {
        pa_log_warn("set_sink_outputdevice_on_range: start and end sink are not in range");
        return false;
    }
    return true;
}

// To set the physical source(input device) for the sources with range based, consider this while adding any new virtual source to the list
static bool set_source_inputdevice_on_range(struct userdata *u, char *inputdevice, int startsourceid, int endsourceid)
{
    pa_log("set_source_inputdevice_on_range: inputdevice:%s startsourceid:%d, endsourceid:%d", inputdevice, startsourceid, endsourceid);
    pa_source *destsource = NULL;
    struct sourceoutputnode *thelistitem = NULL;
    if (startsourceid >= 0 && endsourceid < eVirtualSource_Count)
    {
        for (int i = startsourceid; i <= endsourceid; i++)
        {
            copy_field(u->source_mapping_table[i].inputdevice, sizeof(u->source_mapping_table[i].inputdevice), inputdevice);
            destsource =
                pa_namereg_get(u->core, u->source_mapping_table[i].inputdevice, PA_NAMEREG_SOURCE);
            if (destsource == NULL)
            {
                pa_log_info("set_default_source_routing destsource is null");
                return false;
            }
            /* walk the list of siource-inputs we know about and update their sources */
            struct sourceoutputnode *thelistitem_next;
            PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sourceoutputnodelist)
            {
                if ((int)thelistitem->virtualsourceid == i && !pa_source_output_is_passthrough(thelistitem->sourceoutput) && !(thelistitem->bypassRouting))
                {
                    pa_log_info("moving the virtual source%d to physical source%s:", i, u->source_mapping_table[i].inputdevice);
                    pa_source_output_move_to(thelistitem->sourceoutput, destsource, true);
                }
            }
        }
    }
    else
    {
        pa_log_warn("set_source_inputdevice_on_range: start and end source are not in range");
        return false;
    }
    return true;
}

static bool set_default_sink_routing(struct userdata *u, int startsinkid, int endsinkid)
{
    pa_log("set_default_sink_routing: startsinkid:%d, endsinkid:%d", startsinkid, endsinkid);
    pa_sink *destsink = NULL;
    struct sinkinputnode *thelistitem = NULL;
    if (startsinkid >= 0 && endsinkid < eVirtualSink_Count)
    {
        for (int i = startsinkid; i <= endsinkid; i++)
        {
            copy_field(u->sink_mapping_table[i].outputdevice, sizeof(u->sink_mapping_table[i].outputdevice), u->sink_mapping_table[i].virtualsinkname);
            destsink =
                pa_namereg_get(u->core, u->sink_mapping_table[i].outputdevice, PA_NAMEREG_SINK);
            if (destsink == NULL)
            {
                pa_log_info("set_default_sink_routing destsink is null");
                continue;
            }
            /* walk the list of sink-inputs we know about and update their sinks */
            struct sinkinputnode *thelistitem_next;
            PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sinkinputnodelist)
            {
                if ((int)thelistitem->virtualsinkid == i && !pa_sink_input_is_passthrough(thelistitem->sinkinput))
                {
                    pa_log_info("moving the virtual sink:%d to physical sink:%s:", i, u->sink_mapping_table[i].outputdevice);
                    pa_sink_input_move_to(thelistitem->sinkinput, destsink, true);
                }
            }
        }
    }
    else
    {
        pa_log_warn("set_default_sink_routing: start and end sink are not in range");
        return false;
    }
    return true;
}

static bool set_default_source_routing(struct userdata *u, int startsourceid, int endsourceid)
{
    pa_log("set_default_source_routing: startsourceid:%d, endsourceid:%d", startsourceid, endsourceid);
    pa_source *destsource = NULL;
    struct sourceoutputnode *thelistitem = NULL;
    if (startsourceid >= 0 && endsourceid < eVirtualSource_Count)
    {
        for (int i = startsourceid; i <= endsourceid; i++)
        {
            copy_field(u->source_mapping_table[i].inputdevice, sizeof(u->source_mapping_table[i].inputdevice), u->source_mapping_table[i].virtualsourcename);
            destsource =
                pa_namereg_get(u->core, u->source_mapping_table[i].inputdevice, PA_NAMEREG_SOURCE);
            if (destsource == NULL)
            {
                pa_log_info("set_default_source_routing destsource is null");
                continue;
            }
            /* walk the list of siource-inputs we know about and update their sources */
            struct sourceoutputnode *thelistitem_next;
            PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sourceoutputnodelist)
            {
                if ((int)thelistitem->virtualsourceid == i && !pa_source_output_is_passthrough(thelistitem->sourceoutput))
                {
                    pa_log_info("moving the virtual source%d to physical source%s:", i, u->source_mapping_table[i].inputdevice);
                    pa_source_output_move_to(thelistitem->sourceoutput, destsource, true);
                }
            }
        }
    }
    else
    {
        pa_log_warn("set_default_source_routing: start and end source are not in range");
        return false;
    }
    return true;
}

static bool virtual_source_output_move_inputdevice(int virtualsourceid, char *inputdevice, struct userdata *u)
{
    pa_log_info("virtual_source_output_move_inputdevice for virtualsourceid = %d to inputdevice = %s",
                virtualsourceid, inputdevice);
    struct sourceoutputnode *thelistitem = NULL;
    pa_source *destsource = NULL;
    if (virtualsourceid >= 0 && virtualsourceid < eVirtualSource_Count)
    {
        copy_field(u->source_mapping_table[virtualsourceid].inputdevice, sizeof(u->source_mapping_table[virtualsourceid].inputdevice), inputdevice);
        pa_log_info("virtual_source_output_move_inputdevice name = %s", u->source_mapping_table[virtualsourceid].inputdevice);
        destsource =
            pa_namereg_get(u->core, u->source_mapping_table[virtualsourceid].inputdevice, PA_NAMEREG_SOURCE);
        if (destsource == NULL)
        {
            pa_log_info("virtual_source_output_move_inputdevice destsource is null");
            return false;
        }

        /* walk the list of source-inputs we know about and update their sources */
        struct sourceoutputnode *thelistitem_next;
        PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sourceoutputnodelist)
        {
            if (thelistitem->virtualsourceid == virtualsourceid)
            {
                pa_source_output_move_to(thelistitem->sourceoutput, destsource, true);
            }
        }
    }
    else
    {
        pa_log("virtual_source_input_set_physical_source: source ID out of range");
        return false;
    }
    return true;
}

/* set the mute for all source-outputs associated with a virtual source,
 * sourceid - virtual source on which to set mute
 * mute - 0 unmuted, 1 muted. */
static bool virtual_source_set_mute(int sourceid, int mute, struct userdata *u)
{
    pa_log_info("virtual_source_set_mute for sourceid:%d with mute:%d", sourceid, mute);
    struct sourceoutputnode *thelistitem = NULL;
    if (sourceid >= 0 && sourceid < eVirtualSource_Count)
    {
        for (thelistitem = u->sourceoutputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            pa_log_debug("[%s] Available sourceId:%d name:%s",
                         __func__, thelistitem->virtualsourceid, thelistitem->sourceoutput->source->name);
            if (thelistitem->virtualsourceid == sourceid)
            {
                pa_source_output_set_mute(thelistitem->sourceoutput, mute, TRUE);
                u->source_mapping_table[sourceid].ismuted = mute;
            }
        }
    }
    return true;
}

static bool virtual_sink_input_move_outputdevice(int virtualsinkid, char *outputdevice, struct userdata *u)
{
    pa_log_info("virtual_sink_input_move_outputdevice for virtualsinkid = %d to outputdevice = %s",
                virtualsinkid, outputdevice);
    struct sinkinputnode *thelistitem = NULL;
    pa_sink *destsink = NULL;
    if (virtualsinkid >= 0 && virtualsinkid < eVirtualSink_Count)
    {
        copy_field(u->sink_mapping_table[virtualsinkid].outputdevice, sizeof(u->sink_mapping_table[virtualsinkid].outputdevice), outputdevice);
        pa_log_info("virtual_sink_input_move_outputdevice name = %s", u->sink_mapping_table[virtualsinkid].outputdevice);
        destsink =
            pa_namereg_get(u->core, u->sink_mapping_table[virtualsinkid].outputdevice, PA_NAMEREG_SINK);
        if (destsink == NULL)
        {
            pa_log_info("virtual_sink_input_move_outputdevice  destsink is null");
            return false;
        }
        /* walk the list of sink-inputs we know about and update their sinks */
        struct sinkinputnode *thelistitem_next;
        PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sinkinputnodelist)
        {
            if ((int)thelistitem->virtualsinkid == virtualsinkid && !pa_sink_input_is_passthrough(thelistitem->sinkinput))
            {
                pa_log_info("moving the virtual sink%d to physical sink%s:", virtualsinkid, u->sink_mapping_table[virtualsinkid].outputdevice);
                pa_sink_input_move_to(thelistitem->sinkinput, destsink, true);
            }
        }
    }
    else
    {
        pa_log("virtual_sink_input_move_outputdevice: sink ID out of range");
        return false;
    }
    return true;
}

/* set the volume for all sink-inputs associated with a virtual sink,
 * sinkid - virtual sink on which to set volumes
 * volumetoset - 0..65535 gain setting for pulseaudio to use */

static bool virtual_sink_input_set_ramp_volume(int sinkid, int volumetoset, int volumetable, struct userdata *u)
{
    if (!volume_in_range(volumetable, volumetoset))
        return false;
    struct sinkinputnode *thelistitem = NULL;
    struct pa_cvolume cvolume;

    if (sinkid >= 0 && sinkid < eVirtualSink_Count)
    {
        /* set the default volume on new streams created on
         * this sink, update rules table for final ramped volume */

        /* walk the list of sink-inputs we know about and update their volume */
        for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            if (thelistitem->virtualsinkid == sinkid)
            {
                if (!pa_sink_input_is_passthrough(thelistitem->sinkinput))
                {
                    u->sink_mapping_table[sinkid].volumetable = volumetable;
                    pa_log_debug("volume we are setting is %u, %f db",
                                 pa_sw_volume_from_dB(_mapPercentToPulseRamp
                                                          [volumetable][volumetoset]),
                                 _mapPercentToPulseRamp[volumetable][volumetoset]);
                    pa_cvolume_set(&cvolume,
                                   thelistitem->sinkinput->sample_spec.channels,
                                   pa_sw_volume_from_dB(_mapPercentToPulseRamp[volumetable]
                                                                              [volumetoset]));

                    /* LG's fork ramps this (PALM_UP/DOWN_RAMP_MSEC); upstream
                     * PulseAudio has no ramped sink input volume, so it is set
                     * directly. */
                    if (thelistitem->sinkinput->volume_writable)
                        pa_sink_input_set_volume(thelistitem->sinkinput, &cvolume, TRUE, TRUE);
                    else
                        pa_log_info("volume not writable");
                }
            }
        }
        u->sink_mapping_table[sinkid].volume = volumetoset;
    }
    else
        pa_log("virtual_sink_input_set_volume: sink ID %d out of range", sinkid);
    return true;
}

/* set the volume for all sink-inputs associated with a virtual sink,
 * sinkid - virtual sink on which to set volumes
 * volumetoset - 0..65535 gain setting for pulseaudio to use */

static bool virtual_sink_input_set_volume(int sinkid, int volumetoset, int volumetable, struct userdata *u)
{
    if (!volume_in_range(volumetable, volumetoset))
        return false;
    struct sinkinputnode *thelistitem = NULL;
    struct pa_cvolume cvolume;

    if (sinkid >= 0 && sinkid < eVirtualSink_Count)
    {
        /* set the default volume on new streams created on
         * this sink, update rules table for final ramped volume */

        /* walk the list of sink-inputs we know about and update their volume */
        for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            if (thelistitem->virtualsinkid == sinkid)
            {
                if (!pa_sink_input_is_passthrough(thelistitem->sinkinput))
                {
                    u->sink_mapping_table[sinkid].volumetable = volumetable;
                    pa_log_debug("volume we are setting is %u, %f db",
                                 pa_sw_volume_from_dB(_mapPercentToPulseRamp
                                                          [volumetable][volumetoset]),
                                 _mapPercentToPulseRamp[volumetable][volumetoset]);
                    if (volumetoset)
                        pa_cvolume_set(&cvolume,
                                       thelistitem->sinkinput->sample_spec.channels,
                                       pa_sw_volume_from_dB(_mapPercentToPulseRamp[volumetable]
                                                                                  [volumetoset]));
                    else
                        pa_cvolume_set(&cvolume, thelistitem->sinkinput->sample_spec.channels, 0);
                    // pa_sink_input_set_volume_with_ramping(thelistitem->sinkinput, &cvolume, TRUE, TRUE, 5 * PA_USEC_PER_MSEC);
                    if (thelistitem->sinkinput->volume_writable)
                        pa_sink_input_set_volume(thelistitem->sinkinput, &cvolume, TRUE, TRUE);
                    else
                        pa_log_info("volume not writeable");
                }
            }
        }
        u->sink_mapping_table[sinkid].volume = volumetoset;
    }
    else
        pa_log("virtual_sink_input_set_volume: sink ID %d out of range", sinkid);
    return true;
}

static bool close_playback_by_sink_input(int sinkInputIndex, struct userdata *u)
{
    struct sinkinputnode *thelistitem = NULL;
    pa_client *client = NULL;
    pa_log_info("close_playback_by_sink_input close client associated with sinkinput index %d", sinkInputIndex);
    for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
    {
        if (thelistitem->sinkinputidx == sinkInputIndex)
        {
            client = thelistitem->sinkinput->client;
            break;
        }
    }
    /* Kill outside the walk: pa_client_kill() unlinks the client's streams
     * there and then, and route_sink_input_unlink_hook_callback() frees their
     * nodes - the one the loop would have read ->next from among them. */
    if (client)
        pa_client_kill(client);
    return true;
}

/* set volume based on the sink index */
static bool virtual_sink_input_index_set_volume(int sinkid, int index, int volumetoset, int volumetable, struct userdata *u)
{
    if (!volume_in_range(volumetable, volumetoset))
        return false;
    struct sinkinputnode *thelistitem = NULL;
    struct pa_cvolume cvolume;

    if (sinkid >= 0 && sinkid < eVirtualSink_Count)
    {
        /* set the default volume on new streams created on
         * this sink, update rules table for final ramped volume */
        /* walk the list of sink-inputs we know about and update their volume */
        for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            if ((thelistitem->virtualsinkid == sinkid) && (thelistitem->sinkinputidx == index))
            {
                if (!pa_sink_input_is_passthrough(thelistitem->sinkinput))
                {
                    u->sink_mapping_table[sinkid].volumetable = volumetable;
                    pa_log_debug("volume we are setting is %u, %f db for index %d",
                                 pa_sw_volume_from_dB(_mapPercentToPulseRamp
                                                          [volumetable][volumetoset]),
                                 _mapPercentToPulseRamp[volumetable][volumetoset], thelistitem->sinkinputidx);
                    if (volumetoset)
                        pa_cvolume_set(&cvolume,
                                       thelistitem->sinkinput->sample_spec.channels,
                                       pa_sw_volume_from_dB(_mapPercentToPulseRamp[volumetable]
                                                                                  [volumetoset]));
                    else
                        pa_cvolume_set(&cvolume, thelistitem->sinkinput->sample_spec.channels, 0);
                    if (thelistitem->sinkinput->volume_writable)
                        pa_sink_input_set_volume(thelistitem->sinkinput, &cvolume, TRUE, TRUE);
                    else
                        pa_log_info("volume not writeable");
                }
            }
        }
        u->sink_mapping_table[sinkid].volume = volumetoset;
    }
    else
        pa_log("virtual_sink_input_index_set_volume: sink ID %d out of range", sinkid);
    return true;
}

/* set volume based on the source index */
static void virtual_source_input_index_set_volume(int sourceId, int index, int volumetoset, int volumetable, struct userdata *u)
{
    if (!volume_in_range(volumetable, volumetoset))
        return;

    struct sourceoutputnode *thelistitem = NULL;
    struct pa_cvolume cvolume;
    pa_log_debug("[%s] Requested to set volume for sourceId:%d index:%d volume:%d", __func__, sourceId, index, volumetoset);
    if (sourceId >= 0 && sourceId < eVirtualSource_Count)
    {
        for (thelistitem = u->sourceoutputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            pa_log_debug("[%s] Available sourceId:%d name:%s",
                         __func__, thelistitem->virtualsourceid, thelistitem->sourceoutput->source->name);
            if ((thelistitem->virtualsourceid == sourceId) && (thelistitem->sourceoutputidx == index))
            {
                if (!pa_source_output_is_passthrough(thelistitem->sourceoutput))
                {
                    u->source_mapping_table[sourceId].volumetable = volumetable;
                    pa_log_debug("volume we are setting is %u, %f db",
                                 pa_sw_volume_from_dB(_mapPercentToPulseRamp
                                                          [volumetable][volumetoset]),
                                 _mapPercentToPulseRamp[volumetable][volumetoset]);
                    if (volumetoset)
                        pa_cvolume_set(&cvolume,
                                       thelistitem->sourceoutput->sample_spec.channels,
                                       pa_sw_volume_from_dB(_mapPercentToPulseRamp[volumetable]
                                                                                  [volumetoset]));
                    else
                        pa_cvolume_set(&cvolume, thelistitem->sourceoutput->sample_spec.channels, 0);
                    if (thelistitem->sourceoutput->volume_writable)
                        pa_source_output_set_volume(thelistitem->sourceoutput, &cvolume, TRUE, TRUE);
                    else
                        pa_log_info("volume not writeable");
                }
                else
                {
                    pa_log_debug("setting volume on Compress playback to %d", volumetoset);
                }
            }
        }
        u->source_mapping_table[sourceId].volume = volumetoset;
    }
    else
        pa_log("virtual_source_input_set_volume: sourceId ID %d out of range", sourceId);
}

static bool virtual_source_input_set_volume(int sourceId, int volumetoset, int volumetable, struct userdata *u)
{
    if (!volume_in_range(volumetable, volumetoset))
        return false;
    struct sourceoutputnode *thelistitem = NULL;
    struct pa_cvolume cvolume;
    pa_log_debug("[%s] Requested to set volume for sourceId:%d volume:%d", __func__, sourceId, volumetoset);
    if (sourceId >= 0 && sourceId < eVirtualSource_Count)
    {
        /* set the default volume on new streams created on
         * this sink, update rules table for final ramped volume */

        /* walk the list of sink-inputs we know about and update their volume */
        for (thelistitem = u->sourceoutputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            pa_log_debug("[%s] Available sourceId:%d name:%s",
                         __func__, thelistitem->virtualsourceid, thelistitem->sourceoutput->source->name);
            if (thelistitem->virtualsourceid == sourceId)
            {
                if (!pa_source_output_is_passthrough(thelistitem->sourceoutput))
                {
                    u->source_mapping_table[sourceId].volumetable = volumetable;
                    pa_log_debug("volume we are setting is %u, %f db",
                                 pa_sw_volume_from_dB(_mapPercentToPulseRamp
                                                          [volumetable][volumetoset]),
                                 _mapPercentToPulseRamp[volumetable][volumetoset]);
                    if (volumetoset)
                        pa_cvolume_set(&cvolume,
                                       thelistitem->sourceoutput->sample_spec.channels,
                                       pa_sw_volume_from_dB(_mapPercentToPulseRamp[volumetable]
                                                                                  [volumetoset]));
                    else
                        pa_cvolume_set(&cvolume, thelistitem->sourceoutput->sample_spec.channels, 0);
                    if (thelistitem->sourceoutput->volume_writable)
                        pa_source_output_set_volume(thelistitem->sourceoutput, &cvolume, TRUE, TRUE);
                    else
                        pa_log_info("volume not writeable");
                }
                else
                {
                    pa_log_debug("setting volume on Compress playback to %d", volumetoset);
                }
            }
        }
        u->source_mapping_table[sourceId].volume = volumetoset;
    }
    else
        pa_log("virtual_source_input_set_volume: sourceId ID %d out of range", sourceId);
    return true;
}

/* set the volume for all sink-inputs associated with a virtual sink,
 * sinkid - virtual sink on which to set volumes
 * setmuteval - (true : false) setting for pulseaudio to use */
static bool virtual_sink_input_set_mute(int sinkid, bool mute, struct userdata *u)
{
    pa_log_info("virtual_sink_input_set_mute for sinkid = %d mute = %d",
                sinkid, (int)mute);
    struct sinkinputnode *thelistitem = NULL;
    if (sinkid >= 0 && sinkid < eVirtualSink_Count)
    {
        for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
        {
            pa_log_debug("[%s] Available sinkId:%d name:%s : %d",
                         __func__, thelistitem->virtualsinkid, thelistitem->sinkinput->sink->name, thelistitem->sinkinputidx);
            if (thelistitem->virtualsinkid == sinkid)
            {
                pa_sink_input_set_mute(thelistitem->sinkinput, mute, TRUE);
                u->sink_mapping_table[sinkid].ismuted = mute;
            }
        }
    }
    else
        pa_log("virtual_sink_input_set_mute: sink ID %d out of range", sinkid);
    return true;
}

static bool sink_set_master_volume(const char *outputdevice, int volume, struct userdata *u)
{
    pa_assert(u);
    struct pa_cvolume cvolume;
    pa_sink *destSink = NULL;
    if (!volume_in_range(VOLUMETABLE, volume))
        return false;
    pa_log_debug("Inside sink_set_master_volume : volume requested is %d volume we are setting is %u, %f db",
                 volume,
                 pa_sw_volume_from_dB(_mapPercentToPulseRamp[VOLUMETABLE][volume]),
                 _mapPercentToPulseRamp[VOLUMETABLE][volume]);

    destSink = pa_namereg_get(u->core, outputdevice, PA_NAMEREG_SINK);
    if (NULL != destSink)
    {
        pa_cvolume_set(&cvolume, destSink->sample_spec.channels,
                       pa_sw_volume_from_dB(_mapPercentToPulseRamp[VOLUMETABLE][volume]));
        pa_sink_set_volume(destSink, &cvolume, true, false);
    }
    else
        pa_log_warn("sink_set_master_volume destSink is null");
    return true;
}

// static void source_set_master_volume(const char* source, int volume, struct userdata *u)
static bool source_set_master_volume(const char *source, int volume, struct userdata *u)
{
    pa_assert(u);
    struct pa_cvolume cvolume;
    pa_source *destSource = NULL;
    if (!volume_in_range(VOLUMETABLE, volume))
        return 0;
    pa_log_debug("Inside source_set_master_volume : volume requested is %d volume we are setting is %u, %f db",
                 volume,
                 pa_sw_volume_from_dB(_mapPercentToPulseRamp[VOLUMETABLE][volume]),
                 _mapPercentToPulseRamp[VOLUMETABLE][volume]);

    destSource = pa_namereg_get(u->core, source, PA_NAMEREG_SOURCE);
    if (NULL != destSource)
    {
        pa_cvolume_set(&cvolume, destSource->sample_spec.channels,
                       pa_sw_volume_from_dB(_mapPercentToPulseRamp[VOLUMETABLE][volume]));
        pa_source_set_volume(destSource, &cvolume, true, false);
    }
    else
        pa_log_warn("source_set_master_volume null");
    return true;
}

static bool sink_set_master_mute(const char *outputdevice, bool mute, struct userdata *u)
{
    pa_log_debug("Inside sink_set_master_mute with outputdevice %s and mute %d", outputdevice, mute);
    pa_assert(u);
    pa_sink *destSink = NULL;
    destSink = pa_namereg_get(u->core, outputdevice, PA_NAMEREG_SINK);
    if (NULL != destSink)
    {
        if (mute)
            pa_sink_set_mute(destSink, MUTE, SAVE);
        else
            pa_sink_set_mute(destSink, UNMUTE, SAVE);
    }
    else
        pa_log_warn("sink_set_master_mute destSink is null");
    return true;
}

static bool source_set_master_mute(const char *source, bool mute, struct userdata *u)
{
    pa_log_debug("Inside source_set_master_mute with source %s and mute %d", source, mute);
    pa_assert(u);
    pa_source *destSource = NULL;
    destSource = pa_namereg_get(u->core, source, PA_NAMEREG_SOURCE);
    if (NULL != destSource)
    {
        if (mute)
            pa_source_set_mute(destSource, MUTE, SAVE);
        else
            pa_source_set_mute(destSource, UNMUTE, SAVE);
    }
    else
        pa_log("Valid source is not present for source %s", source);
    return true;
}

static int sink_suspend_request(struct userdata *u)
{
    struct sinkinputnode *thesinklistitem = NULL;
    struct sourceoutputnode *thesourcelistitem = NULL;

    for (thesinklistitem = u->sinkinputnodelist; thesinklistitem != NULL; thesinklistitem = thesinklistitem->next)
    {
        if (thesinklistitem->sinkinput->state == PA_SINK_INPUT_RUNNING)
        {
            pa_log("%s: sink input (%d) is active and running, close and report error",
                   __FUNCTION__, thesinklistitem->virtualsinkid);
            break;
        }
    }

    pa_sink_suspend_all(u->core, true, PA_SUSPEND_IDLE);

    for (thesourcelistitem = u->sourceoutputnodelist; thesourcelistitem != NULL;
         thesourcelistitem = thesourcelistitem->next)
    {
        if (thesourcelistitem->sourceoutput->state == PA_SOURCE_OUTPUT_RUNNING)
        {
            pa_log("%s: source output (%d) is active and running, close and report error",
                   __FUNCTION__, thesourcelistitem->virtualsourceid);
            break;
        }
    }
    pa_source_suspend_all(u->core, true, PA_SUSPEND_IDLE);

    return 0;
}

static int update_sample_spec(struct userdata *u, int rate)
{
    (void)u; /* the body is disabled below */
    (void)rate;
#if 0
    pa_sink *sink;
    pa_source *source;

    uint32_t idx, i, need_unsuspend = 0;
    pa_sample_spec sample_spec;

    for (i = 0; i < ePhysicalSink_Count; i++) {
        for (sink = PA_SINK(pa_idxset_first(u->core->sinks, &idx)); sink;
             sink = PA_SINK(pa_idxset_next(u->core->sinks, &idx))) {
            if (strcmp(u->source_mapping_table[i].outputdevice, sink->name) == 0) {
                if (sink->update_sample_spec && sink->sample_spec.rate != rate) {
                    if (sink->state == PA_SINK_RUNNING) {
                        pa_log_info("Need to suspend then unsuspend device before changing sampling rate");
                        need_unsuspend = 1;
                    }
                    pa_sink_suspend(sink, 1, PA_SUSPEND_IDLE);

                    pa_log_info
                        ("update_sample_spec is available for sink %s, current sample_spec rate %d, changing to %d",
                         sink->name, sink->sample_spec.rate, rate);
                    sample_spec = sink->sample_spec;
                    sample_spec.rate = rate;
                    sink->update_sample_spec(sink, &sample_spec);

                    if (need_unsuspend == 1) {
                        pa_sink_suspend(sink, 0, PA_SUSPEND_IDLE);
                        need_unsuspend = 0;
                    }
                }
            }
        }
    }

    for (i = 0; i < ePhysicalSource_Count; i++) {
        for (source = PA_SOURCE(pa_idxset_first(u->core->sources, &idx));
             source; source = PA_SOURCE(pa_idxset_next(u->core->sources, &idx))) {
            if (strcmp(source_mapping_table[i].inputdevice, source->name) == 0) {
                if (source->update_sample_spec && source->sample_spec.rate != rate) {
                    if (source->state == PA_SOURCE_RUNNING) {
                        pa_log_info("Need to suspend then unsuspend device before changing sampling rate");
                        need_unsuspend = 1;
                    }
                    pa_source_suspend(source, 1, PA_SUSPEND_IDLE);

                    pa_log_info
                        ("update_sample_spec is available for source %s, current sample_spec rate %d, changing to %d",
                         source->name, source->sample_spec.rate, rate);
                    sample_spec = source->sample_spec;
                    sample_spec.rate = rate;
                    source->update_sample_spec(source, &sample_spec);

                    if (need_unsuspend == 1) {
                        pa_source_suspend(source, 0, PA_SUSPEND_IDLE);
                        need_unsuspend = 0;
                    }
                }
            }
        }
    }
#endif
    return 0;
}

static bool load_unicast_rtp_module(struct userdata *u)
{
    char *args = NULL;
    pa_assert(u != NULL);

    /* Request for Unicast RTP
     * Client Provides Destination IP & Port (Optional)
     * Load RTP Module
     * Send Error To AudioD in case RTP Load fails
     */
    if (strcmp(u->connectionType, "unicast") == 0)
    {
        pa_log("[rtp loading begins for Unicast RTP] [AudioD sent] port = %u ip_addr = %s",
               u->connectionPort, u->destAddress);
        if (u->connectionPort < 1 || u->connectionPort > 0xFFFF)
        {
            args = pa_sprintf_malloc("source=%s destination_ip=%s", "rtp.monitor", u->destAddress);
        }
        else
        {
            args = pa_sprintf_malloc("source=%s destination_ip=%s port=%d", "rtp.monitor",
                                     u->destAddress, u->connectionPort);
        }
        pa_module_load(&u->rtp_module, u->core, "module-rtp-send", args);
    }

    if (args)
        pa_xfree(args);

    if (!u->rtp_module)
    {
        pa_log("Error loading in module-rtp-send");
        /* No address to report: the module did not load. */
        struct paReplyToModuleSet moduleSet;
        memset(&moduleSet, 0, sizeof(moduleSet));
        moduleSet.Type = PAUDIOD_REPLY_MODULE_CAST_RTP;
        moduleSet.sink = 0;
        moduleSet.info = 1;
        moduleSet.port = 0;

        if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_MODULE, 0, &moduleSet, sizeof(moduleSet)))
            pa_log("Failed to send message to audiod ");
        else
            pa_log("Error in Loading RTP Module message sent to audiod");
        return false;
    }
    return true;
}

static bool load_multicast_rtp_module(struct userdata *u)
{
    char *args = NULL;
    pa_assert(u != NULL);

    /* Request for Multicast RTP
     * Client Provides Destination IP(Optional) & Port (Optional)
     * Load RTP Module
     * Send Error To AudioD in case RTP Load fails
     */
    if (strcmp(u->connectionType, "multicast") == 0)
    {
        pa_log("[rtp loading begins for Multicast RTP] [AudioD sent] port = %u ip_addr = %s",
               u->connectionPort, u->destAddress);
        if (u->connectionPort < 1 || u->connectionPort > 0xFFFF)
        {
            if (strcmp(u->destAddress, "default") == 0)
            {
                args = pa_sprintf_malloc("source=%s", "rtp.monitor");
            }
            else
            {
                args = pa_sprintf_malloc("source=%s destination_ip=%s", "rtp.monitor", u->destAddress);
            }
        }
        else
        {
            if (strcmp(u->destAddress, "default") == 0)
            {
                args = pa_sprintf_malloc("source=%s port=%d", "rtp.monitor", u->connectionPort);
            }
            else
            {
                args = pa_sprintf_malloc("source=%s destination_ip=%s port=%d", "rtp.monitor",
                                         u->destAddress, u->connectionPort);
            }
        }
        pa_module_load(&u->rtp_module, u->core, "module-rtp-send", args);
    }

    if (args)
        pa_xfree(args);

    if (!u->rtp_module)
    {
        pa_log("Error loading in module-rtp-send");
        /* No address to report: the module did not load. */
        struct paReplyToModuleSet moduleSet;
        memset(&moduleSet, 0, sizeof(moduleSet));
        moduleSet.Type = PAUDIOD_REPLY_MODULE_CAST_RTP;
        moduleSet.sink = 0;
        moduleSet.info = 1;
        moduleSet.port = 0;

        if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_MODULE, 0, &moduleSet, sizeof(moduleSet)))
            pa_log("Failed to send message to audiod ");
        else
            pa_log("Error in Loading RTP Module message sent to audiod");
        return false;
    }
    return true;
}

static bool unload_rtp_module(struct userdata *u)
{
    pa_assert(u);
    pa_assert(u->rtp_module);

    pa_module_unload(u->rtp_module, true);
    pa_log_info("module-rtp-sink unloaded");
    u->rtp_module = NULL;
    return true;
}

static void send_rtp_connection_data_to_audiod(const char *ip, const char *port, struct userdata *u)
{
    pa_assert(ip);
    pa_assert(port);
    int32_t port_value = 0;
    if (pa_atoi(port, &port_value) < 0 || port_value < 0 || port_value > 0xFFFF)
    {
        pa_log_warn("[send_rtp_connection_data_to_audiod] invalid port %s", port);
        port_value = 0;
    }

    pa_log("[send_rtp_connection_data_to_audiod] ip = %s port = %d", ip, (int)port_value);
    /* The connection that module-rtp-send set up: its address and port, as the
     * old text form of this message ("t 0 0 <ip> <port>") carried them. */
    struct paReplyToModuleSet moduleSet;
    memset(&moduleSet, 0, sizeof(moduleSet));
    moduleSet.Type = PAUDIOD_REPLY_MODULE_CAST_RTP;
    moduleSet.sink = 0;
    moduleSet.info = 0;
    moduleSet.port = (uint32_t)port_value;
    snprintf(moduleSet.ip, sizeof(moduleSet.ip), "%s", ip);

    if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_MODULE, 0, &moduleSet, sizeof(moduleSet)))
        pa_log("Failed to send message to audiod ");
    else
        pa_log("Message sent to audiod");
}

static bool load_Bluetooth_module(struct userdata *u)
{
    u->IsBluetoothEnabled = true;
    if (NULL == u->btDiscoverModule)
    {
        pa_module_load(&u->btDiscoverModule, u->core, "module-bluetooth-discover", NULL);
        char physicalSinkBT[BLUETOOTH_SINK_NAME_SIZE];
        char btSinkInit[BLUETOOTH_SINK_INIT_SIZE] = "bluez_sink.";
        btSinkInit[BLUETOOTH_SINK_INIT_SIZE - 1] = '\0';

        int index = 0;
        while (u->address[index] != '\0')
        {
            if (u->address[index] >= 'a' && u->address[index] <= 'z')
            {
                u->address[index] = u->address[index] - 32;
            }
            index++;
        }
        snprintf(physicalSinkBT, sizeof(physicalSinkBT), "%s%s", btSinkInit, u->address);
        for (size_t j = 0; j < strlen(physicalSinkBT); j++)
        {
            if (physicalSinkBT[j] == ':')
                physicalSinkBT[j] = '_';
        }
        copy_field(u->physicalSinkBT, sizeof(u->physicalSinkBT), physicalSinkBT);
        if (NULL == u->btDiscoverModule)
            pa_log_info("%s :module-bluetooth-discover loading failed", __FUNCTION__);
        else
            pa_log_info("%s :module-bluetooth-discover loaded", __FUNCTION__);
    }
    else
        pa_log_info("%s :module-bluetooth-discover already loaded", __FUNCTION__);
    return true;
}

static bool unload_BlueTooth_module(struct userdata *u)
{
    u->IsBluetoothEnabled = false;
    if (u->btDiscoverModule)
    {
        pa_log_info("%s : going to unload BT module ", __FUNCTION__);
        pa_module_unload(u->btDiscoverModule, TRUE);
    }
    else
    {
        pa_log_info("%s :module already unloaded", __FUNCTION__);
    }
    u->btDiscoverModule = NULL;
    return true;
}

static bool load_lineout_alsa_sink(struct userdata *u, int soundcardNo, int deviceNo, int status, int isOutput)
{
    (void)status;
    pa_assert(u);
    /* Request for lineout loading
     * Load Alsa Sink Module
     * Send Error To AudioD in case Alsa sink Load fails
     */
    pa_log("[alsa sink loading begins for lineout] [AudioD sent] cardno = %d playback device number = %d deviceName = %s",
           soundcardNo, deviceNo, u->deviceName);

    if (isOutput)
    {
        multipleDeviceInfo *mdi = u->internalOutputDeviceInfo;
        for (int i = 0; i < mdi->maxDeviceCount; i++)
        {
            deviceInfo *deviceList = (mdi->deviceList + i);
            if (deviceList->alsaModule == NULL)
            {
                char *args = NULL;
                deviceList->cardNumber = soundcardNo;
                deviceList->deviceNumber = deviceNo;
                deviceList->index = i;
                pa_log("%s, %s,%s,%zu", __FUNCTION__, u->deviceName, deviceList->cardName, strlen(u->deviceName));
                copy_field(deviceList->cardName, sizeof(deviceList->cardName), u->deviceName);
                free(deviceList->cardNameDetail);
                deviceList->cardNameDetail = NULL;
                snd_card_get_name(soundcardNo, &(deviceList->cardNameDetail));
                pa_log_info("lineout %s:%s - %zu", deviceList->cardName, deviceList->cardNameDetail, strlen(deviceList->cardName));

                args = pa_sprintf_malloc("device=hw:%d,%d mmap=0 sink_name=%s fragment_size=4096 tsched=0", soundcardNo, deviceNo, u->deviceName);
                pa_module_load(&deviceList->alsaModule, u->core, "module-alsa-sink", args);
                if (args)
                    pa_xfree(args);
                if (!deviceList->alsaModule)
                {
                    device_slot_reset(deviceList);
                    pa_log("Error loading in module-alsa-sink for %s", u->deviceName);
                    return false;
                }

                pa_log_info("module-alsa-sink loaded for %s", u->deviceName);
                pa_log_info("%d %d %d %s %s", deviceList->cardNumber, deviceList->deviceNumber, deviceList->index, deviceList->cardName, deviceList->cardNameDetail);

                u->isPcmOutputConnected = true;
                u->isPcmHeadphoneConnected = true;

                break;
            }
            else
            {
                pa_log("alsa module already loaeded");
            }
        }
    }
    else
    {
        multipleDeviceInfo *mdi = u->internalInputDeviceInfo;
        for (int i = 0; i < mdi->maxDeviceCount; i++)
        {
            deviceInfo *deviceList = (mdi->deviceList + i);
            if (deviceList->alsaModule == NULL)
            {
                char *args = NULL;
                deviceList->cardNumber = soundcardNo;
                deviceList->deviceNumber = deviceNo;
                copy_field(deviceList->cardName, sizeof(deviceList->cardName), u->deviceName);

                free(deviceList->cardNameDetail);
                deviceList->cardNameDetail = NULL;
                snd_card_get_name(soundcardNo, &(deviceList->cardNameDetail));
                pa_log_info("linein %s", deviceList->cardNameDetail);

                args = pa_sprintf_malloc("device=hw:%d,%d mmap=0 source_name=%s fragment_size=4096 tsched=0",
                                         soundcardNo, deviceNo, u->deviceName);
                pa_module_load(&deviceList->alsaModule, u->core, "module-alsa-source", args);
                if (args)
                    pa_xfree(args);
                if (!deviceList->alsaModule)
                {
                    device_slot_reset(deviceList);
                    pa_log("Error loading in module-alsa-source for %s", u->deviceName);
                    return false;
                }
                pa_log_info("module-alsa-source loaded for %s", u->deviceName);
                pa_log_info("%d %d %d %s %s", deviceList->cardNumber, deviceList->deviceNumber, deviceList->index, deviceList->cardName, deviceList->cardNameDetail);
                break;
            }
        }
    }
    multipleDeviceInfo *mdi = u->internalOutputDeviceInfo;
    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        if (deviceList->cardNumber != -1)
        {
            pa_log_debug("%d %d %d %s %s", deviceList->cardNumber, deviceList->deviceNumber, deviceList->index, deviceList->cardName, deviceList->cardNameDetail);
        }
    }
    mdi = u->internalInputDeviceInfo;
    for (int i = 0; i < mdi->maxDeviceCount; i++)
    {
        deviceInfo *deviceList = (mdi->deviceList + i);
        if (deviceList->cardNumber != -1)
        {
            pa_log_debug("%d %d %d %s %s", deviceList->cardNumber, deviceList->deviceNumber, deviceList->index, deviceList->cardName, deviceList->cardNameDetail);
        }
    }
    return true;
}

static bool set_audio_effect(struct userdata *u, const char* effect, int enabled)
{

    int sinkId = u->PreprocessSinkId;
    int sourceId = u->PreprocessSourceId;

    if (!u->preprocess_module && !enabled)
    {
        pa_log_debug("AudioEffect has same status as before");
        return false;
    }

    if(!u->preprocess_module && enabled) {

        char *args = NULL;
        args = pa_sprintf_malloc("sink_master=%s source_master=%s autoloaded=false",
                                u->sink_mapping_table[sinkId].outputdevice, u->source_mapping_table[sourceId].inputdevice);

        pa_log_info("load-module module-preprocess %s", args);
        pa_module_load(&u->preprocess_module, u->core, "module-preprocess-source", args);
        pa_xfree(args);

        struct sinkinputnode *thelistitem = NULL;
        pa_sink *destsink = NULL;

        destsink = pa_namereg_get(u->core, "preprocess_sink", PA_NAMEREG_SINK);

        struct sinkinputnode *thelistitem_next;

        PA_LLIST_FOREACH_SAFE(thelistitem, thelistitem_next, u->sinkinputnodelist) {
            const char *media_name = pa_proplist_gets(thelistitem->sinkinput->proplist, "media.name");
            if ((media_name) && (strcmp(media_name, "preprocess Stream") == 0)) {
                thelistitem->sinkinput->origin_sink = NULL;
                thelistitem->sinkinput->volume_writable = true;
                virtual_sink_input_index_set_volume(thelistitem->virtualsinkid, thelistitem->sinkinputidx, u->sink_mapping_table[thelistitem->virtualsinkid].volume, 0, u);
                if (u->sink_mapping_table[thelistitem->virtualsinkid].ismuted) {
                    pa_sink_input_set_mute(thelistitem->sinkinput, true, TRUE);
                }
            }
            else if (thelistitem->virtualsinkid == sinkId) {
                if(destsink != NULL) {
                    int si_volume = u->sink_mapping_table[thelistitem->virtualsinkid].volume;
                    virtual_sink_input_index_set_volume(thelistitem->virtualsinkid, thelistitem->sinkinputidx, 65535, 0, u);
                    u->sink_mapping_table[thelistitem->virtualsinkid].volume = si_volume;
                    pa_sink_input_set_mute(thelistitem->sinkinput, false, TRUE);
                    thelistitem->virtualsinkid *= -1;
                    pa_log_info("moving the sink input 'voice' idx %d to module-ecnr", thelistitem->sinkinputidx);
                    pa_sink_input_move_to(thelistitem->sinkinput, destsink, true);
                }
            }
        }

        struct sourceoutputnode *item = NULL;
        pa_source *destsource = NULL;

        destsource = pa_namereg_get(u->core, "preprocess-source", PA_NAMEREG_SOURCE);

        struct sourceoutputnode *item_next;

        PA_LLIST_FOREACH_SAFE(item, item_next, u->sourceoutputnodelist) {
            const char *media_name = pa_proplist_gets(item->sourceoutput->proplist, "media.name");
            if ((media_name) && (strcmp(media_name, "preprocess Stream") == 0)) {
                item->sourceoutput->destination_source = NULL;
                item->sourceoutput->volume_writable = true;
                virtual_source_input_index_set_volume(item->virtualsourceid, item->sourceoutputidx, u->source_mapping_table[item->virtualsourceid].volume, 0, u);
                if (u->source_mapping_table[item->virtualsourceid].ismuted) {
                    pa_source_output_set_mute(item->sourceoutput, true, TRUE);
                }
            }
            else if (item->virtualsourceid == sourceId) {
                if(destsource != NULL) {
                    int so_volume = u->source_mapping_table[item->virtualsourceid].volume;
                    virtual_source_input_index_set_volume(item->virtualsourceid, item->sourceoutputidx, 65535, 0, u);
                    u->source_mapping_table[item->virtualsourceid].volume = so_volume;
                    pa_source_output_set_mute(item->sourceoutput, false, TRUE);
                    item->virtualsourceid *= -1;
                    pa_log_info("moving the source output 'voice' idx %d to module-preprocess", item->sourceoutputidx);
                    pa_source_output_move_to(item->sourceoutput, destsource, true);
                }
            }
        }

    }

    char message[SIZE_MESG_TO_PULSE] = {0};
    snprintf(message, sizeof(message), "param %s %d", effect, enabled);
    pa_palm_policy_set_param_data_t *spd;
    spd = pa_xnew0(pa_palm_policy_set_param_data_t, 1);
    if (spd)
    {
        memcpy(spd->keyValuePairs, message, PALM_POLICY_SET_PARAM_DATA_SIZE);
        pa_log(" Sending Audio PreProcess msg %s", message);
        pa_palm_policy_hook_fire_set_parameters(u->palm_policy, spd);
        pa_xfree(spd);
    }

    if (enabled)
    {
        u->enabledEffectsCount++;
    }
    else
    {
        u->enabledEffectsCount--;
        if (!u->enabledEffectsCount)
        {
            //unlode module
            pa_assert(u);
            pa_assert(u->preprocess_module);

            struct sourceoutputnode *item = NULL;
            pa_source *destsource = NULL;

            destsource = pa_namereg_get(u->core, u->source_mapping_table[sourceId].inputdevice, PA_NAMEREG_SOURCE);

            struct sourceoutputnode *item_next;
            PA_LLIST_FOREACH_SAFE(item, item_next, u->sourceoutputnodelist)
            {
                const char *media_name = pa_proplist_gets(item->sourceoutput->proplist, "media.name");
                if ((media_name) && (strcmp(media_name, "preprocess Stream") == 0))
                {
                }
                else if (item->virtualsourceid == -1 * sourceId)
                {
                    item->virtualsourceid *= -1;
                    pa_log_info("moving the source output 'voice' idx %d to physical device", item->sourceoutputidx);
                    pa_source_output_move_to(item->sourceoutput, destsource, true);
                    pa_log_info("virtual_source_input_index_set_volume: %d %d %d", item->virtualsourceid, item->sourceoutputidx, u->source_mapping_table[item->virtualsourceid].volume);
                    virtual_source_input_index_set_volume(item->virtualsourceid, item->sourceoutputidx, u->source_mapping_table[item->virtualsourceid].volume, 0, u);
                    if (u->source_mapping_table[item->virtualsourceid].ismuted)
                    {
                        pa_source_output_set_mute(item->sourceoutput, true, TRUE);
                    }
                }
            }

            pa_log_info("unload-module module-preprocess");
            pa_module_unload(u->preprocess_module, true);
            pa_log_info("unload-module module-preprocess done");
            u->preprocess_module = NULL;
        }
    }

    return true;
}

static bool set_postprocess_effect(struct userdata *u, const char* effect, int enabled)
{
    pa_log_info("postprocess effect param: %s %d", effect, enabled);

    char *args = NULL;
    pa_assert(u);
    if (enabled && (u->postprocess_module == NULL)) {
        //  load audio post process module
        pa_module_load(&u->postprocess_module, u->core, "module-postprocess-sink", args);
        pa_log_info("load-module module-postprocess-sink done");
    }

    char message[SIZE_MESG_TO_PULSE] = {0};
    if (strcmp("dynamic_range_compressor", effect) == 0)
    {
        if (u->IsDRCEnabled == enabled)
        {
            pa_log_debug("AudioEffect has same status as before");
            return true;
        }
        u->IsDRCEnabled = enabled;
        snprintf(message, sizeof(message), "dynamic_range_compressor enable %d", enabled);
    }
    else if (strcmp("equalizer", effect) == 0)
    {
        if (u->IsEqualizerEnabled == enabled)
        {
            pa_log_debug("AudioEffect has same status as before");
            return true;
        }
        u->IsEqualizerEnabled = enabled;
        snprintf(message, sizeof(message), "equalizer enable %d", enabled);
    }
    else if (strcmp("bass_boost", effect) == 0)
    {
        if (u->IsBassBoostEnabled == enabled)
        {
            pa_log_debug("AudioEffect has same status as before");
            return true;
        }
        u->IsBassBoostEnabled = enabled;
        snprintf(message, sizeof(message), "bass_boost enable %d", enabled);
    }

    pa_palm_policy_set_param_data_t *spd;
    spd = pa_xnew0(pa_palm_policy_set_param_data_t, 1);
    if (spd)
    {
        memcpy(spd->keyValuePairs, message, PALM_POLICY_SET_PARAM_DATA_SIZE);
        pa_palm_policy_hook_fire_set_parameters(u->palm_policy, spd);
        pa_xfree(spd);
    }

    char *output = u->sink_mapping_table[eVirtualSink_First].outputdevice;
    set_sink_outputdevice_on_range(u, output, eVirtualSink_First, eVirtualSink_Last);
    pa_xfree(args);

    return true;
}

static bool set_equalizer_param(struct userdata *u, int preset, int band, int level)
{
    pa_log_info("equalizer effect set param: preset[%d] band[%d] level[%d]", preset, band, level);
    if (!u->IsEqualizerEnabled) return false;

    char message[SIZE_MESG_TO_PULSE] = {0};
    snprintf(message, sizeof(message), "equalizer param %d %d %d", preset, band, level);
    pa_palm_policy_set_param_data_t *spd;
    spd = pa_xnew0(pa_palm_policy_set_param_data_t, 1);
    if (spd)
    {
        memcpy(spd->keyValuePairs, message, PALM_POLICY_SET_PARAM_DATA_SIZE);
        pa_palm_policy_hook_fire_set_parameters(u->palm_policy, spd);
        pa_xfree(spd);
    }

    return true;
}

static void send_callback_to_audiod(int id, int returnVal, struct userdata *u)
{
    pa_log_info("%s : %d,%d", __FUNCTION__, id, returnVal);

    struct paReplyToAudiod vSink;
    memset(&vSink, 0, sizeof(vSink));
    vSink.id = id;
    vSink.returnValue = returnVal;

    if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_CALLBACK, 0, &vSink, sizeof(vSink)))
        pa_log("Failed to send message to audiod from pulseaudio");
    else
        pa_log("message sent to audiod from pulseaudio");
}

/* Parse a message sent from audiod and invoke
 * requested changes in pulseaudio
 */
/* Every message from audiod is one record of SIZE_MESG_TO_PULSE bytes, which
 * each of the request types has to fit in. */
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paRoutingSet) <= SIZE_MESG_TO_PULSE, "routing request too large");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paVolumeSet) <= SIZE_MESG_TO_PULSE, "volume request too large");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paDeviceSet) <= SIZE_MESG_TO_PULSE, "device request too large");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paModuleSet) <= SIZE_MESG_TO_PULSE, "module request too large");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paParamSet) <= SIZE_MESG_TO_PULSE, "setparam request too large");
_Static_assert(sizeof(struct paudiodMsgHdr) + sizeof(struct paEffectSet) <= SIZE_MESG_TO_PULSE, "effect request too large");

static void parse_message(char *msgbuf, int bufsize, struct userdata *u)
{
    int HdrLen = sizeof(struct paudiodMsgHdr);

    if (bufsize != SIZE_MESG_TO_PULSE)
    {
        pa_log("parse_message: message of %d bytes, expected %d", bufsize, SIZE_MESG_TO_PULSE);
        return;
    }
    pa_log_info("parse_message: paudio msg hdr length=%d", HdrLen);
    struct paudiodMsgHdr *msgHdr = (struct paudiodMsgHdr *)msgbuf;
    pa_log_info("parse_message: message type=%x, message ID=%x", msgHdr->msgType, msgHdr->msgID);

    int ret;

    switch (msgHdr->msgType)
    {
    // PAUDIOD_MSGTYPE_ROUTING
    case PAUDIOD_MSGTYPE_ROUTING:
    {
        struct paRoutingSet *SndHdr = (struct paRoutingSet *)(msgbuf + HdrLen);
        /* the strings come off the wire: never read past their fields */
        SndHdr->device[sizeof(SndHdr->device) - 1] = '\0';
        pa_log_info("received source routing for device:%s startID:%d,\
                endID:%d",
                    SndHdr->device, SndHdr->startID, SndHdr->endID);
        switch (SndHdr->Type)
        {
        case PAUDIOD_ROUTING_SINKINPUT_MOVE:
        {
            // 'd'
            ret = virtual_sink_input_move_outputdevice(SndHdr->startID, SndHdr->device, u);
            pa_log_info("parse_message: virtual_sink_input_move_outputdevice sink is %d, device %s",
                        SndHdr->startID, SndHdr->device);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SINKINPUT_RANGE:
        {
            // 'o'
            pa_log_info("received sink routing for outputdevice: %s startsinkid:%d, endsinkid:%d",
                        SndHdr->device, SndHdr->startID, SndHdr->endID);
            ret = set_sink_outputdevice_on_range(u, SndHdr->device, SndHdr->startID, SndHdr->endID);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SINKINPUT_DEFAULT:
        {
            // '2'
            pa_log_info("received default sink routing for startID:%d endID:%d",
                        SndHdr->startID, SndHdr->endID);
            ret = set_default_sink_routing(u, SndHdr->startID, SndHdr->endID);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SINKOUTPUT_DEVICE:
        {
            // 'q'
            ret = set_sink_outputdevice(u, SndHdr->device, SndHdr->id);
            pa_log_info("received sink routing for outputdevice: %s sinkid:%d",
                        SndHdr->device, SndHdr->id);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SOURCEOUTPUT_MOVE:
        {
            // 'e'
            /* redirect -  E <virtualsource> <physicalsink> when a source is in running state*/
            /* walk list of sink-inputs on this stream and set
             * their output sink */
            ret = virtual_source_output_move_inputdevice(SndHdr->startID, SndHdr->device, u);
            pa_log_info("parse_message: virtual_source_output_move_inputdevice source is %d and redirect to %s",
                        SndHdr->startID, SndHdr->device);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SOURCEOUTPUT_RANGE:
        {
            // 'a'
            pa_log_info("received source routing for inputdevice:%s startID:%d,\
                            inputdevice:%d",
                        SndHdr->device, SndHdr->startID, SndHdr->endID);
            ret = set_source_inputdevice_on_range(u, SndHdr->device, SndHdr->startID, SndHdr->endID);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SOURCEOUTPUT_DEFAULT:
        {
            // '3'
            pa_log_info("received default source routing for startID:%d endID:%d",
                        SndHdr->startID, SndHdr->endID);
            ret = set_default_source_routing(u, SndHdr->startID, SndHdr->endID);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_ROUTING_SOURCEINPUT_DEVICE:
        {
            // 'y'
            ret = set_source_inputdevice(u, SndHdr->device, SndHdr->id);
            pa_log_info("received Source routing for inputdevice: %s sourceId:%d",
                        SndHdr->device, SndHdr->id);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        default:
            pa_log_info("parse_message: unknown type received");
            break;
        }
    }
    break;
    // PAUDIOD_MSGTYPE_VOLUME
    case PAUDIOD_MSGTYPE_VOLUME:
    {
        struct paVolumeSet *SndHdr = (struct paVolumeSet *)(msgbuf + HdrLen);
        /* the strings come off the wire: never read past their fields */
        SndHdr->device[sizeof(SndHdr->device) - 1] = '\0';
        pa_log_info("parse_message: HDR Volume=%x", SndHdr->volume);
        pa_log_info("parse_message: Volume string name HDR=%d", SndHdr->Type);
        switch (SndHdr->Type)
        {
        case PAUDIOD_VOLUME_SINK_VOLUME:
        {
            // 'n'
            /* set volume on respective display */
            ret = sink_set_master_volume(SndHdr->device, SndHdr->volume, u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SINK_MUTE:
        {
            // 'k'
            /* Mute/Un-mute respective display */
            ret = sink_set_master_mute(SndHdr->device, SndHdr->mute, u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SINKINPUT_VOLUME:
        {
            // 'b'
            /* volume -  B  <value 0 : 65535> ramup/down */
            /* walk list of sink-inputs on this stream and set their volume */
            SndHdr->param2 = CLAMP_VOLUME_TABLE(SndHdr->param2);
            ret = virtual_sink_input_set_ramp_volume(SndHdr->param1, SndHdr->param2, !!SndHdr->param3, u);
            pa_log_info("parse_message: Fade command received, requested volume is %d, headphones:%d, fadeIn:%d", SndHdr->param1, SndHdr->param2, SndHdr->param3);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SINKINPUT_MUTE:
        {
            // 'm'
            /* mute -  M <sink> <state true : false> */
            /* set the mute status for the sink*/
            ret = virtual_sink_input_set_mute(SndHdr->id, SndHdr->mute, u);
            pa_log_info("parse_message: mute command received, sink is %d, muteStatus is %d",
                        SndHdr->id, (int)SndHdr->mute);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SINKINPUT_INDEX:
        {
            // '6'
            /* volume -  V <sink> <value 0 : 65535> */
            /* walk list of sink-inputs on this stream and set
             * their volume */
            SndHdr->param2 = CLAMP_VOLUME_TABLE(SndHdr->param2);
            ret = virtual_sink_input_index_set_volume(SndHdr->id, SndHdr->index, SndHdr->param1, SndHdr->param2, u);
            pa_log_info("parse_message: app volume command received, sink is %d sinkInputIndex:%d, requested volume is %d, headphones:%d",
                        SndHdr->id, SndHdr->index, SndHdr->param1, SndHdr->param2);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SINKINPUT_RAMP_VOLUME:
        {
            // 'r'
            /* ramp -  R <sink> <volume 0 : 100> */
            SndHdr->param2 = CLAMP_VOLUME_TABLE(SndHdr->param2);
            ret = virtual_sink_input_set_ramp_volume(SndHdr->id, SndHdr->param1, SndHdr->param2, u);
            pa_log_info("parse_message: ramp command received, sink is %d, volumetoset:%d, headphones:%d",
                        SndHdr->id, SndHdr->param1, SndHdr->param2);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SINKINPUT_SET_VOLUME:
        {
            // 'v'
            /* volume -  V <sink> <value 0 : 65535> */
            /* walk list of sink-inputs on this stream and set
             * their volume */
            SndHdr->param2 = CLAMP_VOLUME_TABLE(SndHdr->param2);
            ret = virtual_sink_input_set_volume(SndHdr->id, SndHdr->param1, SndHdr->param2, u);
            pa_log_info("parse_message: volume command received, sink is %d, requested volume is %d, headphones:%d",
                        SndHdr->id, SndHdr->param1, SndHdr->param2);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SOURCE_MUTE:
        {
            // '5'
            pa_log_info("muting phyiscal sink %s, mute value = %d", SndHdr->device, SndHdr->mute);
            ret = source_set_master_mute(SndHdr->device, SndHdr->mute, u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SOURCE_MIC_VOLUME:
        {
            // '8'
            /* set Mic volume on respective display */
            pa_log_info("setMicVolume inputDevice %s, Volume value = %d", SndHdr->device, SndHdr->volume);
            ret = source_set_master_volume(SndHdr->device, SndHdr->volume, u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_VOLUME_SOURCEOUTPUT_VOLUME:
        {
            // 'f'
            /* volume -  V <source> <value 0 : 65535> */
            SndHdr->param2 = CLAMP_VOLUME_TABLE(SndHdr->param2);
            if (!SndHdr->ramp)
            {
                ret = virtual_source_input_set_volume(SndHdr->id, SndHdr->param1, SndHdr->param2, u);
                send_callback_to_audiod(msgHdr->msgID, ret, u);
            }
            else
                pa_log_info("parse_message: ramp volume command received, sourceId is %d, requested volume is %d, volumetable:%d",
                            SndHdr->id, SndHdr->param1, SndHdr->param2);
        }
        break;
        case PAUDIOD_VOLUME_SOURCEOUTPUT_MUTE:
        {
            // 'h'
            /* mute source -  H <source> <mute 0 : 1> */
            pa_log_info("parse_message: source mute command received, source is %d, mute %d",
                        SndHdr->id, SndHdr->param1);
            ret = virtual_source_set_mute(SndHdr->id, SndHdr->param1, u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        default:
            pa_log_info("parse_message: unknown command received");
            break;
        }
    }
    break;
    // PAUDIOD_MSGTYPE_DEVICE
    case PAUDIOD_MSGTYPE_DEVICE:
    {
        struct paDeviceSet *SndHdr = (struct paDeviceSet *)(msgbuf + HdrLen);
        /* the strings come off the wire: never read past their fields */
        SndHdr->device[sizeof(SndHdr->device) - 1] = '\0';
        pa_log_info("received lineout loading cmd from Audiod  cardno:%d,deviceno:%d,status:%d,isoutput:%d,name: %s",
                    SndHdr->cardNo, SndHdr->deviceNo, SndHdr->status, SndHdr->isOutput, u->deviceName);
        switch (SndHdr->Type)
        {
        case PAUDIOD_DEVICE_LOAD_LINEOUT_ALSA_SINK:
        {
            // 'i'
            /* keep a copy: the record is reused for the next message */
            copy_field(u->deviceName, DEVICE_NAME_LENGTH, SndHdr->device);
            pa_log_info("received lineout loading cmd from Audiod  cardno:%d,deviceno:%d,status:%d,isoutput:%d,name: %s",
                        SndHdr->cardNo, SndHdr->deviceNo, SndHdr->status, SndHdr->isOutput, u->deviceName);
            if (1 == SndHdr->status)
            {
                ret = load_lineout_alsa_sink(u, SndHdr->cardNo, SndHdr->deviceNo, SndHdr->status, SndHdr->isOutput);
                send_callback_to_audiod(msgHdr->msgID, ret, u);
            }
        }
        break;
        case PAUDIOD_DEVICE_LOAD_INTERNAL_CARD:
        {
            // 'I'
            pa_log_info("received init internal cards");
            ret = initialise_internal_card(u, SndHdr->maxDeviceCnt, SndHdr->isOutput);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_DEVICE_LOAD_PLAYBACK_SINK:
        {
            // 'z'
            pa_log_info("received usb headset routing cmd from Audiod");
            ret = detect_usb_device(u, true, SndHdr->cardNo, SndHdr->deviceNo, SndHdr->status);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_DEVICE_LOAD_USB_MULTIPLE_DEVICE:
        {
            // 'Z'
            pa_log_info("received usb %s device info, deviceMaxCount:%d deviceBaseName:%s", SndHdr->isOutput ? "output" : "input", SndHdr->maxDeviceCnt, SndHdr->device);
            ret = init_multiple_usb_device_info(u, (bool)SndHdr->isOutput, SndHdr->maxDeviceCnt, SndHdr->device);
            if (ret)
                initialize_usb_devices(u, (bool)SndHdr->isOutput);

            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_DEVICE_LOAD_CAPTURE_SOURCE:
        {
            //'j'
            pa_log_info("received mic recording cmd from Audiod");
            ret = detect_usb_device(u, false, SndHdr->cardNo, SndHdr->deviceNo, SndHdr->status);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        default:
            pa_log_info("parse_message: unknown command received");
            break;
        }
    }
    break;
    // PAUDIOD_MSGTYPE_MODULE
    case PAUDIOD_MSGTYPE_MODULE:
    {
        struct paModuleSet *SndHdr = (struct paModuleSet *)(msgbuf + HdrLen);
        /* the strings come off the wire: never read past their fields */
        SndHdr->ip[sizeof(SndHdr->ip) - 1] = '\0';
        SndHdr->device[sizeof(SndHdr->device) - 1] = '\0';
        SndHdr->address[sizeof(SndHdr->address) - 1] = '\0';
        SndHdr->profile[sizeof(SndHdr->profile) - 1] = '\0';
        switch (SndHdr->Type)
        {
        case PAUDIOD_MODULE_RTP_LOAD:
        {
            // 'g'
            pa_log_info("received unload command for RTP module from AudioD");
            ret = unload_rtp_module(u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_MODULE_RTP_SET:
        {
            // 't'
            u->connectionPort = SndHdr->port;
            copy_field(u->connectionType, RTP_CONNECTION_TYPE_STRING_SIZE, SndHdr->device);
            pa_log_info("received rtp load cmd from Audiod");
            pa_log_info("parse_message:received command t FOR RTP module port = %d", u->connectionPort);
            if (strcmp(u->connectionType, "unicast") == 0)
            {
                ret = load_unicast_rtp_module(u);
                send_callback_to_audiod(msgHdr->msgID, ret, u);
            }
            else if (strcmp(u->connectionType, "multicast") == 0)
            {
                ret = load_multicast_rtp_module(u);
                send_callback_to_audiod(msgHdr->msgID, ret, u);
            }
        }
        break;
        case PAUDIOD_MODULE_BLUETOOTH_LOAD:
        {
            // 'l'
            /* walk list of sink-inputs on this stream and set
             * their output sink */
            copy_field(u->address, sizeof(u->address), SndHdr->address);
            pa_log_info("Bluetooth connected address %s", u->address);
            ret = load_Bluetooth_module(u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_MODULE_BLUETOOTH_A2DPSOURCE:
        {
            // 'O'
            u->a2dpSource = SndHdr->a2dpSource;
            pa_log_info("received command to set/reset A2DP source: %d", u->a2dpSource);
        }
        break;
        case PAUDIOD_MODULE_BLUETOOTH_UNLOAD:
        {
            // 'u'
            pa_log_info("received unload command for Bluetooth module from AudioD");
            ret = unload_BlueTooth_module(u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        default:
            pa_log_info("parse_message: unknown command received");
            break;
        }
    }
    break;
    // PAUDIOD_MSGTYPE_SETPARAM
    case PAUDIOD_MSGTYPE_SETPARAM:
    {
        struct paParamSet *SndHdr = (struct paParamSet *)(msgbuf + HdrLen);
        switch (SndHdr->Type)
        {
        case PAUDIOD_SETPARAM_SUSPEND:
        {
            // 's'
            /* suspend -  s */
            /* System is going to sleep, so suspend active modules */
            ret = sink_suspend_request(u);
            if (0 == ret)
                pa_log_info("suspend request failed: %s", strerror(errno));
            pa_log_info("parse_message: suspend command received");
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_SETPARAM_UPDATESAMPLERATE:
        {
            //'x'
            /* update sample rate -  x */
            ret = update_sample_spec(u, SndHdr->param1);
            if (0 == ret)
                pa_log_info("suspend request failed: %s", strerror(errno));
            pa_log_info("parse_message: update sample spec command received");
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_SETPARAM_CLOSE_PLAYBACK:
        {
            // '7'
            ret = close_playback_by_sink_input(SndHdr->ID, u);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        default:
            pa_log_info("parse_message: unknown command received");
            break;
        }
    }
    break;
    // PAUDIOD_MSGTYPE_EFFECT
    case PAUDIOD_MSGTYPE_EFFECT:
    {
        struct paEffectSet *SndHdr = (struct paEffectSet *)(msgbuf + HdrLen);
        switch (SndHdr->Type)
        {
        case PAUDIOD_EFFECT_SPEECH_ENHANCEMENT_LOAD:
        {
            // '4'
            uint32_t enabled;
            enabled = SndHdr->param[0];
            ret = set_audio_effect(u, "speech_enhancement", enabled);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_EFFECT_GAIN_CONTROL_LOAD:
        {
            uint32_t enabled;
            enabled = SndHdr->param[0];
            ret = set_audio_effect(u, "gain_control", enabled);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_EFFECT_BEAMFORMING_LOAD:
        {
            uint32_t enabled;
            enabled = SndHdr->param[0];
            ret = set_audio_effect(u, "beamforming", enabled);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_EFFECT_DYNAMIC_COMPRESSOR_LOAD:
        {
            uint32_t enabled;
            enabled = SndHdr->param[0];
            ret = set_postprocess_effect(u, "dynamic_range_compressor", enabled);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_EFFECT_EQUALIZER_LOAD:
        {
            uint32_t enabled;
            enabled = SndHdr->param[0];
            ret = set_postprocess_effect(u, "equalizer", enabled);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_EFFECT_EQUALIZER_SETPARAM:
        {
            uint32_t preset, band, level;
            preset = SndHdr->param[0];
            band = SndHdr->param[1];
            level = SndHdr->param[2];
            ret = set_equalizer_param(u, preset, band, level);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        case PAUDIOD_EFFECT_BASS_BOOST_LOAD:
        {
            uint32_t enabled;
            enabled = SndHdr->param[0];
            ret = set_postprocess_effect(u, "bass_boost", enabled);
            send_callback_to_audiod(msgHdr->msgID, ret, u);
        }
        break;
        default:
            pa_log_info("parse_message: unknown command received");
            break;
        }
    }
    break;
    default:
        pa_log_info("parse_message: unknown command received");
        break;
    }
}

/* pa_io_event_cb_t - IO event handler for socket,
 * this will create connections and assign an
 * appropriate IO event handler */
static void handle_io_event_socket(pa_mainloop_api *ea, pa_io_event *e, int fd, pa_io_event_flags_t events, void *userdata)
{
    (void)ea;
    (void)e;
    struct userdata *u = userdata;
    struct sockaddr_un peer;
    socklen_t peerlen = sizeof(peer);
    int sink;
    int source;

    pa_assert(u);
    pa_assert(fd == u->sockfd);

    if (events & PA_IO_EVENT_INPUT)
    {
        /* do we have a connection on our socket yet? */
        if (-1 == u->newsockfd)
        {
            if (-1 == (u->newsockfd = accept(u->sockfd, (struct sockaddr *)&peer, &peerlen)))
            {
                pa_log_info("handle_io_event_socket could not create new connection on socket:%s", strerror(errno));
            }
            else
            {
                /* create new io handler to deal with data send to this connection */
                u->connev =
                    u->core->mainloop->io_new(u->core->mainloop, u->newsockfd,
                                              PA_IO_EVENT_INPUT |
                                                  PA_IO_EVENT_HANGUP | PA_IO_EVENT_ERROR,
                                              handle_io_event_connection, u);
                u->connectionactive = true; /* flag that we have an active connection */
                u->rxlen = 0;

                // TODO to check if we need to send sink input index and corresponding app name with one more loop for both sink and source
                /* Tell audiod how many sink of each category is opened */
                for (sink = eVirtualSink_First; sink <= eVirtualSink_Last; sink++)
                {
                    if (u->audiod_sink_input_opened[sink] > 0)
                    {
                        struct paReplyToPolicySet policySet;
                        memset(&policySet, 0, sizeof(policySet));
                        policySet.Type = PAUDIOD_REPLY_POLICY_SINK_CATEGORY;
                        policySet.stream = sink;
                        policySet.count = u->audiod_sink_input_opened[sink];

                        if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)))
                            pa_log("handle_io_event_socket: send failed");
                        else
                            pa_log_info("handle_io_event_socket: stream count for sink %d (%d)",
                                        sink, u->audiod_sink_input_opened[sink]);
                    }
                }

                /* Tell audiod how many source of each category is opened */
                for (source = eVirtualSource_First; source <= eVirtualSource_Last; source++)
                {
                    if (u->audiod_source_output_opened[source] > 0)
                    {
                        struct paReplyToPolicySet policySet;
                        memset(&policySet, 0, sizeof(policySet));
                        policySet.Type = PAUDIOD_REPLY_POLICY_SOURCE_CATEGORY;
                        policySet.stream = source;
                        policySet.count = u->audiod_source_output_opened[source];

                        if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)))
                            pa_log("handle_io_event_socket: send failed");
                        else
                            pa_log_info("handle_io_event_socket: stream count for source %d (%d)",
                                        source, u->audiod_source_output_opened[source]);
                    }
                }
            }
        }
        else
            pa_log("handle_io_event_socket could not create new connection on socket");
    }
}

/* pa_io_event_cb_t - IO event handler for socket
 * connections.  We enforce a single connection to the
 * client (audiod).  This routine will createlisten on
 * the socket and parse and act upon messages sent to
 * the socket connection */
/* Tear down the connection to audiod, so that a new one can be accepted. */
static void close_connection(struct userdata *u)
{
    pa_assert(u);

    if (u->connev)
    {
        /* remove ourselves from the IO list on the main loop */
        u->core->mainloop->io_free(u->connev);
        u->connev = NULL;
    }
    if (u->newsockfd != -1)
    {
        if (-1 == shutdown(u->newsockfd, SHUT_RDWR))
            pa_log_info("Error in shutdown (%d):%s", errno, strerror(errno));
        if (-1 == close(u->newsockfd))
            pa_log_info("Error in close (%d):%s", errno, strerror(errno));
        u->newsockfd = -1;
    }
    u->connectionactive = false;
    u->rxlen = 0;
}

static void handle_io_event_connection(pa_mainloop_api *ea, pa_io_event *e, int fd, pa_io_event_flags_t events, void *userdata)
{
    (void)ea;
    (void)e;
    struct userdata *u = userdata;
    ssize_t bytesread;

    pa_assert(u);
    pa_assert(fd == u->newsockfd);

    if (events & PA_IO_EVENT_INPUT)
    {
        /* audiod sends fixed records of SIZE_MESG_TO_PULSE bytes over a stream
         * socket, which may deliver one in pieces: collect a whole record
         * before acting on it. */
        bytesread = recv(u->newsockfd, u->rxbuf + u->rxlen, sizeof(u->rxbuf) - u->rxlen, 0);
        if (bytesread < 0)
        {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)
                return;
            pa_log_info("handle_io_event_connection Error in recv (%d): %s ", errno, strerror(errno));
            close_connection(u);
            return;
        }
        if (bytesread == 0)
        {
            pa_log_info("handle_io_event_connection audiod closed the connection");
            close_connection(u);
            return;
        }
        u->rxlen += (size_t)bytesread;
        if (u->rxlen == sizeof(u->rxbuf))
        {
            u->rxlen = 0;
            parse_message(u->rxbuf, sizeof(u->rxbuf), u);
        }
    }
    if (events & (PA_IO_EVENT_HANGUP | PA_IO_EVENT_ERROR))
    {
        pa_log_info("handle_io_event_connection Socket is being closed (events 0x%x)", (unsigned)events);
        close_connection(u);
    }
}

static int make_socket(struct userdata *u)
{

    int path_len;

    u->sockfd = -1;
    u->newsockfd = -1; /* set to -1 to indicate no connection */

    /* create a socket for ipc with audiod the policy manager */
    path_len = strlen(PALMAUDIO_SOCK_NAME);
    _MEM_ZERO(u->name);
    u->name.sun_family = AF_UNIX;

    u->name.sun_path[0] = '\0'; /* this is what says "use abstract" */
    path_len++;                 /* Account for the extra nul byte added to the start of sun_path */

    if (path_len > _MAX_NAME_LEN)
    {
        pa_log("%s: socket name is too long", __FUNCTION__);
        goto fail;
    }

    strncpy(&u->name.sun_path[1], PALMAUDIO_SOCK_NAME, path_len);

    /* build the socket */
    if (-1 == (u->sockfd = socket(AF_UNIX, SOCK_STREAM, 0)))
    {
        pa_log("Error in socket (%d) ", errno);
        goto fail;
    }

    /* bind it to a name */
    if (-1 ==
        bind(u->sockfd, (struct sockaddr *)&(u->name), (socklen_t)(offsetof(struct sockaddr_un, sun_path) + (size_t)path_len)))
    {
        pa_log("Error in bind (%d) ", errno);
        goto fail;
    }

    if (-1 == listen(u->sockfd, 5))
    {
        pa_log("Error in listen (%d) ", errno);
        goto fail;
    }

    u->connectionactive = false;
    u->sockev = NULL;
    u->connev = NULL;

    /* register an IO event handler for the socket, deal
     * with new connections in this handler */
    u->sockev =
        u->core->mainloop->io_new(u->core->mainloop, u->sockfd,
                                  PA_IO_EVENT_INPUT | PA_IO_EVENT_HANGUP |
                                      PA_IO_EVENT_ERROR,
                                  handle_io_event_socket, u);
    return 0;
fail:
    if (u->sockfd != -1)
    {
        close(u->sockfd);
        u->sockfd = -1;
    }
    return -1;
}

static void connect_to_hooks(struct userdata *u)
{

    pa_assert(u);

    /* bit early than module-stream-restore:
     * module-stream-restore will try to set the sink if the stream doesn't comes with device set
     * Let palm-policy do the routing before module-stream-restore
     */
    u->sink_input_new_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_INPUT_NEW],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_sink_input_new_hook_callback, u);

    u->sink_input_fixate_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_INPUT_FIXATE],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_sink_input_fixate_hook_callback, u);

    u->source_output_new_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_OUTPUT_NEW],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_source_output_new_hook_callback, u);

    u->source_output_fixate_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_OUTPUT_FIXATE],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_source_output_fixate_hook_callback, u);

    u->source_output_put_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_OUTPUT_PUT],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_source_output_put_hook_callback, u);

    u->source_output_state_changed_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_OUTPUT_STATE_CHANGED], PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_source_output_state_changed_hook_callback, u);

    u->sink_input_put_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_INPUT_PUT],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_sink_input_put_hook_callback, u);

    u->sink_input_unlink_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_INPUT_UNLINK],
                        PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_sink_input_unlink_hook_callback, u);

    u->source_output_unlink_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_OUTPUT_UNLINK],
                        PA_HOOK_EARLY, (pa_hook_cb_t)route_source_output_unlink_hook_callback, u);

    u->sink_input_state_changed_hook_slot =
        pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_INPUT_STATE_CHANGED], PA_HOOK_EARLY - 10, (pa_hook_cb_t)route_sink_input_state_changed_hook_callback, u);

    u->sink_input_move_finish = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_INPUT_MOVE_FINISH], PA_HOOK_EARLY,
                                                (pa_hook_cb_t)route_sink_input_move_finish_cb, u);

    u->source_output_move_finish = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_OUTPUT_MOVE_FINISH], PA_HOOK_EARLY,
                                                   (pa_hook_cb_t)route_source_output_move_finish_cb, u);

    u->sink_unlink_post = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_UNLINK_POST], PA_HOOK_EARLY,
                                          (pa_hook_cb_t)route_sink_unlink_post_cb, u);

    u->source_unlink_post = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_UNLINK_POST], PA_HOOK_EARLY,
                                            (pa_hook_cb_t)route_source_unlink_post_cb, u);
    u->module_unload_hook_slot = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_MODULE_UNLINK],
                                                 PA_HOOK_EARLY, (pa_hook_cb_t)module_unload_subscription_callback, u);

    u->module_load_hook_slot = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_MODULE_NEW],
                                               PA_HOOK_EARLY, (pa_hook_cb_t)module_load_subscription_callback, u);

    u->sink_load_hook_slot = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_FIXATE],
                                             PA_HOOK_EARLY, (pa_hook_cb_t)sink_load_subscription_callback, u);

    u->source_load_hook_slot = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SOURCE_FIXATE],
                                               PA_HOOK_EARLY, (pa_hook_cb_t)source_load_subscription_callback, u);

    u->sink_unlink = pa_hook_connect(&u->core->hooks[PA_CORE_HOOK_SINK_UNLINK], PA_HOOK_EARLY,
                                     (pa_hook_cb_t)route_sink_unlink_cb, u);
}

static void disconnect_hooks(struct userdata *u)
{

    pa_assert(u);

    if (u->sink_input_new_hook_slot)
        pa_hook_slot_free(u->sink_input_new_hook_slot);

    if (u->sink_input_fixate_hook_slot)
        pa_hook_slot_free(u->sink_input_fixate_hook_slot);

    if (u->sink_input_put_hook_slot)
        pa_hook_slot_free(u->sink_input_put_hook_slot);

    if (u->sink_input_state_changed_hook_slot)
        pa_hook_slot_free(u->sink_input_state_changed_hook_slot);

    if (u->sink_input_unlink_hook_slot)
        pa_hook_slot_free(u->sink_input_unlink_hook_slot);

    if (u->source_output_new_hook_slot)
        pa_hook_slot_free(u->source_output_new_hook_slot);

    if (u->source_output_fixate_hook_slot)
        pa_hook_slot_free(u->source_output_fixate_hook_slot);

    if (u->source_output_put_hook_slot)
        pa_hook_slot_free(u->source_output_put_hook_slot);

    if (u->source_output_state_changed_hook_slot)
        pa_hook_slot_free(u->source_output_state_changed_hook_slot);

    if (u->source_output_unlink_hook_slot)
        pa_hook_slot_free(u->source_output_unlink_hook_slot);

    if (u->sink_input_move_finish)
        pa_hook_slot_free(u->sink_input_move_finish);

    if (u->source_output_move_finish)
        pa_hook_slot_free(u->source_output_move_finish);

    if (u->sink_new)
        pa_hook_slot_free(u->sink_new);

    if (u->sink_unlink)
        pa_hook_slot_free(u->sink_unlink);

    /* These were connected in pa__init() but never let go of: once the
     * module was unloaded, the next module, sink or source to come or go -
     * at shutdown, every one unloaded after this module - called back into
     * the freed userdata. */
    if (u->sink_unlink_post)
        pa_hook_slot_free(u->sink_unlink_post);

    if (u->source_unlink_post)
        pa_hook_slot_free(u->source_unlink_post);

    if (u->module_load_hook_slot)
        pa_hook_slot_free(u->module_load_hook_slot);

    if (u->module_unload_hook_slot)
        pa_hook_slot_free(u->module_unload_hook_slot);

    if (u->sink_load_hook_slot)
        pa_hook_slot_free(u->sink_load_hook_slot);

    if (u->source_load_hook_slot)
        pa_hook_slot_free(u->source_load_hook_slot);
}

/* entry point for the module*/
/* The module's state, set up as pa__init() needs it but without hooks, socket
 * or shared objects - so that tests can build one around a core of their own. */
static struct userdata *userdata_new(pa_core *core, pa_module *m)
{
    struct userdata *u = NULL;
    int i;

    u = pa_xnew0(struct userdata, 1);

    u->core = core;
    u->module = m;
    /* no socket and no connection yet, whatever pa__done() is reached from */
    u->sockfd = -1;
    u->newsockfd = -1;

    u->alsa_sink1 = NULL;
    u->alsa_sink2 = NULL;
    u->headphone_sink = NULL;

    PA_LLIST_HEAD_INIT(struct sinkinputnode, u->sinkinputnodelist);
    PA_LLIST_HEAD_INIT(struct sourceoutputnode, u->sourceoutputnodelist);

    /* copy the default sink mapping */
    for (i = 0; i < eVirtualSink_Count; i++)
    {
        u->sink_mapping_table[i].virtualsinkname = virtualsinkmap[i].virtualsinkname;
        u->sink_mapping_table[i].virtualsinkidentifier = virtualsinkmap[i].virtualsinkidentifier;
        copy_field(u->sink_mapping_table[i].outputdevice, sizeof(u->sink_mapping_table[i].outputdevice), virtualsinkmap[i].outputdevice);
        u->sink_mapping_table[i].volumetable = virtualsinkmap[i].volumetable;
        u->sink_mapping_table[i].volume = virtualsinkmap[i].volume;
        u->sink_mapping_table[i].ismuted = virtualsinkmap[i].ismuted;

        // Clear audiod sink opened count
        u->audiod_sink_input_opened[i] = 0;

        // find preprocess sink
        if (strcmp(u->sink_mapping_table[i].virtualsinkname, "voipcall") == 0)
        {
            u->PreprocessSinkId = i;
        }
    }
    u->n_sink_input_opened = 0;

    /* copy the default source mapping */
    for (i = 0; i < eVirtualSource_Count; i++)
    {
        u->source_mapping_table[i].virtualsourcename = virtualsourcemap[i].virtualsourcename;
        u->source_mapping_table[i].virtualsourceidentifier = virtualsourcemap[i].virtualsourceidentifier;
        copy_field(u->source_mapping_table[i].inputdevice, sizeof(u->source_mapping_table[i].inputdevice), virtualsourcemap[i].inputdevice);
        u->source_mapping_table[i].volume = virtualsourcemap[i].volume;
        u->source_mapping_table[i].ismuted = virtualsourcemap[i].ismuted;
        u->source_mapping_table[i].volumetable = virtualsourcemap[i].volumetable;

        // Clear audiod sink opened count
        u->audiod_source_output_opened[i] = 0;

        // find preprocess source
        if (strcmp(u->source_mapping_table[i].virtualsourcename, "webcall") == 0)
        {
            u->PreprocessSourceId = i;
        }
    }
    u->n_source_output_opened = 0;
    u->media_type = edefaultapp;

    u->rtp_module = NULL;
    u->alsa_source = NULL;
    u->default1_alsa_sink = NULL;
    u->default2_alsa_sink = NULL;
    u->postprocess_module = NULL;
    u->preprocess_module = NULL;
    u->destAddress = (char *)pa_xmalloc0(RTP_IP_ADDRESS_STRING_SIZE);
    u->connectionType = (char *)pa_xmalloc0(RTP_CONNECTION_TYPE_STRING_SIZE);
    u->connectionPort = 0;
    u->deviceName = (char *)pa_xmalloc0(DEVICE_NAME_LENGTH);
    /* only ever borrows the name of the sink or source being reported */
    u->callback_deviceName = NULL;

    u->btDiscoverModule = NULL;
    u->IsBluetoothEnabled = false;
    u->a2dpSource = 0;
    u->IsDRCEnabled = false;
    u->IsEqualizerEnabled = false;
    u->IsBassBoostEnabled = false;

    u->isPcmHeadphoneConnected = false;
    u->isPcmOutputConnected = false;

    // allocate memory and initialize
    u->usbOutputDeviceInfo = pa_xnew(multipleDeviceInfo, 1);
    u->usbOutputDeviceInfo->baseName = NULL;
    u->usbOutputDeviceInfo->maxDeviceCount = 0;
    u->usbOutputDeviceInfo->deviceList = NULL;
    u->usbInputDeviceInfo = pa_xnew(multipleDeviceInfo, 1);
    u->usbInputDeviceInfo->baseName = NULL;
    u->usbInputDeviceInfo->maxDeviceCount = 0;
    u->usbInputDeviceInfo->deviceList = NULL;

    u->internalInputDeviceInfo = pa_xnew(multipleDeviceInfo, 1);
    u->internalInputDeviceInfo->baseName = NULL;
    u->internalInputDeviceInfo->maxDeviceCount = 0;
    u->internalInputDeviceInfo->deviceList = NULL;
    u->internalOutputDeviceInfo = pa_xnew(multipleDeviceInfo, 1);
    u->internalOutputDeviceInfo->baseName = NULL;
    u->internalOutputDeviceInfo->maxDeviceCount = 0;
    u->internalOutputDeviceInfo->deviceList = NULL;

    return u;
}

int pa__init(pa_module *m)
{
    struct userdata *u;

    pa_assert(m);
    u = userdata_new(m->core, m);
    m->userdata = u;

    connect_to_hooks(u);

    if (!(u->palm_policy = pa_palm_policy_get(u->core)))
    {
        pa_log_info("pa_palm_policy_get fail");
        goto fail;
    }
    else
    {
        pa_log_info("pa_palm_policy_get success");
    }

    return make_socket(u);

fail:
    return -1;
}

/* callback for stream creation */
static pa_hook_result_t route_sink_input_new_hook_callback(pa_core *c, pa_sink_input_new_data *data,
                                                           struct userdata *u)
{
    int i, sink_index = edefaultapp;
    pa_sink *sink = NULL;
    pa_proplist *type = NULL;

    pa_assert(data);
    pa_assert(u);
    pa_assert(c);
    pa_log("route_sink_input_new_hook_callback");

    type = pa_proplist_new();

    const char *media_name = pa_proplist_gets(data->proplist, "media.name");
    // For handling void stream
    const char *pref_device = pa_proplist_gets(data->proplist, PA_PROP_PREFERRED_DEVICE);
    if (data->sink != NULL && pref_device != NULL)
    {
        const char *actualDeviceName = get_device_name_from_detail(pref_device, u, true);
        pa_proplist_sets(type, "media.type", data->sink->name);
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, type);
        if (actualDeviceName)
        {
            pa_log_info("Preferred device = %s, actualDeviceName=%s", (pref_device ? pref_device : "x"), actualDeviceName);
            pa_log_info("streamtype =%s", data->sink->name);

            sink = pa_namereg_get(c, actualDeviceName, PA_NAMEREG_SINK);

            if (sink && PA_SINK_IS_LINKED(sink->state))
            {
                // data->sink = sink;
                pa_log_info("Preferred device being set %s", sink->name);
                pa_sink_input_new_data_set_sink(data, sink, TRUE, FALSE);
            }
        }
        else
        {
            pa_log("No device found for preferred device : ERROR");
        }
    }
    else if (data->sink == NULL)
    {
        /* redirect everything to the default application stream */
        pa_log_info("THE DEFAULT DEVICE WAS USED TO CREATE THIS STREAM - PLEASE CATEGORIZE USING A VIRTUAL STREAM");
        const char *role = pa_proplist_gets(data->proplist, "media.role");
        const char *virtualsink;
        /* LG's fork meant streams of the music role for btstream (A2DP),
         * but that branch fell through into the default one, so they have
         * always gone to pdefaultapp - which is also what webOS's own
         * module-palm-policy did: its new-stream hook has no role routing
         * and puts every stream that comes without a sink on the default
         * application sink. audiod routes btstream no differently and only
         * keeps a volume policy for it. Only the phone role goes elsewhere. */
        if (role && 0 == strncmp(role, "phone", 5))
        {
            virtualsink = "btcall";
            sink_index = ebtcall;
        }
        else
        {
            virtualsink = "pdefaultapp";
            sink_index = edefaultapp;
        }
        pa_proplist_sets(type, "media.type", virtualsink);
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, type);
        sink = pa_namereg_get(c, virtualsink, PA_NAMEREG_SINK);
        if (sink)
            data->sink = sink;
        else
            pa_log_warn("virtual sink %s does not exist", virtualsink);
        pa_log_info("stream of role %s goes to %s", role ? role : "(none)", virtualsink);
        sink = NULL;

        sink = pa_namereg_get(c, u->sink_mapping_table[sink_index].outputdevice, PA_NAMEREG_SINK);

        if (sink && PA_SINK_IS_LINKED(sink->state))
        {
            pa_sink_input_new_data_set_sink(data, sink, TRUE, FALSE);
        }
    }
    else if ((data->sink != NULL) && (media_name) && (strcmp(media_name, "preprocess Stream") == 0))
    {
        pa_log_info("preprocess Stream");
        pa_log_info("data->sink->name : %s", data->sink->name);
        pa_proplist_sets(type, "media.type", "voipcall");
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, type);

        sink = pa_namereg_get(c, data->sink->name, PA_NAMEREG_SINK);

        if (sink && PA_SINK_IS_LINKED(sink->state))
            pa_sink_input_new_data_set_sink(data, sink, TRUE, FALSE);
    }
    else if ((NULL != data->sink) && sink_index == edefaultapp && (!(pa_streq(data->sink->driver, MODULE_NULL_SINK))))
    {
        pa_log_info("data->sink->name : %s", data->sink->name);
        const char *app_name = pa_proplist_gets(data->proplist, PA_PROP_APPLICATION_NAME);
        if (app_name && !strncmp(app_name, "Chromium", strlen("Chromium")))
        {
            pa_proplist_sets(type, "media.type", "voipcall");
        }
        else
        {
            pa_proplist_sets(type, "media.type", "pdefaultapp");
        }
        pa_proplist_sets(type, PA_PROP_PREFERRED_DEVICE, data->sink->name);
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, type);

        sink = pa_namereg_get(c, data->sink->name, PA_NAMEREG_SINK);

        if (sink && PA_SINK_IS_LINKED(sink->state))
        {
            pa_sink_input_new_data_set_sink(data, sink, TRUE, FALSE);
        }
    }
    else
    {
        pa_log_debug("new stream is opened with sink name : %s", data->sink->name);
        pa_proplist_sets(type, "media.type", data->sink->name);
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, type);
        for (i = eVirtualSink_First; i < eVirtualSink_Count; i++)
        {
            if (pa_streq(data->sink->name, u->sink_mapping_table[i].virtualsinkname))
            {
                pa_log_debug("found virtual sink index on virtual sink %d, name %s, index %d",
                             virtualsinkmap[i].virtualsinkidentifier, data->sink->name, i);
                u->media_type = i;
                break;
            }
        }

        if (i < eVirtualSink_Count)
        {
            /* opened on a virtual sink: route it to where that sink goes */
            sink = pa_namereg_get(c, u->sink_mapping_table[i].outputdevice, PA_NAMEREG_SINK);
            pa_log_info("routing to device:%s", u->sink_mapping_table[i].outputdevice);
            if (sink)
                data->sink = sink;
            else
                pa_log_warn("no sink %s to route %s to", u->sink_mapping_table[i].outputdevice, data->sink->name);
        }
        else
            pa_log_debug("%s is not a virtual sink, leaving the stream on it", data->sink->name);
    }
    if (type)
        pa_proplist_free(type);

    return PA_HOOK_OK;
}

static pa_hook_result_t route_sink_input_fixate_hook_callback(pa_core *c, pa_sink_input_new_data *data,
                                                              struct userdata *u)
{

    int i, sink_index = -1, volumetoset, volumetable;
    int ismute;
    const char *type;
    struct pa_cvolume cvolume;

    pa_assert(c);
    pa_assert(data);
    pa_assert(u);

    type = pa_proplist_gets(data->proplist, "media.type");

    for (i = 0; i < eVirtualSink_Count; i++)
    {
        if (pa_safe_streq(type, u->sink_mapping_table[i].virtualsinkname))
        {
            sink_index = i;
            break;
        }
    }
    if (sink_index == -1)
    {
        /* not a stream type we route: leave its volume and mute alone */
        pa_log_debug("no volume policy for stream type %s", type ? type : "(none)");
        return PA_HOOK_OK;
    }

    ismute = u->sink_mapping_table[sink_index].ismuted;
    pa_log_debug("setting mute %s for stream type %s", (ismute ? "TRUE" : "FALSE"), type);
    pa_sink_input_new_data_set_muted(data, ismute);

    volumetable = u->sink_mapping_table[sink_index].volumetable;
    volumetoset = pa_sw_volume_from_dB(_mapPercentToPulseRamp[volumetable]
                                                             [u->sink_mapping_table[sink_index].volume]);

    pa_log_debug("Setting volume(%d) for stream type(%s)", volumetoset, type);

    pa_cvolume_set(&cvolume, data->channel_map.channels, volumetoset);
    if (data->volume_writable)
        pa_sink_input_new_data_set_volume(data, &cvolume);
    else
        pa_log_debug("sink volume not writable");

    return PA_HOOK_OK;
}

static pa_hook_result_t route_sink_input_put_hook_callback(pa_core *c, pa_sink_input *data, struct userdata *u)
{

    struct sinkinputnode *si_data = NULL;
    const char *si_type;
    pa_sink_input_state_t state;
    int i;

    pa_assert(c);
    pa_assert(u);

    si_data = pa_xnew0(struct sinkinputnode, 1);

    si_data->sinkinput = data;
    si_data->sinkinputidx = data->index;
    si_data->paused = true;
    si_data->virtualsinkid = -1;

    if (pa_proplist_gets(data->proplist, PA_PROP_APPLICATION_NAME) == NULL)
    {
        pa_log("Sink opened, application.name not a VALID key..hence adding empty name");
        copy_field(si_data->appname, sizeof(si_data->appname), "");
    }
    else
    {
        copy_field(si_data->appname, sizeof(si_data->appname), pa_proplist_gets(data->proplist, PA_PROP_APPLICATION_NAME));
    }

    if (pa_proplist_gets(data->proplist, PA_PROP_PREFERRED_DEVICE))
    {
        pa_log("BYPASS enabled");
        si_data->bypassRouting = true;
    }
    else
    {
        pa_log("BYPASS disabled");
        si_data->bypassRouting = false;
    }

    pa_log("Sink opened with application name:%s, sink input index:%d",
           si_data->appname, si_data->sinkinputidx);
    si_type = pa_proplist_gets(data->proplist, "media.type");
    for (i = 0; i < eVirtualSink_Count; i++)
    {
        if (pa_safe_streq(si_type, u->sink_mapping_table[i].virtualsinkname))
        {
            si_data->virtualsinkid = u->sink_mapping_table[i].virtualsinkidentifier;
            break;
        }
    }
    if (si_data->virtualsinkid == -1)
    {
        /* not a stream type we route: nothing to track */
        pa_xfree(si_data);
        return PA_HOOK_OK;
    }
    pa_assert(si_data->virtualsinkid >= eVirtualSink_First);
    pa_assert(si_data->virtualsinkid <= eVirtualSink_Last);

    u->n_sink_input_opened++;
    PA_LLIST_PREPEND(struct sinkinputnode, u->sinkinputnodelist, si_data);

    state = data->state;

    /* send notification to audiod only if sink_input is in uncorked state */
    if (si_data->virtualsinkid != u->PreprocessSinkId && state == PA_SINK_INPUT_CORKED)
    {
        // si_data->paused = true; already done as part of init
        pa_log_debug("stream type (%s) is opened in corked state", si_type);
        return PA_HOOK_OK;
    }

    /* notify audiod of stream open */
    if (u->connectionactive && u->connev)
    {
        int ret;
        /* we have a connection send a message to audioD */
        si_data->paused = false;

        struct paReplyToPolicySet policySet;
        memset(&policySet, 0, sizeof(policySet));
        policySet.Type = PAUDIOD_REPLY_MSGTYPE_SINK_OPEN;
        policySet.id = si_data->virtualsinkid;
        policySet.index = si_data->sinkinputidx;
        copy_field(policySet.appName, sizeof(policySet.appName), si_data->appname);

        u->audiod_sink_input_opened[si_data->virtualsinkid]++;

        ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)) ? 0 : -1;
        if (-1 == ret)
            pa_log("send() failed: %s", strerror(errno));
        else
            pa_log("sent playback stream open message to audiod");
    }

    if (u->preprocess_module && si_data->virtualsinkid == u->PreprocessSinkId)
    {
        const char *media_name = pa_proplist_gets(data->proplist, "media.name");
        if ((media_name) && (strcmp(media_name, "preprocess Stream") == 0))
        {
        }
        else
        {
            pa_sink *destsink = NULL;
            destsink = pa_namereg_get(u->core, "preprocess_sink", PA_NAMEREG_SINK);

            if (destsink != NULL)
            {
                int si_volume = u->sink_mapping_table[si_data->virtualsinkid].volume;
                virtual_sink_input_index_set_volume(si_data->virtualsinkid, si_data->sinkinputidx, 65535, 0, u);
                u->sink_mapping_table[si_data->virtualsinkid].volume = si_volume;
                pa_sink_input_set_mute(si_data->sinkinput, false, TRUE);
                si_data->virtualsinkid *= -1;
                pa_log_info("moving the sink input 'voice' idx %d to module-preprocess", si_data->sinkinputidx);
                pa_sink_input_move_to(data, destsink, true);
            }
        }

    }

    return PA_HOOK_OK;
}

static pa_hook_result_t route_source_output_new_hook_callback(pa_core *c, pa_source_output_new_data *data,
                                                              struct userdata *u)
{
    int i, source_index = erecord;
    pa_proplist *stream_type;
    pa_source *source;
    const char *port, *dest_ip, *prop_name;
    pa_assert(data);
    pa_assert(u);
    pa_assert(c);

    prop_name = pa_strnull(pa_proplist_gets(data->proplist, PA_PROP_MEDIA_NAME));
    stream_type = pa_proplist_new();

    const char *media_name = pa_proplist_gets(data->proplist, "media.name");
    const char *app_name = pa_proplist_gets(data->proplist, PA_PROP_APPLICATION_NAME);
    const char *pref_device = pa_proplist_gets(data->proplist, PA_PROP_PREFERRED_DEVICE);

    if (!strcmp(prop_name, "RTP Monitor Stream"))
    {
        port = pa_strnull(pa_proplist_gets(data->proplist, "rtp.port"));
        dest_ip = pa_strnull(pa_proplist_gets(data->proplist, "rtp.destination"));
        send_rtp_connection_data_to_audiod(dest_ip, port, u);
    }

    if (data->source == NULL)
    {
        /* redirect everything to the default application stream */
        pa_log("THE DEFAULT DEVICE WAS USED TO CREATE THIS STREAM - PLEASE CATEGORIZE USING A VIRTUAL STREAM");
        source_index = erecord;
        pa_proplist_sets(stream_type, "media.type", "record");
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, stream_type);
        source = pa_namereg_get(c, "record", PA_NAMEREG_SOURCE);
        pa_assert(source != NULL);
        data->source = source;
        source = NULL;
    }
    else if (data->source && (!(pa_streq(data->source->driver, MODULE_NULL_SOURCE))))
    {
        pa_log("Physical device is used to create this stream");
        if (app_name && !strncmp(app_name, "Chromium", strlen("Chromium")))
        {
            pa_proplist_sets(stream_type, "media.type", "webcall");
            pa_proplist_sets(stream_type, PA_PROP_PREFERRED_DEVICE, data->source->name);
            pa_proplist_update(data->proplist, PA_UPDATE_MERGE, stream_type);
            source = pa_namereg_get(c, data->source->name, PA_NAMEREG_SOURCE);
        }
        else
        {
            pa_proplist_sets(stream_type, "media.type", "record");
            pa_proplist_sets(stream_type, PA_PROP_PREFERRED_DEVICE, data->source->name);
            pa_proplist_update(data->proplist, PA_UPDATE_MERGE, stream_type);
            source = pa_namereg_get(c, data->source->name, PA_NAMEREG_SOURCE);
        }
        pa_assert(source != NULL);
        data->source = source;
        if (source && PA_SOURCE_IS_LINKED(source->state))
            pa_source_output_new_data_set_source(data, source, true, false);

        source = NULL;
        return PA_HOOK_OK;
    }
    else if (pref_device)
    {
        pa_log("preferred device is used to create this stream");
        const char *actualDeviceName = get_device_name_from_detail(pref_device, u, false);
        if (data->source)
            pa_proplist_sets(stream_type, "media.type", data->source->name);
        else
        {
            if (app_name && !pa_streq(app_name, "Chromium"))
            {
                pa_proplist_sets(stream_type, "media.type", "webcall");
            }
            else
            {
                pa_proplist_sets(stream_type, "media.type", "record");
            }
        }

        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, stream_type);
        if (actualDeviceName)
        {
            pa_log_info("Preferred device = %s, appname = %s, actualDeviceName=%s", (pref_device ? pref_device : "_"), app_name ? app_name : "_", actualDeviceName);
            pa_log_info("streamtype =%s", data->source->name);
            source = pa_namereg_get(c, actualDeviceName, PA_NAMEREG_SOURCE);
            if (source && PA_SOURCE_IS_LINKED(source->state))
                pa_source_output_new_data_set_source(data, source, true, false);
        }
        else
        {
            pa_log("No device found for preferred device : ERROR");
        }

        source = NULL;
        return PA_HOOK_OK;
    }
    else if (strstr(data->source->name, "monitor"))
    {
        pa_log_info("found a monitor source, do not route to hw sink!");
        return PA_HOOK_OK;
    }
    else if ((media_name) && (strcmp(media_name, "preprocess Stream") == 0))
    {
        pa_log_info("preprocess Stream");
        pa_log_info("data->source->name : %s", data->source->name);
        stream_type = pa_proplist_new();
        pa_proplist_sets(stream_type, "media.type", "webcall");
        pa_proplist_update(data->proplist, PA_UPDATE_MERGE, stream_type);

        pa_source *s;
        s = pa_namereg_get(c, data->source->name, PA_NAMEREG_SOURCE);

        if (s && PA_SOURCE_IS_LINKED(s->state))
            pa_source_output_new_data_set_source(data, s, true, false);

        return PA_HOOK_OK;
    }
    else
    {
        for (i = 0; i < eVirtualSource_Count; i++)
        {
            if (pa_streq(data->source->name, virtualsourcemap[i].virtualsourcename))
            {
                pa_log_debug("found virtual source index on virtual source %d, name %s, index %d",
                             virtualsourcemap[i].virtualsourceidentifier, data->source->name, i);
                source_index = i;
                break;
            }
        }
    }

    pa_proplist_sets(stream_type, "media.type", virtualsourcemap[source_index].virtualsourcename);
    pa_proplist_update(data->proplist, PA_UPDATE_MERGE, stream_type);
    pa_proplist_free(stream_type);

    if ((data->source != NULL) && strstr(data->source->name, "bluez_"))
    {
        return PA_HOOK_OK;
    }

    /* implement policy */
    for (i = 0; i < eVirtualSource_Count; i++)
    {
        if (i == (int)virtualsourcemap[source_index].virtualsourceidentifier)
        {
            pa_source *s;

            pa_log_debug("setting data->source (physical) to %s for streams created on %s (virtual)",
                         u->source_mapping_table[i].inputdevice, virtualsourcemap[i].virtualsourcename);

            if (data->source == NULL)
            {
                s = pa_namereg_get(c, PCM_SOURCE_NAME, PA_NAMEREG_SOURCE);
            }
            else
            {
                s = pa_namereg_get(c, u->source_mapping_table[i].inputdevice, PA_NAMEREG_SOURCE);
            }
            if (s && PA_SOURCE_IS_LINKED(s->state))
                pa_source_output_new_data_set_source(data, s, false, true);
            break;
        }
    }
    return PA_HOOK_OK;
}

static pa_hook_result_t route_source_output_fixate_hook_callback(pa_core *c, pa_source_output_new_data *data,
                                                                 struct userdata *u)
{

    pa_assert(c);
    pa_assert(data);
    pa_assert(u);

    /* nothing much to to in fixate as of now */
    return PA_HOOK_OK;
}

static pa_hook_result_t route_source_output_put_hook_callback(pa_core *c, pa_source_output *so, struct userdata *u)
{
    const char *so_type = NULL;
    struct sourceoutputnode *node = NULL;
    int i, source_index = -1;
    pa_source_output_state_t state;

    pa_assert(c);
    pa_assert(so);
    pa_assert(u);

    if (strstr(so->source->name, "monitor"))
        return PA_HOOK_OK; /* nothing to be done for monitor source */

    so_type = pa_proplist_gets(so->proplist, "media.type");
    if (so_type == NULL)
    {
        pa_log_warn("source output %u has no media.type, leaving it alone", so->index);
        return PA_HOOK_OK;
    }

    node = pa_xnew0(struct sourceoutputnode, 1);
    node->virtualsourceid = -1;
    node->sourceoutput = so;
    node->sourceoutputidx = so->index;
    if (pa_proplist_gets(so->proplist, PA_PROP_APPLICATION_NAME) == NULL)
    {
        pa_log("Source opened, application.name not a VALID key..hence adding empty name");
        copy_field(node->appname, sizeof(node->appname), "");
    }
    else
    {
        copy_field(node->appname, sizeof(node->appname), pa_proplist_gets(so->proplist, PA_PROP_APPLICATION_NAME));
    }
    pa_log("Source opened with application name:%s, source output index:%d",
           node->appname, node->sourceoutputidx);

    node->paused = false;

    if (pa_proplist_gets(so->proplist, PA_PROP_PREFERRED_DEVICE))
    {
        node->bypassRouting = true;
    }
    else
    {
        node->bypassRouting = false;
    }

    for (i = 0; i < eVirtualSource_Count; i++)
    {
        if (pa_streq(so_type, u->source_mapping_table[i].virtualsourcename))
        {
            source_index = u->source_mapping_table[i].virtualsourceidentifier;
            break;
        }
    }

    if (source_index == -1)
    {
        /* not a stream type we route; do not take the daemon down over it */
        pa_log_warn("source output %u has unknown media.type %s, leaving it alone", so->index, so_type);
        pa_xfree(node);
        return PA_HOOK_OK;
    }
    node->virtualsourceid = source_index;

    u->n_source_output_opened++;
    PA_LLIST_PREPEND(struct sourceoutputnode, u->sourceoutputnodelist, node);

    state = so->state;
    if (node->virtualsourceid != u->PreprocessSourceId && state == PA_SOURCE_OUTPUT_CORKED)
    {
        node->paused = true;
        pa_log_debug("Record stream of type(%s) is opened in corked state", so_type);
        return PA_HOOK_OK;
    }
    if (u->connectionactive && u->connev)
    {
        int ret;

        struct paReplyToPolicySet policySet;
        memset(&policySet, 0, sizeof(policySet));
        policySet.Type = PAUDIOD_REPLY_MSGTYPE_SOURCE_OPEN;
        policySet.id = node->virtualsourceid;
        policySet.index = node->sourceoutputidx;
        copy_field(policySet.appName, sizeof(policySet.appName), node->appname);

        ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)) ? 0 : -1;
        if (ret == -1)
            pa_log("Record stream type(%s): send failed(%s)", so_type, strerror(errno));
    }
    u->audiod_source_output_opened[source_index]++;

    if(u->preprocess_module && node->virtualsourceid == u->PreprocessSourceId)
    {
        const char *media_name = pa_proplist_gets(so->proplist, "media.name");
        if ((media_name) && (strcmp(media_name, "preprocess Stream") == 0))
        {
        }
        else
        {
            pa_source *destsource = NULL;
            destsource = pa_namereg_get(u->core, "preprocess-source", PA_NAMEREG_SOURCE);

            if (destsource != NULL)
            {
                int so_volume = u->source_mapping_table[node->virtualsourceid].volume;
                virtual_source_input_index_set_volume(node->virtualsourceid, node->sourceoutputidx, 65535, 0, u);
                u->source_mapping_table[node->virtualsourceid].volume = so_volume;
                pa_source_output_set_mute(node->sourceoutput, false, TRUE);
                node->virtualsourceid *= -1;
                pa_log_info("moving the source output 'voice' idx %d preprocess-source", node->sourceoutputidx);
                pa_source_output_move_to(so, destsource, true);
            }
        }
    }
    return PA_HOOK_OK;
}

/* callback for stream deletion */
static pa_hook_result_t route_sink_input_unlink_hook_callback(pa_core *c, pa_sink_input *data, struct userdata *u)
{
    (void)c;
    struct sinkinputnode *thelistitem = NULL;

    pa_assert(data);
    pa_assert(u);

    /* delete the list item */
    for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
    {

        if (thelistitem->sinkinput == data)
        {

            if (u->preprocess_module && thelistitem->virtualsinkid == -1 * u->PreprocessSinkId)
            {
                thelistitem->virtualsinkid = u->PreprocessSinkId;
            }
            /* we have a connection send a message to audioD */
            if (!thelistitem->paused)
            {

                /* notify audiod of stream closure */
                if (u->connectionactive && u->connev != NULL)
                {

                    pa_log("Sink closed with application name:%s, sink input index:%d",
                           thelistitem->appname, thelistitem->sinkinputidx);

                    struct paReplyToPolicySet policySet;
                    memset(&policySet, 0, sizeof(policySet));
                    policySet.Type = PAUDIOD_REPLY_MSGTYPE_SINK_CLOSE;

                    policySet.id = thelistitem->virtualsinkid;
                    policySet.index = thelistitem->sinkinputidx;
                    copy_field(policySet.appName, sizeof(policySet.appName), thelistitem->appname);

                    if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)))
                        pa_log_info("route_sink_input_unlink_hook_callback: send failed: %s", strerror(errno));
                    else
                        pa_log_info("route_sink_input_unlink_hook_callback: sending close notification to audiod");
                }

                // decrease sink opened count, even if audiod doesn't hear from it
                if (thelistitem->virtualsinkid >= eVirtualSink_First && thelistitem->virtualsinkid <= eVirtualSink_Last)
                    u->audiod_sink_input_opened[thelistitem->virtualsinkid]--;
            }

            /* remove this node from the list and free */
            PA_LLIST_REMOVE(struct sinkinputnode, u->sinkinputnodelist, thelistitem);

            pa_xfree(thelistitem);
            pa_assert(u->n_sink_input_opened > 0);
            u->n_sink_input_opened--;
            break;
        }
    }

    return PA_HOOK_OK;
}

/* callback for stream pause/unpause */
static pa_hook_result_t
route_sink_input_state_changed_hook_callback(pa_core *c, pa_sink_input *data, struct userdata *u)
{
    (void)c;
    pa_sink_input_state_t state;
    struct sinkinputnode *thelistitem = NULL;

    pa_assert(data);
    pa_assert(u);

    /* delete the list item */
    for (thelistitem = u->sinkinputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
    {

        if (thelistitem->sinkinput == data)
        {
            bool sinkidReversed = false;
            struct paReplyToPolicySet policySet;
            memset(&policySet, 0, sizeof(policySet));

            if (u->preprocess_module && thelistitem->virtualsinkid == -1 * u->PreprocessSinkId)
            {
                thelistitem->virtualsinkid = u->PreprocessSinkId;
                sinkidReversed = true;
            }

            state = data->state;

            /* we have a connection send a message to audioD */
            if (!thelistitem->paused && state == PA_SINK_INPUT_CORKED)
            {
                thelistitem->paused = true;
                policySet.Type = PAUDIOD_REPLY_MSGTYPE_SINK_CLOSE;

                policySet.id = thelistitem->virtualsinkid;
                policySet.index = thelistitem->sinkinputidx;
                copy_field(policySet.appName, sizeof(policySet.appName), thelistitem->appname);


                // decrease sink opened count, even if audiod doesn't hear from it
                if (thelistitem->virtualsinkid >= eVirtualSink_First && thelistitem->virtualsinkid <= eVirtualSink_Last)
                    u->audiod_sink_input_opened[thelistitem->virtualsinkid]--;
            }
            else if (thelistitem->paused && state != PA_SINK_INPUT_CORKED)
            {
                thelistitem->paused = false;
                policySet.Type = PAUDIOD_REPLY_MSGTYPE_SINK_OPEN;
                policySet.id = thelistitem->virtualsinkid;
                policySet.index = thelistitem->sinkinputidx;
                copy_field(policySet.appName, sizeof(policySet.appName), thelistitem->appname);


                // increase sink opened count, even if audiod doesn't hear from it
                if (thelistitem->virtualsinkid >= eVirtualSink_First && thelistitem->virtualsinkid <= eVirtualSink_Last)
                    u->audiod_sink_input_opened[thelistitem->virtualsinkid]++;
            }
            else
            {
                if (sinkidReversed)
                {
                    thelistitem->virtualsinkid *= -1;
                }
                continue;
            }
            if (sinkidReversed)
            {
                thelistitem->virtualsinkid *= -1;
            }

            /* notify audiod of stream closure */
            if (u->connectionactive && u->connev != NULL)
            {
                if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)))
                    pa_log("route_sink_input_state_changed_hook_callback: send failed");
                else
                    pa_log_info("route_sink_input_state_changed_hook_callback: sending state change notification to audiod");
            }
        }
    }

    return PA_HOOK_OK;
}

static pa_hook_result_t route_source_output_state_changed_hook_callback(pa_core *c, pa_source_output *so, struct userdata *u)
{

    pa_source_output_state_t state;
    struct sourceoutputnode *node;

    pa_assert(c);
    pa_assert(so);
    pa_assert(u);

    state = so->state;

    for (node = u->sourceoutputnodelist; node; node = node->next)
    {
        if (node->sourceoutput == so)
        {
            bool sourceidReversed = false;
            struct paReplyToPolicySet policySet;
            memset(&policySet, 0, sizeof(policySet));
            if(u->preprocess_module && node->virtualsourceid == -1 * u->PreprocessSourceId)
            {
                node->virtualsourceid = u->PreprocessSourceId;
                sourceidReversed = true;
            }

            if (!node->paused && state == PA_SOURCE_OUTPUT_CORKED)
            {
                policySet.Type = PAUDIOD_REPLY_MSGTYPE_SOURCE_CLOSE;
                policySet.id = node->virtualsourceid;
                policySet.index = node->sourceoutputidx;
                copy_field(policySet.appName, sizeof(policySet.appName), node->appname);

                node->paused = true;

                /* decrease source opened count, even if audiod doesn't hear from it */
                if (node->virtualsourceid >= eVirtualSource_First && node->virtualsourceid <= eVirtualSource_Last)
                    u->audiod_source_output_opened[node->virtualsourceid]--;
            }
            else if (node->paused && state == PA_SOURCE_OUTPUT_RUNNING)
            {
                // pa_assert(node->paused == true);
                node->paused = false;
                policySet.Type = PAUDIOD_REPLY_MSGTYPE_SOURCE_OPEN;
                policySet.id = node->virtualsourceid;
                policySet.index = node->sourceoutputidx;
                copy_field(policySet.appName, sizeof(policySet.appName), node->appname);


                /* increase source opened count, even if audiod doesn't hear from it */
                if (node->virtualsourceid >= eVirtualSource_First && node->virtualsourceid <= eVirtualSource_Last)
                    u->audiod_source_output_opened[node->virtualsourceid]++;
            }
            else
            {
                if (sourceidReversed)
                {
                    node->virtualsourceid *= -1;
                }
                continue;
            }
            if (sourceidReversed)
            {
                node->virtualsourceid *= -1;
            }
            if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)))
                pa_log("Error sending recording stream state change to audiod");
            break;
        }
    }
    return PA_HOOK_OK;
}

/* callback for source deletion */
static pa_hook_result_t route_source_output_unlink_hook_callback(pa_core *c, pa_source_output *data, struct userdata *u)
{
    struct sourceoutputnode *thelistitem = NULL;

    pa_assert(data);
    pa_assert(u);
    pa_assert(c);

    /* delete the list item */
    for (thelistitem = u->sourceoutputnodelist; thelistitem != NULL; thelistitem = thelistitem->next)
    {

        if (thelistitem->sourceoutput == data)
        {

            if (u->preprocess_module && thelistitem->virtualsourceid == -1 * u->PreprocessSourceId)
            {
                thelistitem->virtualsourceid = u->PreprocessSourceId;
            }

            if (!thelistitem->paused)
            {
                /* we have a connection send a message to audioD */
                /* notify audiod of stream closure */
                if (u->connectionactive && u->connev != NULL)
                {

                    struct paReplyToPolicySet policySet;
                    memset(&policySet, 0, sizeof(policySet));
                    policySet.Type = PAUDIOD_REPLY_MSGTYPE_SOURCE_CLOSE;

                    policySet.id = thelistitem->virtualsourceid;
                    policySet.index = thelistitem->sourceoutputidx;
                    copy_field(policySet.appName, sizeof(policySet.appName), thelistitem->appname);


                    if (!send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_POLICY, 0, &policySet, sizeof(policySet)))
                        pa_log("route_source_output_unlink_hook_callback: send failed: %s", strerror(errno));
                    else
                        pa_log_info("route_source_output_unlink_hook_callback: sending close notification to audiod");
                }

                // decrease source opened count, even if audiod doesn't hear from it
                if (thelistitem->virtualsourceid >= eVirtualSource_First && thelistitem->virtualsourceid <= eVirtualSource_Last)
                    u->audiod_source_output_opened[thelistitem->virtualsourceid]--;
            }

            /* remove this node from the list and free */
            PA_LLIST_REMOVE(struct sourceoutputnode, u->sourceoutputnodelist, thelistitem);

            pa_xfree(thelistitem);

            // maintain count, even if we can't talk to audiod
            pa_assert(u->n_source_output_opened > 0);
            u->n_source_output_opened--;
            break;
        }
    }

    return PA_HOOK_OK;
}

/* exit and cleanup */

/* Free the module's state as userdata_new() made it, with everything it owns.
 * Hooks, socket and shared objects are pa__done()'s to release first. */
static void userdata_free(struct userdata *u)
{
    struct sinkinputnode *thelistitem = NULL;

    /* free the list of sink-inputs */
    while ((thelistitem = u->sinkinputnodelist) != NULL)
    {
        PA_LLIST_REMOVE(struct sinkinputnode, u->sinkinputnodelist, thelistitem);
        pa_xfree(thelistitem);
    }

    /* and of source-outputs */
    struct sourceoutputnode *node;
    while ((node = u->sourceoutputnodelist) != NULL)
    {
        PA_LLIST_REMOVE(struct sourceoutputnode, u->sourceoutputnodelist, node);
        pa_xfree(node);
    }

    // free memory
    multiple_device_info_free(&u->usbOutputDeviceInfo);

    multiple_device_info_free(&u->usbInputDeviceInfo);

    multiple_device_info_free(&u->internalInputDeviceInfo);

    multiple_device_info_free(&u->internalOutputDeviceInfo);

    pa_xfree(u->destAddress);
    pa_xfree(u->connectionType);
    pa_xfree(u->deviceName);
    pa_xfree(u);
}

void pa__done(pa_module *m)
{
    struct userdata *u;

    pa_assert(m);

    if (!(u = m->userdata))
        return;

    /* a connection exists on the socket, tear down */
    close_connection(u);

    if (u->sockev != NULL)
    {
        /* a socket still exists, tear it down and remove
         * ourselves from the IO list on the pulseaudio
         * main loop */
        u->core->mainloop->io_free(u->sockev);

        /* tear down the connection */
        if (-1 == shutdown(u->sockfd, SHUT_RDWR))
        {
            pa_log_info("Error in shutdown (%d):%s", errno, strerror(errno));
        }
        if (-1 == close(u->sockfd))
        {
            pa_log_info("Error in close (%d):%s", errno, strerror(errno));
        }
    }

    disconnect_hooks(u);

    if (u->palm_policy)
        pa_palm_policy_unref(u->palm_policy);

    userdata_free(u);
    m->userdata = NULL;
}

static pa_hook_result_t route_sink_input_move_finish_cb(pa_core *c, pa_sink_input *data, struct userdata *u)
{
    int i;

    pa_assert(c);
    pa_assert(data);
    pa_assert(u);

    for (i = eVirtualSink_First; i <= eVirtualSink_Last; i++)
        virtual_sink_input_set_volume(i, u->sink_mapping_table[i].volume, 0, u);

    pa_log_debug("moved sink inputs to the destination sink");
    return PA_HOOK_OK;
}

static pa_hook_result_t route_source_output_move_finish_cb(pa_core *c, pa_source_output *data, struct userdata *u)
{
    int i;

    pa_assert(c);
    pa_assert(data);
    pa_assert(u);

    for (i = eVirtualSource_First; i <= eVirtualSource_Last; i++)
        virtual_source_input_set_volume(i, u->source_mapping_table[i].volume, 0, u);

    return PA_HOOK_OK;
}

pa_hook_result_t route_sink_unlink_post_cb(pa_core *c, pa_sink *sink, struct userdata *u)
{
    pa_assert(c);
    pa_assert(sink);
    pa_assert(u);

    if (pa_streq(sink->name, PCM_SINK_NAME))
        u->alsa_sink1 = NULL;

    return PA_HOOK_OK;
}

pa_hook_result_t route_source_unlink_post_cb(pa_core *c, pa_source *source, struct userdata *u)
{
    pa_assert(c);
    pa_assert(source);
    pa_assert(u);

    u->callback_deviceName = source->name;
    pa_log_info("module other = %s %d", source->name, source->index);

    if (strstr(source->name, "preprocess"))
    {
        pa_log_info("preprocess module unload, dont inform audiod");
        return PA_HOOK_OK;
    }
    if (strstr(source->name, ".monitor"))
    {
        pa_log_info("monitor unload, dont inform audiod");
        return PA_HOOK_OK;
    }
    if (NULL != u->callback_deviceName)
    {
        pa_log_debug("module_unloaded with device name:%s", u->callback_deviceName);
        /* notify audiod of device insertion */
        if (u->connectionactive && u->connev)
        {
            int ret = -1;

            struct paReplyToRoutingSet routingSet;
            memset(&routingSet, 0, sizeof(routingSet));
            routingSet.Type = PAUDIOD_REPLY_MSGTYPE_DEVICE_REMOVED;
            routingSet.isOutput = 0;
            copy_field(routingSet.device, sizeof(routingSet.device), u->callback_deviceName);
            routingSet.deviceIcon[0] = '\0';
            routingSet.deviceNameDetail[0] = '\0';

            /* we have a connection send a message to audioD */
            ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_ROUTING, 0, &routingSet, sizeof(routingSet)) ? 0 : -1;
            if (-1 == ret)
                pa_log("send() failed: %s", strerror(errno));
            else
                pa_log_info("sent device unloaded message to audiod");
        }
        else
            pa_log_warn("connectionactive is not active");
    }

    return PA_HOOK_OK;
}

pa_hook_result_t route_sink_unlink_cb(pa_core *c, pa_sink *sink, struct userdata *u)
{
    pa_log_info("route_sink_unlink_cb");
    pa_assert(c);
    pa_assert(sink);
    pa_assert(u);
    if (strstr(sink->name, "preprocess"))
    {
        pa_log_info("preprocess module unload, dont inform audiod");
        return PA_HOOK_OK;
    }
    pa_log_debug("BT sink disconnected with name:%s", sink->name);
    if (strstr(sink->name, "bluez_sink."))
    {
        pa_log_debug("BT sink disconnected with name:%s", sink->name);
        u->callback_deviceName = sink->name;
        if (NULL != u->callback_deviceName)
        {
            pa_log_debug("Bt sink disconnected with name:%s", u->callback_deviceName);
            /* notify audiod of device insertion */
            if (u->connectionactive && u->connev)
            {
                int ret = -1;
                /* we have a connection send a message to audioD */

                struct paReplyToRoutingSet routingSet;
                memset(&routingSet, 0, sizeof(routingSet));
                routingSet.Type = PAUDIOD_REPLY_MSGTYPE_DEVICE_REMOVED;
                routingSet.isOutput = 1;
                routingSet.deviceIcon[0] = '\0';
                routingSet.deviceNameDetail[0] = '\0';

                copy_field(routingSet.device, sizeof(routingSet.device), u->callback_deviceName);

                ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_ROUTING, 0, &routingSet, sizeof(routingSet)) ? 0 : -1;
                if (-1 == ret)
                    pa_log("send() failed: %s", strerror(errno));
                else
                    pa_log_info("sent device unloaded message to audiod");
            }
            else
                pa_log_warn("connectionactive is not active");
        }
        else
            pa_log_warn("error reading device name");
    }
    else
    {
        if (sink->module && pa_streq(sink->module->name, MODULE_ALSA_SINK_NAME))
        {
            u->callback_deviceName = sink->name;
            /* notify audiod of device removal */
            if (u->connectionactive && u->connev)
            {
                int ret = -1;

                struct paReplyToRoutingSet routingSet;
                memset(&routingSet, 0, sizeof(routingSet));
                routingSet.Type = PAUDIOD_REPLY_MSGTYPE_DEVICE_REMOVED;
                routingSet.isOutput = 1;
                routingSet.deviceIcon[0] = '\0';
                routingSet.deviceNameDetail[0] = '\0';
                copy_field(routingSet.device, sizeof(routingSet.device), u->callback_deviceName);

                /* we have a connection send a message to audioD */

                ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_ROUTING, 0, &routingSet, sizeof(routingSet)) ? 0 : -1;
                if (-1 == ret)
                    pa_log("send() failed: %s", strerror(errno));
                else
                    pa_log_info("sent device un-loaded message to audiod");
            }
            else
                pa_log_warn("connectionactive is not active");
        }
    }
    return PA_HOOK_OK;
}

static pa_hook_result_t source_load_subscription_callback(pa_core *c, pa_source_new_data *data, struct userdata *u)
{
    pa_log_info("source_load_subscription_callback");
    pa_assert(c);
    pa_assert(data);
    pa_assert(u);

    /* a source need not belong to a module; only ALSA sources matter here */
    if (data->module && pa_streq(data->module->name, MODULE_ALSA_SOURCE_NAME))
    {
        const char *deviceNameDetail;
        u->callback_deviceName = data->name;
        deviceNameDetail = pa_proplist_gets(data->proplist, ALSA_CARD_NAME);
        const char *deviceIcon = pa_proplist_gets(data->proplist, PA_PROP_DEVICE_ICON_NAME);
        if (!deviceNameDetail)
            deviceNameDetail = data->name;
        /* notify audiod of device insertion */
        if (u->connectionactive && u->connev)
        {
            int ret = -1;

            struct paReplyToRoutingSet routingSet;
            memset(&routingSet, 0, sizeof(routingSet));
            routingSet.Type = PAUDIOD_REPLY_MSGTYPE_DEVICE_CONNECTION;
            copy_field(routingSet.device, sizeof(routingSet.device), u->callback_deviceName);
            copy_field(routingSet.deviceNameDetail, sizeof(routingSet.deviceNameDetail), deviceNameDetail);
            copy_field(routingSet.deviceIcon, sizeof(routingSet.deviceIcon), deviceIcon);
            routingSet.isOutput = 0; // false

            /* we have a connection send a message to audioD */
            ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_ROUTING, 0, &routingSet, sizeof(routingSet)) ? 0 : -1;
            if (-1 == ret)
                pa_log("send() failed: %s", strerror(errno));
            else
                pa_log_info("sent device loaded message to audiod");
        }
        else
            pa_log_warn("connectionactive is not active");
        pa_log_info("source_load_subscription_callback:%s-%s", u->callback_deviceName, deviceNameDetail);
    }
    else
    {
        pa_log_info("source other than alsa source loaded");
    }

    return PA_HOOK_OK;
}

pa_hook_result_t sink_load_subscription_callback(pa_core *c, pa_sink_new_data *data, struct userdata *u)
{
    pa_log_info("sink_load_subscription_callback");
    pa_assert(c);
    pa_assert(data);
    pa_assert(u);
    if (strstr(data->name, "bluez_sink."))
    {
        const char *deviceNameDetail;
        const char *deviceIcon;

        deviceNameDetail = pa_proplist_gets(data->card->proplist, "bluez.alias");
        deviceIcon = pa_proplist_gets(data->card->proplist, PA_PROP_DEVICE_ICON_NAME);
        pa_log_debug("BT sink connected with name:%s : %s,%s", data->name, deviceNameDetail, deviceIcon);
        u->callback_deviceName = data->name;
        if (NULL != u->callback_deviceName)
        {
            /* notify audiod of device insertion */
            if (u->connectionactive && u->connev)
            {
                int ret = -1;

                struct paReplyToRoutingSet routingSet;
                memset(&routingSet, 0, sizeof(routingSet));
                routingSet.Type = PAUDIOD_REPLY_MSGTYPE_DEVICE_CONNECTION;
                copy_field(routingSet.device, sizeof(routingSet.device), u->callback_deviceName);
                copy_field(routingSet.deviceNameDetail, sizeof(routingSet.deviceNameDetail), deviceNameDetail);
                copy_field(routingSet.deviceIcon, sizeof(routingSet.deviceIcon), deviceIcon);
                routingSet.isOutput = 1; // true

                /* we have a connection send a message to audioD */
                ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_ROUTING, 0, &routingSet, sizeof(routingSet)) ? 0 : -1;
                if (-1 == ret)
                    pa_log("send() failed: %s", strerror(errno));
                else
                    pa_log_info("sent device loaded message to audiod");
            }
            else
                pa_log_warn("connectionactive is not active");
        }
        else
            pa_log_warn("error reading device name");
    }
    else
    {
        const char *deviceNameDetail;
        const char *deviceIcon;
        if (pa_streq(data->module->name, MODULE_ALSA_SINK_NAME))
        {
            u->callback_deviceName = data->name;
            deviceNameDetail = pa_proplist_gets(data->proplist, ALSA_CARD_NAME);
            deviceIcon = pa_proplist_gets(data->proplist, PA_PROP_DEVICE_ICON_NAME);
            pa_log_info("deviceIcon:%s", deviceIcon);
            pa_log_info("deviceNameDetail:%s", deviceNameDetail);
            if (!deviceNameDetail)
                deviceNameDetail = data->name;
            /* notify audiod of device insertion */
            if (u->connectionactive && u->connev)
            {
                int ret = -1;

                struct paReplyToRoutingSet routingSet;
                memset(&routingSet, 0, sizeof(routingSet));
                routingSet.Type = PAUDIOD_REPLY_MSGTYPE_DEVICE_CONNECTION;
                copy_field(routingSet.device, sizeof(routingSet.device), u->callback_deviceName);
                copy_field(routingSet.deviceNameDetail, sizeof(routingSet.deviceNameDetail), deviceNameDetail);
                copy_field(routingSet.deviceIcon, sizeof(routingSet.deviceIcon), deviceIcon);
                routingSet.isOutput = 1; // true

                /* we have a connection send a message to audioD */
                ret = send_to_audiod(u, PAUDIOD_REPLY_MSGTYPE_ROUTING, 0, &routingSet, sizeof(routingSet)) ? 0 : -1;
                if (-1 == ret)
                    pa_log("send() failed: %s", strerror(errno));
                else
                    pa_log_info("sent device loaded message to audiod");
            }
            else
            {
                pa_log_warn("connectionactive is not active");
            }
            pa_log("sink_load_subscription_callback : %s-%s", u->callback_deviceName, deviceNameDetail);
        }
    }
    return PA_HOOK_OK;
}

pa_hook_result_t module_unload_subscription_callback(pa_core *c, pa_module *m, struct userdata *u)
{
    pa_log_info("module_unload_subscription_callback");
    pa_assert(c);
    pa_assert(m);
    pa_assert(u);
    pa_log_debug("module_unloaded with index#:%u", m->index);
    if (!strcmp(m->name, MODULE_ALSA_SINK_NAME))
        check_and_remove_usb_device_module(u, true, m);
    if (!strcmp(m->name, MODULE_ALSA_SOURCE_NAME))
        check_and_remove_usb_device_module(u, false, m);

    return PA_HOOK_OK;
}

pa_hook_result_t module_load_subscription_callback(pa_core *c, pa_module *m, struct userdata *u)
{
    pa_log_info("module_load_subscription_callback");
    pa_assert(c);
    pa_assert(m);
    pa_assert(u);
    pa_log_debug("module_loaded with name:%s", m->name);

    return PA_HOOK_OK;
}
