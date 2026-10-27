// SPDX-License-Identifier: GPL-3.0-or-later
/* Report the Snapdragon Sensor Core Hall sensor as an input lid switch.
 *
 * Covers and keyboards hold a magnet over the Hall sensor when closed. The
 * sensor sits on the DSP, so no kernel driver exposes it; a uinput SW_LID
 * device lets logind, UPower and the desktop apply their lid actions.
 *
 * The device is created at startup, open until the first reading, and kept for
 * the lifetime of the process: PowerDevil 6.8 reads whether a lid is present
 * only when it starts, and UPower 1.91 tracks only the first lid device it sees.
 * SSC failures are retried here instead of by restarting the service.
 */
#include "libssc-client-private.h"
#include "libssc-common-private.h"
#include "ssc-sensor-suid.pb-c.h"
#include "monitor.pb-c.h"
#include <glib-unix.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/uinput.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define HALL_EVENT 770  /* sns_hall: state 1 = near (magnet present), 0 = far */
#define ERROR_EVENT 130

static gboolean stopping, discovered, failed;
static guint64 hall_high, hall_low;
static int uinput = -1, lid;

static gboolean stop(gpointer unused)
{
    stopping = TRUE;
    return G_SOURCE_CONTINUE;
}

static gboolean expire(gpointer expired)
{
    *(gboolean *)expired = TRUE;
    return G_SOURCE_REMOVE;
}

/* Iterate the main loop until *ready, a stop request or the timeout. */
static gboolean wait_for(gboolean *ready, guint seconds)
{
    gboolean expired = FALSE;
    guint source = g_timeout_add_seconds(seconds, expire, &expired);
    while (!*ready && !stopping && !expired)
        g_main_context_iteration(NULL, TRUE);
    if (!expired) g_source_remove(source);
    return *ready;
}

static gboolean send_request(SSCClient *client, guint64 high, guint64 low,
                             guint32 id, ProtobufCMessage *message)
{
    g_autoptr(GArray) bytes = NULL;
    if (message) {
        bytes = g_array_new(FALSE, FALSE, 1);
        g_array_set_size(bytes, protobuf_c_message_get_packed_size(message));
        protobuf_c_message_pack(message, (guint8 *)bytes->data);
    }
    SyncContext ctx;
    g_autoptr(GError) error = NULL;
    ssc_common_init_sync_context(&ctx);
    ssc_client_send(client, high, low, id, bytes, NULL,
                    ssc_common_callback_sync_context, &ctx);
    ssc_common_wait_sync_context(&ctx);
    gboolean ok = ssc_client_send_finish(client, ctx.result, &error);
    if (!ok) fprintf(stderr, "Request %u failed: %s\n", id, error->message);
    ssc_common_clear_sync_context(&ctx);
    return ok;
}

static gboolean emit(int fd, unsigned type, unsigned code, int value)
{
    struct input_event event = {.type = type, .code = code, .value = value};
    return write(fd, &event, sizeof(event)) == sizeof(event);
}

static gboolean create_device(void)
{
    uinput = open("/dev/uinput", O_WRONLY | O_CLOEXEC);
    if (uinput < 0) {
        fprintf(stderr, "Cannot open /dev/uinput: %s\n", strerror(errno));
        return FALSE;
    }
    struct uinput_setup setup = {.id = {.bustype = BUS_VIRTUAL}};
    g_strlcpy(setup.name, "SSC Hall Lid Switch", sizeof(setup.name));
    if (ioctl(uinput, UI_SET_EVBIT, EV_SW) < 0 || ioctl(uinput, UI_SET_SWBIT, SW_LID) < 0 ||
        ioctl(uinput, UI_DEV_SETUP, &setup) < 0 || ioctl(uinput, UI_DEV_CREATE) < 0) {
        fprintf(stderr, "Cannot create the lid device: %s\n", strerror(errno));
        close(uinput);
        uinput = -1;
        return FALSE;
    }
    return TRUE;
}

static void set_lid(int closed)
{
    if (closed == lid) return;
    if (!emit(uinput, EV_SW, SW_LID, closed) || !emit(uinput, EV_SYN, SYN_REPORT, 0)) {
        fprintf(stderr, "Cannot report the lid state: %s\n", strerror(errno));
        failed = stopping = TRUE;
        return;
    }
    fprintf(stderr, "Lid %s\n", closed ? "closed" : "open");
    lid = closed;
}

static void report(SSCClient *client, guint32 id, guint64 high, guint64 low,
                   GArray *bytes, gboolean *rejected)
{
    if (high == SSC_SENSOR_UID_SUID_HIGH && low == SSC_SENSOR_UID_SUID_LOW && id == SSC_MSG_RESPONSE_SUID) {
        SscSuidResponse *msg = ssc_suid_response__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        if (!msg || g_strcmp0(msg->data_type, "hall")) { ssc_suid_response__free_unpacked(msg, NULL); return; }
        if (msg->n_uid) {
            hall_high = msg->uid[0]->high;
            hall_low = msg->uid[0]->low;
            discovered = TRUE;
        }
        ssc_suid_response__free_unpacked(msg, NULL);
        return;
    }
    if (!discovered || high != hall_high || low != hall_low) return;
    MonitorState *msg = monitor_state__unpack(NULL, bytes->len, (guint8 *)bytes->data);
    if (!msg) return;
    if (id == ERROR_EVENT) {
        fprintf(stderr, "Hall sensor rejected the request (error %u)\n", msg->value);
        *rejected = TRUE;
    } else if (id == HALL_EVENT && msg->value <= 1)
        set_lid(msg->value);
    monitor_state__free_unpacked(msg, NULL);
}

/* Connect, find the Hall sensor and subscribe. Returns NULL if not ready yet. */
static SSCClient *subscribe(gboolean *rejected, gboolean quiet)
{
    SyncContext ctx;
    g_autoptr(GError) error = NULL;
    ssc_common_init_sync_context(&ctx);
    ssc_client_new(NULL, ssc_common_callback_sync_context, &ctx);
    ssc_common_wait_sync_context(&ctx);
    SSCClient *client = ssc_client_new_finish(ctx.result, &error);
    ssc_common_clear_sync_context(&ctx);
    if (!client) {
        if (!quiet) fprintf(stderr, "Cannot connect to SSC: %s\n", error->message);
        return NULL;
    }
    g_signal_connect(client, "report", G_CALLBACK(report), rejected);
    discovered = FALSE;
    SscSuidRequest req = SSC_SUID_REQUEST__INIT;
    req.data_type = "hall";
    req.has_enable_updates = req.has_only_default_values = TRUE;
    req.enable_updates = FALSE;
    req.only_default_values = TRUE;
    if (!send_request(client, SSC_SENSOR_UID_SUID_HIGH, SSC_SENSOR_UID_SUID_LOW,
                      SSC_MSG_REQUEST_SUID, &req.base) || !wait_for(&discovered, 5)) {
        if (!quiet && !stopping) fputs("No Hall sensor advertised yet\n", stderr);
        g_object_unref(client);
        return NULL;
    }
    *rejected = FALSE;
    if (!send_request(client, hall_high, hall_low, SSC_MSG_REQUEST_ENABLE_REPORT_ON_CHANGE, NULL)) {
        g_object_unref(client);
        return NULL;
    }
    fputs("Monitoring the Hall sensor\n", stderr);
    return client;
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        fprintf(stderr, "Usage: %s\nReport the SSC Hall sensor as an input lid switch.\n", argv[0]);
        return 2;
    }
    setvbuf(stderr, NULL, _IOLBF, 0);
    g_unix_signal_add(SIGINT, stop, NULL);
    g_unix_signal_add(SIGTERM, stop, NULL);
    if (!create_device()) return 1;
    gboolean rejected = FALSE, never = FALSE;
    for (guint attempt = 0; !stopping; attempt++) {
        /* Sensors register some seconds after the listener starts. */
        SSCClient *client = subscribe(&rejected, attempt && attempt % 60);
        if (client) {
            attempt = 0;
            while (!stopping && !rejected)
                g_main_context_iteration(NULL, TRUE);
            if (!rejected && !send_request(client, hall_high, hall_low, SSC_MSG_REQUEST_DISABLE_REPORT, NULL))
                failed = TRUE;
            g_object_unref(client);
        }
        if (!stopping) wait_for(&never, attempt < 60 ? 1 : 30);
    }
    ioctl(uinput, UI_DEV_DESTROY);
    close(uinput);
    return failed ? 1 : 0;
}
