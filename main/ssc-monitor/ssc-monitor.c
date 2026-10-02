// SPDX-License-Identifier: GPL-3.0-or-later
#include "libssc-client-private.h"
#include "ssc-sensor-suid.pb-c.h"
#include "monitor.pb-c.h"
#include <glib-unix.h>
#include <float.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>

typedef struct {
    guint64 high, low;
    char *type, *name;
    int stream;
    float rate;
    gboolean available, attributes, selected, opened, rejected;
    guint events;
} Sensor;

static GPtrArray *sensors;
static gboolean discovered, stopping, failed, raw, all, probe, verbose, json;
static gint64 started;
static int type_width, name_width;

/* Qualcomm sns_std_type.proto: these describe the request, not chip presence. */
static const char *error_names[] = {
    "NO_ERROR", "FAILED", "NOT_SUPPORTED", "INVALID_TYPE",
    "INVALID_STATE", "INVALID_VALUE", "NOT_AVAILABLE", "POLICY"
};

/* Physical and derived sensor streams using standard enable requests.
 * Infrastructure (registry, camera, location, diagnostics) is explicit-only.
 * Named types outside this list can still be tried with standard requests.
 */
static const char *auto_types[] = {
    "accel", "gyro", "mag", "ambient_light", "rgb", "hall", "proximity",
    "pressure", "humidity", "sensor_temperature", "gravity", "linear_accel",
    "rotv", "game_rv", "geomag_rv", "fmv", "gyro_rot_matrix",
    "amd", "rmd", "tilt", "motion_detect", "sig_motion",
    "persist_motion_detect", "persist_stationary_detect", "offbody_detect",
    "pedometer", "pedometer_minute", "device_orient", "free_fall",
    "elevator_detect", "flight_detect", "micro_motion", "motion_recognition",
    "activity_recognition", "oplus_activity_recognition", "explorer_gyro",
    "ccd_gmd", "ccd_walk", "ccd_ttw", "delta_angle", "gyro_cal", "mag_cal", NULL
};

static gboolean stop(gpointer unused)
{
    stopping = TRUE;
    return G_SOURCE_CONTINUE;
}

static gboolean tick(gpointer unused)
{
    return G_SOURCE_CONTINUE;
}

static gboolean wait_for(gboolean *ready, const char *what)
{
    gint64 deadline = g_get_monotonic_time() + 5 * G_USEC_PER_SEC;
    while (!*ready && !stopping && g_get_monotonic_time() < deadline)
        g_main_context_iteration(NULL, TRUE);
    if (*ready) return TRUE;
    if (!stopping) fprintf(stderr, "Timed out waiting for %s\n", what);
    return FALSE;
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

static Sensor *find_sensor(guint64 high, guint64 low)
{
    for (guint i = 0; i < sensors->len; i++) {
        Sensor *s = g_ptr_array_index(sensors, i);
        if (s->high == high && s->low == low) return s;
    }
    return NULL;
}

static void free_sensor(gpointer data)
{
    Sensor *s = data;
    g_free(s->type);
    g_free(s->name);
    g_free(s);
}

/* Fields are rendered as "key=value" text or as members of a JSON object. */
static void add_key(GString *out, gboolean as_json, const char *key)
{
    if (out->len) g_string_append_c(out, as_json ? ',' : ' ');
    g_string_append_printf(out, as_json ? "\"%s\":" : "%s=", key);
}

static void add_string(GString *out, gboolean as_json, const char *value)
{
    g_string_append_printf(out, as_json ? "\"%s\"" : "%s", value);
}

static void add_floats(GString *out, gboolean as_json, const float *data, gsize from, gsize to)
{
    g_string_append_c(out, '[');
    for (gsize i = from; i < to; i++) {
        if (i > from) g_string_append(out, ", ");
        if (as_json && !isfinite(data[i])) g_string_append(out, "null");
        else g_string_append_printf(out, "%.7g", data[i]);
    }
    g_string_append_c(out, ']');
}

static char *json_quote(const char *text)
{
    GString *out = g_string_new("\"");
    for (const char *c = text; *c; c++) {
        if (*c == '"' || *c == '\\') g_string_append_printf(out, "\\%c", *c);
        else if ((guchar)*c < 0x20) g_string_append_printf(out, "\\u%04x", *c);
        else g_string_append_c(out, *c);
    }
    g_string_append_c(out, '"');
    return g_string_free(out, FALSE);
}

static char *format_event(const char *type, guint32 id, GArray *bytes, gboolean include_raw, gboolean as_json)
{
    GString *text = g_string_new(NULL);
    if (id == 1025) {
        MonitorSample *msg = monitor_sample__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        if (msg) {
            const char *unit = NULL;
            gsize count = msg->n_data;
            if (count >= 3) {
                if (!strcmp(type, "accel")) { unit = "m/s²"; count = 3; }
                if (!strcmp(type, "gyro")) { unit = "rad/s"; count = 3; }
                if (!strcmp(type, "mag")) { unit = "µT"; count = 3; }
            }
            if (count >= 1) {
                if (!strcmp(type, "sensor_temperature")) { unit = "°C"; count = 1; }
                if (!strcmp(type, "ambient_light")) { unit = "lux"; count = 1; }
            }
            add_key(text, as_json, "values");
            add_floats(text, as_json, msg->data, 0, count);
            if (unit && as_json) { add_key(text, TRUE, "unit"); add_string(text, TRUE, unit); }
            else if (unit) g_string_append_printf(text, " %s", unit);
            if (count < msg->n_data) {
                add_key(text, as_json, "extra");
                add_floats(text, as_json, msg->data, count, msg->n_data);
            }
            add_key(text, as_json, "status");
            g_string_append_printf(text, "%d", msg->status);
            monitor_sample__free_unpacked(msg, NULL);
        }
    } else if (id == 130) {
        /* Errors only go to stderr as text. */
        MonitorState *msg = monitor_state__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        if (msg) g_string_append_printf(text, "DSP ERROR %s (code=%u)",
            msg->value < G_N_ELEMENTS(error_names) ? error_names[msg->value] : "UNKNOWN", msg->value);
        monitor_state__free_unpacked(msg, NULL);
    } else if ((!strcmp(type, "hall") && id == 770) ||
               (!strcmp(type, "amd") && id == 772) ||
               (!strcmp(type, "pedometer") && (id == 1028 || id == 775))) {
        MonitorState *msg = monitor_state__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        if (msg) {
            const char *amd_states[] = {"unknown", "stationary", "motion"};
            if (!strcmp(type, "hall") && msg->value <= 1) {
                add_key(text, as_json, "state");
                add_string(text, as_json, msg->value ? "near" : "far");
            } else if (!strcmp(type, "amd") && msg->value <= 2) {
                add_key(text, as_json, "state");
                add_string(text, as_json, amd_states[msg->value]);
            } else {
                add_key(text, as_json, !strcmp(type, "pedometer") ? (id == 775 ? "initial_steps" : "steps") : "state");
                g_string_append_printf(text, "%u", msg->value);
            }
            if (msg->has_status) {
                add_key(text, as_json, "status");
                g_string_append_printf(text, "%d", msg->status);
            }
            monitor_state__free_unpacked(msg, NULL);
        }
    }
    if (!text->len || include_raw) {
        add_key(text, as_json, "event");
        g_string_append_printf(text, "%u", id);
        add_key(text, as_json, "hex");
        GString *hex = g_string_new(NULL);
        for (guint i = 0; i < bytes->len; i++)
            g_string_append_printf(hex, "%02x", (guint8)bytes->data[i]);
        add_string(text, as_json, hex->len || as_json ? hex->str : "<empty>");
        g_string_free(hex, TRUE);
    }
    return g_string_free(text, FALSE);
}

static void report(SSCClient *client, guint32 id, guint64 high, guint64 low,
                   GArray *bytes, gpointer unused)
{
    if (high == SSC_SENSOR_UID_SUID_HIGH && low == SSC_SENSOR_UID_SUID_LOW && id == SSC_MSG_RESPONSE_SUID) {
        SscSuidResponse *msg = ssc_suid_response__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        if (!msg) { fprintf(stderr, "Malformed discovery response\n"); failed = stopping = TRUE; return; }
        for (gsize i = 0; i < msg->n_uid; i++) {
            if (find_sensor(msg->uid[i]->high, msg->uid[i]->low)) continue;
            Sensor *s = g_new0(Sensor, 1);
            s->high = msg->uid[i]->high;
            s->low = msg->uid[i]->low;
            s->stream = -1;
            g_ptr_array_add(sensors, s);
        }
        discovered = TRUE;
        ssc_suid_response__free_unpacked(msg, NULL);
        return;
    }
    Sensor *s = find_sensor(high, low);
    if (!s) { fprintf(stderr, "Response from unknown sensor\n"); failed = stopping = TRUE; return; }
    if (id == SSC_MSG_RESPONSE_GET_ATTRIBUTES && !s->attributes) {
        SscAttrResponse *msg = ssc_attr_response__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        if (!msg) { fprintf(stderr, "Malformed attribute response\n"); failed = stopping = TRUE; return; }
        for (gsize i = 0; i < msg->n_attr; i++) {
            SscAttr *attr = msg->attr[i];
            if (!attr->value_array->n_v) continue;
            SscAttrValue *v = attr->value_array->v[0];
            if (attr->id == SSC_ATTRIBUTE_TYPE && v->s) { g_free(s->type); s->type = g_strdup(v->s); }
            if (attr->id == SSC_ATTRIBUTE_NAME && v->s) { g_free(s->name); s->name = g_strdup(v->s); }
            if (attr->id == SSC_ATTRIBUTE_AVAILABLE && v->has_b) s->available = v->b;
            if (attr->id == SSC_ATTRIBUTE_STREAM_TYPE && v->has_i) s->stream = v->i;
            if (attr->id == SSC_ATTRIBUTE_SAMPLE_RATE) {
                for (gsize j = 0; j < attr->value_array->n_v; j++) {
                    v = attr->value_array->v[j];
                    if (v->has_f && isfinite(v->f) && v->f > 0 && (!s->rate || v->f < s->rate)) s->rate = v->f;
                }
            }
        }
        s->attributes = TRUE;
        ssc_attr_response__free_unpacked(msg, NULL);
        if (!s->type || !s->name || s->stream < 0 || s->stream > 2) {
            fprintf(stderr, "Missing or unsupported sensor attributes\n");
            failed = stopping = TRUE;
        }
        return;
    }
    if (!s->selected || s->rejected) return;
    if (id == 130) {
        s->rejected = TRUE;
        /* Probing accepts any rejection; --all only NOT_SUPPORTED/NOT_AVAILABLE. */
        MonitorState *msg = monitor_state__unpack(NULL, bytes->len, (guint8 *)bytes->data);
        gboolean skip = msg && (probe || (all && (msg->value == 2 || msg->value == 6)));
        monitor_state__free_unpacked(msg, NULL);
        g_autofree char *text = format_event(s->type, id, bytes, raw, FALSE);
        if (!skip) {
            fprintf(stderr, "%s/%s: %s\n", s->type, s->name, text);
            failed = TRUE;
        } else if (verbose)
            fprintf(stderr, "Skipping %s/%s: %s\n", s->type, s->name, text);
        return;
    }
    s->events++;
    if (probe) return;
    double time = (g_get_monotonic_time() - started) / 1e6;
    g_autofree char *text = format_event(s->type, id, bytes, raw, json);
    int written;
    if (json) {
        g_autofree char *type = json_quote(s->type), *name = json_quote(s->name);
        written = printf("{\"time\":%.3f,\"type\":%s,\"name\":%s,%s}\n", time, type, name, text);
    } else
        written = printf("%8.3f %-*s %-*s %s\n", time, type_width, s->type, name_width, s->name, text);
    if (written < 0) stopping = failed = TRUE;
}

static gint sensor_compare(gconstpointer a, gconstpointer b)
{
    const Sensor *sa = *(Sensor *const *)a, *sb = *(Sensor *const *)b;
    int result = strcmp(sa->type, sb->type);
    return result ? result : strcmp(sa->name, sb->name);
}

static void fit_columns(gboolean selected_only)
{
    type_width = name_width = 4;
    for (guint i = 0; i < sensors->len; i++) {
        Sensor *s = g_ptr_array_index(sensors, i);
        if (selected_only && !s->selected) continue;
        type_width = MAX(type_width, (int)strlen(s->type));
        name_width = MAX(name_width, (int)strlen(s->name));
    }
}

int main(int argc, char **argv)
{
    gboolean list = FALSE;
    double rate = 0, timeout = 0;
    g_auto(GStrv) names = NULL;
    GOptionEntry options[] = {
        {"list", 'l', 0, G_OPTION_ARG_NONE, &list, "List advertised sensors", NULL},
        {"probe", 'p', 0, G_OPTION_ARG_NONE, &probe, "With --list, briefly enable each sensor and hide rejected ones", NULL},
        {"all", 'a', 0, G_OPTION_ARG_NONE, &all, "Monitor every type marked ALL in --list, skipping unsupported ones", NULL},
        {"rate", 'r', 0, G_OPTION_ARG_DOUBLE, &rate, "Sample rate for continuous sensors (default: lowest advertised)", "HZ"},
        {"timeout", 't', 0, G_OPTION_ARG_DOUBLE, &timeout, "Stop after SECONDS (default: until Ctrl+C; 2 with --probe)", "SECONDS"},
        {"raw", 0, 0, G_OPTION_ARG_NONE, &raw, "Show event IDs, payload hex and sensor UIDs", NULL},
        {"json", 'j', 0, G_OPTION_ARG_NONE, &json, "Print JSON Lines instead of text", NULL},
        {"verbose", 'v', 0, G_OPTION_ARG_NONE, &verbose, "Report opened, skipped and closed streams", NULL},
        {G_OPTION_REMAINING, 0, 0, G_OPTION_ARG_STRING_ARRAY, &names, NULL, NULL},
        {NULL}
    };
    g_autoptr(GOptionContext) options_ctx = g_option_context_new("[TYPE…]");
    g_option_context_set_summary(options_ctx, "Monitor Qualcomm Snapdragon Sensor Core (SSC) sensors.");
    g_option_context_add_main_entries(options_ctx, options, NULL);
    g_option_context_set_description(options_ctx,
        "Examples:\n"
        "  ssc-monitor --list --probe\n"
        "  ssc-monitor gyro hall rgb\n"
        "  ssc-monitor --all --timeout 10\n"
        "\n"
        "TYPE is a type from --list and selects all its instances. Any advertised\n"
        "type can be named, even if it is not marked ALL; the DSP may reject it.\n"
        "The PROBE column shows whether a probed sensor sent any event.\n"
        "Messages go to stderr, so stdout stays parseable with --json.\n"
        "Sample status: 0=unreliable, 1=low, 2=medium, 3=high.");
    g_autoptr(GError) error = NULL;
    if (!g_option_context_parse(options_ctx, &argc, &argv, &error)) {
        fprintf(stderr, "%s\n", error->message);
        return 2;
    }
    if (list + all + (names != NULL) != 1 || (probe && !list)) {
        fprintf(stderr, "Use one of --list [--probe], --all or TYPE…; see --help\n");
        return 2;
    }
    if (!(rate >= 0 && rate <= FLT_MAX) || !(timeout >= 0 && isfinite(timeout))) {
        fprintf(stderr, "--rate and --timeout must be non-negative numbers\n");
        return 2;
    }
    if (probe && !timeout) timeout = 2;
    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGPIPE, SIG_IGN);
    guint int_source = g_unix_signal_add(SIGINT, stop, NULL);
    guint term_source = g_unix_signal_add(SIGTERM, stop, NULL);
    guint tick_source = g_timeout_add(100, tick, NULL);
    sensors = g_ptr_array_new_with_free_func(free_sensor);
    SyncContext ctx;
    ssc_common_init_sync_context(&ctx);
    ssc_client_new(NULL, ssc_common_callback_sync_context, &ctx);
    ssc_common_wait_sync_context(&ctx);
    SSCClient *client = ssc_client_new_finish(ctx.result, &error);
    ssc_common_clear_sync_context(&ctx);
    gulong handler = 0;
    if (!client) { fprintf(stderr, "Cannot connect to SSC: %s\n", error->message); failed = TRUE; goto cleanup; }
    handler = g_signal_connect(client, "report", G_CALLBACK(report), NULL);
    SscSuidRequest req = SSC_SUID_REQUEST__INIT;
    req.data_type = "";
    req.has_enable_updates = req.has_only_default_values = TRUE;
    req.enable_updates = req.only_default_values = FALSE;
    if (!send_request(client, SSC_SENSOR_UID_SUID_HIGH, SSC_SENSOR_UID_SUID_LOW,
                       SSC_MSG_REQUEST_SUID, &req.base) || !wait_for(&discovered, "sensor discovery")) {
        failed = TRUE; goto cleanup;
    }
    if (!sensors->len) { fprintf(stderr, "SSC advertised no sensors\n"); failed = TRUE; goto cleanup; }
    for (guint i = 0; i < sensors->len && !stopping; i++) {
        Sensor *s = g_ptr_array_index(sensors, i);
        SscAttrRequest attr = SSC_ATTR_REQUEST__INIT;
        attr.has_enable_updates = TRUE;
        if (!send_request(client, s->high, s->low, SSC_MSG_REQUEST_GET_ATTRIBUTES, &attr.base) ||
            !wait_for(&s->attributes, "sensor attributes")) { failed = TRUE; goto cleanup; }
    }
    if (stopping) goto cleanup;
    g_ptr_array_sort(sensors, sensor_compare);
    if (list && !probe) goto show_list;
    /* Resolve every requested name before enabling any stream. */
    for (char **name = names; name && *name; name++) {
        gboolean found = FALSE;
        for (guint i = 0; i < sensors->len; i++) {
            Sensor *s = g_ptr_array_index(sensors, i);
            if (!strcmp(*name, s->type)) { s->selected = TRUE; found = TRUE; }
        }
        if (!found) { fprintf(stderr, "Unknown sensor type '%s'; see --list\n", *name); failed = TRUE; }
    }
    guint selected = 0;
    for (guint i = 0; i < sensors->len; i++) {
        Sensor *s = g_ptr_array_index(sensors, i);
        if (all) s->selected = s->available && g_strv_contains(auto_types, s->type);
        if (probe) s->selected = TRUE;
        if (!s->selected) continue;
        const char *problem = !s->available ? "not available" :
            s->stream == 0 && !rate && !s->rate ? "no advertised sample rate; use --rate" : NULL;
        if (problem) {
            if (!probe) failed = TRUE;
            if (!probe || verbose) fprintf(stderr, "%s%s/%s: %s\n", probe ? "Skipping " : "", s->type, s->name, problem);
            s->selected = FALSE;
            continue;
        }
        selected++;
    }
    if (!selected) { fprintf(stderr, "No sensors selected\n"); failed = TRUE; }
    if (failed) goto cleanup;
    fit_columns(TRUE);
    fprintf(stderr, "%s %u sensor%s ", probe ? "Probing" : "Monitoring", selected, selected == 1 ? "" : "s");
    if (timeout) fprintf(stderr, "for %g s\n", timeout);
    else fputs("until Ctrl+C\n", stderr);
    started = g_get_monotonic_time();
    for (guint i = 0; i < sensors->len && !stopping; i++) {
        Sensor *s = g_ptr_array_index(sensors, i);
        if (!s->selected) continue;
        if (verbose) fprintf(stderr, "Opening %s/%s\n", s->type, s->name);
        s->opened = TRUE;
        gboolean ok;
        if (s->stream == 0) {
            SscEnableConfigRequest config = SSC_ENABLE_CONFIG_REQUEST__INIT;
            config.sample_rate = rate ? rate : s->rate;
            ok = send_request(client, s->high, s->low, SSC_MSG_REQUEST_ENABLE_REPORT_CONTINUOUS, &config.base);
        } else {
            ok = send_request(client, s->high, s->low, SSC_MSG_REQUEST_ENABLE_REPORT_ON_CHANGE, NULL);
        }
        if (!ok) { failed = stopping = TRUE; break; }
    }
    if (!stopping) {
        gint64 watch_start = g_get_monotonic_time();
        while (!stopping && (!timeout || (g_get_monotonic_time() - watch_start) / 1e6 < timeout)) {
            guint active = 0;
            for (guint i = 0; i < sensors->len; i++) {
                Sensor *s = g_ptr_array_index(sensors, i);
                if (!s->opened) continue;
                if (s->rejected) {
                    /* Disable outside the report callback: send_request dispatches
                     * the main loop and must not reenter report during teardown. */
                    if (!send_request(client, s->high, s->low, SSC_MSG_REQUEST_DISABLE_REPORT, NULL)) {
                        failed = stopping = TRUE;
                        break;
                    }
                    s->opened = FALSE;
                } else active++;
            }
            if (stopping) break;
            if (!active) {
                fprintf(stderr, "All streams were rejected\n");
                failed = TRUE;
                break;
            }
            g_main_context_iteration(NULL, TRUE);
        }
    }

show_list:
    if (list && !stopping) {
        if (!probe) fit_columns(FALSE);
        const char *modes[] = {"continuous", "on-change", "one-shot"};
        if (!json) {
            printf("%-*s %-*s %-10s %6s %-5s %-3s", type_width, "TYPE", name_width, "NAME",
                   "MODE", "MIN-HZ", "AVAIL", "ALL");
            if (probe) printf(" %-6s", "PROBE");
            if (raw) printf(" UID");
            puts("");
        }
        for (guint i = 0; i < sensors->len; i++) {
            Sensor *s = g_ptr_array_index(sensors, i);
            if (probe && (!s->selected || s->rejected)) continue;
            gboolean in_all = g_strv_contains(auto_types, s->type);
            g_autofree char *uid = g_strdup_printf("%016" G_GINT64_MODIFIER "x:%016" G_GINT64_MODIFIER "x",
                                                   s->high, s->low);
            if (json) {
                g_autofree char *type = json_quote(s->type), *name = json_quote(s->name);
                g_autofree char *hz = s->rate ? g_strdup_printf("%g", s->rate) : g_strdup("null");
                printf("{\"type\":%s,\"name\":%s,\"mode\":\"%s\",\"min_hz\":%s,\"available\":%s,\"all\":%s",
                       type, name, modes[s->stream], hz, s->available ? "true" : "false", in_all ? "true" : "false");
                if (probe) printf(",\"events\":%u", s->events);
                printf(",\"uid\":\"%s\"}\n", uid);
                continue;
            }
            g_autofree char *hz = s->rate ? g_strdup_printf("%g", s->rate) : g_strdup("-");
            printf("%-*s %-*s %-10s %6s %-5s %-3s", type_width, s->type, name_width, s->name,
                   modes[s->stream], hz, s->available ? "yes" : "no", in_all ? "yes" : "no");
            if (probe) printf(" %-6s", s->events ? "events" : "quiet");
            if (raw) printf(" %s", uid);
            puts("");
        }
        if (ferror(stdout)) failed = TRUE;
    }

cleanup:
    for (guint i = 0; i < sensors->len; i++) {
        Sensor *s = g_ptr_array_index(sensors, i);
        if (!s->opened) continue;
        if (send_request(client, s->high, s->low, SSC_MSG_REQUEST_DISABLE_REPORT, NULL)) {
            if (verbose) fprintf(stderr, "Closed %s/%s (%u events)\n", s->type, s->name, s->events);
        } else {
            fprintf(stderr, "Failed to close %s/%s\n", s->type, s->name);
            failed = TRUE;
        }
    }
    if (handler) g_signal_handler_disconnect(client, handler);
    g_clear_object(&client);
    g_ptr_array_unref(sensors);
    g_source_remove(int_source);
    g_source_remove(term_source);
    g_source_remove(tick_source);
    return failed ? 1 : 0;
}
