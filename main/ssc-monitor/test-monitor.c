// SPDX-License-Identifier: GPL-3.0-or-later
// Decode regressions using payloads captured from the OnePlus Pad 2.
#define main monitor_main
#include "ssc-monitor.c"
#undef main

static GArray *parse_hex(const char *hex)
{
    GArray *bytes = g_array_new(FALSE, FALSE, 1);
    for (gsize i = 0; i < strlen(hex); i += 2) {
        guint8 value = (g_ascii_xdigit_value(hex[i]) << 4) | g_ascii_xdigit_value(hex[i + 1]);
        g_array_append_val(bytes, value);
    }
    return bytes;
}

static void check(const char *type, guint id, const char *hex, const char *expected, gboolean with_raw)
{
    g_autoptr(GArray) bytes = parse_hex(hex);
    g_autofree char *text = format_event(type, id, bytes, with_raw, FALSE);
    g_assert_cmpstr(text, ==, expected);
}

static void check_json(const char *type, guint id, const char *hex, const char *expected)
{
    g_autoptr(GArray) bytes = parse_hex(hex);
    g_autofree char *text = format_event(type, id, bytes, FALSE, TRUE);
    g_assert_cmpstr(text, ==, expected);
}

/* Deliver a DSP error event to the only sensor; returns whether it counted as a failure. */
static gboolean rejection_fails(const char *hex)
{
    Sensor *s = g_ptr_array_index(sensors, 0);
    g_autoptr(GArray) bytes = parse_hex(hex);
    s->rejected = failed = FALSE;
    report(NULL, 130, s->high, s->low, bytes, NULL);
    g_assert_true(s->rejected);
    g_assert_true(s->opened); /* The main loop closes the request, not the callback. */
    g_assert_cmpuint(s->events, ==, 0); /* Errors are not sensor data. */
    return failed;
}

int main(void)
{
    check("sensor_temperature", 1025, "0a045f1d07421000", "values=[33.77868] °C status=0", FALSE);
    check("gyro", 1025, "0a0c32c8523b2e154e3cd3caa33c1000",
          "values=[0.003216278, 0.01257829, 0.01999418] rad/s status=0", FALSE);
    check("hall", 770, "08001003", "state=far status=3", FALSE);
    check("ambient_light", 1025, "0a140000b6420000000000000000000000000000803f1003",
          "values=[91] lux extra=[0, 0, 0, 1] status=3", FALSE);
    check("hall", 770, "08011003", "state=near status=3 event=770 hex=08011003", TRUE);
    check("pedometer", 1028, "0800", "steps=0", FALSE);
    check("vendor", 888, "0800", "event=888 hex=0800", FALSE);
    check("tilt", 774, "", "event=774 hex=<empty>", FALSE);
    check("gyro", 1025, "0a0c32", "event=1025 hex=0a0c32", FALSE);
    check("gyro", 130, "0803", "DSP ERROR INVALID_TYPE (code=3)", FALSE);
    check("gyro", 130, "0802", "DSP ERROR NOT_SUPPORTED (code=2)", FALSE);
    check("gyro", 130, "0806", "DSP ERROR NOT_AVAILABLE (code=6)", FALSE);
    check("gyro", 130, "0863", "DSP ERROR UNKNOWN (code=99)", FALSE);
    check("pedometer", 775, "0805", "initial_steps=5", FALSE);

    check_json("gyro", 1025, "0a0c32c8523b2e154e3cd3caa33c1000",
               "\"values\":[0.003216278, 0.01257829, 0.01999418],\"unit\":\"rad/s\",\"status\":0");
    check_json("ambient_light", 1025, "0a140000b6420000000000000000000000000000803f1003",
               "\"values\":[91],\"unit\":\"lux\",\"extra\":[0, 0, 0, 1],\"status\":3");
    check_json("rgb", 1025, "0a040000c07f1000", "\"values\":[null],\"status\":0");
    check_json("hall", 770, "08001003", "\"state\":\"far\",\"status\":3");
    check_json("tilt", 774, "", "\"event\":774,\"hex\":\"\"");
    g_autofree char *quoted = json_quote("A \"b\"\\\n");
    g_assert_cmpstr(quoted, ==, "\"A \\\"b\\\"\\\\\\u000a\"");

    sensors = g_ptr_array_new_with_free_func(free_sensor);
    Sensor *sensor = g_new0(Sensor, 1);
    sensor->type = g_strdup("delta_angle");
    sensor->name = g_strdup("delta_angle");
    sensor->high = 1;
    sensor->low = 2;
    sensor->selected = sensor->opened = TRUE;
    g_ptr_array_add(sensors, sensor);

    all = TRUE;
    g_assert_false(rejection_fails("0802")); /* --all skips NOT_SUPPORTED */
    g_assert_false(rejection_fails("0806")); /* and NOT_AVAILABLE */
    g_assert_true(rejection_fails("0801"));  /* but not real failures. */
    failed = FALSE;
    g_autoptr(GArray) error = parse_hex("0801");
    report(NULL, 130, 1, 2, error, NULL);
    g_assert_false(failed); /* A rejected stream ignores further errors. */

    all = FALSE;
    g_assert_true(rejection_fails("0802")); /* Explicitly named sensors keep errors. */

    probe = TRUE;
    g_assert_false(rejection_fails("0802")); /* Every DSP rejection is a probe result. */
    g_assert_false(rejection_fails("0863"));
    g_assert_false(rejection_fails("0801"));
    g_assert_true(rejection_fails(""));      /* Malformed errors still fail. */
    sensor->rejected = FALSE;
    g_autoptr(GArray) event = parse_hex("0800");
    report(NULL, 888, 1, 2, event, NULL);
    g_assert_cmpuint(sensor->events, ==, 1); /* Probing counts events without printing them. */

    g_ptr_array_unref(sensors);
    return 0;
}
