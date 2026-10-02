# ssc-monitor

Lists and monitors Qualcomm Snapdragon Sensor Core (SSC) sensors over QRTR. The sensor
DSP and its registry must already be running.

```sh
ssc-monitor --list                  # advertised sensors
ssc-monitor --list --probe          # ...minus those that reject a request
ssc-monitor gyro hall rgb           # monitor until Ctrl+C
ssc-monitor --all --timeout 10      # every type marked ALL, for 10 s
ssc-monitor accel gyro --rate 25
ssc-monitor pedometer --raw
```

## Listing

`--list` prints each advertised sensor's type, name, mode, lowest advertised
rate, availability and whether `--all` includes it (`ALL`). `--raw` adds UIDs.
These are advertised attributes; they do not prove that a sensor works.

`--probe` enables every available sensor for two seconds (or `--timeout`),
then lists those the DSP did not reject. `PROBE` is `events` if any event
arrived and `quiet` otherwise; quiet is not evidence either way. `-v` shows
the skipped sensors and why.

## Monitoring

Names are DSP data types from `--list`, such as `gyro`, `mag`, `rotv` or
`ambient_light`. A type selects all of its instances. Any advertised type can
be named; it gets the standard continuous or on-change enable request, which
some types reject. `--all` covers the physical and derived sensor types and
skips those that reply `NOT_SUPPORTED` or `NOT_AVAILABLE`. For example, this
tablet's `delta_angle` rejects the standard request with `NOT_SUPPORTED`.

Continuous sensors use their lowest advertised rate unless `--rate` is given;
the firmware decides the actual rate. One-shot sensors are armed once.

Each line shows seconds since start, type, name and the decoded event:

- float samples in their original order: XYZ for accel, gyro and mag (µT),
  XYZW quaternions for rotation vectors; vendor fields beyond the known
  reading appear as `extra`
- `status`: 0=unreliable, 1=low, 2=medium, 3=high
- hall near/far, AMD motion/stationary, pedometer step counts
- anything else as event ID and payload hex (`--raw` adds hex to every event)

## JSON

`--json` prints one JSON object per line on stdout; messages stay on stderr.
Listing objects have `type`, `name`, `mode`, `min_hz` (null if not
advertised), `available`, `all`, `uid` and, with `--probe`, an `events` count.
Event objects have `time`, `type` and `name`, plus the same fields as the
text output (`values`, `unit`, `extra`, `status`, `state`, `steps`,
`initial_steps`, `event`, `hex`). Non-finite samples become `null`.

```sh
ssc-monitor --json accel | jq -c .values
```

## Errors and exit status

A DSP error closes its stream; the others keep running. On exit, including
Ctrl+C and SIGTERM, the tool disables only its own subscriptions.

Exit status is 1 if any stream failed (expected `--all` and probe rejections
excepted) or if every stream was rejected, and 2 for usage errors.

## Notes

libssc keeps its transport and discovery API private, so the tool builds
unchanged libssc 0.4.4 sources into the executable. It does not replace
libssc or iio-sensor-proxy. Event layouts follow Qualcomm's
[sensinghub v2.3.0 protocols](https://github.com/qualcomm/sensinghub/tree/v2.3.0/apis/proto)
(`sns_std_sensor`, `sns_std_type`, `sns_hall`, `sns_amd`, `sns_pedometer`).
