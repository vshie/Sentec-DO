# Sentec DO Extension for BlueOS

A BlueOS extension that reads dissolved oxygen (DO) and temperature from a
Sentec OXYnor RS485 sensor over a BLUART USB-to-RS485 adapter, graphs the data
in the BlueOS web UI, logs to CSV, and forwards values to the autopilot's
`.BIN` log via MAVLink2Rest.

## Features

- Modbus RTU communication with the Sentec OXYnor (default 19200 8N2, slave ID 1)
- Real-time dashboard for DO and temperature
- `/widget` endpoint for embedding the live DO graph in BlueOS
- CSV logging with automatic rotation at 10 MB
- Forwards `DO` (dissolved oxygen) and `TDO` (temperature) to MAVLink2Rest as
  `NAMED_VALUE_FLOAT` so they show up in the autopilot's `.BIN` log
- Selectable serial port (defaults to `/dev/ttyUSB0`)
- GPS position logged alongside each measurement when available

## Hardware

- Sentec OXYnor RS485 dissolved oxygen sensor
- Blue Robotics BLUART USB-to-RS485 adapter (FTDI FT232 based)
- A BlueOS host (Raspberry Pi)

## Installation

The published Docker image is **`vshie/blueos-sentec-do`** on Docker Hub.

### From the BlueOS Extension Manager (recommended)

Once Blue Robotics indexes the extension, it will appear in the BlueOS
**Extensions** tab. Click *Install* and the manager handles everything.

### Manual install (until the extension is indexed)

In BlueOS, open **Extensions -> Installed -> + (Add Extension)** and use:

- **Extension Identifier**: `vshie.blueos-sentec-do`
- **Extension Name**: `Sentec DO`
- **Docker image**: `vshie/blueos-sentec-do`
- **Docker tag**: `main`
- **Original Settings (Permissions)**: paste the JSON below

```json
{
  "ExposedPorts": {
    "6438/tcp": {}
  },
  "HostConfig": {
    "CpuPeriod": 100000,
    "CpuQuota": 100000,
    "Binds": [
      "/usr/blueos/extensions/sentec-do:/app/logs",
      "/dev/ttyUSB0:/dev/ttyUSB0",
      "/dev/ttyUSB1:/dev/ttyUSB1",
      "/dev/ttyUSB2:/dev/ttyUSB2",
      "/dev/ttyUSB3:/dev/ttyUSB3",
      "/dev/ttyACM0:/dev/ttyACM0",
      "/dev/ttyACM1:/dev/ttyACM1"
    ],
    "ExtraHosts": ["host.docker.internal:host-gateway"],
    "PortBindings": {
      "6438/tcp": [
        {
          "HostPort": ""
        }
      ]
    },
    "NetworkMode": "host",
    "Privileged": true
  }
}
```

The deploy action tags the image with the branch name, so commits on `main`
publish `vshie/blueos-sentec-do:main`.

After install, open the extension card and click *View* to access the
dashboard. The live DO graph for embedding (e.g. in a BlueOS cockpit) is
served at `/widget`.

## Sensor protocol

This extension uses the OXYnor Modbus RTU protocol (the alternative ASCII
protocol is not used). Default serial framing is `19200 baud, 8 data bits,
2 stop bits, no parity` with slave ID `1`. The poll loop reads holding-register
block `4895` (14 registers) every 5 seconds and decodes:

| Field | Offset (regs) | Type |
| --- | --- | --- |
| Pressure (hPa) | 0 | float |
| Reference amplitude (uV) | 2 | float |
| Oxygen amplitude (uV) | 4 | float |
| Oxygen phase shift (deg) | 6 | float |
| Temperature (deg C) | 8 | float |
| Calculated oxygen | 10 | float |
| Error register | 12 | integer |

Floats are 2 registers wide with the **low word first**, big-endian within each
word. The oxygen unit code is read from register `2089` and reflected in the
UI label (`% air saturation`, `mg/L`, etc.).

## MAVLink2Rest

Two `NAMED_VALUE_FLOAT` messages are published every poll (5 s):

- `DO`  - dissolved oxygen in the sensor's currently configured unit
- `TDO` - sensor temperature in degrees C

They are sent with `system_id` 255 and a **unique `component_id` per metric**
(`DO` = 70, `TDO` = 71). This matters: mavlink2rest keys its store by
system/component/message_type, so sending both names from one component makes
them overwrite each other (only the last survives), and `component_id` 0
(`MAV_COMP_ID_ALL`) is an invalid source the autopilot ignores. With distinct
non-zero components both values persist in the inspector
(`/v1/mavlink/vehicles/255/components/70|71/messages/NAMED_VALUE_FLOAT`) and are
logged to the autopilot `.BIN`. The chosen base (70) stays clear of the BlueOS
PH/TEMP/SAL/COND range (25-28) and the Mikrotik-Monitor range (60-66).

## Build / deploy

Pushed commits trigger the `Deploy BlueOS Extension Image` GitHub action.
Configure the following repository secrets/variables before the first push:

- Secrets: `DOCKER_USERNAME`, `DOCKER_PASSWORD`
- (Optional) Variables: `IMAGE_NAME` (defaults to `sentec-do`), `MY_NAME`,
  `MY_EMAIL`, `ORG_NAME`, `ORG_EMAIL`

## Reference utilities

The `tools/` directory contains the standalone Python scripts used to probe
and verify the sensor:

- `tools/probe_sensor.py` - tries Modbus RTU and ASCII at several baud rates
- `tools/read_do.py` - decodes the live measurement block from the sensor

Run on the BlueOS host with:

```bash
python3 tools/read_do.py /dev/ttyUSB0
```
