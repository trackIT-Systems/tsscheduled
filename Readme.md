tsOS Schedule Daemon (tsschedule)
---

This repository provides a schedule-based power management daemon for tsOS, supporting multiple hardware backends including WittyPi 4 and Raspberry Pi 5.

tsschedule enables automated power management based on time-based schedules, supporting:
- Absolute time schedules (e.g., "10:00" to "13:00")
- Astronomical event schedules (e.g., "sunrise-01:00" to "sunset+00:30")
- Multiple overlapping schedule entries
- Manual power-on with configurable delay
- Real-time clock (RTC) integration for scheduled wake-ups

# Installation

Install the library using pip or pdm:

```bash
# Using pip
pip install -e .

# Using pdm
pdm install
```

# Python API

This repository includes a Python library for programmatic control of schedule-based power management with support for multiple hardware backends. The library **automatically detects** available hardware (WittyPi 4 or Raspberry Pi 5).

## Quick Example

### Schedule Configuration

```python
from tsschedule import ScheduleConfiguration
import datetime

# Load schedule configuration
config = {
    'lat': 50.85318, 'lon': 8.78735,
    'tz': 'Europe/Berlin',  # optional, defaults to system timezone
    'schedule': [
        {'name': 'morning', 'start': 'sunrise-01:00', 'stop': '12:00'},
        {'name': 'evening', 'start': '18:00', 'stop': 'sunset+01:00'}
    ]
}
sc = ScheduleConfiguration(config)

# Get next scheduled startup and shutdown times
print(f"Next startup: {sc.next_startup()}")
print(f"Next shutdown: {sc.next_shutdown()}")
print(f"Currently active: {sc.active()}")
```

### Hardware Backend Usage with Auto-Detection

```python
from tsschedule import detect_hardware
from tsschedule.backends.raspberrypi5 import RaspberryPi5
from tsschedule.backends.wittypi4 import WittyPi4
import smbus2
import datetime

# Automatically detect hardware
hardware_type = detect_hardware()
if hardware_type == "wittypi4":
    bus = smbus2.SMBus(1, force=True)
    device = WittyPi4(bus)
elif hardware_type == "raspberrypi5":
    device = RaspberryPi5()
else:
    raise RuntimeError("No supported hardware detected")

# Use common interface
device.set_startup_datetime(datetime.datetime.now() + datetime.timedelta(hours=1))
```

For hardware-specific backend usage and setup instructions, see the backend documentation:
- [WittyPi 4 Setup](docs/WittyPi4.md) - Hardware setup for WittyPi 4 power management board
- [Raspberry Pi 5 Setup](docs/RaspberryPi5.md) - Hardware setup for Raspberry Pi 5 with built-in RTC

## Running the Daemon

The `tsscheduled` daemon manages schedules automatically and **automatically detects** which hardware backend is available (WittyPi 4 or Raspberry Pi 5):

```bash
# Run with default schedule.yml
tsscheduled

# Run with custom schedule file
tsscheduled -s /path/to/schedule.yml

# Verbose output
tsscheduled -vv
```

The daemon:
- **Automatically detects** available hardware (WittyPi 4 or Raspberry Pi 5)
- Validates RTC time against system clock sources
- Loads schedule configuration from YAML file
- Continuously updates startup/shutdown alarms based on schedule
- Handles manual power-on events with configurable delay
- Responds to SIGTERM/SIGINT for graceful shutdown

For production use, install as a systemd service. See backend-specific documentation for hardware setup details.

## Schedule Configuration

Schedules are defined in YAML format. See [schedule.yml](schedule.yml) for an example configuration.

Key features:
- **Location-based**: Configure latitude/longitude for astronomical calculations
- **Timezone**: Configure timezone (e.g., "Europe/Berlin") for schedule calculations (optional, defaults to system timezone)
- **Multiple schedules**: Define multiple overlapping time windows
- **Astronomical events**: Use `sunrise`, `sunset`, `dawn`, `dusk` with offsets
- **Force-on mode**: Disable automatic shutdowns when needed
- **Button delay**: Configure how long to stay on after manual power-on

Example schedule entry:
```yaml
schedule:
  - name: morning
    start: sunrise-01:00
    stop: 12:00
  - name: evening
    start: 18:00
    stop: sunset+01:00
```

## Documentation

For complete information, see:
- [Python API Documentation](docs/API.md) - Complete API reference for developers
- [WittyPi 4 Setup](docs/WittyPi4.md) - Hardware-specific setup for WittyPi 4 power management board
- [Raspberry Pi 5 Setup](docs/RaspberryPi5.md) - Hardware-specific setup for Raspberry Pi 5 with built-in RTC
- [schedule.yml](schedule.yml) - Example schedule configuration
