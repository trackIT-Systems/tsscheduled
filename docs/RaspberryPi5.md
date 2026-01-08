# Raspberry Pi 5 Hardware Setup

This document provides hardware-specific setup instructions for using tsschedule with the Raspberry Pi 5.

## Overview

The Raspberry Pi 5 includes a built-in real-time clock (RTC) that can be used for scheduled wake-ups. Unlike add-on boards like WittyPi 4, the Pi 5's RTC is integrated into the main board and requires an external battery for operation when the main power is disconnected.

## Real-Time Clock (RTC)

The Raspberry Pi 5 features an on-board RTC (Real-Time Clock) that requires a CR2032 coin cell battery to maintain time when the system is powered off. The RTC is accessible via the standard Linux RTC interface.

### RTC Battery Installation

1. Locate the battery holder on the Raspberry Pi 5 board (near the GPIO header)
2. Insert a CR2032 coin cell battery with the positive (+) side facing up
3. Ensure the battery is securely seated

> **Note**: Without a battery, the RTC will not maintain time when the Pi is powered off, and scheduled wake-ups will not function correctly.

### Enabling the RTC

The RTC should be automatically detected by the Raspberry Pi OS kernel. To verify it's working:

```bash
# Check if RTC device is present
ls -l /dev/rtc*

# Read current RTC time
sudo hwclock -r

# Sync system time to RTC
sudo hwclock -w

# Sync RTC to system time (after NTP sync)
sudo hwclock -s
```

The RTC device is typically available as `/dev/rtc0` or `/dev/rtc`.

### RTC Alarm Configuration

The Raspberry Pi 5 RTC supports alarm functionality. However, note that Raspberry Pi does not support ACPI power states (suspend/hibernate), so `rtcwake` cannot be used to suspend the system.

> **Note**: Raspberry Pi does not support ACPI suspend states. The `rtcwake` command will not work for suspending the system. Use the sysfs interface directly instead.

For programmatic access, the RTC can be accessed via `/sys/class/rtc/rtc0/`:

```bash
# Read current RTC time
cat /sys/class/rtc/rtc0/time
cat /sys/class/rtc/rtc0/date

# Set alarm time
echo $(date -d "tomorrow 08:00" +%s) > /sys/class/rtc/rtc0/wakealarm

# Enable alarm
echo 0 > /sys/class/rtc/rtc0/wakealarm  # Clear first
echo $(date -d "tomorrow 08:00" +%s) > /sys/class/rtc/rtc0/wakealarm
```

## Power Management

### Power Button

The Raspberry Pi 5 includes a physical power button that can be used for:
- Power on (when system is off)
- Graceful shutdown (when system is running)

The power button functionality is handled by the firmware and requires no additional configuration.

### GPIO-Based Shutdown

The Raspberry Pi 5's built-in power button is connected to GPIO 3 (RUN pin) and is **automatically configured by the firmware**. No manual configuration is required for the power button to function.

If you need to add an **additional** GPIO pin for remote or automated shutdown (beyond the built-in power button), you can use the `gpio-shutdown` device tree overlay:

```ini
# Add to /boot/firmware/config.txt (only if you need an additional shutdown GPIO)
dtoverlay=gpio-shutdown,gpio_pin=<pin_number>,active_low=1
```

> **Note**: The built-in power button on GPIO 3 is handled automatically by the firmware and does not require this overlay configuration.

### Wake from RTC Alarm

The Raspberry Pi 5 **can wake from a powered-off state** using the built-in RTC alarm. This requires configuring the EEPROM to enable power-off on halt, which allows the Pi to enter a very low-power state (~0.01W) when shut down.

#### Enabling RTC Wake-Up

To enable RTC alarm wake-up, configure the EEPROM settings:

```bash
# Edit EEPROM configuration
sudo rpi-eeprom-config -e
```

Add or ensure these settings are present:

```ini
[all]
POWER_OFF_ON_HALT=1
WAKE_ON_GPIO=0
```

After saving and rebooting, the Raspberry Pi will power off completely when halted (instead of consuming 1.2-1.6W in a shutdown state). The RTC alarm can then wake the system back up.

> **Note**: By default, `POWER_OFF_ON_HALT=0` to maintain compatibility with some HATs that require the 3.3V rail to remain active. Setting it to `1` enables true power-off and RTC wake-up, but may cause issues with certain HATs.

#### Setting an RTC Alarm

To set an RTC alarm for scheduled wake-up:

```bash
# Clear any existing alarm
echo 0 > /sys/class/rtc/rtc0/wakealarm

# Set alarm for a specific time (Unix timestamp)
echo $(date -d "tomorrow 08:00" +%s) > /sys/class/rtc/rtc0/wakealarm

# Or set alarm relative to now (e.g., 10 minutes)
echo +600 > /sys/class/rtc/rtc0/wakealarm

# Verify alarm is set
cat /sys/class/rtc/rtc0/wakealarm
```

#### Testing RTC Wake-Up

To test the wake-up functionality:

```bash
# Set alarm for 10 minutes from now
echo +600 | sudo tee /sys/class/rtc/rtc0/wakealarm

# Shutdown the system (will enter low-power state)
sudo halt
```

The system will wake and restart automatically when the alarm fires. The power button continues to work normally for manual power-on.

> **Reference**: For more details on power consumption and RTC wake-up, see [Jeff Geerling's article on reducing Raspberry Pi 5 power consumption](https://www.jeffgeerling.com/blog/2023/reducing-raspberry-pi-5s-power-consumption-140x/).

## Python API Usage

The tsschedule library provides a `RaspberryPi5` backend that interfaces with the Linux RTC automatically.

### Quick Example

```python
from tsschedule.backends.raspberrypi5 import RaspberryPi5
import datetime

# Initialize Raspberry Pi 5 backend
pi5 = RaspberryPi5()

# Read RTC time
print(f"RTC Time: {pi5.rtc_datetime}")

# Schedule wake in 1 hour
pi5.set_startup_datetime(datetime.datetime.now() + datetime.timedelta(hours=1))

# Schedule shutdown in 30 minutes
pi5.set_shutdown_datetime(datetime.datetime.now() + datetime.timedelta(minutes=30))
```

### Using Hardware Detection

For code that works with both WittyPi 4 and Raspberry Pi 5:

```python
from tsschedule import detect_hardware
from tsschedule.backends.raspberrypi5 import RaspberryPi5
from tsschedule.backends.wittypi4 import WittyPi4
import smbus2

# Automatically detect hardware
hardware_type = detect_hardware()
if hardware_type == "wittypi4":
    bus = smbus2.SMBus(1, force=True)
    device = WittyPi4(bus)
elif hardware_type == "raspberrypi5":
    device = RaspberryPi5()
else:
    raise RuntimeError("No supported hardware detected")

# Common interface works for both
device.set_startup_datetime(...)
device.set_shutdown_datetime(...)
```

For complete API reference, see the [Python API Documentation](API.md).

## Hardware Specifications

### RTC Details
- **Type**: On-board RTC with external battery backup
- **Battery**: CR2032 coin cell (3V, ~220mAh)
- **Interface**: Linux RTC subsystem (`/dev/rtc0`, `/sys/class/rtc/rtc0/`)
- **Alarm Support**: Yes, via wakealarm interface
- **Accuracy**: Typically ±20ppm (about 1.7 seconds per day)

### Power Requirements
- **Input**: 5V/5A DC via USB-C
- **Power Delivery**: Supports USB Power Delivery (PD)
- **Power Button**: Physical button on board
- **Low Power States**: Suspend-to-RAM supported

## Troubleshooting

### RTC Not Maintaining Time

1. Verify battery is installed correctly (positive side up)
2. Check battery voltage (should be ~3V)
3. Ensure RTC is being used: `sudo hwclock -r`
4. Sync system time to RTC after NTP sync: `sudo hwclock -w`

### Alarm Not Waking System

1. Verify alarm is set: `cat /sys/class/rtc/rtc0/wakealarm`
2. Check RTC time is correct: `cat /sys/class/rtc/rtc0/time`
3. Ensure system supports wake from RTC (check BIOS/UEFI settings if applicable)
4. For full power-off scenarios, verify external power control hardware is configured

### RTC Device Not Found

1. Check kernel modules: `lsmod | grep rtc`
2. Verify device tree: `ls -l /dev/rtc*`
3. Check dmesg for RTC initialization: `dmesg | grep rtc`
4. Ensure Raspberry Pi OS is up to date

## Resources

### Official Documentation
- [Raspberry Pi 5 Product Brief](https://datasheets.raspberrypi.com/rpi5/raspberry-pi-5-product-brief.pdf)
- [Raspberry Pi 5 Documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi-5.html)

### Linux RTC Documentation
- [Linux RTC Subsystem](https://www.kernel.org/doc/html/latest/driver-api/rtc.html)
- [rtcwake man page](https://manpages.debian.org/rtcwake)

