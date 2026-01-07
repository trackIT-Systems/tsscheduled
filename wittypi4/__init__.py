"""WittyPi 4 Python Library.

This module provides a Python interface for controlling and monitoring power
management hardware for Raspberry Pi. It supports:

- WittyPi 4 HAT: Full-featured power management with RTC, voltage/current
  monitoring, and temperature sensing
- Raspberry Pi 5: Onboard RTC-based sleep/wake functionality

Features:
- Real-time clock (RTC) operations
- Scheduled startup and shutdown via alarms
- Schedule-based power management with sunrise/sunset support
- Voltage, current, and temperature monitoring (WittyPi 4 only)
- Low voltage protection and power mode management (WittyPi 4 only)

Main Classes:
    PowerManager: Base class for power management devices
    WittyPi4: Primary interface to WittyPi 4 hardware
    RaspberryPi5: Interface to Raspberry Pi 5 onboard RTC
    ScheduleConfiguration: Manages scheduling logic for startup/shutdown
    ActionReason: Enum of possible startup/shutdown reasons
    ButtonEntry: Handles manual power-on scheduling

Example:
    Basic usage with WittyPi 4:

    >>> import smbus2
    >>> from wittypi4 import WittyPi4
    >>> bus = smbus2.SMBus(1, force=True)
    >>> wp = WittyPi4(bus)
    >>> print(wp.voltage_in, wp.voltage_out, wp.current_out)
    >>> wp.set_startup_datetime(datetime.datetime.now() + datetime.timedelta(hours=1))

    Basic usage with Raspberry Pi 5:

    >>> from wittypi4 import RaspberryPi5
    >>> pi5 = RaspberryPi5()
    >>> pi5.set_startup_datetime(datetime.datetime.now() + datetime.timedelta(hours=1))
"""

import collections.abc
import datetime
import enum
import importlib
import logging
import os
import pathlib
import platform
import subprocess
import tempfile
import time

import astral
import pytimeparse
import smbus2
from scheduleparse import ScheduleEntry

__version__ = importlib.metadata.version(__name__)

logger = logging.getLogger("wittypi4")

# Device Adress
I2C_MC_ADDRESS = 0x08

# Read registers
I2C_ID = 0
I2C_VOLTAGE_IN_I = 1
I2C_VOLTAGE_IN_D = 2
I2C_VOLTAGE_OUT_I = 3
I2C_VOLTAGE_OUT_D = 4
I2C_CURRENT_OUT_I = 5
I2C_CURRENT_OUT_D = 6
I2C_POWER_MODE = 7
I2C_LV_SHUTDOWN = 8
I2C_ALARM1_TRIGGERED = 9
I2C_ALARM2_TRIGGERED = 10
I2C_ACTION_REASON = 11
I2C_FW_REVISION = 12

# Configuration registers
I2C_CONF_ADDRESS = 16
I2C_CONF_DEFAULT_ON = 17
I2C_CONF_PULSE_INTERVAL = 18
I2C_CONF_LOW_VOLTAGE = 19
I2C_CONF_BLINK_LED = 20
I2C_CONF_POWER_CUT_DELAY = 21
I2C_CONF_RECOVERY_VOLTAGE = 22
I2C_CONF_DUMMY_LOAD = 23
I2C_CONF_ADJ_VIN = 24
I2C_CONF_ADJ_VOUT = 25
I2C_CONF_ADJ_IOUT = 26

# Startup alarm
I2C_CONF_SECOND_ALARM1 = 27
I2C_CONF_MINUTE_ALARM1 = 28
I2C_CONF_HOUR_ALARM1 = 29
I2C_CONF_DAY_ALARM1 = 30
I2C_CONF_WEEKDAY_ALARM1 = 31

# Shutdown alarm
I2C_CONF_SECOND_ALARM2 = 32
I2C_CONF_MINUTE_ALARM2 = 33
I2C_CONF_HOUR_ALARM2 = 34
I2C_CONF_DAY_ALARM2 = 35
I2C_CONF_WEEKDAY_ALARM2 = 36

I2C_CONF_RTC_OFFSET = 37
I2C_CONF_RTC_ENABLE_TC = 38
I2C_CONF_FLAG_ALARM1 = 39
I2C_CONF_FLAG_ALARM2 = 40

I2C_CONF_IGNORE_POWER_MODE = 41
I2C_CONF_IGNORE_LV_SHUTDOWN = 42

I2C_CONF_BELOW_TEMP_ACTION = 43
I2C_CONF_BELOW_TEMP_POINT = 44
I2C_CONF_OVER_TEMP_ACTION = 45
I2C_CONF_OVER_TEMP_POINT = 46
I2C_CONF_DEFAULT_ON_DELAY = 47
I2C_CONF_MISC = 48
I2C_CONF_GUARANTEED_WAKE = 49

I2C_LM75B_TEMPERATURE = 50
I2C_LM75B_CONF = 51
I2C_LM75B_THYST = 52
I2C_LM75B_TOS = 53

I2C_RTC_CTRL1 = 54
I2C_RTC_CTRL2 = 55
I2C_RTC_OFFSET = 56
I2C_RTC_RAM_BYTE = 57
I2C_RTC_SECONDS = 58
I2C_RTC_MINUTES = 59
I2C_RTC_HOURS = 60
I2C_RTC_DAYS = 61
I2C_RTC_WEEKDAYS = 62
I2C_RTC_MONTHS = 63
I2C_RTC_YEARS = 64
I2C_RTC_SECOND_ALARM = 65
I2C_RTC_MINUTE_ALARM = 66
I2C_RTC_HOUR_ALARM = 67
I2C_RTC_DAY_ALARM = 68
I2C_RTC_WEEKDAY_ALARM = 69
I2C_RTC_TIMER_VALUE = 70
I2C_RTC_TIMER_MODE = 71

# GPIO Pins
HALT_PIN = 4  # halt by GPIO-4 (BCM naming)
SYSUP_PIN = 17  # output SYS_UP signal on GPIO-17 (BCM naming)
CHRG_PIN = 5  # input to detect charging status
STDBY_PIN = 6  # input to detect standby status

# Values
ALARM_RESET = 80


def bcd2bin(value):
    """Convert Binary-Coded Decimal (BCD) to binary integer.

    Args:
        value: BCD encoded value (e.g., 0x23 represents decimal 23)

    Returns:
        Integer representation of the BCD value
    """
    return value - 6 * (value >> 4)


def bin2bcd(value):
    """Convert binary integer to Binary-Coded Decimal (BCD).

    Args:
        value: Integer value to convert (0-99)

    Returns:
        BCD encoded value (e.g., 23 becomes 0x23)
    """
    return value + 6 * (value // 10)


class ActionReason(enum.Enum):
    """Enumeration of possible reasons for WittyPi 4 power state changes.

    These values indicate why the Raspberry Pi was powered on or off, and can be
    read from the WittyPi4.action_reason property.

    Attributes:
        ALARM_STARTUP: Scheduled startup via Alarm 1
        ALARM_SHUTDOWN: Scheduled shutdown via Alarm 2
        BUTTON_CLICK: Manual power button press
        LOW_VOLTAGE: Shutdown triggered by low input voltage
        VOLTAGE_RESTORE: Startup after voltage restored above threshold
        OVER_TEMPERATURE: Shutdown triggered by high temperature
        BELOW_TEMPERATURE: Shutdown triggered by low temperature
        ALARM_STARTUP_DELAYED: Startup alarm with configured delay
        POWER_CONNECTED: Power source connected
        REBOOT: System reboot
        GUARANTEED_WAKE: Startup via guaranteed wake feature
    """

    REASON_NA = 0x00
    ALARM_STARTUP = 0x01
    ALARM_SHUTDOWN = 0x02
    BUTTON_CLICK = 0x03
    LOW_VOLTAGE = 0x04
    VOLTAGE_RESTORE = 0x05
    OVER_TEMPERATURE = 0x06
    BELOW_TEMPERATURE = 0x07
    ALARM_STARTUP_DELAYED = 0x08
    POWER_CONNECTED = 0x0A
    REBOOT = 0x0B
    GUARANTEED_WAKE = 0x0C

    @classmethod
    def _missing_(cls, value: object) -> int:
        if isinstance(value, int):
            # For integer values, return a custom object
            logger.warning("ActionReason %i is unknown!", value)
            return 0
        else:
            # For non-integer values, raise a ValueError
            raise ValueError(f"{value} is not a valid {cls.__name__}")


class WittyPiException(Exception):
    """Exception raised for WittyPi 4 hardware communication errors.

    This exception is raised when there are issues communicating with the
    WittyPi 4 hardware, such as I2C errors or unexpected firmware responses.
    """

    pass


def _parse_geolocation_file(path: str = "/etc/geolocation") -> tuple[float, float] | None:
    """Parse geoclue-2.0 format static location file.

    Reads location from a geoclue-2.0 compatible file with format:
    - Line 1: Latitude (float, positive=north, negative=south)
    - Line 2: Longitude (float, positive=east, negative=west)
    - Line 3: Altitude (optional, ignored)
    - Line 4: Accuracy radius (optional, ignored)

    Comments (starting with #) and whitespace are ignored.

    Args:
        path: Path to geolocation file (default: /etc/geolocation)

    Returns:
        Tuple of (latitude, longitude) or None if file doesn't exist or is invalid
    """
    try:
        with open(path, "r") as f:
            lines = []
            for line in f:
                # Remove comments and strip whitespace
                line = line.split("#", 1)[0].strip()
                # Skip empty lines
                if line:
                    lines.append(line)

            # Need at least 2 lines for lat/lon
            if len(lines) < 2:
                logger.warning("Geolocation file %s has insufficient data (need lat/lon)", path)
                return None

            lat = float(lines[0])
            lon = float(lines[1])

            # Basic validation
            if not (-90 <= lat <= 90):
                logger.warning("Invalid latitude in %s: %f (must be -90 to 90)", path, lat)
                return None
            if not (-180 <= lon <= 180):
                logger.warning("Invalid longitude in %s: %f (must be -180 to 180)", path, lon)
                return None

            logger.debug("Parsed geolocation from %s: lat=%f, lon=%f", path, lat, lon)
            return (lat, lon)

    except FileNotFoundError:
        logger.debug("Geolocation file %s not found", path)
        return None
    except (ValueError, IndexError) as e:
        logger.warning("Failed to parse geolocation file %s: %s", path, e)
        return None
    except Exception as e:
        logger.warning("Error reading geolocation file %s: %s", path, e)
        return None


class ButtonEntry(ScheduleEntry):
    """Schedule entry for manual button-triggered startups.

    This class represents a schedule entry created when the WittyPi is powered on
    manually via button press, voltage restore, or power connection. It allows the
    system to stay on for a configurable delay period before the next scheduled
    shutdown.

    Args:
        button_delay: How long to keep system on after manual power-on.
                     None disables automatic shutdown after button press.
        tz: Timezone for schedule calculations. Defaults to system local timezone.

    Attributes:
        button_delay: Duration to stay powered on after manual start
        boot_ts: Timestamp when the system booted
    """

    def __init__(
        self,
        button_delay: datetime.timedelta | None,
        tz: datetime.tzinfo = None,
    ):
        # get local timezone
        if not tz:
            tz = datetime.datetime.now().astimezone().tzinfo

        self.button_delay = button_delay
        self._tz = tz

    @property
    def boot_ts(self):
        return datetime.datetime.now(tz=self._tz) - datetime.timedelta(seconds=time.monotonic())

    def prev_start(self, now: datetime.datetime | None = None) -> datetime.datetime:
        return self.boot_ts

    def next_stop(self, now: datetime.datetime | None = None) -> datetime.datetime | None:
        return None

    def next_start(self, now: datetime.datetime | None = None) -> datetime.datetime | None:
        return None

    def prev_stop(self, now: datetime.datetime | None = None) -> datetime.datetime | None:
        if not self.button_delay:
            return None

        now = now or datetime.datetime.now(tz=self._tz)
        next_stop = self.boot_ts + self.button_delay
        if next_stop < now:
            return None

        return next_stop

    def active(self, now: datetime.datetime | None = None) -> bool:
        now = now or datetime.datetime.now(tz=self._tz)
        prev_stop = self.prev_stop(now)
        if prev_stop:
            return prev_stop > now
        else:
            return False

    def __repr__(self):
        return f"{self.__class__.__name__}(prev_start={self.prev_start()}, next_stop={self.next_stop()})"


class ScheduleConfiguration:
    """Manages startup/shutdown scheduling based on time and astronomical events.

    This class parses schedule configuration from YAML/dict format and calculates
    when the system should be powered on or off. Supports:
    - Absolute time schedules (e.g., "10:00" to "13:00")
    - Relative to sunrise/sunset (e.g., "sunrise-01:00" to "sunset+00:30")
    - Multiple overlapping schedule entries
    - Force-on mode to disable automatic shutdowns
    - Button delay for manual power-ons

    Location Configuration:
        Location coordinates for astronomical calculations are obtained in order:
        1. From config dict (lat/lon keys)
        2. From /etc/geolocation file (geoclue-2.0 format)
        If neither is available, relative schedules (sunrise/sunset) are disabled.

    Args:
        config: Dictionary containing schedule configuration with keys:
            - lat/lon: Location coordinates for astronomical calculations (optional)
            - force_on: If True, system stays on indefinitely (optional, default False)
            - button_delay: Duration string (e.g., "00:30") to stay on after button press
            - schedule: List of schedule entry dicts with 'name', 'start', 'stop'
        tz: Timezone for schedule calculations. Defaults to system local timezone.

    Attributes:
        force_on: If True, system never shuts down automatically
        button_delay: Timedelta to stay on after manual power-on
        entries: List of ScheduleEntry objects

    Example:
        >>> config = {
        ...     'lat': 50.85318, 'lon': 8.78735,
        ...     'force_on': False,
        ...     'button_delay': '00:30',
        ...     'schedule': [
        ...         {'name': 'morning', 'start': 'sunrise-01:00', 'stop': '12:00'},
        ...         {'name': 'evening', 'start': '18:00', 'stop': 'sunset+01:00'}
        ...     ]
        ... }
        >>> sc = ScheduleConfiguration(config)
        >>> print(sc.next_startup())
    """

    def __init__(
        self,
        config: dict,
        tz: datetime.tzinfo = None,
    ):
        # get local timezone
        if not tz:
            tz = datetime.datetime.now().astimezone().tzinfo

        self._tz = tz

        logger.debug(config)

        # parsing location information
        self._location: astral.LocationInfo | None = None

        # First, try to get location from config
        if ("lat" in config) and ("lon" in config):
            self._location = astral.LocationInfo(platform.node(), "", tz, config["lat"], config["lon"])
            logger.info("Times relative to %s (from config)", self._location)
        # Fall back to /etc/geolocation file
        else:
            geoloc = _parse_geolocation_file()
            if geoloc:
                self._location = astral.LocationInfo(platform.node(), "", tz, geoloc[0], geoloc[1])
                logger.info("Times relative to %s (from /etc/geolocation)", self._location)

        if not self._location:
            logger.warning(
                "No location configured (neither lat/lon in config nor /etc/geolocation), relative schedules are disabled."
            )

        self.force_on: bool = False
        if "force_on" in config:
            if bool(config["force_on"]):
                self.force_on = True
                logger.info("Force on is enabled (%s)", config["force_on"])
            else:
                logger.debug("Force on is disabled (%s)", config["force_on"])

        try:
            self.button_delay = datetime.timedelta(
                seconds=pytimeparse.parse(config["button_delay"], granularity="minutes")
            )
        except Exception:
            self.button_delay = None
        logger.debug("Using button delay of %s", self.button_delay)

        self.entries: list[ScheduleEntry] = []

        if "schedule" not in config or not isinstance(config["schedule"], collections.abc.Iterable):
            logger.warning("Schedule missing in configuration, setting force_on.")
            self.force_on = True
        else:
            for entry_raw in config["schedule"]:
                try:
                    entry = ScheduleEntry(**entry_raw, location=self._location, tz=self._tz)
                    self.entries.append(entry)
                except AttributeError:
                    logger.warning("Schedule doesn't contain lat/lon information, ignoring %s", entry_raw)
            if not self.entries:
                logger.warning("No schedules found, setting force_on.")
                self.force_on = True

        logger.info(
            "ScheduleConfiguration loaded - active: %s, next_shutdown: %s, next_startup: %s",
            self.active(),
            self.next_shutdown(),
            self.next_startup(),
        )
        for entry in self.entries:
            logger.info("%s", entry)

    def next_startup(self, now: datetime.datetime | None = None) -> datetime.datetime | None:
        """Calculate the next scheduled startup time.

        Args:
            now: Reference time for calculation. Defaults to current time.

        Returns:
            Datetime of next scheduled startup, or None if no startup scheduled.
        """
        now = now or datetime.datetime.now(tz=self._tz)
        # as all next_starts are in the future, pick the most recent start
        try:
            return min([e.next_start(now) for e in self.entries if e.next_start(now)])
        except ValueError:
            return None

    def next_shutdown(self, now: datetime.datetime | None = None) -> datetime.datetime | None:
        """Calculate the next scheduled shutdown time.

        Considers all active schedule entries and finds the next time when no
        schedule entry is active (i.e., system should shut down).

        Args:
            now: Reference time for calculation. Defaults to current time.

        Returns:
            Datetime of next scheduled shutdown, or None if force_on is enabled
            or no shutdown needed in next 24 hours.
        """
        now = now or datetime.datetime.now(tz=self._tz)
        if self.force_on:
            return None

        try:
            next_ts = now
            while self.active(next_ts):
                stop_list = []
                for e in self.entries:
                    if e.active(next_ts):
                        e_stop = e.prev_stop(next_ts)
                    else:
                        e_stop = e.next_stop(next_ts)

                    logger.debug("Entry %s (active: %s): %s", e, e.active(next_ts), e_stop)
                    if e_stop and e_stop > now:
                        stop_list.append(e_stop)

                if not stop_list:
                    logger.info("No stop events found, we are online for over 1 day")
                    return None

                next_ts = min(stop_list)
                logger.debug("Next stop event would be %s, are we active then? %s", next_ts, self.active(next_ts))

                if next_ts - now >= datetime.timedelta(days=1):
                    logger.debug("No shutdown required, we are online for over 1 day")
                    return None

            return next_ts

        except ValueError:
            return None

    def active(self, now: datetime.datetime | None = None):
        """Check if system should be powered on at given time.

        Args:
            now: Time to check. Defaults to current time.

        Returns:
            True if system should be on (force_on or any schedule entry active),
            False otherwise.
        """
        now = now or datetime.datetime.now(tz=self._tz)
        return self.force_on or any([e.active(now) for e in self.entries])


def detect_hardware(manual_override: str | None = None) -> str | None:
    """Detect which power management hardware is present.

    Checks for WittyPi 4 hardware first, then Raspberry Pi 5. Returns the
    detected hardware type or None if neither is found.

    Args:
        manual_override: If provided, return this value without detection.
                        Valid values: "wittypi4", "raspberrypi5"

    Returns:
        "wittypi4" if WittyPi hardware detected, "raspberrypi5" if Raspberry Pi 5
        detected, or None if neither detected.
    """
    if manual_override:
        if manual_override in ("wittypi4", "raspberrypi5"):
            logger.info("Using manual hardware override: %s", manual_override)
            return manual_override
        else:
            logger.warning("Invalid manual override '%s', detecting hardware", manual_override)

    # Try to detect WittyPi 4 first
    try:
        bus = smbus2.SMBus(1, force=True)
        try:
            firmware_id = bus.read_byte_data(I2C_MC_ADDRESS, I2C_ID)
            if firmware_id == 0x26:
                logger.info("Detected WittyPi 4 hardware (firmware ID: 0x%02x)", firmware_id)
                bus.close()
                return "wittypi4"
        finally:
            bus.close()
    except (OSError, IOError) as e:
        logger.debug("WittyPi 4 not detected: %s", e)

    # Try to detect Raspberry Pi 5
    try:
        # Check device tree model
        model_path = pathlib.Path("/proc/device-tree/model")
        if model_path.exists():
            model_text = model_path.read_text()
            if "Raspberry Pi 5" in model_text:
                # Check for RTC wakealarm support
                wakealarm_path = pathlib.Path("/sys/class/rtc/rtc0/wakealarm")
                if wakealarm_path.exists():
                    logger.info("Detected Raspberry Pi 5 with RTC support")
                    return "raspberrypi5"
                else:
                    logger.debug("Raspberry Pi 5 detected but RTC wakealarm not available")
    except (OSError, IOError) as e:
        logger.debug("Raspberry Pi 5 detection failed: %s", e)

    logger.warning("No supported power management hardware detected")
    return None


class PowerManager:
    """Base class for power management hardware with RTC and alarm capabilities.

    This abstract base class defines the common interface for power management
    devices that support RTC operations and wake/sleep scheduling. Subclasses
    implement platform-specific functionality.

    Common interface:
    - RTC datetime read/write
    - Startup/shutdown alarm configuration
    - Action reason detection
    - RTC/system clock synchronization checking
    """

    def __init__(self, tz=datetime.UTC):
        """Initialize power manager.

        Args:
            tz: Timezone for RTC operations (default: UTC)
        """
        self._tz = tz

    @property
    def rtc_datetime(self) -> datetime.datetime:
        """Get the current RTC datetime.

        Returns:
            Current datetime from RTC, converted to configured timezone.
        """
        raise NotImplementedError

    @rtc_datetime.setter
    def rtc_datetime(self, value: datetime.datetime):
        """Set the RTC datetime.

        Args:
            value: Datetime to set (will be converted to configured timezone).
        """
        raise NotImplementedError

    def set_startup_datetime(self, ts: datetime.datetime | None):
        """Set the scheduled startup time.

        Args:
            ts: Datetime to power on, or None to disable alarm.
        """
        raise NotImplementedError

    def get_startup_datetime(self) -> datetime.datetime | None:
        """Get the currently configured startup time.

        Returns:
            Datetime when next startup is scheduled, or None if alarm disabled.
        """
        raise NotImplementedError

    def set_shutdown_datetime(self, ts: datetime.datetime | None):
        """Set the scheduled shutdown time.

        Args:
            ts: Datetime to shut down, or None to disable alarm.
        """
        raise NotImplementedError

    def get_shutdown_datetime(self) -> datetime.datetime | None:
        """Get the currently configured shutdown time.

        Returns:
            Datetime when next shutdown is scheduled, or None if alarm disabled.
        """
        raise NotImplementedError

    def clear_flags(self):
        """Clear all alarm flags.

        Should be called after boot to acknowledge alarm triggers.
        """
        raise NotImplementedError

    @property
    def action_reason(self) -> ActionReason:
        """Get the reason for the current power state.

        Returns:
            ActionReason enum indicating why the system was powered on.
        """
        raise NotImplementedError

    def rtc_sysclock_match(self, threshold=datetime.timedelta(seconds=2)) -> bool:
        """Check if RTC time matches system clock within threshold.

        Args:
            threshold: Maximum allowed time difference (default: 2 seconds)

        Returns:
            True if RTC and system clock are within threshold, False otherwise.
        """
        return abs(self.rtc_datetime - datetime.datetime.now(tz=self._tz)) < threshold


class WittyPi4(PowerManager):
    """Main interface to WittyPi 4 power management hardware.

    This class provides complete access to WittyPi 4 functionality including:
    - Hardware monitoring (voltage, current, temperature)
    - RTC operations (read/write time, alarms)
    - Power management (startup/shutdown scheduling)
    - Configuration (voltage thresholds, delays, power modes)

    The WittyPi 4 is accessed via I2C and includes:
    - Microcontroller with firmware v0x26
    - Real-time clock (PCF85063A)
    - Temperature sensor (LM75B)
    - Voltage/current monitoring
    - Configurable alarms for automatic power control

    Args:
        bus: SMBus instance for I2C communication. If None, creates SMBus(1, force=True)
        addr: I2C address of WittyPi microcontroller (default: 0x08)
        tz: Timezone for RTC operations (default: UTC)

    Raises:
        WittyPiException: If device not found or firmware ID doesn't match (expected 0x26)

    Example:
        >>> import smbus2
        >>> from wittypi4 import WittyPi4
        >>> bus = smbus2.SMBus(1, force=True)
        >>> wp = WittyPi4(bus)
        >>> print(f"Input: {wp.voltage_in}V, Output: {wp.voltage_out}V @ {wp.current_out}A")
        >>> print(f"Temperature: {wp.lm75b_temperature}°C")
        >>> print(f"RTC Time: {wp.rtc_datetime}")
        >>>
        >>> # Schedule startup in 1 hour
        >>> import datetime
        >>> wp.set_startup_datetime(datetime.datetime.now() + datetime.timedelta(hours=1))
    """

    def __init__(
        self,
        bus: smbus2.SMBus | None = None,
        addr: int = I2C_MC_ADDRESS,
        tz=datetime.UTC,
    ):
        super().__init__(tz)
        self._bus = bus or smbus2.SMBus(1, force=True)
        self._addr = addr

        try:
            firmware_id = self.firmware_id
            if firmware_id != 0x26:
                raise WittyPiException("unknown Firmware Id (got 0x%02x, expected 0x26)" % firmware_id)
        except OSError as ex:
            raise WittyPiException("error reading address 0x%02x, check device connection" % self._addr) from ex

        logger.info(
            "WittyPi 4 probed successfully, id: 0x%02x, revision 0x%02x", self.firmware_id, self.firmware_revision
        )

    def __del__(self):
        self._bus.close()

    # Read registers
    @property
    def firmware_id(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_ID)

    @property
    def voltage_in(self) -> float:
        return self._bus.read_byte_data(self._addr, I2C_VOLTAGE_IN_I) + (
            self._bus.read_byte_data(self._addr, I2C_VOLTAGE_IN_D) / 100
        )

    @property
    def voltage_out(self) -> float:
        return self._bus.read_byte_data(self._addr, I2C_VOLTAGE_OUT_I) + (
            self._bus.read_byte_data(self._addr, I2C_VOLTAGE_OUT_D) / 100
        )

    @property
    def current_out(self) -> float:
        return self._bus.read_byte_data(self._addr, I2C_CURRENT_OUT_I) + (
            self._bus.read_byte_data(self._addr, I2C_CURRENT_OUT_D) / 100
        )

    @property
    def watts_out(self) -> float:
        return self.voltage_out * self.current_out

    @property
    def power_ldo(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_POWER_MODE))

    @property
    def lv_shutdown(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_LV_SHUTDOWN))

    @property
    def alarm1_triggered(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_ALARM1_TRIGGERED))

    @property
    def alarm2_triggered(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_ALARM2_TRIGGERED))

    @property
    def action_reason(self) -> ActionReason:
        return ActionReason(self._bus.read_byte_data(self._addr, I2C_ACTION_REASON))

    @property
    def firmware_revision(self):
        return self._bus.read_byte_data(self._addr, I2C_FW_REVISION)

    # Configuration Registers
    @property
    def default_on(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_CONF_DEFAULT_ON))

    @default_on.setter
    def default_on(self, value: bool):
        self._bus.write_byte_data(self._addr, I2C_CONF_DEFAULT_ON, value)

    @property
    def pulse_interval(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_CONF_PULSE_INTERVAL)

    @pulse_interval.setter
    def pulse_interval(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_PULSE_INTERVAL, value)

    @property
    def lv_threshold(self) -> float:
        thres = self._bus.read_byte_data(self._addr, I2C_CONF_LOW_VOLTAGE)
        return 0.0 if (thres == 255) else (thres / 10)

    @lv_threshold.setter
    def lv_threshold(self, value: float):
        write_value = 255 if (value == 0.0) else int(value * 10)
        self._bus.write_byte_data(self._addr, I2C_CONF_LOW_VOLTAGE, write_value)

    @property
    def blink_led(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_CONF_BLINK_LED)

    @blink_led.setter
    def blink_led(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_BLINK_LED, value)

    @property
    def power_cut_delay(self) -> float:
        return self._bus.read_byte_data(self._addr, I2C_CONF_POWER_CUT_DELAY) / 10

    @power_cut_delay.setter
    def power_cut_delay(self, value: float):
        self._bus.write_byte_data(self._addr, I2C_CONF_POWER_CUT_DELAY, max(0, min(int(value * 10), 250)))

    @property
    def recovery_voltage(self) -> float:
        thres = self._bus.read_byte_data(self._addr, I2C_CONF_RECOVERY_VOLTAGE)
        return 0.0 if (thres == 255) else (thres / 10)

    @recovery_voltage.setter
    def recovery_voltage(self, value: float):
        write_value = 255 if (value == 0.0) else int(value * 10)
        self._bus.write_byte_data(self._addr, I2C_CONF_RECOVERY_VOLTAGE, write_value)

    @property
    def dummy_load(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_CONF_DUMMY_LOAD)

    @dummy_load.setter
    def dummy_load(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_DUMMY_LOAD, value)

    @staticmethod
    def _from_adj(value: int) -> float:
        if value > 127:
            return (value - 255) / 100
        else:
            return value / 100

    @staticmethod
    def _to_adj(value: float) -> int:
        value = int(value * 100)
        if value < 0:
            return value + 255
        else:
            return value

    @property
    def adj_vin(self) -> float:
        return self._from_adj(self._bus.read_byte_data(self._addr, I2C_CONF_ADJ_VIN))

    @adj_vin.setter
    def adj_vin(self, value: float):
        write_value = self._to_adj(value)
        self._bus.write_byte_data(self._addr, I2C_CONF_ADJ_VIN, write_value)

    @property
    def adj_vout(self) -> float:
        return self._from_adj(self._bus.read_byte_data(self._addr, I2C_CONF_ADJ_VOUT))

    @adj_vout.setter
    def adj_vout(self, value: float):
        write_value = self._to_adj(value)
        self._bus.write_byte_data(self._addr, I2C_CONF_ADJ_VOUT, write_value)

    @property
    def adj_iout(self) -> float:
        return self._from_adj(self._bus.read_byte_data(self._addr, I2C_CONF_ADJ_IOUT))

    @adj_iout.setter
    def adj_iout(self, value: float):
        write_value = self._to_adj(value)
        self._bus.write_byte_data(self._addr, I2C_CONF_ADJ_IOUT, write_value)

    # Startup Alarm
    @property
    def alarm1_second(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_SECOND_ALARM1))

    @alarm1_second.setter
    def alarm1_second(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_SECOND_ALARM1, bin2bcd(value))

    @property
    def alarm1_minute(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_MINUTE_ALARM1))

    @alarm1_minute.setter
    def alarm1_minute(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_MINUTE_ALARM1, bin2bcd(value))

    @property
    def alarm1_hour(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_HOUR_ALARM1))

    @alarm1_hour.setter
    def alarm1_hour(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_HOUR_ALARM1, bin2bcd(value))

    @property
    def alarm1_day(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_DAY_ALARM1))

    @alarm1_day.setter
    def alarm1_day(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_DAY_ALARM1, bin2bcd(value))

    @property
    def alarm1_weekday(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_WEEKDAY_ALARM1))

    @alarm1_weekday.setter
    def alarm1_weekday(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_WEEKDAY_ALARM1, bin2bcd(value))

    @property
    def alarm2_second(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_SECOND_ALARM2))

    @alarm2_second.setter
    def alarm2_second(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_SECOND_ALARM2, bin2bcd(value))

    @property
    def alarm2_minute(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_MINUTE_ALARM2))

    @alarm2_minute.setter
    def alarm2_minute(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_MINUTE_ALARM2, bin2bcd(value))

    @property
    def alarm2_hour(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_HOUR_ALARM2))

    @alarm2_hour.setter
    def alarm2_hour(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_HOUR_ALARM2, bin2bcd(value))

    @property
    def alarm2_day(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_DAY_ALARM2))

    @alarm2_day.setter
    def alarm2_day(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_DAY_ALARM2, bin2bcd(value))

    @property
    def alarm2_weekday(self) -> int:
        return bcd2bin(self._bus.read_byte_data(self._addr, I2C_CONF_WEEKDAY_ALARM2))

    @alarm2_weekday.setter
    def alarm2_weekday(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_WEEKDAY_ALARM2, bin2bcd(value))

    # High level interface for alarms
    def _timer_next_ts(self, day: int, weekday: int, hour: int, minute: int, second: int) -> None | datetime.datetime:
        ts = self.rtc_datetime.astimezone(self._tz)

        # if all register are unset, return None
        if (
            (day == ALARM_RESET)
            and (weekday == ALARM_RESET)
            and (hour == ALARM_RESET)
            and (minute == ALARM_RESET)
            and (second == ALARM_RESET)
        ):
            return None
        if day == 0:
            return None

        logger.debug("Iterating %s to match day %02i %02i:%02i:%02i", ts, day, hour, minute, second)

        # get current date and iterate until day matches
        while (second != ts.second) and (second != ALARM_RESET):
            ts += datetime.timedelta(seconds=1)
        while (minute != ts.minute) and (minute != ALARM_RESET):
            ts += datetime.timedelta(minutes=1)
        while (hour != ts.hour) and (hour != ALARM_RESET):
            ts += datetime.timedelta(hours=1)
        while (weekday != ts.weekday()) and (weekday != ALARM_RESET):
            ts += datetime.timedelta(days=1)
        while (day != ts.day) and (day != ALARM_RESET):
            ts += datetime.timedelta(days=1)

        return ts.astimezone()

    def set_startup_datetime(self, ts: datetime.datetime | None):
        """Set the scheduled startup time (Alarm 1).

        Configures when the WittyPi should power on the Raspberry Pi. Pass None
        to disable the startup alarm.

        Args:
            ts: Datetime to power on, or None to disable alarm.
                Will be converted to the configured timezone.

        Note:
            Logs a warning if the specified time is in the past.
        """
        if ts is None:
            self.alarm1_day = ALARM_RESET
            self.alarm1_weekday = ALARM_RESET
            self.alarm1_hour = ALARM_RESET
            self.alarm1_minute = ALARM_RESET
            self.alarm1_second = ALARM_RESET
            return

        ts = ts.astimezone(self._tz)
        if ts < self.rtc_datetime:
            logger.warning("startup time is in the past.")

        self.alarm1_day = ts.day
        self.alarm1_weekday = ALARM_RESET
        self.alarm1_hour = ts.hour
        self.alarm1_minute = ts.minute
        self.alarm1_second = ts.second

    def get_startup_datetime(self) -> datetime.datetime | None:
        """Get the currently configured startup time (Alarm 1).

        Returns:
            Datetime when next startup is scheduled, or None if alarm disabled.
        """
        return self._timer_next_ts(
            day=self.alarm1_day,
            weekday=self.alarm1_weekday,
            hour=self.alarm1_hour,
            minute=self.alarm1_minute,
            second=self.alarm1_second,
        )

    def set_shutdown_datetime(self, ts: datetime.datetime | None):
        """Set the scheduled shutdown time (Alarm 2).

        Configures when the WittyPi should initiate shutdown of the Raspberry Pi.
        Pass None to disable the shutdown alarm.

        Args:
            ts: Datetime to shut down, or None to disable alarm.
                Will be converted to the configured timezone.

        Note:
            Logs a warning if the specified time is in the past.
        """
        if ts is None:
            self.alarm2_day = ALARM_RESET
            self.alarm2_weekday = ALARM_RESET
            self.alarm2_hour = ALARM_RESET
            self.alarm2_minute = ALARM_RESET
            self.alarm2_second = ALARM_RESET
            return

        ts = ts.astimezone(self._tz)
        if ts < self.rtc_datetime:
            logger.warning("startup time is in the past.")

        self.alarm2_day = ts.day
        self.alarm2_weekday = ALARM_RESET  # ignore weekday in this mode
        self.alarm2_hour = ts.hour
        self.alarm2_minute = ts.minute
        self.alarm2_second = ts.second

    def get_shutdown_datetime(self) -> datetime.datetime | None:
        """Get the currently configured shutdown time (Alarm 2).

        Returns:
            Datetime when next shutdown is scheduled, or None if alarm disabled.
        """
        return self._timer_next_ts(
            day=self.alarm2_day,
            weekday=self.alarm2_weekday,
            hour=self.alarm2_hour,
            minute=self.alarm2_minute,
            second=self.alarm2_second,
        )

    @property
    def rtc_offset(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_CONF_RTC_OFFSET)

    @rtc_offset.setter
    def rtc_offset(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_RTC_OFFSET, value)

    @property
    def rtc_tc(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_CONF_RTC_ENABLE_TC))

    @rtc_tc.setter
    def rtc_tc(self, value: bool):
        self._bus.write_byte_data(self._addr, I2C_CONF_RTC_ENABLE_TC, value)

    @property
    def alarm1_flag(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_CONF_FLAG_ALARM1))

    @alarm1_flag.setter
    def alarm1_flag(self, value: bool):
        self._bus.write_byte_data(self._addr, I2C_CONF_FLAG_ALARM1, value)

    @property
    def alarm2_flag(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_CONF_FLAG_ALARM2))

    @alarm2_flag.setter
    def alarm2_flag(self, value: bool):
        self._bus.write_byte_data(self._addr, I2C_CONF_FLAG_ALARM2, value)

    @property
    def ignore_power_mode(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_CONF_IGNORE_POWER_MODE))

    @ignore_power_mode.setter
    def ignore_power_mode(self, value: bool):
        self._bus.write_byte_data(self._addr, I2C_CONF_IGNORE_POWER_MODE, value)

    @property
    def ignore_lv_shutdown(self) -> bool:
        return bool(self._bus.read_byte_data(self._addr, I2C_CONF_IGNORE_LV_SHUTDOWN))

    @ignore_lv_shutdown.setter
    def ignore_lv_shutdown(self, value: bool):
        self._bus.write_byte_data(self._addr, I2C_CONF_IGNORE_LV_SHUTDOWN, value)

    @property
    def below_temperature_action(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_CONF_BELOW_TEMP_ACTION)

    @below_temperature_action.setter
    def below_temperature_action(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_BELOW_TEMP_ACTION, value)

    @property
    def below_temperature_threshold(self) -> int:
        value = self._bus.read_byte_data(self._addr, I2C_CONF_BELOW_TEMP_POINT)
        if value > 80:
            return value - 256
        else:
            return value

    @below_temperature_threshold.setter
    def below_temperature_threshold(self, value: int):
        if value < 0:
            value += 256
        self._bus.write_byte_data(self._addr, I2C_CONF_BELOW_TEMP_POINT, value)

    @property
    def over_temperature_action(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_CONF_OVER_TEMP_ACTION)

    @over_temperature_action.setter
    def over_temperature_action(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_OVER_TEMP_ACTION, value)

    @property
    def over_temperature_threshold(self) -> int:
        value = self._bus.read_byte_data(self._addr, I2C_CONF_OVER_TEMP_POINT)
        if value > 80:
            return value - 256
        else:
            return value

    @over_temperature_threshold.setter
    def over_temperature_threshold(self, value: int):
        if value < 0:
            value += 256
        self._bus.write_byte_data(self._addr, I2C_CONF_OVER_TEMP_POINT, value)

    @property
    def default_on_delay(self):
        return self._bus.read_byte_data(self._addr, I2C_CONF_DEFAULT_ON_DELAY)

    @default_on_delay.setter
    def default_on_delay(self, value: int):
        self._bus.write_byte_data(self._addr, I2C_CONF_DEFAULT_ON_DELAY, value)

    # LM75B Temperature Sensor
    @staticmethod
    def _lm75b_celsius(data):
        return int.from_bytes(data.to_bytes(2, "little"), signed=True) / 256

    @property
    def lm75b_temperature(self):
        return self._lm75b_celsius(self._bus.read_word_data(self._addr, I2C_LM75B_TEMPERATURE))

    # RTC PCF85063
    @property
    def rtc_datetime(self) -> datetime.datetime:
        return datetime.datetime(
            year=2000 + bcd2bin(self._bus.read_byte_data(self._addr, I2C_RTC_YEARS)),
            month=bcd2bin(self._bus.read_byte_data(self._addr, I2C_RTC_MONTHS)),
            day=bcd2bin(self._bus.read_byte_data(self._addr, I2C_RTC_DAYS)),
            hour=bcd2bin(self._bus.read_byte_data(self._addr, I2C_RTC_HOURS)),
            minute=bcd2bin(self._bus.read_byte_data(self._addr, I2C_RTC_MINUTES)),
            second=bcd2bin(self._bus.read_byte_data(self._addr, I2C_RTC_SECONDS)),
            tzinfo=self._tz,
        ).astimezone()

    @rtc_datetime.setter
    def rtc_datetime(self, value: datetime.datetime):
        ts = value.astimezone(self._tz)
        self._bus.write_byte_data(self._addr, I2C_RTC_YEARS, bin2bcd(ts.year - 2000))
        self._bus.write_byte_data(self._addr, I2C_RTC_MONTHS, bin2bcd(ts.month))
        self._bus.write_byte_data(self._addr, I2C_RTC_WEEKDAYS, bin2bcd(ts.weekday()))
        self._bus.write_byte_data(self._addr, I2C_RTC_DAYS, bin2bcd(ts.day))
        self._bus.write_byte_data(self._addr, I2C_RTC_HOURS, bin2bcd(ts.hour))
        self._bus.write_byte_data(self._addr, I2C_RTC_MINUTES, bin2bcd(ts.minute))
        self._bus.write_byte_data(self._addr, I2C_RTC_SECONDS, bin2bcd(ts.second))

    @property
    def rtc_ctrl1(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_RTC_CTRL1)

    @property
    def rtc_ctrl2(self) -> int:
        return self._bus.read_byte_data(self._addr, I2C_RTC_CTRL2)

    def rtc_ctrl2_clear_alarm(self):
        ctrl2_value = self._bus.read_byte_data(self._addr, I2C_RTC_CTRL2)
        ctrl2_value &= 0b10111111
        self._bus.write_byte_data(self._addr, I2C_RTC_CTRL2, ctrl2_value)

    def clear_flags(self):
        """Clear all alarm flags.

        Resets RTC control register alarm flag and both WittyPi alarm flags.
        Should be called after boot to acknowledge alarm triggers.
        """
        self.rtc_ctrl2_clear_alarm()
        self.alarm1_flag = False
        self.alarm2_flag = False

    def rtc_sysclock_match(self, threshold=datetime.timedelta(seconds=2)) -> bool:
        """Check if RTC time matches system clock within threshold.

        Useful for verifying that RTC and system time are synchronized,
        which is important for reliable scheduling.

        Args:
            threshold: Maximum allowed time difference (default: 2 seconds)

        Returns:
            True if RTC and system clock are within threshold, False otherwise.
        """
        return abs(self.rtc_datetime - datetime.datetime.now(tz=self._tz)) < threshold

    def dump_config(self) -> dict:
        """Dump all readable configuration values.

        Returns:
            Dictionary with all non-private, non-callable attributes and their values.
            Useful for debugging and configuration backup.
        """
        return {
            prop: getattr(self, prop)
            for prop in dir(self)
            if not (prop.startswith("_") or callable(getattr(self, prop)))
        }

    def get_status(self) -> dict[str, float | int]:
        """Get key hardware status information.

        Returns:
            Dictionary containing:
                - Id: Firmware ID (should be 0x26)
                - Input Voltage (V): Supply voltage
                - Output Voltage (V): Output to Raspberry Pi
                - Output Current (A): Current draw
                - Power Mode: LDO mode (bool)
                - Revision: Firmware revision number
                - Temperature (°C): Onboard temperature sensor reading
        """
        return {
            "Id": self.firmware_id,
            "Input Voltage (V)": self.voltage_in,
            "Output Voltage (V)": self.voltage_out,
            "Output Current (A)": self.current_out,
            "Power Mode": self.power_ldo,
            "Revision": self.firmware_revision,
            "Temperature (°C)": self.lm75b_temperature,
        }


class RaspberryPi5(PowerManager):
    """Interface to Raspberry Pi 5 onboard RTC for sleep/wake operations.

    This class provides RTC-based power management for Raspberry Pi 5 using the
    onboard real-time clock. It supports:
    - RTC datetime operations
    - Wake alarm configuration via sysfs
    - System shutdown with wake scheduling

    The Raspberry Pi 5 RTC functionality is more limited than WittyPi hardware:
    - No voltage/current monitoring
    - No temperature monitoring
    - No hardware power cut delays
    - No wake reason detection (always returns REASON_NA)

    Requires:
    - EEPROM configuration: POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0
    - Root privileges for EEPROM configuration (checked automatically)

    Args:
        tz: Timezone for RTC operations (default: UTC)
        check_eeprom: If True (default), check and configure EEPROM settings

    Raises:
        WittyPiException: If RTC wakealarm interface is not available
    """

    RTC_PATH = "/sys/class/rtc/rtc0"
    RTC_WAKEALARM_PATH = RTC_PATH + "/wakealarm"

    def __init__(self, tz=datetime.UTC, check_eeprom: bool = True):
        """Initialize Raspberry Pi 5 RTC interface.

        Args:
            tz: Timezone for RTC operations (default: UTC)
            check_eeprom: If True, check and configure EEPROM settings (default: True)
        """
        super().__init__(tz)

        # Check if RTC wakealarm is available
        wakealarm_path = pathlib.Path(self.RTC_WAKEALARM_PATH)
        if not wakealarm_path.exists():
            raise WittyPiException(
                f"RTC wakealarm not available at {self.RTC_WAKEALARM_PATH}. "
                "Ensure Raspberry Pi 5 RTC is enabled."
            )

        # Check and configure EEPROM if requested
        if check_eeprom:
            self._check_eeprom_config()

        # Detect action reason on initialization
        self._shutdown_datetime: datetime.datetime | None = None

        logger.info("Raspberry Pi 5 RTC interface initialized")

    def _check_eeprom_config(self):
        """Check and update EEPROM configuration for sleep/wake support.

        Verifies that POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0 are set.
        Updates configuration if needed using rpi-eeprom-config.
        """
        try:
            # Create temporary file for bootconf
            with tempfile.NamedTemporaryFile(mode="w+", suffix=".txt", delete=False) as tmp_file:
                bootconf_path = tmp_file.name

            try:
                # Dump current configuration
                try:
                    subprocess.run(
                        ["rpi-eeprom-config", "--out", bootconf_path],
                        check=True,
                        capture_output=True,
                        text=True,
                    )
                except FileNotFoundError:
                    logger.warning(
                        "rpi-eeprom-config not found. EEPROM configuration check skipped. "
                        "Ensure POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0 are set manually."
                    )
                    return
                except subprocess.CalledProcessError as e:
                    logger.warning("Failed to dump EEPROM config: %s", e)
                    return
                except PermissionError:
                    logger.warning(
                        "Permission denied accessing EEPROM. Run as root or ensure "
                        "POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0 are set manually."
                    )
                    return

                # Read and parse configuration
                config_updated = False
                lines = []
                power_off_set = False
                wake_on_gpio_set = False

                try:
                    with open(bootconf_path, "r") as f:
                        for line in f:
                            line_stripped = line.strip()
                            # Skip comments and empty lines
                            if not line_stripped or line_stripped.startswith("#"):
                                lines.append(line)
                                continue

                            # Check POWER_OFF_ON_HALT
                            if line_stripped.startswith("POWER_OFF_ON_HALT="):
                                power_off_set = True
                                if line_stripped != "POWER_OFF_ON_HALT=1":
                                    logger.info("Updating POWER_OFF_ON_HALT from '%s' to 'POWER_OFF_ON_HALT=1'", line_stripped)
                                    lines.append("POWER_OFF_ON_HALT=1\n")
                                    config_updated = True
                                else:
                                    lines.append(line)
                            # Check WAKE_ON_GPIO
                            elif line_stripped.startswith("WAKE_ON_GPIO="):
                                wake_on_gpio_set = True
                                if line_stripped != "WAKE_ON_GPIO=0":
                                    logger.info("Updating WAKE_ON_GPIO from '%s' to 'WAKE_ON_GPIO=0'", line_stripped)
                                    lines.append("WAKE_ON_GPIO=0\n")
                                    config_updated = True
                                else:
                                    lines.append(line)
                            else:
                                lines.append(line)

                    # Add missing settings
                    if not power_off_set:
                        logger.info("Adding POWER_OFF_ON_HALT=1")
                        lines.append("POWER_OFF_ON_HALT=1\n")
                        config_updated = True

                    if not wake_on_gpio_set:
                        logger.info("Adding WAKE_ON_GPIO=0")
                        lines.append("WAKE_ON_GPIO=0\n")
                        config_updated = True

                    # Write updated configuration if needed
                    if config_updated:
                        with open(bootconf_path, "w") as f:
                            f.writelines(lines)

                        # Apply configuration
                        try:
                            subprocess.run(
                                ["rpi-eeprom-config", "--apply", bootconf_path],
                                check=True,
                                capture_output=True,
                                text=True,
                            )
                            logger.info("EEPROM configuration updated successfully. Reboot required for changes to take effect.")
                        except subprocess.CalledProcessError as e:
                            logger.error("Failed to apply EEPROM configuration: %s", e)
                        except PermissionError:
                            logger.warning(
                                "Permission denied applying EEPROM config. Run as root or "
                                "set POWER_OFF_ON_HALT=1 and WAKE_ON_GPIO=0 manually."
                            )
                    else:
                        logger.debug("EEPROM configuration already correct")

                except IOError as e:
                    logger.warning("Failed to read/write bootconf file: %s", e)

            finally:
                # Clean up temporary file
                try:
                    os.unlink(bootconf_path)
                except OSError:
                    pass

        except Exception as e:
            logger.warning("Unexpected error during EEPROM configuration check: %s", e)

    @property
    def rtc_datetime(self) -> datetime.datetime:
        """Get the current RTC datetime.

        Reads from RTC sysfs date and time files.

        Returns:
            Current datetime from RTC, converted to configured timezone.
        """
        date_path = pathlib.Path(f"{self.RTC_PATH}/date")
        time_path = pathlib.Path(f"{self.RTC_PATH}/time")
    
        # Read date (format: YYYY-MM-DD)
        date_str = date_path.read_text().strip()
        # Read time (format: HH:MM:SS)
        time_str = time_path.read_text().strip()
        
        # Parse date and time
        date_parts = date_str.split("-")
        time_parts = time_str.split(":")
        
        if len(date_parts) != 3 or len(time_parts) != 3:
            raise ValueError(f"Unexpected date/time format: {date_str} {time_str}")
        
        year = int(date_parts[0])
        month = int(date_parts[1])
        day = int(date_parts[2])
        hour = int(time_parts[0])
        minute = int(time_parts[1])
        second = int(time_parts[2])
        
        # Create datetime in UTC (RTC sysfs provides UTC time)
        rtc_dt = datetime.datetime(year, month, day, hour, minute, second, tzinfo=datetime.timezone.utc)
        
        # Convert to configured timezone
        return rtc_dt.astimezone(self._tz)


    def set_startup_datetime(self, ts: datetime.datetime | None):
        """Set the scheduled startup time (wake alarm).

        Configures when the Raspberry Pi 5 should wake from sleep. Pass None
        to disable the wake alarm.

        Args:
            ts: Datetime to wake up, or None to disable alarm.
                Will be converted to the configured timezone.

        Note:
            Logs a warning if the specified time is in the past.
            On Raspberry Pi 5, the alarm must be cleared before setting a new value.
        """
        wakealarm_path = pathlib.Path(self.RTC_WAKEALARM_PATH)

        if ts is None:
            # Clear alarm by writing 0
            try:
                wakealarm_path.write_text("0")
                logger.debug("Wake alarm cleared")
            except (IOError, OSError) as e:
                raise WittyPiException(f"Failed to clear wake alarm: {e}") from e
            return

        ts = ts.astimezone(self._tz)
        if ts < datetime.datetime.now(tz=self._tz):
            logger.warning("Wake time is in the past: %s", ts)

        # Convert to Unix timestamp (seconds since epoch)
        wake_timestamp = int(ts.timestamp())

        try:
            # Check current alarm value
            current_alarm = None
            try:
                current_alarm_str = wakealarm_path.read_text().strip()
                if current_alarm_str and current_alarm_str != "0":
                    current_alarm = int(current_alarm_str)
            except (IOError, OSError, ValueError):
                pass

            # Only update if the value is different
            if current_alarm == wake_timestamp:
                logger.debug("Wake alarm already set to %s (timestamp: %d)", ts, wake_timestamp)
                return

            # Clear existing alarm first (required on Raspberry Pi 5)
            try:
                wakealarm_path.write_text("0")
            except (IOError, OSError) as e:
                logger.warning("Failed to clear existing alarm before setting new one: %s", e)
                # Continue anyway - might work if alarm was already cleared

            # Set new alarm value
            wakealarm_path.write_text(str(wake_timestamp))
            logger.info("Wake alarm set for %s (timestamp: %d)", ts, wake_timestamp)
        except (IOError, OSError) as e:
            raise WittyPiException(f"Failed to set wake alarm: {e}") from e

    def get_startup_datetime(self) -> datetime.datetime | None:
        """Get the currently configured startup time (wake alarm).

        Returns:
            Datetime when next wake is scheduled, or None if alarm disabled.
        """
        wakealarm_path = pathlib.Path(self.RTC_WAKEALARM_PATH)

        try:
            alarm_value = wakealarm_path.read_text().strip()
            if not alarm_value or alarm_value == "0":
                return None

            wake_timestamp = int(alarm_value)
            return datetime.datetime.fromtimestamp(wake_timestamp, tz=self._tz)
        except (IOError, OSError, ValueError) as e:
            logger.debug("Failed to read wake alarm: %s", e)
            return None

    def set_shutdown_datetime(self, ts: datetime.datetime | None):
        """Set the scheduled shutdown time.

        Configures when the Raspberry Pi 5 should shut down. When the time
        arrives, the system will be halted (entering low-power mode if
        POWER_OFF_ON_HALT=1 is set).

        Args:
            ts: Datetime to shut down, or None to disable alarm.
                Will be converted to the configured timezone.

        Note:
            This stores the shutdown time but does not set a hardware alarm.
            The daemon should call this periodically and trigger shutdown
            when the time arrives.
        """
        if ts is None:
            self._shutdown_datetime = None
            return

        ts = ts.astimezone(self._tz)
        if ts < datetime.datetime.now(tz=self._tz):
            logger.warning("Shutdown time is in the past: %s", ts)

        self._shutdown_datetime = ts

    def get_shutdown_datetime(self) -> datetime.datetime | None:
        """Get the currently configured shutdown time.

        Returns:
            Datetime when next shutdown is scheduled, or None if alarm disabled.
        """
        return getattr(self, "_shutdown_datetime", None)

    def clear_flags(self):
        """Clear all alarm flags.

        For Raspberry Pi 5, this is a no-op as wake alarm flags are automatically
        managed by the kernel. Kept for interface compatibility.
        """
        pass

    @property
    def action_reason(self) -> ActionReason:
        """Get the reason for the current power state.

        Returns:
            ActionReason.REASON_NA (always, as wake reason detection is not reliable
            on Raspberry Pi 5)
        """
        return ActionReason.REASON_NA
