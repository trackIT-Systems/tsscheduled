"""tsOS Schedule Daemon - Python Library.

This module provides a Python interface for schedule-based power management.
It supports multiple hardware backends and provides:

- Schedule-based power management with sunrise/sunset support
- Real-time clock (RTC) operations
- Scheduled startup and shutdown via alarms
- Multiple backend support (WittyPi 4, Raspberry Pi 5, etc.)

Main Classes:
    ScheduleConfiguration: Manages scheduling logic for startup/shutdown
    ActionReason: Enum of possible startup/shutdown reasons
    ButtonEntry: Handles manual power-on scheduling

Backends:
    Backend implementations are available in the backends subpackage.
    Example: from tsschedule.backends.wittypi4 import WittyPi4

Example:
    Basic usage with schedule configuration:

    >>> from tsschedule import ScheduleConfiguration
    >>> config = {'schedule': [{'name': 'morning', 'start': '08:00', 'stop': '12:00'}]}
    >>> sc = ScheduleConfiguration(config)
    >>> print(sc.next_startup())
"""

import collections.abc
import datetime
import enum
import importlib
import logging
import platform
import time
from zoneinfo import ZoneInfo

import astral
import astral.sun
import pytimeparse
from scheduleparse import ScheduleEntry

__version__ = importlib.metadata.version(__name__)

logger = logging.getLogger("tsschedule")

# Device Address (WittyPi 4 specific, but kept for compatibility)
I2C_MC_ADDRESS = 0x08

# GPIO Pins (WittyPi 4 specific, but kept for compatibility)
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
    """Enumeration of possible reasons for power state changes.

    These values indicate why the system was powered on or off, and can be
    read from backend hardware action_reason properties.

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

    This class represents a schedule entry created when the system is powered on
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
            - tz: Timezone name (e.g., "Europe/Berlin", "America/New_York") for schedule calculations (optional, defaults to system timezone)
            - force_on: If True, system stays on indefinitely (optional, default False)
            - button_delay: Duration string (e.g., "00:30") to stay on after button press (optional, default: "00:10")
            - schedule: List of schedule entry dicts with 'name', 'start', 'stop'

    Attributes:
        force_on: If True, system never shuts down automatically
        button_delay: Timedelta to stay on after manual power-on
        entries: List of ScheduleEntry objects

    Example:
        >>> config = {
        ...     'lat': 50.85318, 'lon': 8.78735,
        ...     'tz': 'Europe/Berlin',  # optional, defaults to system timezone
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
    ):
        # get timezone from config if provided, otherwise use system timezone
        if "tz" in config:
            try:
                tz = ZoneInfo(config["tz"])
                logger.info("Using timezone from config: %s", config["tz"])
            except Exception as e:
                logger.warning("Invalid timezone '%s' in config: %s, using system timezone", config["tz"], e)
                tz = datetime.datetime.now().astimezone().tzinfo
        else:
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
            # Default to 10 minutes if not specified or invalid
            self.button_delay = datetime.timedelta(minutes=10)
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


__all__ = [
    "ActionReason",
    "ButtonEntry",
    "ScheduleConfiguration",
    "ALARM_RESET",
    "I2C_MC_ADDRESS",
    "HALT_PIN",
    "SYSUP_PIN",
    "CHRG_PIN",
    "STDBY_PIN",
    "bcd2bin",
    "bin2bcd",
]
