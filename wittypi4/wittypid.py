#!/usr/bin/env python3
"""WittyPi 4 Daemon - Automated schedule management service.

This daemon runs as a system service to automatically manage WittyPi 4 startup
and shutdown schedules. It:
- Validates RTC time against known system time sources
- Loads schedule configuration from YAML file
- Continuously updates startup/shutdown alarms based on schedule
- Handles manual power-on events with configurable delay
- Responds to SIGTERM/SIGINT for graceful shutdown

The daemon should be run as a systemd service for automatic startup.
"""

import argparse
import datetime
import io
import logging
import os
import pathlib
import signal
import threading

import smbus2
import yaml

from . import (
    ActionReason,
    ButtonEntry,
    PowerManager,
    RaspberryPi5,
    ScheduleConfiguration,
    WittyPi4,
    WittyPiException,
    detect_hardware,
)
from .__main__ import parser

parser.prog = "wittypid"
parser.usage = "daemon to configure and handle WittyPi schedules"
parser.add_argument(
    "-s", "--schedule", type=argparse.FileType("r"), help="YML schedule configuration", default="schedule.yml"
)

logger = logging.getLogger(parser.prog)


def fake_hwclock() -> datetime.datetime:
    """Read time from fake-hwclock timestamp file.

    The fake-hwclock package saves system time to a file on shutdown,
    allowing restoration on systems without battery-backed RTC.

    Returns:
        Datetime read from /etc/fake-hwclock.data

    Raises:
        FileNotFoundError: If fake-hwclock.data doesn't exist
    """
    path = pathlib.Path("/etc/fake-hwclock.data")
    with path.open(encoding="ascii") as fp:
        data = fp.read()

    ts = datetime.datetime.strptime(data, "%Y-%m-%d %H:%M:%S\n").replace(tzinfo=datetime.timezone.utc).astimezone()
    logger.info("Read fake_hwclock: %s", ts)
    return ts


def systemd_timesync_clock() -> datetime.datetime:
    """Read last time synchronization from systemd-timesyncd.

    Uses modification time of systemd's timesync clock file to determine
    when NTP last synchronized the system clock.

    Returns:
        Datetime of last systemd timesync update

    Raises:
        FileNotFoundError: If timesync clock file doesn't exist
    """
    # get last modification date of /var/lib/systemd/timesync/clock
    path = pathlib.Path("/var/lib/systemd/timesync/clock")
    ts = datetime.datetime.fromtimestamp(path.stat().st_mtime).astimezone()
    logger.info("Read systemd_timesync_clock: %s", ts)
    return ts


def chrony_drift_clock() -> datetime.datetime:
    """Read last time synchronization from chrony.

    Uses modification time of chrony's drift file to determine when chrony
    last updated the system clock.

    Returns:
        Datetime of last chrony drift file update

    Raises:
        FileNotFoundError: If chrony drift file doesn't exist
    """
    # get last modification date of /var/lib/chrony/chrony.drift
    path = pathlib.Path("/var/lib/chrony/chrony.drift")
    ts = datetime.datetime.fromtimestamp(path.stat().st_mtime).astimezone()
    logger.info("Read chrony_drift_clock: %s", ts)
    return ts


def last_known_time() -> datetime.datetime:
    """Determine the most recent plausible system time from available sources.

    Checks multiple time sources in order to validate RTC time:
    1. fake-hwclock (if available)
    2. systemd-timesyncd (if available)
    3. chrony (if available)

    Returns:
        Most recent timestamp from available clock sources

    Raises:
        RuntimeError: If no clock source files are available
    """
    # read all three clocks and return the most recent one
    # note: files might not exist, which will throw an exception that needs to be caught
    clocks = []

    # Try to read fake_hwclock
    try:
        clocks.append(fake_hwclock())
    except (FileNotFoundError, OSError) as e:
        logger.debug("Could not read fake_hwclock: %s", e)

    # Try to read systemd_timesync_clock
    try:
        clocks.append(systemd_timesync_clock())
    except (FileNotFoundError, OSError) as e:
        logger.debug("Could not read systemd_timesync_clock: %s", e)

    # Try to read chrony_drift_clock
    try:
        clocks.append(chrony_drift_clock())
    except (FileNotFoundError, OSError) as e:
        logger.debug("Could not read chrony_drift_clock: %s", e)

    if not clocks:
        raise RuntimeError("No clock sources available - all clock files are missing")

    return max(clocks)


class PowerManagerDaemon(threading.Thread):
    """Daemon thread for managing power management schedules automatically.

    This class works with PowerManager instances (WittyPi4 or RaspberryPi5) to:
    - Run as a background thread
    - Validate RTC time on startup
    - Load and apply schedule configuration
    - Continuously update alarms based on schedule
    - Handle manual power-on with button delay
    - Respond to SIGTERM/SIGINT for graceful shutdown

    Args:
        device: PowerManager instance (WittyPi4 or RaspberryPi5)
        schedule: Open file handle to YAML schedule configuration
    """

    def __init__(self, device: PowerManager, schedule: io.TextIOWrapper):
        super().__init__()
        self._device = device
        self._stop = threading.Event()
        self._schedule = schedule

    def terminate(self, sig):
        """Handle termination signals gracefully.

        Args:
            sig: Signal number received (SIGTERM or SIGINT)
        """
        logger.warning("Caught %s, terminating.", signal.Signals(sig).name)
        self._stop.set()

    def run(self):
        """Main daemon loop.

        Performs startup validation, loads schedule configuration, and continuously
        updates startup/shutdown alarms. Runs until terminated by signal.

        Exit codes:
            3: RTC time validation failed (implausible time or not synchronized)
        """
        signal.signal(signal.SIGINT, lambda sig, _: self.terminate(sig))
        signal.signal(signal.SIGTERM, lambda sig, _: self.terminate(sig))

        logger.info("Welcome to %s, action reason: %s", parser.prog, self._device.action_reason)
        self._device.clear_flags()

        # Hardware-specific configuration (only for WittyPi4)
        if isinstance(self._device, WittyPi4):
            # setting default on
            self._device.default_on = True
            self._device.default_on_delay = 1

            # setting power cut delay
            self._device.power_cut_delay = 25

        try:
            # check clock plausibility
            if self._device.rtc_datetime < last_known_time():
                logger.warning(
                    "RTC is implausible (%s). Connect to GPS/internet and wait for timesync", self._device.rtc_datetime
                )
                exit(3)

            # check RTC and systemclock matching
            if not self._device.rtc_sysclock_match():
                logger.warning("RTC does not match system clock, check system configuration")
                exit(3)

            # set clock synced
            sync_path = pathlib.Path("/run/systemd/timesync/synchronized")
            logger.info("RTC is valid, setting %s", sync_path)
            if not sync_path.parent.exists():
                logger.info("Creating for /run/systemd/timesync/...")
                sync_path.parent.mkdir(parents=True, exist_ok=True)
            sync_path.touch()
        except ValueError:
            logger.error("RTC is unset. Connect to GPS/internet, and wait for timesync")
            exit(3)

        # read schedule configuration
        schedule_raw: dict = yaml.safe_load(self._schedule)
        sc = ScheduleConfiguration(schedule_raw)

        if self._device.action_reason in [
            ActionReason.REASON_NA,
            ActionReason.BUTTON_CLICK,
            ActionReason.VOLTAGE_RESTORE,
            ActionReason.POWER_CONNECTED,
        ]:
            button_entry = ButtonEntry(sc.button_delay)
            logger.info("Started by %s, adding %s", self._device.action_reason, button_entry)
            sc.entries.append(button_entry)

        shutdown_delay_s = 30

        while not self._stop.is_set():
            now = self._device.rtc_datetime
            next_startup = sc.next_startup(now)
            next_shutdown = sc.next_shutdown(now)

            logger.info("Setting next_shutdown: %s, next_startup: %s", next_shutdown, next_startup)
            self._device.set_startup_datetime(next_startup)
            self._device.set_shutdown_datetime(next_shutdown)

            # Check if shutdown time has arrived
            shutdown_dt = self._device.get_shutdown_datetime()
            if shutdown_dt and shutdown_dt <= now:
                logger.warning("Shutdown time has arrived, shutting down")
                os.system("shutdown 0")
                break

            # somehow we're here while shouldn't be active, setting shutdown with delay
            if not sc.active(now):
                logger.info("Shouldn't be active, scheduling shutdown in %ss", shutdown_delay_s)
                self._device.set_shutdown_datetime(now + datetime.timedelta(seconds=shutdown_delay_s))

            # Check for hardware-specific shutdown triggers (WittyPi only)
            elif self._device.action_reason in [
                ActionReason.ALARM_SHUTDOWN,
                ActionReason.LOW_VOLTAGE,
                ActionReason.OVER_TEMPERATURE,
            ]:
                logger.warning("Alarm %s fired, shutting down", self._device.action_reason)
                os.system("shutdown 0")
                break

            # wait for 60s or until signal
            self._stop.wait(60)

        self._device.set_shutdown_datetime(None)
        self._device.set_startup_datetime(sc.next_startup())
        logger.info(
            "Terminating, set ScheduleConfiguration shutdown: %s, startup: %s",
            self._device.get_shutdown_datetime(),
            self._device.get_startup_datetime(),
        )
        logger.info("Bye from wittypid")


def main():
    """Entry point for wittypid daemon.

    Parses command-line arguments, initializes logging, detects hardware,
    connects to power management hardware, and starts the daemon loop.

    Exit codes:
        1: Failed to connect to power management hardware
        2: No supported hardware detected
        3: RTC validation failed (set by daemon.run())
    """
    args = parser.parse_args()

    # configure logging
    logging_level = max(0, logging.WARN - (args.verbose * 10))
    logging_stderr = logging.StreamHandler()
    logging_stderr.setLevel(logging_level)
    logging.basicConfig(level=logging.DEBUG, handlers=[logging_stderr])

    # Detect hardware
    hardware_type = detect_hardware()
    if not hardware_type:
        logger.error("No supported power management hardware detected. Terminating.")
        exit(2)

    logger.info("Detected hardware: %s", hardware_type)

    # Initialize appropriate device
    try:
        if hardware_type == "wittypi4":
            bus = smbus2.SMBus(bus=args.bus, force=args.force)
            device = WittyPi4(bus, args.addr)
        elif hardware_type == "raspberrypi5":
            device = RaspberryPi5()
        else:
            logger.error("Unknown hardware type: %s", hardware_type)
            exit(2)
    except WittyPiException as ex:
        logger.error("Couldn't connect to power management hardware (%s), terminating.", ex)
        exit(1)

    # Create and run daemon
    daemon = PowerManagerDaemon(device, args.schedule)
    daemon.run()


if __name__ == "__main__":
    main()
