import argparse
import datetime
import logging

import smbus2

from . import detect_hardware
from .backends.raspberrypi5 import RaspberryPi5
from .backends.wittypi4 import WittyPi4, WittyPiException

logger = logging.getLogger("tsschedule")

parser = argparse.ArgumentParser(
    "tsschedule",
    description="Control schedule-based power management devices",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)
parser.add_argument("-v", "--verbose", help="increase output verbosity", action="count", default=0)

parser.add_argument(
    "--force",
    help="force I2C bus access, required when using with RTC kernel module",
    default=True,
    action=argparse.BooleanOptionalAction,
)
parser.add_argument("--bus", help="I2C bus to be used", default=1, type=int)
parser.add_argument("--addr", help="Hardware I2C address", default=8, type=int)


if __name__ == "__main__":
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

    # Print status information (WittyPi4-specific features)
    logger.info("RTC Time: %s", device.rtc_datetime)
    logger.info("Startup Reason: %s", device.action_reason)

    if isinstance(device, WittyPi4):
        if logging_level <= logging.DEBUG:
            for prop, val in device.dump_config().items():
                logger.debug("%s: %s", prop, val)

        logger.info("RTC Control 1: %s", format(device.rtc_ctrl1, "08b"))
        logger.info("RTC Control 2: %s", format(device.rtc_ctrl2, "08b"))

    device.clear_flags()

    # schedule next startup
    startup_s = 20
    startup = device.rtc_datetime + datetime.timedelta(seconds=startup_s)
    logger.warning("Scheduling startup in %s seconds @%s", startup_s, startup)
    device.set_startup_datetime(startup)

    # schedule next shutdown
    shutdown_s = 10
    shutdown = device.rtc_datetime + datetime.timedelta(seconds=shutdown_s)
    logger.warning("Scheduling shutdown in %s seconds @%s", shutdown_s, shutdown)
    device.set_shutdown_datetime(shutdown)

    if isinstance(device, WittyPi4):
        logger.info("Power Cut Delay: %s", device.power_cut_delay)
        device.power_cut_delay = 30
        logger.info("Power Cut Delay: %s (newly set)", device.power_cut_delay)

    # debug print info
    logger.info("Next Startup: %s", device.get_startup_datetime())
    logger.info("Next Shutdown: %s", device.get_shutdown_datetime())

    if isinstance(device, WittyPi4):
        logger.info("%s", device.get_status())

