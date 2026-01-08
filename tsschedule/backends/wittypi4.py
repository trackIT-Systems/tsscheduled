"""WittyPi 4 Backend - Hardware-specific implementation.

This module provides the WittyPi 4 hardware backend for the tsOS Schedule Daemon.
"""

import datetime
import logging

import smbus2

from .. import ActionReason, ALARM_RESET, I2C_MC_ADDRESS, bcd2bin, bin2bcd
from .base import PowerManager

logger = logging.getLogger("tsschedule.backends.wittypi4")

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


class WittyPiException(Exception):
    """Exception raised for WittyPi 4 hardware communication errors.

    This exception is raised when there are issues communicating with the
    WittyPi 4 hardware, such as I2C errors or unexpected firmware responses.
    """

    pass


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
        >>> from tsschedule.backends.wittypi4 import WittyPi4
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

