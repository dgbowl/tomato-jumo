from datetime import datetime
import serial
import minimalmodbus
import time
from threading import Thread, current_thread, RLock
from tomato.driverinterface_2_1 import Attr, ModelInterface, ModelDevice, Task
from tomato.driverinterface_2_1.decorators import coerce_val
from functools import wraps
import pint
import logging
import xarray as xr

pint.set_application_registry(pint.UnitRegistry(autoconvert_offset_to_baseunit=True))
logger = logging.getLogger(__name__)
# Known values from:
# JUMO Quantrol LC100/LC200/LC300
# Universal PID Controller Series
# B 702030.2.0
# Interface Description Modbus
# Section 4: Modbus addresses
# 0x3200 - RAM setpoint (RW), set to 200001 to use controller setpoint
# 0x0031 - controller value, i.e. the process variable
# 0x0035 - controller setpoint (RW), persistent but EEPROM-based, use RAM setpoint instead
# 0x0037 - controller output in %
# 0x004E - ramp rate (RW), not used.
REGISTER_MAP = {
    0x3200: "setpoint",
    0x0031: "temperature",
    0x0035: "eeprom_setpoint",
    0x0037: "duty_cycle",
}

PARAM_MAP = {v: k for k, v in REGISTER_MAP.items()}

MODBUS_DELAY = 0.02


def modbus_delay(func):
    @wraps(func)
    def wrapper(self: ModelDevice, **kwargs):
        with self.portlock:
            if time.perf_counter() - self.last_action < MODBUS_DELAY:
                time.sleep(MODBUS_DELAY)
            return func(self, **kwargs)

    return wrapper


class DriverInterface(ModelInterface):
    idle_measurement_interval = 10

    def DeviceFactory(self, key, **kwargs):
        return Device(self, key, **kwargs)


class Device(ModelDevice):
    s: serial.Serial
    """:class:`serial.Serial` port, used for communication with the device."""

    instrument: minimalmodbus.Instrument
    """:class:`minimalmodbus.Instrument`, used for communication with the device over MODBUS"""

    portlock: RLock
    """:class:`threading.RLock`, used to ensure exclusive access to the serial port"""

    last_action: float
    """a timestamp of last MODBUS read/write obtained using :func:`time.perf_counter`"""

    ramp_rate: pint.Quantity
    """the rate of increase in temperature, in K/min, for ``temperature_ramp`` tasks"""

    ramp_target: pint.Quantity
    """the target temperature of a ramp, in degC, for ``temperature_ramp`` tasks"""

    ramp_task: Thread
    """the :class:`Thread` for the ``temperature_ramp`` task."""

    @property
    @modbus_delay
    def temperature(self) -> pint.Quantity:
        val = self.instrument.read_float(
            registeraddress=PARAM_MAP["temperature"],
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
        )
        self.last_action = time.perf_counter()
        return pint.Quantity(val, "degC")

    @property
    @modbus_delay
    def setpoint(self) -> pint.Quantity:
        val = self.instrument.read_float(
            registeraddress=PARAM_MAP["setpoint"],
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
        )
        # Check for controller setpoint - read EEPROM value as fallback
        if val == 200001:
            val = self.instrument.read_float(
                registeraddress=PARAM_MAP["eeprom_setpoint"],
                byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
            )
        self.last_action = time.perf_counter()
        return pint.Quantity(val, "degC")

    @property
    @modbus_delay
    def duty_cycle(self) -> pint.Quantity:
        val = self.instrument.read_float(
            registeraddress=PARAM_MAP["duty_cycle"],
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
        )
        self.last_action = time.perf_counter()
        return pint.Quantity(val, "percent")

    def __init__(self, driver: ModelInterface, key: tuple[str, int], **kwargs: dict):
        address, channel = key
        try:
            self.s = serial.Serial(
                port=address,
                baudrate=9600,
                bytesize=8,
                parity="N",
                stopbits=1,
                exclusive=True,
            )
        except serial.SerialException as e:
            logger.error(e, exc_info=True)
            raise RuntimeError(str(e)) from e
        self.instrument = minimalmodbus.Instrument(
            port=self.s, slaveaddress=int(channel)
        )
        self.ramp_target = pint.Quantity("20 degC")
        self.ramp_rate = pint.Quantity("0 K/min")
        self.portlock = RLock()
        self.last_action = time.perf_counter()
        self.ramp_task = Thread(target=self._temperature_ramp, daemon=True)
        self.ramp_task.do_run = False
        super().__init__(driver, key, **kwargs)

    def attrs(self, **kwargs) -> dict[str, Attr]:
        """Returns a dict of available attributes for the device."""
        attrs_dict = {
            "setpoint": Attr(
                type=pint.Quantity,
                units="degC",
                status=True,
                rw=True,
                minimum=pint.Quantity("0 degC"),
            ),
            "ramp_rate": Attr(
                type=pint.Quantity,
                units="kelvin/min",
                rw=True,
                maximum=pint.Quantity("600 K/min"),
            ),
            "ramp_target": Attr(
                type=pint.Quantity,
                units="degC",
                rw=True,
                minimum=pint.Quantity("0 degC"),
            ),
            "duty_cycle": Attr(
                type=pint.Quantity,
                units="percent",
                status=False,
            ),
        }
        return attrs_dict

    @coerce_val
    @modbus_delay
    def set_attr(self, attr: str, val: pint.Quantity, **kwargs: dict) -> pint.Quantity:
        if attr in {"ramp_rate", "ramp_target"}:
            setattr(self, attr, val)
        else:
            register_nr = PARAM_MAP[attr]
            self.instrument.write_float(
                registeraddress=register_nr,
                value=val.to("degC").m,
                byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
            )
            self.last_action = time.perf_counter()
        return val

    @modbus_delay
    def get_attr(self, attr: str, **kwargs: dict) -> pint.Quantity:
        if attr not in self.attrs():
            raise AttributeError(f"unknown attr: {attr!r}")
        return getattr(self, attr)

    def capabilities(self, **kwargs) -> set:
        caps = {"constant_temperature", "temperature_ramp"}
        return caps

    def do_measure(self, **kwargs) -> None:
        setp = self.setpoint
        temp = self.temperature
        duty = self.duty_cycle
        r_rt = self.ramp_rate
        r_tt = self.ramp_target
        uts = datetime.now().timestamp()
        data_vars = {
            "setpoint": (["uts"], [setp.m], {"units": str(setp.u)}),
            "duty_cycle": (["uts"], [duty.m], {"units": str(duty.u)}),
            "temperature": (["uts"], [temp.m], {"units": str(temp.u)}),
            "ramp_rate": (["uts"], [r_rt.m], {"units": str(r_rt.u)}),
            "ramp_target": (["uts"], [r_tt.m], {"units": str(r_tt.u)}),
        }

        self.last_data = xr.Dataset(
            data_vars=data_vars,
            coords={"uts": (["uts"], [uts])},
        )

    def prepare_task(self, task: Task, **kwargs: dict) -> None:
        logger.critical("preparing task (parent fn)")
        super().prepare_task(task=task, **kwargs)
        if self.ramp_task.do_run is True:
            logger.critical("joining ramp task")
            self.ramp_task.do_run = False
            self.ramp_task.join()
        if task.technique_name in {"temperature_ramp"}:
            logger.critical("preparing ramp task")
            self.ramp_task = Thread(target=self._temperature_ramp, daemon=True)
            self.ramp_task.do_run = True
            self.ramp_task.start()
        logger.critical("task prepared")

    def stop_task(self, **kwargs: dict) -> None:
        logger.critical("stopping task (parent fn)")
        super().stop_task(**kwargs)
        if self.ramp_task.do_run is True:
            logger.critical("joining ramp task")
            self.ramp_task.do_run = False
            self.ramp_task.join()
        logger.critical("task stopped")

    def reset(self, **kwargs) -> None:
        logger.critical("resetting component (parent fn)")
        super().reset(**kwargs)
        if self.ramp_task.do_run is True:
            logger.critical("joining ramp task")
            self.ramp_task.do_run = False
            self.ramp_task.join()
        logger.critical("resetting setpoint")
        self.set_attr(attr="setpoint", val=200001)
        logger.critical("component reset")

    def _temperature_ramp(self) -> None:
        thread = current_thread()
        T_start = self.temperature
        T_end = self.ramp_target
        if T_end > T_start:
            sign = 1
        else:
            sign = -1

        t_start = time.perf_counter()
        t_prev = t_start

        while thread.do_run:
            t_now = time.perf_counter()
            if t_now - t_prev >= 2.0:
                dt = pint.Quantity(t_now - t_start, "s")
                delta = dt * self.ramp_rate
                setpoint = (T_start.to("K") + sign * delta).to("degC")
                if sign > 0 and setpoint >= T_end:
                    self.set_attr(attr="setpoint", val=T_end)
                    break
                elif sign < 0 and setpoint <= T_end:
                    self.set_attr(attr="setpoint", val=T_end)
                    break
                else:
                    self.set_attr(attr="setpoint", val=setpoint)
                t_prev = t_now
            time.sleep(0.1)
