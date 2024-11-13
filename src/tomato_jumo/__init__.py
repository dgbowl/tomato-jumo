from datetime import datetime
from typing import Any
import serial
import minimalmodbus
import time
from threading import Thread, current_thread, RLock
from tomato.driverinterface_1_0 import Attr, ModelInterface, Task
from functools import wraps

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
    0x0037: "duty_cycle",
}

PARAM_MAP = {v: k for k, v in REGISTER_MAP.items()}

MODBUS_DELAY = 0.02


def modbus_delay(func):
    @wraps(func)
    def wrapper(self: ModelInterface.DeviceManager, **kwargs):
        with self.portlock:
            if time.perf_counter() - self.last_action < MODBUS_DELAY:
                time.sleep(MODBUS_DELAY)
            return func(self, **kwargs)

    return wrapper


class DriverInterface(ModelInterface):
    class DeviceManager(ModelInterface.DeviceManager):
        instrument: minimalmodbus.Instrument
        """minimalmodbus.Instrument, used for communication with the device"""

        portlock: RLock
        """threading.RLock, used to ensure exclusive access to the serial port"""

        last_action: float
        """a timestamp of last MODBUS read/write obtained using time.perf_counter()"""

        _ramp_rate: float

        _ramp_target: float

        _ramp_task: Thread

        @property
        def _temperature(self):
            val = self.instrument.read_float(
                registeraddress=PARAM_MAP["temperature"],
                byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
            )
            self.last_action = time.perf_counter()
            return val

        @property
        def _setpoint(self):
            val = self.instrument.read_float(
                registeraddress=PARAM_MAP["setpoint"],
                byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
            )
            self.last_action = time.perf_counter()
            return val

        @property
        def _duty_cycle(self):
            val = self.instrument.read_float(
                registeraddress=PARAM_MAP["duty_cycle"],
                byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
            )
            self.last_action = time.perf_counter()
            return val / 100.0

        def __init__(
            self, driver: ModelInterface, key: tuple[str, int], **kwargs: dict
        ):
            super().__init__(driver, key, **kwargs)
            address, channel = key
            s = serial.Serial(
                port=address,
                baudrate=9600,
                bytesize=8,
                parity="N",
                stopbits=1,
                exclusive=True,
            )
            self.instrument = minimalmodbus.Instrument(port=s, slaveaddress=channel)
            self._ramp_target = None
            self._ramp_rate = None
            self.portlock = RLock()
            self.last_action = time.perf_counter()

        def attrs(self, **kwargs) -> dict[str, Attr]:
            """Returns a dict of available attributes for the device."""
            attrs_dict = {
                "temperature": Attr(type=float, units="Celsius", status=True),
                "setpoint": Attr(type=float, units="Celsius", status=True, rw=True),
                "ramp_rate": Attr(type=float, units="Celsius/min", rw=True),
                "ramp_target": Attr(type=float, units="Celsius", rw=True),
                "duty_cycle": Attr(type=float, units="%", status=True),
            }
            return attrs_dict

        @modbus_delay
        def set_attr(self, attr: str, val: Any, **kwargs: dict):
            if attr in self.attrs() and self.attrs()[attr].rw:
                if attr in {"ramp_rate", "ramp_target"}:
                    setattr(self, f"_{attr}", val)
                else:
                    register_nr = PARAM_MAP[attr]
                    self.instrument.write_float(
                        registeraddress=register_nr,
                        value=val,
                        byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
                    )
                    self.last_action = time.perf_counter()
            else:
                raise ValueError(f"Unknown attr: {attr!r}")

        @modbus_delay
        def get_attr(self, attr: str, **kwargs: dict) -> Any:
            """
            Retrieves the value of an attribute from the instrument.

            Checks whether the attribute is in allowed attrs. Converts return values to
            expected types using maps.

            """
            if attr in self.attrs():
                return getattr(self, f"_{attr}")
            else:
                raise ValueError(f"Unknown attr: {attr!r}")

        def capabilities(self, **kwargs) -> set:
            """Returns a set of capabilities supported by this device."""
            caps = {"constant_temperature", "temperature_ramp"}
            return caps

        def do_task(self, task: Task, **kwargs):
            """
            Iterate over all attrs and get their values.

            TODO: The read can be batched.
            """
            uts = datetime.now().timestamp()
            self.data["uts"].append(uts)
            for key in self.attrs(**kwargs):
                val = self.get_attr(attr=key)
                self.data[key].append(val)

        def prepare_task(self, task: Task, **kwargs: dict):
            super().prepare_task(task=task, **kwargs)
            if task.technique_name in {"temperature_ramp"}:
                self._ramp_task = Thread(target=self._temperature_ramp, daemon=True)
                self._ramp_task.do_run = True
                self._ramp_task.start()

        def reset(self, **kwargs):
            super().reset(**kwargs)
            self.set_attr(attr="setpoint", val=200001)

        def _temperature_ramp(self):
            thread = current_thread()
            T_start = self.get_attr(attr="temperature")
            T_end = self.get_attr(attr="ramp_target")
            if T_end > T_start:
                sign = 1
            else:
                sign = -1

            t_start = time.perf_counter()
            t_prev = t_start

            while getattr(thread, "do_run"):
                t_now = time.perf_counter()
                if t_now - t_prev >= 2.0:
                    dt = t_now - t_start
                    delta = dt * self.get_attr(attr="ramp_rate") / 60.0
                    setpoint = T_start + sign * delta
                    if sign > 0 and setpoint >= T_end:
                        self.set_attr(attr="setpoint", val=T_end)
                        break
                    elif sign < 0 and setpoint <= T_end:
                        self.set_attr(attr="setpoint", val=T_end)
                        break
                    else:
                        self.set_attr(attr="setpoint", val=setpoint)
                    t_prev = t_now
                time.sleep(0.2)