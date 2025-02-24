from datetime import datetime
import serial
import minimalmodbus
import time
from threading import Thread, current_thread, RLock
from tomato.driverinterface_2_0 import Attr, ModelInterface, DriverModel, Val, Task
from functools import wraps
import pint
import xarray as xr

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
    def DeviceFactory(self, key, **kwargs):
        return Device(self, key, **kwargs)


class Device(DriverModel):
    instrument: minimalmodbus.Instrument
    """minimalmodbus.Instrument, used for communication with the device"""

    portlock: RLock
    """threading.RLock, used to ensure exclusive access to the serial port"""

    last_action: float
    """a timestamp of last MODBUS read/write obtained using time.perf_counter()"""

    ramp_rate: pint.Quantity

    ramp_target: pint.Quantity

    ramp_task: Thread

    @property
    def temperature(self) -> pint.Quantity:
        val = self.instrument.read_float(
            registeraddress=PARAM_MAP["temperature"],
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
        )
        self.last_action = time.perf_counter()
        return pint.Quantity(val, "degC")

    @property
    def setpoint(self) -> pint.Quantity:
        val = self.instrument.read_float(
            registeraddress=PARAM_MAP["setpoint"],
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
        )
        self.last_action = time.perf_counter()
        return pint.Quantity(val, "degC")

    @property
    def duty_cycle(self) -> pint.Quantity:
        val = self.instrument.read_float(
            registeraddress=PARAM_MAP["duty_cycle"],
            byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP,
        )
        self.last_action = time.perf_counter()
        return pint.Quantity(val / 100.0, "percent")

    def __init__(self, driver: ModelInterface, key: tuple[str, int], **kwargs: dict):
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
        self.ramp_target = None
        self.ramp_rate = None
        self.portlock = RLock()
        self.last_action = time.perf_counter()

    def attrs(self, **kwargs) -> dict[str, Attr]:
        """Returns a dict of available attributes for the device."""
        attrs_dict = {
            "setpoint": Attr(
                type=pint.Quantity,
                units="degC",
                status=True,
                rw=True,
                minimum=pint.Quantity(0, "degC"),
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
                minimum=pint.Quantity(0, "degC"),
            ),
        }
        return attrs_dict

    @modbus_delay
    def set_attr(self, attr: str, val: Val, **kwargs: dict) -> Val:
        assert attr in self.attrs(), f"unknown attr: {attr!r}"
        props = self.attrs()[attr]
        assert props.rw

        # First coerce val to correct type:
        if not isinstance(val, props.type):
            val = props.type(val)
        if isinstance(val, pint.Quantity):
            if val.dimensionless and props.units is not None:
                val = pint.Quantity(val.m, props.units)
            assert val.dimensionality == pint.Quantity(props.units).dimensionality, (
                f"attr {attr!r} has the wrong dimensionality {str(val.dimensionality)}"
            )
        assert props.minimum is None or val > props.minimum, (
            f"attr {attr!r} is smaller than {props.minimum}"
        )
        assert props.maximum is None or val < props.maximum, (
            f"attr {attr!r} is greater than {props.maximum}"
        )

        # Then set val
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
    def get_attr(self, attr: str, **kwargs: dict) -> Val:
        """
        Retrieves the value of an attribute from the instrument.

        Checks whether the attribute is in allowed attrs. Converts return values to
        expected types using maps.

        """
        assert attr in self.attrs(), f"unknown attr: {attr!r}"
        return getattr(self, attr)

    def capabilities(self, **kwargs) -> set:
        """Returns a set of capabilities supported by this device."""
        caps = {"constant_temperature", "temperature_ramp"}
        return caps

    def do_measure(self, **kwargs):
        setp = self.setpoint
        temp = self.temperature
        r_rt = self.ramp_rate
        r_tt = self.ramp_target
        uts = datetime.now().timestamp()
        data_vars = {
            "setpoint": (["uts"], [setp.m], {"units": str(setp.u)}),
            "temperature": (["uts"], [temp.m], {"units": str(temp.u)}),
        }
        if r_rt is not None:
            data_vars["ramp_rate"] = (["uts"], [r_rt.m], {"units": str(r_rt.u)})
        if r_tt is not None:
            data_vars["ramp_target"] = (["uts"], [r_tt.m], {"units": str(r_tt.u)})

        self.last_data = xr.Dataset(
            data_vars=data_vars,
            coords={"uts": (["uts"], [uts])},
        )

    def prepare_task(self, task: Task, **kwargs: dict):
        super().prepare_task(task=task, **kwargs)
        if task.technique_name in {"temperature_ramp"}:
            self.ramp_task = Thread(target=self._temperature_ramp, daemon=True)
            self.ramp_task.do_run = True
            self.ramp_task.start()

    def reset(self, **kwargs):
        super().reset(**kwargs)
        self.ramp_task.do_run = False
        self.set_attr(attr="setpoint", val=200001)

    def _temperature_ramp(self):
        thread = current_thread()
        T_start = self.temperature
        T_end = self.ramp_target
        if T_end > T_start:
            sign = 1
        else:
            sign = -1

        t_start = time.perf_counter()
        t_prev = t_start

        while getattr(thread, "do_run"):
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
            time.sleep(0.2)
