from tomato_jumo import DriverInterface
from dgbowl_schemas.tomato.payload import Task
import time

kwargs = dict(address="COM6", channel=1)


if __name__ == "__main__":
    interface = DriverInterface()
    print(f"{interface=}")
    ret = interface.cmp_register(**kwargs)
    print(f"{interface.devmap=}")
    print(f"{interface.devmap[("COM6", 1)].setpoint=}")
    print(f"{interface.devmap[("COM6", 1)].temperature=}")
    print(f"{interface.cmp_attrs(**kwargs)=}")
    print(f"{interface.cmp_measure(**kwargs)=}")
    time.sleep(0.5)
    print(f"{interface.cmp_last_data(**kwargs)=}")
    task = Task(
        component_tag="a1",
        max_duration=3.0,
        sampling_interval=1.0,
        technique_name="constant_temperature",
        task_params={"setpoint": 40.0},
    )
    print(f"{interface.task_start(**kwargs, task=task)=}")
    time.sleep(5)
    print(f"{interface.task_data(**kwargs)=}")
    print(f"{interface.cmp_last_data(**kwargs)=}")
    task = Task(
        component_tag="a1",
        max_duration=3.0,
        sampling_interval=1.0,
        technique_name="temperature_ramp",
        task_params={"ramp_rate": "300 K/min", "ramp_target": 30},
    )
    print(f"{interface.task_start(**kwargs, task=task)=}")
    time.sleep(5)
    print(f"{interface.task_data(**kwargs)=}")
    print(f"{interface.cmp_last_data(**kwargs)=}")
