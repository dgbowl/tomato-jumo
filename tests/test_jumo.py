from tomato_jumo import DriverInterface, Device

if __name__ == "__main__":
    import time

    kwargs = dict(address="COM3", channel="80")
    # interface = DriverInterface()
    device = Device(driver="jumo", key=("COM3", "80"))
    print(f"{device=}")
    # print(f"{interface=}")
    # print(f"{interface.cmp_register(**kwargs)=}")
    print(f"{device.set_attr(attr="setpoint", val=10.0)=}")
