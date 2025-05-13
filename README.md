# tomato-jumo
`tomato` driver for Jumo Quantrol heater controllers.

This driver is based on the [`minimalmodbus`](https://github.com/pyhys/minimalmodbus) library. This driver is developed by the [ConCat lab at TU Berlin](https://tu.berlin/en/concat).

## Supported functions

### Capabilities
- `constant_temperature` which sets the setpoint to the required value,
- `temperature_ramp` for a gradual heating / cooling followed by a hold

### Attributes
- `setpoint` which is the current temperature setpoint, `pint.Quantity(float, "degC")`
- `duty_cycle` which is the current load requested by the controller, `pint.Quantity(float, "percent")`
- `ramp_rate` which is the temperature ramp rate, `pint.Quantity(float, "K/min")`
- `ramp_target` which is the target of the temperature ramp, `pint.Quantity(float, "degC")`

## Contributors

- Peter Kraus
