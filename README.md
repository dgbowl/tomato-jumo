# tomato-jumo
`tomato` driver for Jumo Quantrol heater controllers.

This driver is based on the [`minimalmodbus`](https://github.com/pyhys/minimalmodbus) library. This driver is developed by the [ConCat lab at TU Berlin](https://tu.berlin/en/concat).

## Supported functions

### Capabilities
- `constant_temperature` which sets the setpoint to the required value,
- `temperature_ramp` for a gradual heating / cooling followed by a hold

### Attributes
- `temperature` which is the temperature of the process thermocouple, `RO`, `float`
- `setpoint` which is the current temperature setpoint, `RW`, `float`
- `ramp_rate` which is the temperature ramp rate, `RW`, `float`
- `ramp_target` which is the target of the temperature ramp, `RW`, `float`
- `duty_cycle` which is the current duty cycle of the heater, `RO`, `float`

## Contributors

- Peter Kraus
