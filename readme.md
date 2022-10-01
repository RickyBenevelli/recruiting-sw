# Recruiting microcontrollers

## Settings

- PC10 - GPIO Output (for checking if the timer is working) [CN7 pin 1]
- PC11 - GPIO Output (LED undervoltage) [CN7 pin 2]
- PC12 - GPIO Output (LED overvoltage) [CN7 pin 3]
- PC13 - GPIO EXTI (blue button)
- PA0 - ADC1 (sensor) [CN8 pin A0]
- TIM6 - Timer (for ADC sampling)

The timer is set with a prescaler of 1000-1 and a counter period of 8500-1, so that it is triggered every 50 ms. Every time the timer is triggered it increases a counter that modifies the status of two flags that control the sensor and voltage reading.

The adc reading is done using the polling method.

---

### Yet to be implemented:

If the voltage detected is outside the required parameters, two LEDs still to be set will light up.
The finite state machine will be managed with ``switch`` condition.

- [ ] Finite State Machine
- [ ] checking the range values of sensor
- [ ] checking the range values of voltage
- [x] LEDs
- [ ] wainting state
- [ ] convert volt to gauss