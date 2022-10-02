# Recruiting microcontrollers

## Settings

- PC10 - GPIO Output (for checking if the timer is working) [CN7 pin 1]
- PC11 - GPIO Output (LED undervoltage) [CN7 pin 2]
- PC12 - GPIO Output (LED overvoltage) [CN7 pin 3]
- PC13 - GPIO EXTI (blue button)
- PA0 - ADC1 IN1 (sensor) [CN8 pin A0]
- PA1 - ADC1 IN2 (voltage) [CN8 pin A1]
- TIM6 - Timer (for ADC sampling)

The timer is set with a prescaler of 1000-1 and a counter period of 8500-1, so that it is triggered every 50 ms. Every time the timer is triggered it increases a counter that modifies the status of two flags that control the sensor and voltage reading.

The adc reading is done using the polling method.

Every 200 ms the sensor input is read, every 350 ms the voltage value is read. If the value of the expansion is out of the range (1.8 - 2.7) the DANGER state is activated and the corresponding LED is turned on.

If the button is pressed the WAITING state is activated and the waiting message is transmitted every 500 ms until the button is pressed again

![flowchart](./media/flowchart.png)