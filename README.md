## durandal_play 

An environment for experimenting with rust on 
the [Durandal](https://shop.holybro.com/c/durandal_0505)
 stm32h7-based flight controller.

For installation and debugging use either 
openocd (built with stm32h743 support)or the 
[daily build of the Black Magic Probe firmware](https://github.com/blacksphere/blackmagic/wiki/Upgrading-Firmware)
(which also requires recently introduced stm32h743 support).
We've used the 
[Zubax Drone Code Probe](https://kb.zubax.com/display/MAINKB/Dronecode+Probe+documentation)
successfully with this latest firmware.



## Status

This is very much work-in-progress

- [x] Debug build runs on Durandal
- [x] Access external "I2C A" bus (`I2C4`)
- [x] ssd1306 external i2C OLED driver 
- [ ] 3 LED indicators
- [ ] Internal SPI bus access
- [ ] External SPI bus access
- [ ] Internal I2C bus access
- [ ] ms5611 internal barometer readings
- [ ] ICM20689 internal 6DOF reading (no existing driver)
- [ ] BMI088 internal 6DOF reading interal SPI (no existing driver)
- [ ] IST8310 internal mag reading (no existing driver)
- [ ] Running on RTOS
- [ ] USB-C serial I/O
- [ ] S.Bus or CPPM input (radio control)
- [ ] GPS port access
- [ ] External GPS reading (eg ublox Neo-M8N GPS, see ublox crate)
- [ ] FMU PWM output 
- [ ] IO PWM output
- [ ] CAN bus access
- [ ] Calibration routine for IMUs
- [ ] CI
- [ ] Documentation
- [ ] IST8310 external i2c mag (no existing driver?)
- [ ] hmc5883 external mag (no existing driver?)
- [ ] qmc5883 external mag (no existing driver?)
- [ ] LIS3MDL external mag (no existing driver? see LSM303c?)


## Clocks
- Durandal has an external oscillator crystal at 16 MHz (HSE)
- Also has a low speed 32.768 kHz crystal (LSE)
- `SYSCLK` should run at 480 MHz (400 MHz min)

## Notes on buses
###  I2C Buses
Format: `(SCL, SDA)`
- External i2c bus labeled "GPS" is `I2C1`: `(PB8, PB9)`
- External i2c bus labeled I2CB??? is  `I2C2`: `(PF1, PF0)`
- Internal i2c bus is `I2C3`: `(PH7, PH8)`
- External i2c bus labeled "I2C A" is `I2C4`: `(PF14, PF15)`
 
### SPI Buses
Format: `(MISO, MOSI, SCK)`
- SPI1 is 2 MHz internal sensors: `(PA6, PD7, PG11)`
  - ICM20689 6DOF CS1: `PF2` , DRDY1:`PB4`  
  - BMI088 gyro CS3: `PF4` , DRDY2: `PB14` 
  - BMI088 gyro DRDY5: `PC13` 
  - BMI088 accel CS4:  `PG10` , DRDY3: `PB15`
  - BMI088 accel DRDY6: `PD10`
  - CS5: aux memory??? `PH5`

- SPI2 is dedicated to FRAM: `(PI2, PI3, PI1)`
  - FRAM CS: `PF5` 
- SPI4 is internal barometer:  `(PE13, PE6, PE2)`
  - MS5611 CS1: `PF10` 
- SPI5 is SPI bus EXTERNAL1: `(PF8, PF9, PF7)`
  - CS1: `PI4`
  - CS2 : `PI10` 
  - DRDY7: `PD15`
- SPI6 is SPI bus EXTERNAL2: `(PG12, PB5, PG13)`
  - CS1: `PI6`
  - CS2: `PI7`
  - CS3: `PI8`


### CAN buses
Format: `(RX, TX)`
- CAN1: `(PI9, PH13)`
- CAN2: `(PB12, PB13)`

### USB OTG
- `OTG_FS_DM` `PA11`
- `OTG_FS_DP` `PA12`
- `VBUS` `PA9`

## License

BSD-3-Clause, see `LICENSE` file. 