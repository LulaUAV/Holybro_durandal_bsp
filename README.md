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
- [x] Release build runs on Durandal
- [x] Access external "I2C A" bus (`I2C4`)
- [x] ssd1306 external i2C OLED driver 
- [x] Internal SPI4 bus access
- [x] ms5611 internal barometer readings (requires updated ms5611-ehal driver)
- [x] blink front panel indicator LED
- [x] Debug serial console output (on dronecode connector serial port `UART7`)
- [x] Access internal `I2C3` bus
- [x] IST8310 internal magnetometer reading (use new ist8310 crate)
- [x] initial clock configuration to match max Durandal specs
- [x] All internal SPI bus access (`SPI1`, `SPI4`)
- [x] BMI088 internal 6DOF reading internal SPI (use new BMI088 driver)
- [ ] ICM20689 internal 6DOF reading (new driver is unconfirmed)
- [ ] External SPI bus access
- [ ] Internal I2C bus access
- [ ] External "I2C B" bus access (`I2C2`)
- [ ] Telem 1, 2, 3, 4 serial port access (uart)
- [ ] SDMMC read/write
- [ ] Running on RTOS
- [ ] USB-C serial I/O
- [ ] S.Bus or CPPM input (radio control)
- [ ] GPS port access
- [ ] External GPS reading (eg ublox Neo-M8N GPS, see ublox crate)
- [ ] FMU PWM output 
- [ ] IO PWM output
- [ ] CAN bus access
- [ ] Calibration routine for IMUs
- [ ] Semihosting debug support via dronecode debug port
- [ ] CI
- [ ] Documentation
- [ ] IST8310 external i2c mag (use ist8310 crate)
- [ ] hmc5883 external mag (no existing ehal driver?)
- [ ] qmc5883 external mag (no existing ehal driver?)
- [ ] LIS3MDL external mag (no existing ehal driver? see LSM303c?)


## Clocks
- Durandal has an external oscillator crystal at 16 MHz (HSE)
- Also has a low speed 32.768 kHz crystal (LSE)
- `SYSCLK` should run at 480 MHz (400 MHz min)

## Notes on buses
###  I2C Buses
Format: `(SCL, SDA)`
- External i2c bus labeled "GPS" is `I2C1`: `(PB8, PB9)`
- External i2c bus labeled "I2C B" is  `I2C2`: `(PF1, PF0)`
- Internal i2c bus is `I2C3`: `(PH7, PH8)` ... used by mag
- External i2c bus labeled "I2C A" is `I2C4`: `(PF14, PF15)`

### SPI Buses
Format:  `(SCK, MISO, MOSI)` 
- SPI1 is 2 MHz internal sensors: `(PG11, PA6, PD7)`
  - ICM20689 6DOF CS1: `PF2` , DRDY1:`PB4`  
  - BMI088 gyro CS3: `PF4` , DRDY2: `PB14` 
  - BMI088 gyro DRDY5: `PC13` 
  - BMI088 accel CS4:  `PG10` , DRDY3: `PB15`
  - BMI088 accel DRDY6: `PD10`
  - CS5: aux memory??? `PH5`

- SPI2 is dedicated to FRAM: `(PI1, PI2, PI3)`
  - FRAM CS: `PF5` 
  - 8.0 MHz, MODE_3 

- SPI4 is internal barometer:  `(PE2, PE13, PE6)`
  - MS5611 CS1: `PF10` 
  - 2.0 MHz, MODE_3  (or is it 20 MHz?)
      
- SPI5 is SPI bus EXTERNAL1: `(PF7, PF8, PF9 )`
  - CS1: `PI4`
  - CS2 : `PI10` 
  - DRDY7: `PD15`
  
- SPI6 is SPI bus EXTERNAL2: `(PG13, PG12, PB5)`
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

### LEDs
- `PC7` is the blue led marked "ACT"
- `PC6` is the green led marked "PWR"
- `PB1` is the red led marked "B/E"

### UARTs / USARTs
Format: `(RX, TX)` , `(CTS, RTS)`

- USART1 is GPS1: `(PB7, PB6)`
- USART2 is Telem1: `(PD6, PD5)` , `(PD3, PD4)`
- USART3 is Telem2: `(PD9, PD8)` , `(PD11, PD12)`
- UART4 is GPS2: `(PD0, PD1)` 
- USART6 is Telem3: `(PG9, PG14)` with special needs
- UART7 is debug (dronecode port): `(PF6, PE8)`
- UART8 is IOMCU: `(PE0, PE1)`


## License

BSD-3-Clause, see `LICENSE` file. 