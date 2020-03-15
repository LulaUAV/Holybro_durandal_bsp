use stm32h7xx_hal as p_hal;
use p_hal::stm32;

use stm32::I2C1;
// use stm32::I2C2;
// use stm32::I2C3;
use stm32::I2C4;


pub type I2c1PortType = p_hal::i2c::I2c<I2C1,
    (p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
pub type ExternGPSI2cPortType = I2c1PortType;

pub type I2c4PortType = p_hal::i2c::I2c<I2C4,
    (p_hal::gpio::gpiof::PF14<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiof::PF15<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
pub type ExternI2cPortAType = I2c4PortType;

