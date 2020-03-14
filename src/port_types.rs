use stm32h7xx_hal as p_hal;
use p_hal::prelude::*;
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


pub trait embedded_hal::blocking::spi::Transfer<_> + embedded_hal::blocking::spi::Write<_>

// pub type InternSpiPort4Type p_hal::spi::Spi<SPI4,
// (p_hal::gpio::gpioe::PE2<p_hal::gpio::Alternate<p_hal::gpio::AF45>
//
// >;
//        let miso = gpioe.pe13.into_alternate_af5();
//         let mosi = gpioe.pe6.into_alternate_af5();
//         let sck = gpioe.pe2.into_alternate_af5();
//p_hal::spi::Spi::spi4(dp.SPI4, (sck, miso, mosi), embedded_hal::spi::MODE_0, 2.mhz(), &ccdr)

//// #define GPIO_SPI4_CS1_MS5611      /* PF10 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET|GPIO_PORTF|GPIO_PIN10)