use stm32h7xx_hal as p_hal;
use p_hal::stm32;

use stm32::I2C1;
// use stm32::I2C2;
use stm32::I2C3;
use stm32::I2C4;

use stm32::SPI4;


type I2c1PortType = p_hal::i2c::I2c<I2C1,
    (p_hal::gpio::gpiob::PB8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiob::PB9<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
pub type ExternGPSI2cPortType = I2c1PortType;


type I2c3PortType = p_hal::i2c::I2c<I2C3,
    (p_hal::gpio::gpioh::PH7<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpioh::PH8<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
pub type InternalI2cPortType = I2c3PortType;


type I2c4PortType = p_hal::i2c::I2c<I2C4,
    (p_hal::gpio::gpiof::PF14<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiof::PF15<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
pub type ExternI2cPortAType = I2c4PortType;


//- SPI4 is internal barometer:  `(PE13, PE6, PE2)`
//   - MS5611 CS1: `PF10`
//Pins<SPI> for (SCK, MISO, MOSI)
pub type Spi4PortType = p_hal::spi::Spi<SPI4,
    (p_hal::gpio::gpioe::PE2<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioe::PE13<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioe::PE6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
    )
>;


// let spi4_port =  {
// let miso = gpioe.pe13.into_alternate_af5();
// let mosi = gpioe.pe6.into_alternate_af5();
// let sck = gpioe.pe2.into_alternate_af5();
// p_hal::spi::Spi::spi4(dp.SPI4, (sck, miso, mosi), embedded_hal::spi::MODE_0, 2.mhz(), &ccdr)
//
// };
