use stm32h7xx_hal as p_hal;
use p_hal::stm32;

use stm32::I2C1;
// use stm32::I2C2;
use stm32::I2C3;
use stm32::I2C4;

use stm32::SPI4;
use stm32::UART7;

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


pub type Spi4PortType = p_hal::spi::Spi<SPI4,
    (p_hal::gpio::gpioe::PE2<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioe::PE13<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioe::PE6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
    )
>;



pub type Uart7PortType = p_hal::serial::Serial<UART7,
    ( p_hal::gpio::gpioe::PE8<p_hal::gpio::Alternate<p_hal::gpio::AF7>>,
    p_hal::gpio::gpiof::PF6<p_hal::gpio::Alternate<p_hal::gpio::AF7>>
    )
    >;