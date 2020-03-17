use embedded_hal as ehal;

use stm32h7xx_hal as p_hal;
use p_hal::stm32;

use stm32::I2C1;
// use stm32::I2C2;
use stm32::I2C3;
use stm32::I2C4;

use stm32::SPI1;
use stm32::SPI4;
use stm32::UART7;
// use shared_bus::BusManager;

pub type HalI2cError = p_hal::i2c::Error;
pub type HalSpiError = p_hal::spi::Error;

pub struct SpiConfig<SPI, CSN, DRDY> {
    /// the SPI port to use when communicating
    spi: SPI,
    /// the Chip Select pin (GPIO output) to use when communicating
    csn: CSN,
    /// an input pin (GPIO input) that indicates when data is available (Data Ready)
    drdy: DRDY,
}

/// A filler type for when the CSN pin is unnecessary
pub struct NoCsn;

/// A filler type for when the DRDY pin is unnecessary
pub struct NoDrdy;


// type SharedBusType<T> = BusManager<shared_bus::BusMutex<cell::RefCell<T>>, T>;



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
// pub type InternalI2cBusType = SharedBusType<InternalI2cPortType>;


type I2c4PortType = p_hal::i2c::I2c<I2C4,
    (p_hal::gpio::gpiof::PF14<p_hal::gpio::Alternate<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiof::PF15<p_hal::gpio::Alternate<p_hal::gpio::AF4>>)
>;
pub type ExternI2cPortAType = I2c4PortType;
// pub type ExternI2cBusAType = SharedBusType<ExternI2cPortAType>;


pub type Spi1PortType = p_hal::spi::Spi<SPI1,
    (p_hal::gpio::gpiog::PG11<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioa::PA6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpiod::PD7<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
    )
>;

pub type Spi4PortType = p_hal::spi::Spi<SPI4,
    (p_hal::gpio::gpioe::PE2<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioe::PE13<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
     p_hal::gpio::gpioe::PE6<p_hal::gpio::Alternate<p_hal::gpio::AF5>>,
    )
>;
// pub type Spi4BusType = SharedBusType<Spi4PortType>;


pub type Uart7PortType = p_hal::serial::Serial<UART7,
    ( p_hal::gpio::gpioe::PE8<p_hal::gpio::Alternate<p_hal::gpio::AF7>>,
    p_hal::gpio::gpiof::PF6<p_hal::gpio::Alternate<p_hal::gpio::AF7>>
    )
    >;