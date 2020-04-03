/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

use p_hal::{prelude::*, stm32};
use stm32h7xx_hal as p_hal;

use ehal::blocking::delay::{DelayMs, DelayUs};
use ehal::digital::v2::OutputPin;
use ehal::digital::v2::ToggleableOutputPin;
use embedded_hal as ehal;


use crate::port_types::{DbgUartPortType, Gps1PortType, HalGpioError, HalI2cError, HalSpiError};

use p_hal::pwr::VoltageScale;
use p_hal::rcc::PllConfigStrategy;
use p_hal::serial::config::{Parity, StopBits, WordLength};

// pub fn set_max_bps(port: &mut Gps1PortType, bps: u32) {
//
//     //get the initial baud rate
//     //set a series of baud rates until we find a speed that works
//
//     let config = p_hal::serial::config::Config {
//         baudrate: bps.bps(),
//         wordlength: WordLength::DataBits8,
//         parity: Parity::ParityNone,
//         stopbits: StopBits::STOP1,
//     };
//     port.set_config(config);
//
// }

pub fn setup_peripherals() -> (
    // i2c1
    impl ehal::blocking::i2c::Read<Error = HalI2cError>
        + ehal::blocking::i2c::Write<Error = HalI2cError>
        + ehal::blocking::i2c::WriteRead<Error = HalI2cError>,
    // i2c2
    impl ehal::blocking::i2c::Read<Error = HalI2cError>
        + ehal::blocking::i2c::Write<Error = HalI2cError>
        + ehal::blocking::i2c::WriteRead<Error = HalI2cError>,
    // i2c3
    // InternalI2cPortType
    impl ehal::blocking::i2c::Read<Error = HalI2cError>
        + ehal::blocking::i2c::Write<Error = HalI2cError>
        + ehal::blocking::i2c::WriteRead<Error = HalI2cError>,
    // i2c4: ExternI2cPortAType,
    impl ehal::blocking::i2c::Read<Error = HalI2cError>
        + ehal::blocking::i2c::Write<Error = HalI2cError>
        + ehal::blocking::i2c::WriteRead<Error = HalI2cError>,
    // spi1 :Spi1PortType,
    impl ehal::blocking::spi::Transfer<u8, Error = HalSpiError>
        + ehal::blocking::spi::Write<u8, Error = HalSpiError>,
    // SPI pins for ICM20689
    impl OutputPin<Error = HalGpioError>, // ICM20689 CS
    // SPI pins for BMI088
    (
        impl OutputPin<Error = HalGpioError>, // BMI088 gyro CS
        impl OutputPin<Error = HalGpioError>, // BMI088 accel CS
    ),
    // spi2 : Spi2PortType,
    impl ehal::blocking::spi::Transfer<u8, Error = HalSpiError>
        + ehal::blocking::spi::Write<u8, Error = HalSpiError>,
    // spi2_cs1
    impl OutputPin<Error = HalGpioError>,
    // spi4 : Spi4PortType,
    impl ehal::blocking::spi::Transfer<u8, Error = HalSpiError>
        + ehal::blocking::spi::Write<u8, Error = HalSpiError>,
    // spi4_cs1
    impl OutputPin,
    // UART1 -- GPS1 serial port
    Gps1PortType,
    // UART7 -- debug serial port
    DbgUartPortType,
    // user_led1
    impl OutputPin + ToggleableOutputPin,
    // delay_source
    impl DelayMs<u8> + DelayUs<u32>,
) {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    // --- Clock configuration
    // Set up the system clock
    // For Durandal we know we have:
    // - 16 MHz xtal HSE
    // - SYSCLK of 480 MHz (processor max)
    // - HCLK of SYSCLK/2 (240 MHz)
    // - (PCLK1, PCLK2, PCLK3, PCLK4) is HCLK/2 (120 MHz)
    // - PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz
    // - PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
    // - PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
    const LE_SYSCLK: u32 = 480;
    const LE_HCLK: u32 = LE_SYSCLK / 2;
    const LE_PCLK: u32 = LE_HCLK / 2;
    let rcc = dp
        .RCC
        .constrain()
        .use_hse(16.mhz()) // Durandal has 16 MHz xtal HSE
        .sysclk(LE_SYSCLK.mhz())
        .hclk(LE_HCLK.mhz())
        .pll1_p_ck(480.mhz())
        .pll1_q_ck(240.mhz())
        .pll1_r_ck(120.mhz())
        .pll1_strategy(PllConfigStrategy::Iterative)
        .pclk1(LE_PCLK.mhz())
        .pclk2(LE_PCLK.mhz())
        .pclk3(LE_PCLK.mhz())
        .pclk4(LE_PCLK.mhz());

    let pwr = dp.PWR.constrain();
    let _vos = pwr.freeze();
    //TODO vos defaults to Scale1 but needs to upgrade to Scale0 to boost to 480 MHz
    let vos = VoltageScale::Scale0; //may force higher? or just allow asserts to pass?

    //TODO need to write : self.rb.d3cr.write(|w| unsafe { w.vos().bits(0b11) });
    // see "VOS0 activation/deactivation sequence" in RM0433

    let mut ccdr = rcc.freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;
    // --- Clock configuration

    let delay_source = p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpioa = dp.GPIOA.split(&mut ccdr.ahb4);
    let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
    //let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);
    let gpiod = dp.GPIOD.split(&mut ccdr.ahb4);
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);
    let gpiof = dp.GPIOF.split(&mut ccdr.ahb4);
    let gpiog = dp.GPIOG.split(&mut ccdr.ahb4);
    let gpioh = dp.GPIOH.split(&mut ccdr.ahb4);
    let gpioi = dp.GPIOI.split(&mut ccdr.ahb4);

    let user_led1 = gpiob.pb1.into_push_pull_output(); // FMU "B/E" light on durandal

    // TODO order of I2C peripheral initialization is apparently critical for this MCU: I2C1 last

    //I2C4 is external "I2C A" port
    let i2c4_port = {
        let scl = gpiof.pf14.into_alternate_af4().set_open_drain();
        let sda = gpiof.pf15.into_alternate_af4().set_open_drain();
        dp.I2C4.i2c((scl, sda), 400.khz(), &ccdr)
    };

    // I2C3 is internal... used by mag
    let i2c3_port = {
        let scl = gpioh.ph7.into_alternate_af4().set_open_drain();
        let sda = gpioh.ph8.into_alternate_af4().set_open_drain();
        dp.I2C3.i2c((scl, sda), 400.khz(), &ccdr)
    };

    // I2C2 is on "Telem 4 / I2C B" external port
    let i2c2_port = {
        let scl = gpiof.pf1.into_alternate_af4().set_open_drain();
        let sda = gpiof.pf0.into_alternate_af4().set_open_drain();
        dp.I2C2.i2c((scl, sda), 400.khz(), &ccdr)
    };

    //I2C1 is on "GPS" external port
    // used by eg rgbled and ist8310 in Holybor M8N Pixhawk GPS module
    let i2c1_port = {
        let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
        let sda = gpiob.pb9.into_alternate_af4().set_open_drain();
        dp.I2C1.i2c((scl, sda), 100.khz(), &ccdr)
    };

    //setup SPI1 for the bulk of SPI-connected internal sensors
    // TODO need to increase SPI1 clock speed?
    let spi1_port = {
        let sck = gpiog.pg11.into_alternate_af5();
        let miso = gpioa.pa6.into_alternate_af5();
        let mosi = gpiod.pd7.into_alternate_af5();
        dp.SPI1
            .spi((sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
    };
    //PF2 is CS for TDK ICM20689 (2 MHz - 8 MHz)
    let mut spi1_cs_tdk = gpiof
        .pf2
        .into_push_pull_output()
        .set_speed(p_hal::gpio::Speed::Low); //TODO verify: should be 2 MHz
    spi1_cs_tdk.set_high().unwrap();

    //PB4 is DRDY for TDK ICM20689
    //let spi1_drdy_tdk = gpiob.pb4.into_floating_input();

    // PF4 is SPI1 CS4 BMI088 gyro
    let mut spi1_cs_bmi088_gyro = gpiof.pf4.into_push_pull_output();
    spi1_cs_bmi088_gyro.set_high().unwrap();

    // PG10 is SPI1 CS4 BMI088 accel
    let mut spi1_cs_bmi088_accel = gpiog.pg10.into_push_pull_output();
    spi1_cs_bmi088_accel.set_high().unwrap();

    // setup SPI2 for serial flash ram (FRAM)
    let spi2_port = {
        let sck = gpioi.pi1.into_alternate_af5();
        let miso = gpioi.pi2.into_alternate_af5();
        let mosi = gpioi.pi3.into_alternate_af5();
        dp.SPI2
            .spi((sck, miso, mosi), ehal::spi::MODE_3, 10.mhz(), &ccdr)
    };
    // PF5 is SPI2 CS1 for FRAM
    let mut spi2_cs1 = gpiof.pf5.into_push_pull_output();
    spi2_cs1.set_high().unwrap();

    let spi4_port = {
        let sck = gpioe.pe2.into_alternate_af5();
        let miso = gpioe.pe13.into_alternate_af5();
        let mosi = gpioe.pe6.into_alternate_af5();
        dp.SPI4
            .spi((sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
    };
    let mut spi4_cs1 = gpiof.pf10.into_push_pull_output();
    spi4_cs1.set_high().unwrap();

    // UART7 is debug serial console (dronecode debug port)
    let uart7_port = {
        let config = p_hal::serial::config::Config {
            baudrate: 57_600_u32.bps(),
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        };
        let rx = gpiof.pf6.into_alternate_af7();
        let tx = gpioe.pe8.into_alternate_af7();
        dp.UART7.usart((tx, rx), config, &mut ccdr).unwrap()
    };

    // UART8 is the serial connection to the px4io IO coprocessor
    let _uart8_port = {
        let config = p_hal::serial::config::Config {
            baudrate: 1_500_000_u32.bps(),
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        };
        let rx = gpioe.pe0.into_alternate_af8();
        let tx = gpioe.pe1.into_alternate_af8();
        dp.UART8.usart((tx, rx), config, &mut ccdr).unwrap()
    };

    // USART1 is GPS1 port:
    let gps1_port = {
        let config = p_hal::serial::config::Config::default().baudrate(115200.bps());
        let rx = gpiob.pb7.into_alternate_af7();
        let tx = gpiob.pb6.into_alternate_af7();
        dp.USART1.usart((tx, rx), config, &mut ccdr).unwrap()
    };

    (
        i2c1_port,
        i2c2_port,
        i2c3_port,
        i2c4_port,
        spi1_port,
        spi1_cs_tdk,
        (spi1_cs_bmi088_gyro, spi1_cs_bmi088_accel),
        spi2_port,
        spi2_cs1,
        spi4_port,
        spi4_cs1,
        gps1_port,
        uart7_port,
        user_led1,
        delay_source,
    )
}
