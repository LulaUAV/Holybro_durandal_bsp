#![no_std]
#![no_main]

// pick a panicking behavior
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support

#[cfg(not(debug_assertions))]
extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
#[cfg(debug_assertions)]
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

use cortex_m_rt::{entry, ExceptionFrame};
use stm32h7xx_hal as p_hal;

#[macro_use]
mod macros;


use p_hal::{stm32, prelude::*};


use embedded_hal as ehal;
use ehal::digital::v2::OutputPin;
use ehal::digital::v2::ToggleableOutputPin;
use ehal::blocking::delay::DelayMs;

// SSD1306 external OLED display (for debug)
use ssd1306::prelude::*;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;


use core::fmt;
use core::fmt::Write;
use arrayvec::ArrayString;

use p_hal::time::{U32Ext};

/// Sensors
use ms5611_spi as ms5611;
use ms5611::{Ms5611, Oversampling};
use ist8310::{IST8310};
// use icm20689::{ICM20689};

#[macro_use]
extern crate cortex_m_rt;

#[allow(dead_code)]
mod port_types;

use crate::port_types::{Uart7PortType, HalI2cError, HalSpiError, Spi1PortType };
use p_hal::serial::config::{WordLength, Parity, StopBits};
use embedded_hal::digital::v2::InputPin;
use cortex_m::asm::bkpt;
use stm32h7xx_hal::rcc::PllConfigStrategy;
use stm32h7xx_hal::pwr::VoltageScale;


// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// // we can override this in debug mode for handy debugging
// #[exception]
// fn DefaultHandler(_irqn: i16) {
//     bkpt();
//     d_println!(get_debug_log(), "IRQn = {}", _irqn);
// }


// // cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

// const LABEL_TEXT_HEIGHT: i32 = 5;
// const SCREEN_WIDTH: i32 = 128;
const SCREEN_HEIGHT: i32 = 32;



fn setup_peripherals() ->  (
    // i2c3
    // InternalI2cPortType
    impl ehal::blocking::i2c::Read<Error = HalI2cError> + ehal::blocking::i2c::Write<Error = HalI2cError> + ehal::blocking::i2c::WriteRead<Error = HalI2cError>,
    // i2c4
    // ExternI2cPortAType,
    impl ehal::blocking::i2c::Read<Error = HalI2cError> + ehal::blocking::i2c::Write<Error = HalI2cError> + ehal::blocking::i2c::WriteRead<Error = HalI2cError>,

    // spi1
    //Spi1PortType,
    impl ehal::blocking::spi::Transfer<u8, Error=HalSpiError> + ehal::blocking::spi::Write<u8, Error=HalSpiError>,

    (
        //spi1_cs_tdk
        impl OutputPin,

        //spi1_drdy_tdk
        impl InputPin,
    ),
    // spi4
    //Spi4PortType,
    impl ehal::blocking::spi::Transfer<u8, Error=HalSpiError> + ehal::blocking::spi::Write<u8, Error=HalSpiError>,

    // spi4_cs1
    impl OutputPin,
    // UART7 -- debug serial port
    Uart7PortType,
    // user_led1
    impl OutputPin + ToggleableOutputPin,
    // delay_source
    impl  DelayMs<u8>,
) {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    // For Durandal we know we have:
    // - 16 MHz xtal HSE
    // - SYSCLK of 480 MHz (processor max)
    // - HCLK of SYSCLK/2 (240 MHz)
    // - (PCLK1, PCLK2, PCLK3, PCLK4) is HCLK/2 (120 MHz)
    // - PLL1P = PLL1_VCO/2  = 960 MHz / 2   = 480 MHz
    // - PLL1Q = PLL1_VCO/4  = 960 MHz / 4   = 240 MHz
    // - PLL1R = PLL1_VCO/8  = 960 MHz / 8   = 120 MHz
    const LE_SYSCLK:u32 = 480;
    const LE_HCLK:u32 = LE_SYSCLK/2;
    const LE_PCLK:u32 = LE_HCLK/2;
    let rcc = dp.RCC
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


    //let rcc = dp.RCC.constrain();

    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();
    //TODO vos is coming back as Scale1 but needs to be Scale0 to get 480 MHz ?
    let vos = VoltageScale::Scale0; //force higher?
    // For SPI1 need clock source of:  STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL2

    bkpt();
    //use the existing sysclk
    let mut ccdr = rcc.freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpioa = dp.GPIOA.split(&mut ccdr.ahb4);
    let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
    //let gpioc = dp.GPIOC.split(&mut ccdr.ahb4);
    let gpiod = dp.GPIOD.split(&mut ccdr.ahb4);
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);
    let gpiof = dp.GPIOF.split(&mut ccdr.ahb4);
    let gpiog = dp.GPIOG.split(&mut ccdr.ahb4);
    let gpioh = dp.GPIOH.split(&mut ccdr.ahb4);

    let user_led1 = gpiob.pb1.into_push_pull_output(); // FMU "B/E" light on durandal

    //I2C3 is internal (PH7, PH8) ... used by mag
    let i2c3_port = {
        let scl = gpioh.ph7.into_alternate_af4().set_open_drain();
        let sda = gpioh.ph8 .into_alternate_af4().set_open_drain();
        dp.I2C3.i2c((scl, sda), 400.khz(), &ccdr)
    };

    //I2C4 is external "I2C A" port
    let i2c4_port = {
        let scl = gpiof.pf14.into_alternate_af4().set_open_drain();
        let sda = gpiof.pf15 .into_alternate_af4().set_open_drain();
        dp.I2C4.i2c((scl, sda), 400.khz(), &ccdr)
    };

    //bkpt();
    //setup SPI1 for the bulk of SPI-connected internal sensors
    let spi1_port =  {
        let sck = gpiog.pg11.into_alternate_af5();
        let miso = gpioa.pa6.into_alternate_af5();
        let mosi = gpiod.pd7.into_alternate_af5();
        dp.SPI1.spi((sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
    };
    //PF2 is CS for TDK IMU ICM20689
    // TODO setup at 2 MHz?
    let mut spi1_cs_tdk = gpiof.pf2.into_push_pull_output();
    //PB4 is DRDY for TDK IMU ICM20689
    let mut spi1_drdy_tdk = gpiob.pb4.into_floating_input();

    let spi4_port =  {
        let sck = gpioe.pe2.into_alternate_af5();
        let miso = gpioe.pe13.into_alternate_af5();
        let mosi = gpioe.pe6.into_alternate_af5();
        dp.SPI4.spi((sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
    };
    let mut spi4_cs1 = gpiof.pf10.into_push_pull_output();
    spi4_cs1.set_high().unwrap();



    //TODO setup BMI088 GYRO SP1 CS pin PF4 at 2 MHz
    //TODO setup BMI088 GYRO SP1 CS pin PG10 at 2 MHz

    //UART7 is debug (dronecode port): `(PF6, PE8)`
    let uart7_port = {
        let config =   p_hal::serial::config::Config {
            baudrate: 57_600_u32.bps(),
            wordlength: WordLength::DataBits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1
        };
        let rx = gpiof.pf6.into_alternate_af7();
        let tx = gpioe.pe8.into_alternate_af7();
        dp.UART7.usart((tx,rx),config, &mut ccdr).unwrap()
    };


    (i2c3_port ,
     i2c4_port,
     spi1_port,
     (spi1_cs_tdk, spi1_drdy_tdk),
     spi4_port, spi4_cs1,
     uart7_port,
     user_led1,
     delay_source)
}



#[entry]
fn main() -> ! {

    let (i2c3_port,
        i2c4_port,
        spi1_port,
        (spi1_cs_tdk, spi1_drdy_tdk),
        spi4_port,
        spi4_cs1,
        uart7_port,
        mut user_led1,
        mut delay_source) =
        setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let spi_bus4 = shared_bus::CortexMBusManager::new(spi4_port);
    let i2c_bus3 = shared_bus::CortexMBusManager::new(i2c3_port);
    let i2c_bus4 = shared_bus::CortexMBusManager::new(i2c4_port);

    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    // let iface = icm20689::SpiInterface::new(
    //     spi_bus1.acquire(),
    //     spi1_cs_tdk,
    //     spi1_drdy_tdk);

    //bkpt();
    let mut tdk_6dof = icm20689::Builder::new_spi(
        spi_bus1.acquire(),
        spi1_cs_tdk,
    );
        // spi1_drdy_tdk);

    bkpt();
    let check = tdk_6dof.probe();
    if !check {
        panic!("probe failed");
    }

    let mut format_buf = ArrayString::<[u8; 20]>::new();
    let mut disp: GraphicsMode<_> = ssd1306::Builder::new().connect_i2c(i2c_bus4.acquire()).into();
    disp.init().unwrap();
    disp.set_rotation(DisplayRotation::Rotate0).unwrap();
    disp.flush().unwrap();

    let mut mag = IST8310::default(i2c_bus3.acquire()).unwrap();

    let mut msbaro = Ms5611::new(spi_bus4.acquire(),
                                 spi4_cs1,
                                 &mut delay_source).unwrap();

    let (mut po_tx, mut _po_rx) = uart7_port.split();

    let _ = user_led1.set_low();



    let mut loop_count = 0;
    loop {
        let mag_sample = mag.get_mag_vector(&mut delay_source).unwrap();

        let ms_sample = msbaro
            .get_second_order_sample(Oversampling::OS_2048, &mut delay_source);
        let ms_press = if ms_sample.is_ok() {
            ms_sample.unwrap().pressure
        }
        else { 0 };

        //let bmp_press = 10.0 * barometer.pressure_one_shot();
        //debug_println!("press: {:.2}", ms_press);

        format_buf.clear();
        if fmt::write(&mut format_buf,
                      format_args!("{} {} {} \r\n", mag_sample[0],mag_sample[1],mag_sample[2]))
            .is_ok() {
            let le_str = format_buf.as_str();
            //write on console out
            po_tx.write_str(le_str).unwrap();

            // draw on the oled display
            disp.draw(
                Font6x8::render_str(le_str)
                    .with_stroke(Some(1u8.into()))
                    .translate(Coord::new(20, 8))
                    .into_iter(),
            );
            disp.flush().unwrap();
        }

        let _ = user_led1.toggle();
        delay_source.delay_ms(1u8);
        loop_count +=1;
    }

}





