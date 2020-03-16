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

#[macro_use]
extern crate cortex_m_rt;

#[allow(dead_code)]
mod port_types;

use port_types::{ExternI2cPortAType, Spi4PortType};
use p_hal::serial::config::{WordLength, Parity, StopBits};
use crate::port_types::Uart7PortType;


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



fn setup_peripherals() ->  (
    // i2c4:
    ExternI2cPortAType,
    // spi4_port
    Spi4PortType,
    // spi4_cs1
    impl OutputPin + ToggleableOutputPin ,
    // UART7 -- debug serial port
    Uart7PortType,
    //impl ehal::serial::Read<u8> + ehal::serial::Write<u8> ,
    // user_led1
    impl OutputPin + ToggleableOutputPin,
    // delay_source
    impl  DelayMs<u8>,
) {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();

    let pwr = dp.PWR.constrain();
    let vos = pwr.freeze();

    //use the existing sysclk
    let mut ccdr = rcc.freeze(vos, &dp.SYSCFG);
    let clocks = ccdr.clocks;

    let delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split(&mut ccdr.ahb4);
    let _gpioc = dp.GPIOC.split(&mut ccdr.ahb4);
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);
    let gpiof = dp.GPIOF.split(&mut ccdr.ahb4);

    let user_led1 = gpiob.pb1.into_push_pull_output(); // FMU "B/E" light on durandal

    let i2c4_port = {
        let scl = gpiof.pf14.into_alternate_af4().set_open_drain();
        let sda = gpiof.pf15 .into_alternate_af4().set_open_drain();
        //p_hal::i2c::I2c::i2c4(dp.I2C4, (scl, sda), 400.khz(), &ccdr)
        dp.I2C4.i2c((scl, sda), 400.khz(), &ccdr)
    };

    //TODO bump this to 20 MHz?
    let spi4_port =  {
        let sck = gpioe.pe2.into_alternate_af5();
        let miso = gpioe.pe13.into_alternate_af5();
        let mosi = gpioe.pe6.into_alternate_af5();
        dp.SPI4.spi((sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
        //p_hal::spi::Spi::spi4(dp.SPI4, (sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
    };
    let mut spi4_cs1 = gpiof.pf10.into_push_pull_output();
    spi4_cs1.set_high().unwrap();

    //UART7 is debug (dronecode port): `(PF6, PE8)`
    //        pins: PINS,
    //         config: config::Config,
    //         ccdr: &mut Ccdr,
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


    // let serial = dp
    //     .USART3
    //     .usart((tx, rx), serial::config::Config::default(), &mut ccdr)
    //     .unwrap();

    (i2c4_port,  spi4_port, spi4_cs1,  uart7_port, user_led1, delay_source)
}

// const LABEL_TEXT_HEIGHT: i32 = 5;
// const SCREEN_WIDTH: i32 = 128;
const SCREEN_HEIGHT: i32 = 32;



#[entry]
fn main() -> ! {

    let (i2c4_port,  spi4_port, spi4_cs1,
         uart7_port,mut user_led1, mut delay_source) =
        setup_peripherals();

    let i2c_bus4 = shared_bus::CortexMBusManager::new(i2c4_port);

    let mut format_buf = ArrayString::<[u8; 20]>::new();
    let mut disp: GraphicsMode<_> = ssd1306::Builder::new().connect_i2c(i2c_bus4.acquire()).into();
    disp.init().unwrap();
    disp.set_rotation(DisplayRotation::Rotate0).unwrap();
    disp.flush().unwrap();

    let (mut po_tx, mut _po_rx) = uart7_port.split();

    let _ = user_led1.set_low();

    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    let mut msbaro = Ms5611::new(spi4_port,
                        spi4_cs1,
                        &mut delay_source).unwrap();

    let mut loop_count = 0;
    loop {

        let ms_sample = msbaro
            .get_second_order_sample(Oversampling::OS_2048, &mut delay_source);
        let ms_press = if ms_sample.is_ok() {
            ms_sample.unwrap().pressure
        }
        else { 0 };

        //let bmp_press = 10.0 * barometer.pressure_one_shot();
        //debug_println!("press: {:.2}", ms_press);

        //overdraw the label
        format_buf.clear();
        if fmt::write(&mut format_buf, format_args!("{}", ms_press)).is_ok() {
            let le_str = format_buf.as_str();
            po_tx.write_str(le_str).unwrap();
            po_tx.write_str("\r\n").unwrap();
            //writeln!(po_tx, le_str).unwrap();

            disp.draw(
                Font6x8::render_str(le_str)
                    .with_stroke(Some(1u8.into()))
                    .translate(Coord::new(20, SCREEN_HEIGHT  / 2))
                    .into_iter(),
            );
        }
        disp.flush().unwrap();

        let _ = user_led1.toggle();
        delay_source.delay_ms(1u8);
        loop_count +=1;
    }

}





