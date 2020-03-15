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

use p_hal::prelude::*;
use p_hal::stm32;


use embedded_hal::digital::v1_compat::OldOutputPin;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::digital::v2::ToggleableOutputPin;
use embedded_hal::blocking::delay::DelayMs;

// SSD1306 external OLED display (for debug)
use ssd1306::prelude::*;
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;
// use embedded_graphics::primitives::{Rect};
use core::fmt;
use arrayvec::ArrayString;


// use cortex_m::asm::bkpt;
use p_hal::time::{U32Ext, Hertz};

/// Sensors
use ms5611_spi::{Ms5611, Oversampling};

#[macro_use]
extern crate cortex_m_rt;

mod port_types;
use port_types::{ExternI2cPortAType};

struct LameError {

}

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
    //impl embedded_hal::blocking::i2c::Read + embedded_hal::blocking::i2c::Write + embedded_hal::blocking::i2c::WriteRead,
    // spi4_port
    impl  embedded_hal::blocking::spi::Write<u8> + embedded_hal::blocking::spi::Transfer<u8>,
    // spi4_cs1
    impl OutputPin + ToggleableOutputPin ,
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
    let gpioe = dp.GPIOE.split(&mut ccdr.ahb4);
    let gpiof = dp.GPIOF.split(&mut ccdr.ahb4);

    let user_led1 = gpiob.pb0.into_push_pull_output(); //TODO

    let i2c4_port = {
        let scl = gpiof.pf14.into_alternate_af4().set_open_drain();
        let sda = gpiof.pf15 .into_alternate_af4().set_open_drain();
        p_hal::i2c::I2c::i2c4(dp.I2C4, (scl, sda), 400.khz(), &ccdr)
    };

    let spi4_port =  {
        let miso = gpioe.pe13.into_alternate_af5();
        let mosi = gpioe.pe6.into_alternate_af5();
        let sck = gpioe.pe2.into_alternate_af5();
        p_hal::spi::Spi::spi4(dp.SPI4, (sck, miso, mosi), embedded_hal::spi::MODE_0, 2.mhz(), &ccdr)
        // miso, mosi, sck: (PE13, PE6, PE2)
    };

    let mut spi4_cs1 = gpiof.pf10.into_push_pull_output();
    spi4_cs1.set_high().unwrap();

    (i2c4_port,  spi4_port, spi4_cs1,  user_led1, delay_source)
}

// const LABEL_TEXT_HEIGHT: i32 = 5;
const SCREEN_WIDTH: i32 = 128;
const SCREEN_HEIGHT: i32 = 32;


#[entry]
fn main() -> ! {

    let (i2c4_port,  spi4_port, spi4_cs1,  mut user_led1, mut delay_source) =
        setup_peripherals();
    #[cfg(debug_assertions)]

    //TODO need SPI pins for this

    let i2c_bus4 = shared_bus::CortexMBusManager::new(i2c4_port);

    let mut format_buf = ArrayString::<[u8; 20]>::new();
    let mut disp: GraphicsMode<_> = ssd1306::Builder::new().connect_i2c(i2c_bus4.acquire()).into();
    disp.init().unwrap();
    disp.set_rotation(DisplayRotation::Rotate0).unwrap();
    disp.flush().unwrap();

    let _ = user_led1.set_low();
    //d_println!(log, "ready!");
    delay_source.delay_ms(1u8);

    //MS5611 CS1: `PF10`
    //let mut msbaro  = Ms5611::new(spi4_port, spi4_cs1.into(), delay_source).unwrap();

    let spi4_cs1_v1 = OldOutputPin::new(spi4_cs1);
    let mut msbaro  = Ms5611::new(spi4_port,
                                  spi4_cs1_v1,
                                  delay_source).unwrap();

    let _sample = msbaro
        .get_second_order_sample(Oversampling::OS_2048)
        .unwrap();

    let mut loop_count = 0;
    loop {
        // let abs_press = 10.0 * barometer.pressure_one_shot();
        // #[cfg(debug_assertions)]
        // hprintln!("press: {:.2}", abs_press).unwrap();


        //overdraw the label
        format_buf.clear();
        if fmt::write(&mut format_buf, format_args!("{}", loop_count)).is_ok() {
            disp.draw(
                Font6x8::render_str(format_buf.as_str())
                    .with_stroke(Some(1u8.into()))
                    .translate(Coord::new(20, SCREEN_HEIGHT  / 2))
                    .into_iter(),
            );
        }
        disp.flush().unwrap();

        // xpos = xpos + BAR_WIDTH;
        // if xpos > SCREEN_WIDTH { xpos = 0; }

        let _ = user_led1.toggle();
        delay_source.delay_ms(250u8);
        loop_count +=1;
    }

}





