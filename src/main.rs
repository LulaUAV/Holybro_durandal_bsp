/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/



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

// #[macro_use]
// mod macros;

use p_hal::{prelude::*, stm32};

use ehal::blocking::delay::DelayMs;
use ehal::digital::v2::OutputPin;
use ehal::digital::v2::ToggleableOutputPin;
use embedded_hal as ehal;

// SSD1306 external OLED display (for debug)
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;
use ssd1306::prelude::*;
use ssd1306::interface::DisplayInterface;

use arrayvec::ArrayString;
use core::fmt;
use core::fmt::{Arguments, Write};

use p_hal::time::U32Ext;

/// Sensors
use ist8310::IST8310;
use ms5611_spi as ms5611;
use ms5611::{Ms5611, Oversampling};

#[macro_use]
extern crate cortex_m_rt;

#[allow(dead_code)]
mod port_types;

use crate::port_types::{HalGpioError, HalI2cError, HalSpiError, Uart7PortType};
use cortex_m::asm::bkpt;

use p_hal::pwr::VoltageScale;
use p_hal::rcc::PllConfigStrategy;
use p_hal::serial::config::{Parity, StopBits, WordLength};
use spi_memory::series25::Identification;


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


fn setup_peripherals() -> (
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
    // UART7 -- debug serial port
    Uart7PortType,
    // user_led1
    impl OutputPin + ToggleableOutputPin,
    // delay_source
    impl DelayMs<u8>,
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

    //I2C3 is internal (PH7, PH8) ... used by mag
    let i2c3_port = {
        let scl = gpioh.ph7.into_alternate_af4().set_open_drain();
        let sda = gpioh.ph8.into_alternate_af4().set_open_drain();
        dp.I2C3.i2c((scl, sda), 400.khz(), &ccdr)
    };

    //I2C4 is external "I2C A" port
    let i2c4_port = {
        let scl = gpiof.pf14.into_alternate_af4().set_open_drain();
        let sda = gpiof.pf15.into_alternate_af4().set_open_drain();
        dp.I2C4.i2c((scl, sda), 400.khz(), &ccdr)
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
        dp.SPI2.spi((sck, miso, mosi), ehal::spi::MODE_3, 10.mhz(), &ccdr)
    };
    // PF5 is SPI2 CS1 for FRAM
    let mut spi2_cs1 = gpiof.pf5.into_push_pull_output();
    spi2_cs1.set_high().unwrap();

    let spi4_port = {
        let sck = gpioe.pe2.into_alternate_af5();
        let miso = gpioe.pe13.into_alternate_af5();
        let mosi = gpioe.pe6.into_alternate_af5();
        dp.SPI4.spi((sck, miso, mosi), ehal::spi::MODE_3, 2.mhz(), &ccdr)
    };
    let mut spi4_cs1 = gpiof.pf10.into_push_pull_output();
    spi4_cs1.set_high().unwrap();

    //UART7 is debug (dronecode port): `(PF6, PE8)`
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

    (
        i2c3_port,
        i2c4_port,
        spi1_port,
        spi1_cs_tdk,
        (spi1_cs_bmi088_gyro, spi1_cs_bmi088_accel),
        spi2_port,
        spi2_cs1,
        spi4_port,
        spi4_cs1,
        uart7_port,
        user_led1,
        delay_source,
    )
}

fn local_println(po_tx: &mut (impl Write + ehal::serial::Write<u8>), args: Arguments<'_>) {
    let mut format_buf = ArrayString::<[u8; 64]>::new();
    format_buf.clear();
    if fmt::write(&mut format_buf, args).is_ok() {
        let le_str = format_buf.as_str();
        //write on console out
        let _ = po_tx.write_str(le_str);
        let _ = po_tx.flush();
    }
}

fn oled_print<DI: DisplayInterface>(disp: &mut GraphicsMode<DI>, y_pos: i32, args: Arguments<'_>) {
    let mut format_buf = ArrayString::<[u8; 16]>::new();
    format_buf.clear();
    if fmt::write(&mut format_buf, args).is_ok() {
        let le_str = format_buf.as_str();
        disp.draw(
            Font6x8::render_str(le_str)
                .with_stroke(Some(1u8.into()))
                .translate(Coord::new(20, y_pos))
                .into_iter(),
        );
        let _ = disp.flush();
    }
}

/// Render formatted text to an external oled screen
fn render_vec3<DI: DisplayInterface>(disp: &mut GraphicsMode<DI>, start_y: i32, _label: &str, buf: &[i16]) {
    const LINE_HEIGHT: i32 = 10;
    let mut y_pos = start_y;
    //TODO dynamically reformat depending on display size
    // oled_print(disp, y_pos, format_args!("{}", label)); y_pos += LINE_HEIGHT;
    oled_print(disp, y_pos, format_args!("X: {}", buf[0] )); y_pos += LINE_HEIGHT;
    oled_print(disp, y_pos, format_args!("Y: {}", buf[1] )); y_pos += LINE_HEIGHT;
    oled_print(disp, y_pos, format_args!("Z: {}", buf[2] ));
}


#[entry]
fn main() -> ! {
    let (
        i2c3_port,
        i2c4_port,
        spi1_port,
        spi1_cs_tdk,
        (spi1_cs_bmi088_gyro, spi1_cs_bmi088_accel),
        spi2_port,
        spi2_cs1,
        spi4_port,
        spi4_cs1,
        uart7_port,
        mut user_led1,
        mut delay_source,
    ) = setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let i2c_bus3 = shared_bus::CortexMBusManager::new(i2c3_port);
    let i2c_bus4 = shared_bus::CortexMBusManager::new(i2c4_port);

    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    let (mut po_tx, mut _po_rx) = uart7_port.split();
    local_println(&mut po_tx, format_args!("\r\n---BEGIN---\r\n"));

    let mut disp: GraphicsMode<_> = ssd1306::Builder::new()
        .with_size(DisplaySize::Display128x32)
        .connect_i2c(i2c_bus4.acquire())
        .into();
    disp.init().unwrap();
    disp.set_rotation(DisplayRotation::Rotate0).unwrap();
    disp.flush().unwrap();

    let mut mag = IST8310::default(i2c_bus3.acquire()).unwrap();
    let mut msbaro = Ms5611::new(spi4_port, spi4_cs1, &mut delay_source).unwrap();
    let mut flash = spi_memory::series25::Flash::init(spi2_port, spi2_cs1).unwrap();
    if let Ok(flash_id) = flash.read_jedec_id() {
        // we expect 0xC2, 0x22, 0x08 for Cypress FM25V02A identifier: 7F 7F 7F 7F 7F 7F C2 22 08
        if flash_id.mfr_code() != 0xC2 {
            local_println(&mut po_tx, format_args!("unexpected flash_id: {:?}\r\n", flash_id));
        }
    }

    let mut bmi088_a = bmi088::Builder::new_accel_spi(spi_bus1.acquire(), spi1_cs_bmi088_accel);
    if bmi088_a.setup(&mut delay_source).is_err() {
        local_println(&mut po_tx, format_args!("bmi088_a failed\r\n"));
    }

    let mut bmi088_g = bmi088::Builder::new_gyro_spi(spi_bus1.acquire(), spi1_cs_bmi088_gyro);
    if bmi088_g.setup(&mut delay_source).is_err() {
        local_println(&mut po_tx, format_args!("bmi088_g failed\r\n"));
    }

    // TODO troubleshoot icm20689  -- probe is consistently failing
    let mut tdk_6dof = icm20689::Builder::new_spi(spi_bus1.acquire(), spi1_cs_tdk);
    if tdk_6dof.setup(&mut delay_source).is_err() {
        local_println(&mut po_tx, format_args!("icm20689 failed\r\n"));
    }


   // bkpt();

    let _ = user_led1.set_low();
    let mut last_accel: [i16; 3] = [0; 3];
    let mut last_gyro: [i16; 3] = [0; 3];
    let mut last_mag: [i16; 3] = [0; 3];
    let mut last_press;
    loop {

        if let Ok(flash_status) = flash.read_status() {
            local_println(&mut po_tx, format_args!("flash_status: {:?}\r\n", flash_status));
        }

        // if let Ok(gyro_sample) = tdk_6dof.get_gyro() {
        //     last_gyro = gyro_sample;
        //     local_println(&mut po_tx, format_args!("gyro_i: {:?}\r\n", last_gyro));
        // }
        // if let Ok(accel_sample) = tdk_6dof.get_accel() {
        //     last_accel = accel_sample;
        //     local_println(&mut po_tx, format_args!("accel_i: {:?}\r\n", last_accel));
        // }
        if let Ok(gyro_sample) = bmi088_g.get_gyro() {
            last_gyro = gyro_sample;
            local_println(&mut po_tx, format_args!("gyro: {:?}\r\n", last_gyro));
        }
        if let Ok(accel_sample) = bmi088_a.get_accel() {
            last_accel = accel_sample;
            local_println(&mut po_tx, format_args!("accel: {:?}\r\n", last_accel));
            render_vec3(&mut disp, 2, "accel" , &last_accel);
        }
        if let Ok(mag_sample) = mag.get_mag_vector(&mut delay_source) {
            last_mag = mag_sample;
            local_println(&mut po_tx, format_args!("mag: {:?}\r\n", last_mag));
        }
        if let Ok(press_sample) =
            msbaro.get_second_order_sample(Oversampling::OS_2048, &mut delay_source)
        {
            last_press = press_sample.pressure;
            local_println(&mut po_tx, format_args!("press: {}\r\n", last_press));
        }

        let _ = user_led1.toggle();
        delay_source.delay_ms(1u8);
    }
}
