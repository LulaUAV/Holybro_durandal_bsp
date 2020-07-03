/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]
#![no_main]


use panic_rtt_core::{self, rprintln, rtt_init_print};


use cortex_m_rt::{entry, exception, ExceptionFrame};

use durandal_bsp::peripherals;

use ehal::blocking::delay::DelayMs;
use ehal::digital::v2::OutputPin;
use ehal::digital::v2::ToggleableOutputPin;
use embedded_hal as ehal;

// SSD1306 external OLED display (for debug)
use embedded_graphics::fonts::Font6x8;
use embedded_graphics::prelude::*;
use ssd1306::interface::DisplayInterface;
use ssd1306::prelude::*;

use arrayvec::ArrayString;
use core::fmt;
use core::fmt::{Arguments, Write};

// use p_hal::time::U32Ext;

/// Sensors
use ist8310::IST8310;
use ms5611::{Ms5611, Oversampling};
use ms5611_spi as ms5611;

// use ncp5623c::NCP5623C;
use tca62724fmg::TCA62724FMG;



// use crate::port_types::{HalGpioError, HalI2cError, HalSpiError, DbgUartPortType, Gps1PortType};
//
// use p_hal::pwr::VoltageScale;
// use p_hal::rcc::PllConfigStrategy;
// use p_hal::serial::config::{Parity, StopBits, WordLength};
// use spi_memory::series25::Identification;

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

fn console_print(po_tx: &mut (impl Write + ehal::serial::Write<u8>), args: Arguments<'_>) {
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
fn render_vec3<DI: DisplayInterface>(
    disp: &mut GraphicsMode<DI>,
    start_y: i32,
    _label: &str,
    buf: &[i16],
) {
    const LINE_HEIGHT: i32 = 10;
    let mut y_pos = start_y;
    //TODO dynamically reformat depending on display size
    // oled_print(disp, y_pos, format_args!("{}", label)); y_pos += LINE_HEIGHT;
    oled_print(disp, y_pos, format_args!("X: {}", buf[0]));
    y_pos += LINE_HEIGHT;
    oled_print(disp, y_pos, format_args!("Y: {}", buf[1]));
    y_pos += LINE_HEIGHT;
    oled_print(disp, y_pos, format_args!("Z: {}", buf[2]));
}

#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("-- > MAIN --");

    let (
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
        mut user_led1,
        mut delay_source,
    ) = peripherals::setup_peripherals();

    let spi_bus1 = shared_bus::CortexMBusManager::new(spi1_port);
    let i2c_bus1 = shared_bus::CortexMBusManager::new(i2c1_port);
    let _i2c_bus2 = shared_bus::CortexMBusManager::new(i2c2_port);

    let i2c_bus3 = shared_bus::CortexMBusManager::new(i2c3_port);
    let i2c_bus4 = shared_bus::CortexMBusManager::new(i2c4_port);

    // wait a bit for sensors to power up
    delay_source.delay_ms(250u8);

    let (mut po_tx, mut _po_rx) = uart7_port.split();
    console_print(&mut po_tx, format_args!("\r\n---BEGIN---\r\n"));

    let mut rgbled = TCA62724FMG::default(i2c_bus1.acquire()).unwrap();
    let _ = rgbled.set_color_brightness(0x01, 0, 0x01);
    rgbled.set_enabled(true).unwrap();
    for _ in 0..5 {
        let _ = rgbled.toggle();
        delay_source.delay_ms(100u8);
    }
    let _ = rgbled.set_color_brightness(0, 0x01, 0);
    let _ = rgbled.set_enabled(true);

    let mut mag_ext = IST8310::default(i2c_bus1.acquire()).unwrap();
    if let Ok(mag_sample) = mag_ext.get_mag_vector(&mut delay_source) {
        console_print(&mut po_tx, format_args!("mag_ext: {:?}\r\n", mag_sample));
    }

    let _ = user_led1.set_low();

    let mut ublox = ublox_core::new_serial_driver(gps1_port);
    ublox.setup(&mut delay_source).unwrap();

    let mut disp: GraphicsMode<_> = ssd1306::Builder::new()
        .with_size(DisplaySize::Display128x32)
        .connect_i2c(i2c_bus4.acquire())
        .into();
    let have_ext_display = disp.init().is_ok();
    if have_ext_display {
        disp.set_rotation(DisplayRotation::Rotate0).unwrap();
        disp.flush().unwrap();
        oled_print(&mut disp, 10, format_args!("hello"));
    }

    let mut mag_int = IST8310::default(i2c_bus3.acquire()).unwrap();
    let mut msbaro = Ms5611::new(spi4_port, spi4_cs1, &mut delay_source).unwrap();
    let mut flash = spi_memory::series25::Flash::init(spi2_port, spi2_cs1).unwrap();
    if let Ok(flash_id) = flash.read_jedec_id() {
        // we expect 0xC2, 0x22, 0x08 for Cypress FM25V02A identifier: 7F 7F 7F 7F 7F 7F C2 22 08
        if flash_id.mfr_code() != 0xC2 {
            console_print(
                &mut po_tx,
                format_args!("unexpected flash_id: {:?}\r\n", flash_id),
            );
        }
    }

    let mut bmi088_a = bmi088::Builder::new_accel_spi(spi_bus1.acquire(), spi1_cs_bmi088_accel);
    if bmi088_a.setup(&mut delay_source).is_err() {
        console_print(&mut po_tx, format_args!("bmi088_a failed\r\n"));
    }

    let mut bmi088_g = bmi088::Builder::new_gyro_spi(spi_bus1.acquire(), spi1_cs_bmi088_gyro);
    if bmi088_g.setup(&mut delay_source).is_err() {
        console_print(&mut po_tx, format_args!("bmi088_g failed\r\n"));
    }

    // TODO troubleshoot icm20689  -- probe is consistently failing
    let mut tdk_6dof = icm20689::Builder::new_spi(spi_bus1.acquire(), spi1_cs_tdk);
    if tdk_6dof.setup(&mut delay_source).is_err() {
        console_print(&mut po_tx, format_args!("icm20689 failed\r\n"));
    }

    let _ = user_led1.set_low();
    let mut last_accel: [i16; 3] = [0; 3];
    let mut last_gyro: [i16; 3] = [0; 3];
    let mut last_mag: [i16; 3] = [0; 3];
    let mut last_press;
    loop {
        // if let Ok(flash_status) = flash.read_status() {
        //     local_println(
        //         &mut po_tx,
        //         format_args!("flash_status: {:?}\r\n", flash_status),
        //     );
        // }

        if let Ok(gyro_sample) = tdk_6dof.get_gyro() {
            last_gyro = gyro_sample;
            console_print(&mut po_tx, format_args!("gyro_i: {:?}\r\n", last_gyro));
        }
        if let Ok(accel_sample) = tdk_6dof.get_accel() {
            last_accel = accel_sample;
            console_print(&mut po_tx, format_args!("accel_i: {:?}\r\n", last_accel));
        }
        if let Ok(gyro_sample) = bmi088_g.get_gyro() {
            last_gyro = gyro_sample;
            console_print(&mut po_tx, format_args!("gyro: {:?}\r\n", last_gyro));
        }
        if let Ok(accel_sample) = bmi088_a.get_accel() {
            last_accel = accel_sample;
            console_print(&mut po_tx, format_args!("accel: {:?}\r\n", last_accel));
            if have_ext_display {
                render_vec3(&mut disp, 2, "accel", &last_accel);
            }
        }
        if let Ok(mag_sample) = mag_int.get_mag_vector(&mut delay_source) {
            last_mag = mag_sample;
            console_print(&mut po_tx, format_args!("mag_int: {:?}\r\n", mag_sample));
        }
        if let Ok(mag_sample) = mag_ext.get_mag_vector(&mut delay_source) {
            console_print(&mut po_tx, format_args!("mag_ext: {:?}\r\n", mag_sample));
        }
        if let Ok(press_sample) =
            msbaro.get_second_order_sample(Oversampling::OS_2048, &mut delay_source)
        {
            last_press = press_sample.pressure;
            console_print(&mut po_tx, format_args!("press: {}\r\n", last_press));
        }

        // // check GNSS
        // if let Ok(msg_count) = ublox.handle_one_message() {
        //     //console_print(&mut po_tx, format_args!(">>> msg_count: {} \r\n", msg_count));
        //     if msg_count > 0 {
        //         if let Some(nav_pvt) = ublox.take_last_nav_pvt() {
        //             console_print(
        //                 &mut po_tx,
        //                 format_args!(
        //                     ">>> nav_pvt lat, lon: {}, {} \r\n",
        //                     nav_pvt.lat,
        //                     nav_pvt.lon,
        //                 ),
        //             );
        //         }
        //         if let Some(nav_dop) = ublox.take_last_nav_dop() {
        //             console_print(
        //                 &mut po_tx,
        //                 format_args!(">>> nav_dop {} \r\n", nav_dop.itow),
        //             );
        //         }
        //         if let Some(mon_hw) = ublox.take_last_mon_hw() {
        //             console_print(
        //                 &mut po_tx,
        //                 format_args!(">>> mon_hw jam: {} \r\n", mon_hw.jam_ind),
        //             );
        //         }
        //     }
        // }

        let _ = user_led1.toggle();
        delay_source.delay_ms(1u8);
    }
}
