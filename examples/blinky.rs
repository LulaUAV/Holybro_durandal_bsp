/*
Copyright (c) 2020 Todd Stellanova
LICENSE: BSD3 (see LICENSE file)
*/

#![no_std]
#![no_main]


use panic_rtt_core::{self, rprint, rprintln, rtt_init_print};


use cortex_m_rt::{entry, exception, ExceptionFrame};

use durandal_bsp::peripherals;

use ehal::blocking::delay::DelayMs;
use ehal::digital::v2::OutputPin;
use ehal::digital::v2::ToggleableOutputPin;
use embedded_hal as ehal;


#[entry]
fn main() -> ! {
    rtt_init_print!(NoBlockTrim);
    rprintln!("--> MAIN --");

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

    let _ = user_led1.set_low();

    loop {
        let _ = user_led1.toggle();
        rprint!(".");
        delay_source.delay_ms(250u8);
    }
}
