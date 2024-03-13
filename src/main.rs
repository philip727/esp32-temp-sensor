#![no_std]
#![no_main]

pub mod aht10;

use aht10::AHT10;
use esp32_hal::{
    clock::ClockControl,
    i2c::{self, I2C},
    peripherals::Peripherals,
    prelude::*,
    Delay, IO,
};
use esp_backtrace as _;
use esp_println::println;

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();

    let clocks = ClockControl::max(system.clock_control).freeze();
    let mut delay = Delay::new(&clocks);

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let sda = io.pins.gpio21;
    let scl = io.pins.gpio22;

    let i2c = I2C::new(peripherals.I2C0, sda, scl, 100u32.kHz(), &clocks);
    let aht10 = AHT10::new(i2c, delay);

    if let Err(ref e) = aht10 {
        println!("Error whilst running AHT10 driver: {:?}", e);
        restart_msg(&mut delay);
    }

    let mut aht10 = aht10.unwrap();

    loop {
        delay.delay_ms(500u32);

        let humidity = aht10.get_humidity();
        match humidity {
            Ok(humidity) => {
                println!("Humidity: {}", humidity.relative());
            }
            Err(e) => {
                println!("Error whilst getting AHT10 humidity: {:?}", e);
            }
        }

        let temperature = aht10.get_temperature();
        match temperature {
            Ok(temperature) => {
                println!("Temperature: {}", temperature.celsius());
            }
            Err(e) => {
                println!("Error whilst getting AHT10 temperature: {:?}", e);
            }
        }
    }
}

fn restart_msg(delay: &mut Delay) {
    loop {
        println!("Please restart and ensure all modules are connected");
        delay.delay_ms(5000u32);
    }
}
