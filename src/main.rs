#![no_std]
#![no_main]

use core::fmt::Write;

use hal::{
    adc::{self, Adc, CommonAdc},
    serial::{self, Serial},
};
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics

use cortex_m_rt::entry;

use stm32f3xx_hal::{self as hal, pac, prelude::*};

#[entry]
fn main() -> ! {
    let periphs = pac::Peripherals::take().unwrap();

    let mut rcc = periphs.RCC.constrain();
    let mut flash = periphs.FLASH.constrain();

    // Configure clocks
    let clocks = rcc
        .cfgr
        .use_pll()
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .pclk2(64.MHz())
        .freeze(&mut flash.acr);

    // Split GPIO bank A for the ADC and USART
    let mut gpioa = periphs.GPIOA.split(&mut rcc.ahb);

    // ADC setup
    // Pin configuration
    // TODO: Clean this up
    let mut adc1_channel1 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc1_channel2 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    // PA2 reserved for VCP TX
    let mut adc1_channel4 = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel1 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel2 = gpioa.pa5.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel3 = gpioa.pa6.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel4 = gpioa.pa7.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // Peripheral init
    let adc_common = CommonAdc::new(periphs.ADC1_2, &clocks, &mut rcc.ahb);
    let mut adc1 = Adc::new(
        periphs.ADC1,
        adc::config::Config::default(),
        &clocks,
        &adc_common,
    )
    .into_oneshot();
    let mut adc2 = Adc::new(
        periphs.ADC2,
        adc::config::Config::default(),
        &clocks,
        &adc_common,
    )
    .into_oneshot();

    // Log ADC values
    let usart_pins = (
        gpioa
            .pa2
            .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl),
        gpioa
            .pa15
            .into_af_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh),
    );

    let mut usart = Serial::new(
        periphs.USART2,
        usart_pins,
        serial::config::Config::default(),
        clocks,
        &mut rcc.apb1,
    );
    usart.write_str("Test write").unwrap();

    loop {
        let adc1_channel1_counts: u16 = adc1.read(&mut adc1_channel1).unwrap();
        let adc1_channel2_counts: u16 = adc1.read(&mut adc1_channel2).unwrap();
        let adc1_channel4_counts: u16 = adc1.read(&mut adc1_channel4).unwrap();
        let adc2_channel1_counts: u16 = adc2.read(&mut adc2_channel1).unwrap();
        let adc2_channel2_counts: u16 = adc2.read(&mut adc2_channel2).unwrap();
        let adc2_channel3_counts: u16 = adc2.read(&mut adc2_channel3).unwrap();
        let adc2_channel4_counts: u16 = adc2.read(&mut adc2_channel4).unwrap();
        write!(
            usart,
            "Reporting counts\na1c1: {}\na1c2: {}\na1c4: {}\na2c1: {}\na2c2: {}\na2c3: {}\na2c4: {}\n",
            adc1_channel1_counts,
            adc1_channel2_counts,
            adc1_channel4_counts,
            adc2_channel1_counts,
            adc2_channel2_counts,
            adc2_channel3_counts,
            adc2_channel4_counts,
        )
        .unwrap();
    }
}
