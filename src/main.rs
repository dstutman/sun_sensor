#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use defmt;
use defmt_rtt as _;
use hal::{
    adc::{self, Adc, CommonAdc},
    i2c::I2c,
    serial::{self, Serial},
};
use nalgebra::SMatrix;
use panic_probe as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

mod attitude;
mod inverse_embedding;
mod lsm9ds1;

use crate::{attitude::AttitudePipeline, lsm9ds1::Lsm9ds1};

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static TIMEBASE: AtomicU32 = AtomicU32::new(0); // NOTE: clients must handle counter overflow

#[exception]
fn SysTick() {
    TIMEBASE.fetch_add(1, Ordering::AcqRel);
}

#[entry]
fn main() -> ! {
    defmt::info!("Sun sensor starting...");

    let mut cp = pac::CorePeripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    defmt::debug!("Configuring peripherals...");

    // Configure clocks
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let clocks = rcc
        .cfgr
        .use_pll()
        .sysclk(64.MHz())
        .pclk1(32.MHz())
        .pclk2(64.MHz())
        .freeze(&mut flash.acr);

    // Set up a timebase
    cp.SYST.set_clock_source(SystClkSource::External); // RCC feeds SysTick with (ahb_clock = 64 MHz)/8 = 8 MHz
    cp.SYST.set_reload(8000); // Tick once per millisecond
    cp.SYST.clear_current();
    cp.SYST.enable_interrupt();
    cp.SYST.enable_counter();

    // Split GPIO bank A for the ADC and USART
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    // LED setup
    let mut green_led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut yellow_led = gpiob
        .pb4
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

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
    let adc_common = CommonAdc::new(dp.ADC1_2, &clocks, &mut rcc.ahb);
    let mut adc1 = Adc::new(
        dp.ADC1,
        adc::config::Config::default(),
        &clocks,
        &adc_common,
    )
    .into_oneshot();
    let mut adc2 = Adc::new(
        dp.ADC2,
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
        dp.USART2,
        usart_pins,
        serial::config::Config::default(),
        clocks,
        &mut rcc.apb1,
    );

    // LSM9DS1 setup
    // Pin configuration
    let scl = gpiob
        .pb6
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    // Peripheral init
    let i2c = I2c::new(dp.I2C1, (scl, sda), 400000.Hz(), clocks, &mut rcc.apb1);
    let mut lsm9ds1 = Lsm9ds1::new(i2c, true, false);
    if let Ok(true) = lsm9ds1.whoami_matches() {
        defmt::info!("LSM9DS1 presence check : PASS")
    } else {
        defmt::warn!("LSM9DS1 presence check : FAIL")
    }

    green_led.set_high().unwrap(); // Status OK
    defmt::info!("Entering run-loop...");

    let attitude_pipeline = AttitudePipeline::new_filled_hexagon(1.0);

    let mut last_time = TIMEBASE.load(Ordering::SeqCst);

    loop {
        if TIMEBASE.load(Ordering::SeqCst) - last_time < 1000 {
            continue;
        }
        last_time = TIMEBASE.load(Ordering::SeqCst);
        yellow_led.toggle().unwrap(); // Running

        // Inner sensor first, then counter clockwise from East-Northeast sensor
        let readings = SMatrix::<f32, 7, 1>::from([[
            adc1.read(&mut adc1_channel1).unwrap(),
            adc1.read(&mut adc1_channel2).unwrap(),
            adc1.read(&mut adc1_channel4).unwrap(),
            adc2.read(&mut adc2_channel1).unwrap(),
            adc2.read(&mut adc2_channel2).unwrap(),
            adc2.read(&mut adc2_channel3).unwrap(),
            adc2.read(&mut adc2_channel4).unwrap(),
        ]]);
        defmt::info!(
            "\nADC0 Reading: {}\nADC1 Reading: {}\nADC2 Reading: {}\nADC3 Reading: {}\nADC4 Reading: {}\nADC5 Reading: {}\nADC6 Reading: {}",
            readings[0],
            readings[1],
            readings[2],
            readings[3],
            readings[4],
            readings[5],
            readings[6],
        );
        // Compute and log the centroid of the incident intensity
        // attitude_pipeline.update(readings);
        // TODO: Implement outlier detection and sanity checking
        //write!(
        //    usart,
        //    "Intensity centroid:\n[{}\n {}]",
        //    centroid[(0, 0)],
        //    centroid[(1, 0)]
        //)
        //.unwrap();
    }
}
