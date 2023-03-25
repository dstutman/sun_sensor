#![no_std]
#![no_main]

use core::f32::consts::PI;
use core::fmt::Write;
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
use libm::powf;
use lsm9ds1::CalibrationParameters;
use nalgebra::{SMatrix, Vector3};
use panic_probe as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

mod attitude;
mod inverse_embedding;
mod lsm9ds1;

use crate::lsm9ds1::CorrectedLsm9ds1;
use crate::{
    attitude::AttitudePipeline,
    inverse_embedding::{resistive_divider_inverse_embedding, InverseEmbeddingTable},
    lsm9ds1::Lsm9ds1,
};

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
    let mut lsm9ds1 = CorrectedLsm9ds1::new(
        Lsm9ds1::new(i2c, true, true).unwrap(),
        CalibrationParameters::new(
            Vector3::new(-0.0440121, 0.03978766, 0.01647095),
            Vector3::new(0.04544277, 0.03340622, 0.00087193),
            Vector3::new(0.23530056, -0.3293044, -0.49914486),
        ),
    );

    green_led.set_high().unwrap(); // Status OK
    defmt::info!("Entering run-loop...");

    let inverse_embedding = resistive_divider_inverse_embedding(6E3, 1.0, 0.0);
    let mut attitude_pipeline = AttitudePipeline::new_filled_hexagon(
        InverseEmbeddingTable::new([
            &inverse_embedding,
            &inverse_embedding,
            &inverse_embedding,
            &inverse_embedding,
            &inverse_embedding,
            &inverse_embedding,
            &inverse_embedding,
        ]),
        1.0,
    );

    let mut last_time = TIMEBASE.load(Ordering::SeqCst);
    let mut last_log_time = TIMEBASE.load(Ordering::SeqCst);

    loop {
        if TIMEBASE.load(Ordering::SeqCst) - last_time < 50 {
            continue;
        }
        last_time = TIMEBASE.load(Ordering::SeqCst);

        if TIMEBASE.load(Ordering::SeqCst) - last_log_time > 5000 {
            last_log_time = TIMEBASE.load(Ordering::SeqCst);
            yellow_led.toggle().unwrap(); // Running

            // Log current estimate
            defmt::info!(
                "\nCurrent estimate : \n\tazimuth : {}\n\televation : {}",
                attitude_pipeline.current_estimate.unwrap().azimuth * 180.0 / PI,
                attitude_pipeline.current_estimate.unwrap().elevation * 180.0 / PI,
            );
        }
        // Inner sensor first, then counter clockwise from marked sensor
        // In fractional-full-scale (thus divison by 2^12).
        let samples = SMatrix::<f32, 7, 1>::from([[
            adc2.read(&mut adc2_channel2).unwrap(), // A4
            adc2.read(&mut adc2_channel1).unwrap(), // A3
            adc2.read(&mut adc2_channel3).unwrap(), // A5
            adc1.read(&mut adc1_channel2).unwrap(), // A1
            adc1.read(&mut adc1_channel4).unwrap(), // A2
            adc1.read(&mut adc1_channel1).unwrap(), // A0
            adc2.read(&mut adc2_channel4).unwrap(), // A6
        ]]) / powf(2.0, 12.0);

        attitude_pipeline.update(samples);

        let acceleration = lsm9ds1.read_accel().unwrap();
        let angular_rates = lsm9ds1.read_gyro().unwrap();
        let magnetic_field = lsm9ds1.read_mag().unwrap();

        // Log sensor readings
        defmt::info!(
            "\nAccelerometer x : {}, y : {}, z : {}\nGyroscope x : {}, y : {}, z : {}\nMagnetometer x : {}, y : {}, z : {}",
            acceleration.x,
            acceleration.y,
            acceleration.z,
            angular_rates.x,
            angular_rates.y,
            angular_rates.z,
            magnetic_field.x,
            magnetic_field.y,
            magnetic_field.z
        );
        // Log the data for calibration
        write!(
            usart,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\n",
            samples[0],
            samples[1],
            samples[2],
            samples[3],
            samples[4],
            samples[5],
            samples[6],
            acceleration.x,
            acceleration.y,
            acceleration.z,
            angular_rates.x,
            angular_rates.y,
            angular_rates.z,
            magnetic_field.x,
            magnetic_field.y,
            magnetic_field.z,
        )
        .unwrap();
    }
}
