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
use nalgebra::{SMatrix, SVector};
use panic_probe as _;
use stm32f3xx_hal::{self as hal, pac, prelude::*};

mod attitude;
mod bno055;
mod fault_detector;
mod inverse_embedding;
mod ldr_array;

use crate::bno055::Bno055;
use crate::inverse_embedding::SensorInverseEmbedding;

#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

// In 1E-5 second ticks
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

    defmt::trace!("Configuring clocks and timebase...");

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
    cp.SYST.set_reload(80); // Tick a hundred times per millisecond
    cp.SYST.clear_current();
    cp.SYST.enable_interrupt();
    cp.SYST.enable_counter();

    defmt::trace!("Configuring peripherals...");
    // Acquire handles to GPIO banks
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb);
    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);

    // Setup LEDs
    let mut green_led = gpiob
        .pb3
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut yellow_led = gpiob
        .pb4
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    // Configure IO pins
    let mut adc1_channel1 = gpioa.pa0.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    //let mut adc1_channel11 = gpiob.pb0.into_analog(&mut gpiob.moder, &mut gpiob.pupdr);
    let mut adc1_channel2 = gpioa.pa1.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    //let mut adc1_channel12 = gpiob.pb1.into_analog(&mut gpiob.moder, &mut gpiob.pupdr);
    // PA2 reserved for VCP TX
    let mut adc1_channel4 = gpioa.pa3.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel1 = gpioa.pa4.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel2 = gpioa.pa5.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel3 = gpioa.pa6.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);
    let mut adc2_channel4 = gpioa.pa7.into_analog(&mut gpioa.moder, &mut gpioa.pupdr);

    // Setup ADC
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

    // Setup UART
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
        serial::config::Config::default().baudrate(2000000.Bd()),
        clocks,
        &mut rcc.apb1,
    );

    // Setup I2C
    // Pin configuration
    let scl = gpiob
        .pb6
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda = gpiob
        .pb7
        .into_af_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    // Peripheral init
    let i2c = I2c::new(dp.I2C1, (scl, sda), 400000.Hz(), clocks, &mut rcc.apb1);

    // Setup IMU
    let mut imu = Bno055::new(i2c).unwrap();

    // Initialization complete
    green_led.set_high().unwrap(); // Status OK
    defmt::info!("Entering run-loop...");

    let inverse_embedding = SensorInverseEmbedding::new([
        (5567.013095, 2.180124E5),
        (28886.092920, 1.063636E6),
        (12527.469362, 2.000733E5),
        (17162.474828, 2.705480E6),
        (5567.013095, 3.801876E6),
        (21255.180764, 1.948000E6),
    ]);

    let mut last_time = TIMEBASE.load(Ordering::SeqCst);
    let mut last_log_time = TIMEBASE.load(Ordering::SeqCst);

    loop {
        let dt = 1E-5 * (TIMEBASE.load(Ordering::SeqCst) - last_time) as f32;
        if dt < 1E0 {
            continue;
        }
        last_time = TIMEBASE.load(Ordering::SeqCst);

        // Inner sensor first, then counter clockwise from marked sensor
        // In fractional-full-scale (thus divison by 2^12).
        let reading = SMatrix::<u16, 6, 1>::from([[
            adc1.read(&mut adc1_channel1).unwrap(), // A0
            adc1.read(&mut adc1_channel2).unwrap(), // A1
            adc1.read(&mut adc1_channel4).unwrap(), // A2
            adc2.read(&mut adc2_channel1).unwrap(), // A3
            adc2.read(&mut adc2_channel2).unwrap(), // A4
            adc2.read(&mut adc2_channel3).unwrap(), // A5
        ]]);

        let faults = fault_detector::Faults::from_reading(reading);

        let sensor_illuminations = inverse_embedding.invert(reading);

        // Map the sensors to the faces
        let face_illuminations = SVector::<f32, 3>::new(
            (sensor_illuminations[(0, 0)] + sensor_illuminations[(1, 0)]) / 2.0,
            (sensor_illuminations[(4, 0)] + sensor_illuminations[(5, 0)]) / 2.0,
            (sensor_illuminations[(2, 0)] + sensor_illuminations[(3, 0)]) / 2.0,
        );

        let attitude = attitude::Attitude::from_illuminations(
            SMatrix::<f32, 3, 3>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            face_illuminations,
        );
    }
}
