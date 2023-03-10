#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use defmt::{debug, error, info, trace, warn};
use defmt_rtt as _;
use hal::{
    adc::{self, Adc, CommonAdc},
    serial::{self, Serial},
};
use libm::sqrtf;
use panic_halt as _; // you can put a breakpoint on `rust_begin_unwind` to catch panics
use stm32f3xx_hal::{self as hal, pac, prelude::*};

mod linalg;
use linalg::Matrix;

static TIMEBASE: AtomicU32 = AtomicU32::new(0);

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
    let mut yellow_led = gpiob
        .pb4
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut green_led = gpiob
        .pb5
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

    green_led.set_high().unwrap(); // Status OK
    defmt::info!("Entering run-loop...");

    let mut last_time = TIMEBASE.load(Ordering::SeqCst);

    loop {
        if TIMEBASE.load(Ordering::SeqCst) - last_time < 1000 {
            continue;
        }
        last_time = TIMEBASE.load(Ordering::SeqCst);
        yellow_led.toggle().unwrap();
        // Inner sensor first, then counter clockwise from East-Northeast sensor
        let sensor_vectors = Matrix::from([
            [0.0, 0.0],
            [sqrtf(3.0) / 2.0, 1.0 / 2.0],
            [0.0, 1.0],
            [-sqrtf(3.0) / 2.0, 1.0 / 2.0],
            [-sqrtf(3.0) / 2.0, -1.0 / 2.0],
            [0.0, -1.0],
            [sqrtf(3.0) / 2.0, -1.0 / 2.0],
        ]);
        let sensor_readings: Matrix<f32, 7, 1> = Matrix::from([[
            adc1.read(&mut adc1_channel1).unwrap(),
            adc1.read(&mut adc1_channel2).unwrap(),
            adc1.read(&mut adc1_channel4).unwrap(),
            adc2.read(&mut adc2_channel1).unwrap(),
            adc2.read(&mut adc2_channel2).unwrap(),
            adc2.read(&mut adc2_channel3).unwrap(),
            adc2.read(&mut adc2_channel4).unwrap(),
        ]]);
        // Compute and log the centroid of the incident intensity
        let centroid = sensor_vectors * sensor_readings;
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
