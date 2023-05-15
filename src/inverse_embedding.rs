use core::ops::Div;

use defmt::info;
use libm::powf;
use nalgebra::{SMatrix, SVector, SimdPartialOrd};

/// Maps from measured ADC values to abstract linear "scale" values
/// *For a divider with the LDR at the top of the divider (nearest-Vdd)*
///
/// The LDR resistance is modelled as R_ldr = 1/alpha * I + beta
/// where:
///     - *I* is the intensity of incident light
///     - *alpha* is a model parameter
///     - *beta* is a model parameter
///
/// This application uses "pseudo" alpha and betas, which map the expected
/// range of intesity onto a closed range from 0 to 1 and are recovered through
/// offline system identification.
///
/// Arguments:
/// - `paired_resistance`: The non-LDR resistance component of the bridge
/// - `alpha`: The constant inverse-linear coefficient relating intensity to
///            resistance
/// - `beta`: The constant offset from intensity to resistance
/// Returns:
/// - Fn(f32) -> f32
///     - Argument 0 is the measured voltage at the divider in fraction of
///       full-scale
pub const fn resistive_divider_inverse(
    paired_resistance: f32,
    alpha: f32,
    beta: f32,
) -> impl Fn(f32) -> f32 {
    move |val| alpha * ((1.0 - val) * paired_resistance - beta)
}

pub const fn linear_range_inverse(min: f32, max: f32) -> impl Fn(f32) -> f32 {
    move |val| val.clamp(min, max) / (max - min)
}

pub struct InverseEmbeddingTable<'a, const N: usize> {
    inverse_embeddings: [&'a dyn Fn(f32) -> f32; N],
}

impl<'a, const N: usize> InverseEmbeddingTable<'a, N> {
    pub fn new(mappings: [&'a dyn Fn(f32) -> f32; N]) -> Self {
        Self {
            inverse_embeddings: mappings,
        }
    }

    pub fn invert_embedding(&self, data: SMatrix<f32, N, 1>) -> SMatrix<f32, N, 1> {
        data.map_with_location(|row, _, val| self.inverse_embeddings[row](val))
    }
}

pub struct SensorInverseEmbedding<const N: usize> {
    resistance_ranges: [(f32, f32); N],
}

impl<const N: usize> SensorInverseEmbedding<N> {
    fn resistance_from_adc(adc_reading: f32) -> f32 {
        // Given Vadc = Vdd Rldr/(Rldr+Rfxd)
        // then Rldr = (Vdd/Vadc - 1) Rl
        return 1E5 * (1.0 / adc_reading - 1.0); // Fixed resistor is 100 kR
    }

    pub fn new(resistance_ranges: [(f32, f32); N]) -> Self {
        Self { resistance_ranges }
    }

    pub fn invert(&self, reading: SVector<u16, N>) -> SVector<f32, N> {
        // The ADC uses a 12 bit word, but we want to work with a float in [0, 1]
        let scaled_reading = reading.map(|e| e as f32 / powf(2.0, 12.0));

        let mut illuminations = SVector::<f32, N>::zeros();

        for i in 0..scaled_reading.len() {
            let (min, max) = self.resistance_ranges[i];

            info!("{}", Self::resistance_from_adc(scaled_reading[i]));

            // For now assume the LDR resistance is inversely proportional to the incident power
            let sample = if scaled_reading[i] > max || scaled_reading[i] < min {
                defmt::warn!("Clamped reading, was: {}", scaled_reading[i]);
                scaled_reading[i].clamp(min, max)
            } else {
                scaled_reading[i]
            };
            illuminations[i] = (max - Self::resistance_from_adc(sample)) / (max - min);
        }

        illuminations
    }
}
