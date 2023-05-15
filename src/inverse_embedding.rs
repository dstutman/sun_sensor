use libm::powf;
use nalgebra::SVector;

pub struct SensorInverseEmbedding<const N: usize> {
    resistance_ranges: [(f32, f32); N],
}

impl<const N: usize> SensorInverseEmbedding<N> {
    fn resistance_from_adc(adc_reading: f32) -> f32 {
        // Given Vadc = Vdd Rldr/(Rldr+Rfxd)
        // then Rldr = (Vdd/Vadc - 1) Rl
        1E5 * (1.0 / adc_reading - 1.0) // Fixed resistor is 100 kR
    }

    pub fn new(resistance_ranges: [(f32, f32); N]) -> Self {
        Self { resistance_ranges }
    }

    pub fn invert(&self, reading: SVector<u16, N>) -> SVector<f32, N> {
        // The ADC uses a 12 bit word, but we want to work with a float in [0, 1]
        let scaled_reading = reading.map(|e| e as f32 / powf(2.0, 12.0));

        let mut illuminations = SVector::<f32, N>::zeros();

        let resistances = scaled_reading.map(|e| Self::resistance_from_adc(e));

        for i in 0..resistances.len() {
            let (min, max) = self.resistance_ranges[i];

            // For now assume the LDR resistance is inversely proportional to the incident power
            let resistance = if resistances[i] > max || resistances[i] < min {
                defmt::warn!("Clamped reading, was: {}", resistances[i]);
                resistances[i].clamp(min, max)
            } else {
                resistances[i]
            };
            illuminations[i] = (max - resistance) / (max - min);
        }

        illuminations
    }
}
