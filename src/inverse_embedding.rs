use nalgebra::SMatrix;

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
pub const fn resistive_divider_inverse_embedder(
    paired_resistance: f32,
    alpha: f32,
    beta: f32,
) -> impl Fn(f32) -> f32 {
    move |val| alpha * ((1.0 - val) * paired_resistance - beta)
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

    pub fn invert_embedding(&self, data: SMatrix<f32, 1, N>) -> SMatrix<f32, 1, N> {
        data.map_with_location(|_, col, val| self.inverse_embeddings[col](val))
    }
}
