use nalgebra::SMatrix;

/// Maps from measured ADC values to abstract linear "scale" values
///
/// - `paired_resistance`: The non-LDR resistance component of the bridge
/// - `alpha`: The constant inverse-linear coefficient relating intensity to
///            resistance
/// - `beta`: The constant offset from intensity to resistance
pub const fn resistive_divider_inverse_embedder(
    paired_resistance: f32,
    alpha: f32,
    beta: f32,
) -> impl Fn(f32) -> f32 {
    |val| unimplemented!()
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
