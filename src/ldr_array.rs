use nalgebra::SMatrix;

pub type Reading<const N: usize> = SMatrix<u16, N, 1>;
