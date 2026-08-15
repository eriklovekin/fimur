use pyo3::prelude::*;

/// A Python module implemented in Rust.
#[pymodule]
mod fusion_py {
    use pyo3::prelude::*;

    #[pyfunction]
    fn hello_world() {
        println!("hello world!");
    }

    // Given relative positions of all IMUs in array, compute constant matrices

    // Given constant matrices and most recent sample, compute fused measurement

    // Operator to remove effects of centripetal acceleration
}
