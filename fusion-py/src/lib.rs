use pyo3::prelude::*;

#[pymodule]
mod fusion_py {
    use pyo3::prelude::*;
    use fusion_core::{
        FusionCore,
        THREE_N,
    };
    use numpy::{
        PyReadonlyArray2,
        PyArray2,
        IntoPyArray,
    };
    use ndarray::{
        Array2,
    };
    use pyo3::exceptions::PyValueError;
    use nalgebra::{
        SMatrix,
    };

    #[pyfunction]
    fn hello_world() {
        println!("hello world!");
    }

    #[pyclass]
    pub struct PyFusionCore {
        inner: FusionCore,
    }

    #[pymethods]
    impl PyFusionCore {
        // Defines the Python constructor: __init__
        #[new]
        fn init_from_geom(n: PyReadonlyArray2<f64>, p: PyReadonlyArray2<f64>) -> PyResult<Self> {
            let nr: SMatrix<f64, THREE_N, 3> = to_smatrix::<THREE_N, 3>(&n)?;
            let pr: SMatrix<f64, THREE_N, 1> = to_smatrix::<THREE_N, 1>(&p)?;

            FusionCore::init_from_geom(&nr, &pr)
                .map(|inner| PyFusionCore { inner })
                .map_err(PyValueError::new_err)
        }

        fn fuse(&self, py: Python<'_>, a: PyReadonlyArray2<f64>, w: PyReadonlyArray2<f64>) -> PyResult<(Py<PyArray2<f64>>, Py<PyArray2<f64>>)> {
            let ar: SMatrix<f64,THREE_N,1> = to_smatrix::<THREE_N, 1>(&a)?;
            let wr: SMatrix<f64,THREE_N,1> = to_smatrix::<THREE_N, 1>(&w)?;
            let (avr,wvr) = self.inner.fuse(&ar,&wr);
            let av = to_py_array2::<3,1>(py,&avr)?;
            let wv = to_py_array2::<3,1>(py,&wvr)?;
            Ok((av,wv))
        }

    }

    fn to_smatrix<const R: usize, const C: usize>(
        arr: &PyReadonlyArray2<f64>,
    ) -> PyResult<SMatrix<f64, R, C>> {
        let view = arr.as_array();
        if view.shape() != [R, C] {
            return Err(PyValueError::new_err(format!(
                "expected shape [{}, {}], got {:?}",
                R, C, view.shape()
            )));
        }

        Ok(SMatrix::<f64, R, C>::from_row_iterator(view.iter().cloned()))
    }

    fn to_py_array2<const R: usize, const C: usize>(
        py: Python<'_>,
        mat: &SMatrix<f64, R, C>,
    ) -> PyResult<Py<PyArray2<f64>>> {
        let arr = Array2::from_shape_fn((R, C), |(r, c)| mat[(r, c)]);
        Ok(arr.into_pyarray(py).into())
    }
}
