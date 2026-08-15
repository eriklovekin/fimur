#![no_std]
#![no_main]

use nalgebra::{
    Vector3,
    Matrix3, 
    SMatrix, 
    stack,
};

const N_IMUS: usize = 12;
const THREE_N: usize = 3*N_IMUS;
const THREE_N_M3: usize = THREE_N-3;
const THREE_P_M3: usize = THREE_N+3;


/// Implementation of [Ref 8](https://hollydinkel.github.io/assets/pdf/AAS2025.pdf)
pub struct FusionCore {
    /// p: 3mx1 position vector of Sensor i in Filter frame
    p: SMatrix<f64,THREE_N,1>,
    
    /// N: 3mx3 matrix rotating Filter frame 
    /// to Sensor frame i
    n: SMatrix<f64,THREE_N,3>,

    /// N+: 3x3m Moore-Penrose inverse of N
    np: SMatrix<f64,3, THREE_N>,

    /// T: Eliminates Euler acceleration effects
    t: SMatrix<f64,3,THREE_N>,
}

impl FusionCore {
    pub fn new() -> Self {
        Self {
            p: SMatrix::zeros(),
            n: SMatrix::zeros(),
            np: SMatrix::zeros(),
            t: SMatrix::zeros(),
        }
    }

    pub fn fuse(&self, a: &SMatrix<f64,THREE_N,1>, w: &SMatrix<f64,THREE_N,1>) -> (SMatrix<f64,3,1>, SMatrix<f64,3,1>) {
        let wv = self.np * *a;
        let av = self.t * (*a - self.s(w));
        (av,wv)
    }

    /// Populate N, p, Np and T fields of struct
    /// n: 3n x 3 matrix stack of DCMs mapping filter frame to Sensor frame of ith sensor (f2s)
    /// p: 3n x 1 matrix stack of position of ith sensor in Filter frame
    pub fn init_geom(&mut self, n: &SMatrix<f64, THREE_N, 3>, p: &SMatrix<f64,THREE_N,1>) {
        self.np = self.compute_np(n);
        let y = self.compute_y(n,p);
        let zt = self.compute_zt(&y);
        debug_assert!((zt * y).norm() < 1e-9, "Z is not orthogonal to Y — null space extraction failed");
        self.t = self.compute_t(n,&zt);
    }

    fn compute_np(&self, n: &SMatrix<f64, THREE_N,3>) -> SMatrix<f64,3, THREE_N> {
        let svd = n.svd(true,true);
        let eps: f64 = 1e-7;
        svd.pseudo_inverse(eps).unwrap()
    }

    fn compute_y(&self,n: &SMatrix<f64, THREE_N,3>, p: &SMatrix<f64,THREE_N,1>) -> SMatrix<f64, THREE_N, 3> {
        let mut ret: SMatrix<f64, THREE_N,3> = SMatrix::zeros();

        for i in 0..N_IMUS {
            let ri: Matrix3<f64> = n.fixed_view::<3, 3>(i*3, 0).into_owned();
            let pi = p.fixed_view::<3, 1>(i*3, 0).into_owned();
            let product: Matrix3<f64> = ri*pi.cross_matrix();
            ret.fixed_view_mut::<3, 3>(i*3, 0).copy_from(&product);        
        }
        ret
    }

    fn compute_zt(&self, y: &SMatrix<f64, THREE_N, 3>) -> SMatrix<f64, THREE_N_M3,THREE_N>{
        let yloc = *y;
        
        let identity = SMatrix::<f64, THREE_N, THREE_N>::identity();
        let augmented: SMatrix<f64,THREE_N,THREE_P_M3> = stack!(yloc,identity);

        let qr = augmented.qr();
        let q_full = qr.q();

        let z = q_full.fixed_columns::<THREE_N_M3>(3).into_owned(); // SMatrix<f64, THREE_N, {THREE_N-3}>
        z.transpose()
    }

    fn compute_t(&self, n: &SMatrix<f64, THREE_N,3>, zt: &SMatrix<f64, THREE_N_M3, THREE_N>) -> SMatrix<f64, 3, THREE_N>{
        let prod = zt*n;
        let svd = prod.svd(true,true);
        let eps: f64 = 1e-7;
        let pseudo_inv = svd.pseudo_inverse(eps).unwrap();
        pseudo_inv*zt
    }

    pub fn s(&self,x: &SMatrix<f64,THREE_N,1>) -> SMatrix<f64,THREE_N,1> {
        let mut ret: SMatrix<f64,THREE_N,1> = SMatrix::zeros();
        for i in 0..N_IMUS {
            let xi = x.fixed_view::<3, 1>(i*3, 0).into_owned();
            let ni = self.n.fixed_view::<3, 3>(i*3, 0).into_owned();
            let pi = self.p.fixed_view::<3, 1>(i*3, 0).into_owned();
            let product:SMatrix<f64,3,1> = 
                ni *
                xi.cross_matrix() *
                xi.cross_matrix() *
                pi;
            ret.fixed_view_mut::<3, 1>(i*3, 0).copy_from(&product);        
        }
        ret
    }
}