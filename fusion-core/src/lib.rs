#![no_std]
#![no_main]

use nalgebra::{
    Matrix3, 
    SMatrix, 
    stack,
};

const N_IMUS: usize = 12;
pub const THREE_N: usize = 3*N_IMUS;
pub const THREE_N_M3: usize = THREE_N-3;
pub const THREE_P_M3: usize = THREE_N+3;


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

    /// Create new and populate N, p, Np and T fields
    /// n: 3n x 3 matrix stack of DCMs mapping filter frame to Sensor frame of ith sensor (f2s)
    /// p: 3n x 1 matrix stack of position of ith sensor in Filter frame
    pub fn init_from_geom(n: &SMatrix<f64, THREE_N, 3>, p: &SMatrix<f64,THREE_N,1>) -> Result<Self,&'static str>{
        let mut core = FusionCore {
            n: *n,
            p: *p,
            np: SMatrix::zeros(),
            t: SMatrix::zeros(),
        };
        
        let y = core.compute_y(n,p);
        let zt = core.compute_zt(&y);
        assert!((zt * y).norm() < 1e-9, "Z is not orthogonal to Y — null space extraction failed");
        core.np = core.compute_np(n)?;
        core.t = core.compute_t(n,&zt)?;
        Ok(core)
    }

    pub fn fuse(&self, a: &SMatrix<f64,THREE_N,1>, w: &SMatrix<f64,THREE_N,1>) -> (SMatrix<f64,3,1>, SMatrix<f64,3,1>) {
        let wv = self.np * *w;
        let av = self.t * (*a - self.s(&wv));
        (av,wv)
    }

    fn compute_np(&self, n: &SMatrix<f64, THREE_N,3>) -> Result<SMatrix<f64,3, THREE_N>, &'static str> {
        let svd = n.svd(true,true);
        let eps: f64 = 1e-7;
        Ok(svd.pseudo_inverse(eps)?)
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

    fn compute_t(&self, n: &SMatrix<f64, THREE_N,3>, zt: &SMatrix<f64, THREE_N_M3, THREE_N>) -> Result<SMatrix<f64, 3, THREE_N>,&'static str>{
        let prod = zt*n;
        let svd = prod.svd(true,true);
        let eps: f64 = 1e-7;
        let pseudo_inv = svd.pseudo_inverse(eps)?;
        Ok(pseudo_inv*zt)
    }

    fn s(&self,x: &SMatrix<f64,3,1>) -> SMatrix<f64,THREE_N,1> {
        let mut ret: SMatrix<f64,THREE_N,1> = SMatrix::zeros();
        for i in 0..N_IMUS {
            let ni = self.n.fixed_view::<3, 3>(i*3, 0).into_owned();
            let pi = self.p.fixed_view::<3, 1>(i*3, 0).into_owned();
            let product:SMatrix<f64,3,1> = 
                ni *
                x.cross_matrix() *
                x.cross_matrix() *
                pi;
            ret.fixed_view_mut::<3, 1>(i*3, 0).copy_from(&product);        
        }
        ret
    }
}