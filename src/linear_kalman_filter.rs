use nalgebra::{DMatrix, DVector};
use std::mem::drop;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

pub use crate::linear_kalman_filter_c_bindings;

type DynTsMatrix = Arc<Mutex<DMatrix<f64>>>;
type DynTsVector = Arc<Mutex<DVector<f64>>>;
// DynTs = Dynamic Threadsafe

pub struct LinearKalmanFilter {
    state_vector_length: usize,
    num_inputs: usize,
    num_measurements: usize,

    state_vector: DynTsVector,
    state_covariance: DynTsMatrix,

    transition_matrix: DynTsMatrix, // the F matrix
    input_matrix: DynTsMatrix, // the G matrix
    process_noise_matrix: DynTsMatrix, // Q matrix

    measurement_matrix: DynTsMatrix, // the H matrix
    measurement_noise_matrix: DynTsMatrix, // R matrix

    update_rate: u64,
}

impl LinearKalmanFilter {
    pub fn new(
        mut update_rate: u64,
        state_vector_length: usize,
        num_inputs: usize,
        num_measurements: usize,
    ) -> Self {
        if update_rate == 0 {
            update_rate = 1;
        }
        Self {
            // initialize all matrices to zero
            state_vector_length,
            num_inputs,
            num_measurements,

            state_vector: Arc::new(Mutex::new(DVector::<f64>::zeros(state_vector_length))),
            state_covariance: Arc::new(Mutex::new(DMatrix::<f64>::zeros(
                state_vector_length,
                state_vector_length,
            ))),

            transition_matrix: Arc::new(Mutex::new(DMatrix::<f64>::zeros(
                state_vector_length,
                state_vector_length,
            ))),
            input_matrix: Arc::new(Mutex::new(DMatrix::<f64>::zeros(
                state_vector_length,
                num_inputs,
            ))),
            process_noise_matrix: Arc::new(Mutex::new(DMatrix::<f64>::zeros(
                state_vector_length,
                state_vector_length,
            ))),

            measurement_matrix: Arc::new(Mutex::new(DMatrix::<f64>::zeros(
                num_measurements,
                state_vector_length,
            ))),
            measurement_noise_matrix: Arc::new(Mutex::new(DMatrix::<f64>::zeros(
                num_measurements,
                num_measurements,
            ))),

            update_rate,
        }
    }

    pub fn set_transition_matrix(&self, new_transition_matrix: DMatrix<f64>) -> Result<(), &str> {
        if new_transition_matrix.shape() == (self.state_vector_length, self.state_vector_length) {
            *(self.transition_matrix.lock().unwrap()) = new_transition_matrix;
            Ok(())
        } else {
            Err("Entered matrix doesn't match the required dimensions to be a transition matrix(F) for the system")
        }
    }

    pub fn set_input_matrix(&self, new_input_matrix: DMatrix<f64>) -> Result<(), &str> {
        if new_input_matrix.shape() == (self.state_vector_length, self.num_inputs) {
            *(self.input_matrix.lock().unwrap()) = new_input_matrix;
            Ok(())
        } else {
            Err("Entered matrix doesn't match the required dimensions to be an input matrix(G) for the system")
        }
    }

    pub fn set_measurement_matrix(&self, new_measurement_matrix: DMatrix<f64>) -> Result<(), &str> {
        if new_measurement_matrix.shape() == (self.num_measurements, self.state_vector_length) {
            *(self.measurement_matrix.lock().unwrap()) = new_measurement_matrix;
            Ok(())
        } else {
            Err("Entered matrix doesn't match the required dimensions to be a measurement matrix(H) for the system")
        }
    }

    pub fn set_state(&self, new_state: DVector<f64>) -> Result<(), &str> {
        if new_state.shape() == (self.state_vector_length, 1) {
            *(self.state_vector.lock().unwrap()) = new_state;
            Ok(())
        } else {
            Err("Entered vector doesn't match the required dimensions to be a state vector for the system")
        }
    }

    pub fn set_process_noise_matrix(&self, new_process_noise: DMatrix<f64>) -> Result<(), &str> {
        if new_process_noise.shape() == (self.state_vector_length, self.state_vector_length) {
            *(self.process_noise_matrix.lock().unwrap()) = new_process_noise;
            Ok(())
        } else {
            Err("Entered matrix doesn't match the required dimensions to be a process noise covariance matrix(Q) for the system")
        }
    }

    pub fn set_measurement_noise_matrix(
        &self,
        new_measurement_noise_matrix: DMatrix<f64>,
    ) -> Result<(), &str> {
        if new_measurement_noise_matrix.shape() == (self.num_measurements, self.num_measurements) {
            *(self.measurement_noise_matrix.lock().unwrap()) = new_measurement_noise_matrix;
            Ok(())
        } else {
            Err("Entered matrix doesn't match the required dimensions to be a measurement noise matrix(R) for the system")
        }
    }

    pub fn initialize_filter(
        &self,
        transition_matrix: DMatrix<f64>,
        input_matrix: DMatrix<f64>,
        measurement_matrix: DMatrix<f64>,
        initial_state: DVector<f64>,
        process_noise_matrix: DMatrix<f64>,
        measurement_noise_matrix: DMatrix<f64>,
    ) -> Result<(), &str> {
        self.set_transition_matrix(transition_matrix).unwrap();
        self.set_input_matrix(input_matrix).unwrap();
        self.set_measurement_matrix(measurement_matrix).unwrap();
        self.set_state(initial_state).unwrap();
        self.set_process_noise_matrix(process_noise_matrix).unwrap();
        self.set_measurement_noise_matrix(measurement_noise_matrix)
            .unwrap();
        Ok(())
    }

    pub fn predict(&self, input: Option<DMatrix<f64>>) {
        let x_mut: &mut DVector<f64> = &mut (self.state_vector.lock().unwrap());
        let p_mut: &mut DMatrix<f64> = &mut (self.state_covariance.lock().unwrap());
        let f_lock: &DMatrix<f64> = &(self.transition_matrix.lock().unwrap());
        let q_lock: &DMatrix<f64> = &(self.process_noise_matrix.lock().unwrap());

        *x_mut = f_lock * &(*x_mut);

        // if input is supplied
        if let Some(input) = input {
            let input_matrix_lock = &*(self.input_matrix.lock().unwrap());
            *x_mut += input_matrix_lock * input;
        }

        *p_mut = (f_lock * &(*p_mut) * f_lock.transpose()) + q_lock;

        // debug
        println!("{:?}", &x_mut);
    }

    pub fn measure(&self, measurement: DMatrix<f64>) -> Result<(), &str> {
        if measurement.shape() == (self.num_measurements, self.state_vector_length) {
            let h_lock: &DMatrix<f64> = &(self.measurement_matrix.lock().unwrap());
            let r_lock: &DMatrix<f64> = &(self.measurement_noise_matrix.lock().unwrap());
            let x_mut: &mut DVector<f64> = &mut(self.state_vector.lock().unwrap());
            let p_mut: &mut DMatrix<f64> = &mut(self.state_covariance.lock().unwrap());

            let estimated_measurement = h_lock * &(*x_mut);
            let innovation = measurement - estimated_measurement;
            let k = ( &(*p_mut) * h_lock.transpose()) * (h_lock * &(*p_mut) * h_lock.transpose() + r_lock).try_inverse().unwrap();
            assert_eq!(innovation.shape(), (self.num_measurements, self.state_vector_length));

            let I = DMatrix::<f64>::identity(self.state_vector_length, self.state_vector_length);
            *x_mut = &(*x_mut) + (&k * &innovation);
            *p_mut = (&I - (&k * h_lock)) * &(*p_mut) * (&I - (&k * h_lock)).transpose() + &k * r_lock * &k.transpose();
            Ok(())
        } else {
            Err("The measurement has an invalid shape")
        }
    }

    pub fn start_filter(&'static self /*TODO accept initial conditions*/) {
        thread::spawn(move || {
            // start loop
            loop {
                self.predict(None);
                thread::sleep(Duration::from_nanos(((1f64/(self.update_rate as f64)) as u64) * 10^9));
            }
        });
    }
}

/*
 *
 * measure functions
 *
 * TODO add state_covariance matrix
 *
 * */

#[cfg(test)]
mod tests {
    use crate::linear_kalman_filter::LinearKalmanFilter;
    #[test]
    fn initialize_lkf() {
        let lkf = LinearKalmanFilter::new(250u64, 3usize, 1usize, 1usize);
        let state_vector_shape = lkf.state_vector.lock().unwrap().shape();
        let state_covariance_shape = lkf.state_covariance.lock().unwrap().shape();
        let transition_matrix_shape = lkf.transition_matrix.lock().unwrap().shape();
        let measurement_matrix_shape = lkf.measurement_matrix.lock().unwrap().shape();
        let measurement_noise_matrix_shape = lkf.measurement_noise_matrix.lock().unwrap().shape();

        assert_eq!(state_vector_shape, (3, 1));
        assert_eq!(state_covariance_shape, (3, 3));
        assert_eq!(transition_matrix_shape, (3, 3));
        assert_eq!(measurement_matrix_shape, (1, 3));
        assert_eq!(measurement_noise_matrix_shape, (1, 1));
    }

    #[test]
    fn test_predict() {
        let lkf = LinearKalmanFilter::new(250u64, 3usize, 1usize, 1usize);
    }
}
