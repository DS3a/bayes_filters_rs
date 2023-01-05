use std::thread;
use nalgebra::{DMatrix, DVector};
use std::sync::{Arc, Mutex};
use std::mem::drop;

pub use crate::linear_kalman_filter_c_bindings;

pub struct LinearKalmanFilter {
  state_vector_length: usize,
  num_inputs: usize,
  state_vector: Arc<Mutex<DVector<f64>>>,
  state_covariance: Arc<Mutex<DMatrix<f64>>>,
  transition_matrix: Arc<Mutex<DMatrix<f64>>>,
  update_rate: u64,
}

impl LinearKalmanFilter {
  pub fn new(mut update_rate: u64, state_vector_length: usize, num_inputs: usize) -> Self {
    if update_rate == 0 {
        update_rate = 1;
    }
    Self {
      state_vector_length,
      num_inputs,
      state_vector: Arc::new(Mutex::new(DVector::<f64>::zeros(state_vector_length))),
      state_covariance: Arc::new(Mutex::new(DMatrix::<f64>::zeros(state_vector_length, state_vector_length))),
      transition_matrix: Arc::new(Mutex::new(DMatrix::<f64>::zeros(state_vector_length, state_vector_length))),
      update_rate,
    }
  }

  pub fn set_transition_matrix(&mut self, new_transition_matrix: DMatrix<f64>) {
    if (new_transition_matrix.shape().0 == self.state_vector_length) &&
       (new_transition_matrix.shape().1 == self.state_vector_length) {
      *(self.transition_matrix.lock().unwrap()) = new_transition_matrix;
    }
  }

  pub fn predict(&self, input: Option<f64>, input_covariance: Option<f64>) {
    let mut state_vector_lock = &*(self.state_vector.lock().unwrap());
    let mut transition_matrix_lock = &*(self.transition_matrix.lock().unwrap());
    let binding = transition_matrix_lock * state_vector_lock;
    state_vector_lock = &binding;

/*    if let Some(input) = input {
      // assuming velocity input
      state_vector_lock = state_vector_lock + Vector3::<f64>::new(0.0, input, 0.0);
    }
*/
    drop(state_vector_lock);
    drop(transition_matrix_lock);


    // TODO add covariance matrix
    println!("{:?}", &self.state_vector);
  }

  pub fn measure(&self) {
    // TODO add measurement equation
  }

  pub fn start_filter(/*initial conditions*/) {
    thread::spawn(move || {
      // start loop
      loop {
        // predict state
        // update covariance matrix
        // wait for 1/f time units
      }
    });
  }
}

/*
 * ensure update_rate is higher than 0 /////done
 * TODO start_kf function, which updates irrespective of input or measurements
 *
 *
 * measure functions
 * 
 * TODO add state_covariance matrix
 * 
 * */
