use std::thread;
use nalgebra::{Matrix3, Vector3, SMatrix};
use std::sync::{Arc, Mutex};
use std::mem::drop;

#[derive(Default)]
pub struct LinearKalmanFilter {
  state_vector: Arc<Mutex<SMatrix<f64, 3, 1>>>,
  state_covariance: Arc<Mutex<Matrix3<f64>>>,
  transition_matrix: Arc<Mutex<Matrix3<f64>>>,
  update_rate: u64,
}

impl LinearKalmanFilter {
  pub fn new(update_rate: u64) -> Self {
    let mut lkf_instance = Self::default();

    lkf_instance.update_rate = 1u64;
    return lkf_instance;
  }

  pub fn update_transition_matrix() {

  }

  pub fn predict(&self, dt: Option<f64>, input: Option<f64>, input_covariance: Option<f64>) {
    let mut state_vector_lock = *(self.state_vector.lock().unwrap());
    let mut transition_matrix_lock = *(self.transition_matrix.lock().unwrap());

    if let Some(dt) = dt {
      /*Update the transition matrix*/
    }
    state_vector_lock = transition_matrix_lock * state_vector_lock;

    if let Some(input) = input {
      // assuming velocity input
      state_vector_lock = state_vector_lock + Vector3::<f64>::new(0.0, input, 0.0);
    }
    drop(state_vector_lock);
    drop(transition_matrix_lock);


    // TODO add covariance matrix
    println!("{:?}", &self.state_vector);
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
