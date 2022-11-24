use nalgebra::{Matrix3, Vector3, SMatrix};

#[derive(Default)]
pub struct LinearKalmanFilter {
  state_vector: SMatrix<f64, 3, 1>,
  transition_matrix: Matrix3<f64>,
  update_rate: u64,
}

impl LinearKalmanFilter {
  pub fn predict(&mut self, dt: f64, input: Option<f64>, input_covariance: Option<f64>) {
    self.state_vector = self.transition_matrix * self.state_vector;

    if let Some(input) = input {
      self.state_vector = self.state_vector + Vector3::<f64>::new(0.0, input, 0.0);
    }
    println!("{:?}", &self.state_vector);
  }
}

/*
 * ensure update_rate is higher than 0
 * TODO start_kf function, which updates irrespective of input or measurements
 *
 *
 * measure functions
 * 
 * TODO add state_covariance matrix
 * 
 * */
