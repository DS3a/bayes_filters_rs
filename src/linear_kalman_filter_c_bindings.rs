use crate::linear_kalman_filter;
use std::ffi;

#[no_mangle]
/*
extern "C" fn init_linear_kalman_filter() -> *mut linear_kalman_filter::LinearKalmanFilter {
  unsafe{
    (&mut linear_kalman_filter::LinearKalmanFilter::default()) as *mut linear_kalman_filter::LinearKalmanFilter
  }
}
*/
#[no_mangle]
extern "C" fn linear_kalman_predict(kf: *mut linear_kalman_filter::LinearKalmanFilter) {
  unsafe {
    (&mut *kf).predict(None, None);
  }
}

