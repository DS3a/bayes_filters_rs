#include "kalman_filter.h"

int main() {

  void *kf_obj = init_linear_kalman_filter();
  linear_kalman_predict(kf_obj);
  return 0;
}


