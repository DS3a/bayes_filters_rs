#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

void *init_linear_kalman_filter();
void linear_kalman_predict(void *kf);

#endif
