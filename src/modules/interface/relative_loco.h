/**
Shushuai li
 *
 */

#ifndef RELATIVELOCO_H_
#define RELATIVELOCO_H_

// #define EKF_N 4
// #define EKF_M 3
// typedef struct discrete_ekf {
//   float X[EKF_N];  // state X
//   float Xp[EKF_N]; // state prediction
//   float Zp[EKF_M]; // measurement prediction
//   float P[EKF_N][EKF_N]; // state covariance matrix
//   float Pp[EKF_N][EKF_N]; // predicted state covariance matrix
//   float Q[EKF_N][EKF_N]; // proces covariance noise
//   float R[EKF_M][EKF_M]; // measurement covariance noise
//   float H[EKF_M][EKF_N]; // jacobian of the measure wrt X
//   float Ht[EKF_N][EKF_M]; // transpose of H
//   float A[EKF_N][EKF_N];
//   float E[EKF_M][EKF_M];
//   float K[EKF_N][EKF_M];
//   float dt;
// } discrete_ekf;
// typedef struct ekf {
//   float X[EKF_N];  // state X
//   float Xp[EKF_N]; // state prediction
//   float Zp[EKF_M]; // measurement prediction
//   float P[EKF_N][EKF_N]; // state covariance matrix
//   float Pp[EKF_N][EKF_N]; // predicted state covariance matrix
//   float Q[EKF_N][EKF_N]; // proces covariance noise
//   float R[EKF_M][EKF_M]; // measurement covariance noise
//   float H[EKF_M][EKF_N]; // jacobian of the measure wrt X
//   float Ht[EKF_N][EKF_M]; // transpose of H
//   float A[EKF_N][EKF_N];
//   float E[EKF_M][EKF_M];
//   float K[EKF_N][EKF_M];
//   float dt;
// } ekf;
void relativeLocoInit(void);
void relativeLocoTask(void* arg);
// void discrete_ekf_new(ekf* filter);
// void discrete_ekf_predict(ekf* filter);
// void discrete_ekf_update(ekf* filter, float *Z);
// void linear_filter(ekf* filter, float dt);
// void linear_measure(ekf* filter);
#endif /* SRC_MODULES_INTERFACE_RSSI_CRAZYRADIO_LOCALIZATION_H_ */
