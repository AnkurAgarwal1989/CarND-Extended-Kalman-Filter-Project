#include "kalman_filter.h"
#include<iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
  MatrixXd &H_laser_in, MatrixXd &R_laser_in, MatrixXd &H_radar_in,
  MatrixXd &R_radar_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_laser_ = H_laser_in;
  R_laser_ = R_laser_in;
  H_radar_ = H_radar_in;
  R_radar_ = R_radar_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  //State Prediction
  /*Linear State Predicition Equations:
  x_new = F*x + u;
  P_new = F*P*Ft + Q;
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

//Utility function called by both Update and UpdateEKF
void KalmanFilter::_Update(const VectorXd &y, const MatrixXd &H, const MatrixXd &R) {
  //Measurement Update
  /*
  S = H*P_new*Ht + R
  K = P_new*Ht*inv(S)
  x = x_new + K*y
  P = (I - K*H)*P_new
  */
  MatrixXd Ht = H.transpose();
  MatrixXd PHt = P_*Ht;
  MatrixXd S = H*PHt + R;
  MatrixXd K = PHt*S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + K*y;
  P_ = (I - K*H)*P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /*Linear State Update equations:
  y = z - H*x_new
  */
  VectorXd y = z - H_laser_*x_;
  _Update(y, H_laser_, R_laser_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /*Non-Linear State Update equations (EKF):
  y = z - Hj*x_new
  */
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  //pre-compute a set of terms to avoid repeated calculation
  //Converting from Cartesian state to polar coordinates
  float a = sqrt(px*px + py*py);
  float b = atan2(py, px);
  float c = (px*vx + py*vy)/(a + 1e-7); 
  VectorXd hjx(3);
  hjx << a, b, c;

  VectorXd y = z - hjx;
  _Update(y, H_radar_, R_radar_);
}
