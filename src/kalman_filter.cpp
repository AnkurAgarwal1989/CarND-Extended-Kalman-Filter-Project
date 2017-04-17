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

void KalmanFilter::Update(const VectorXd &z) {
  //Measurement Update
  /*Linear State Update equations:
  y = z - H*x_new
  S = H*P_new*Ht + R
  K = P_new*Ht*inv(S)
  
  x = x_new + K*y
  P = (I - K*H)*P_new
  */

  VectorXd y = z - H_laser_*x_;
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_*P_*Ht + R_laser_;
  MatrixXd K = P_*Ht*S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + K*y;
  P_ = (I - K*H_laser_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  //Measurement Update
  /*Non-Linear State Update equations (EKF):
  y = z - Hj*x_new
  S = H*P_new*Ht + R
  K = P_new*Ht*inv(S)

  x = x_new + K*y
  P = (I - K*H)*P_new
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
  
  MatrixXd Ht = H_radar_.transpose();
  MatrixXd S = H_radar_*P_*Ht + R_radar_;
  MatrixXd K = P_*Ht*S.inverse();

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  x_ = x_ + K*y;
  P_ = (I - K*H_radar_)*P_;
}
