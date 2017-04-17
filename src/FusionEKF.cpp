#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices

  VectorXd x_ = VectorXd::Ones(4);  //Initial state
  //State Transition Matrix
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

  MatrixXd P_ = MatrixXd(4, 4); //Initial State CoVar Matrix
  P_ << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1000, 0,
        0, 0, 0, 1000;

  MatrixXd Q_ = MatrixXd::Zero(4, 4); //Initial Process CoVar Matrix

  R_laser_ = MatrixXd(2, 2); //Laser: Measurement covariance
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  H_laser_ = MatrixXd(2, 4); //Laser: Measurement Function
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  R_radar_ = MatrixXd(3, 3); //Radar: Measurement covariance
  R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;

  H_radar_ = tools.CalculateJacobian(x_);      //Radar: Measurement Jacobian
    
  //Initializing the EKF
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, H_radar_, R_radar_, Q_);
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  VectorXd meas = measurement_pack.raw_measurements_;
  
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    //cout << "EKF Initialization" << endl;
    
    previous_timestamp_ = measurement_pack.timestamp_;
    //Initialize state with measurements
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      //Radar data is rho and theta. Convert from polar to cartesian
      //cout << "Initializing with Radar Data" << endl;
      float rho, theta, d_rho;
      rho = meas(0);
      theta = meas(1);
      d_rho = meas(2);
      ekf_.x_ << rho*cos(theta), rho*sin(theta), d_rho*cos(theta), d_rho*sin(theta);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      //cout << "Initializing with Laser Data" << endl;
      ekf_.x_ << meas(0), meas(1), 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    //cout << "x_ = " << ekf_.x_ << endl;
    return;
  }

  /*****************************************************************************
   * Prediction
   * Update F and Q
   ****************************************************************************/
  //cout << "Predicting New State" << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_);
  previous_timestamp_ = measurement_pack.timestamp_;
  dt /= 1000000.0;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  //Update the State Transition Matrix F with dt
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  float noise_ax = 9;
  float noise_ay = 9;
  //Update the Process Noise covariance matrix Q
  ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
             0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
             dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
             0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;
  //cout << "Q_ = " << ekf_.Q_ << endl;
  ekf_.Predict();
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
  /*****************************************************************************
   * Update
   * * Use the sensor type to perform the update step.
      * Update the state and covariance matrices.
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "Updating with Radar Data" << endl;
    ekf_.H_radar_ = tools.CalculateJacobian(ekf_.x_);      //Radar: Measurement Jacobian
    ekf_.UpdateEKF(meas);
  }
  else {
    // Laser updates
    //cout << "Updating with Laser Data" << endl;
    ekf_.Update(meas);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
