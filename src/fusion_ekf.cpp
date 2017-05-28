/*
 * fusion_ekf.cpp
 *
 *  Created on: May 19, 2017
 *      Author: pierluigiferrari
 */

#include "fusion_ekf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {

  is_initialized_ = false;
  previous_timestamp_ = 0;

  // Choose an initial state covariance matrix P
  MatrixXd P(4, 4);
  P << 1, 0, 0, 0,
       0, 1, 0, 0,
       0, 0, 1000, 0,
       0, 0, 0, 1000;

  // Initialize the measurement covariance matrices R for LIDAR and RADAR
  MatrixXd R_laser(2, 2);
  MatrixXd R_radar(3, 3);
  R_laser << 0.0225, 0,
             0, 0.0225;
  R_radar << 0.09, 0, 0,
             0, 0.0009, 0,
             0, 0, 0.09;

  // Set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;

  ekf_ = KalmanFilter(P,
                      R_laser,
                      R_radar,
                      noise_ax,
                      noise_ay);

}

/*
 * Destructor
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   * Initialization
   *****************************************************************************/

  if (!is_initialized_) {
    std::cout << "EKF: " << std::endl;
    ekf_.InitializeState(measurement_pack); // Set the initial state vector x
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true; // Initialization complete, no need to predict or update anything
    return;
  }

  /*****************************************************************************
   * Prediction
   *****************************************************************************/

  // Compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; // dt expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.Predict(dt);

  /*****************************************************************************
   * Update
   *****************************************************************************/

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  ekf_.Print(); // Print the current state x and P

}

