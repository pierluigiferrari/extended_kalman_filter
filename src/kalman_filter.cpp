/*
 * KalmanFilter3.cpp
 *
 *  Created on: May 16, 2017
 *      Author: pierluigiferrari
 */

#include "kalman_filter.h"
#include <cmath>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::KalmanFilter(Eigen::MatrixXd &P_in,
                           Eigen::MatrixXd &R_laser_in,
                           Eigen::MatrixXd &R_radar_in,
                           const float noise_ax_in,
                           const float noise_ay_in) {
  x_ = VectorXd(4);
  x_ << 0, 0, 0, 0; // We don't know where we are intially, so this is just a dummy value
  P_ = P_in;
  F_ = MatrixXd(4, 4);
  F_ << 1, 0, 1, 0, // The two ones that are not on the main diagonal will be replaced by the time elapsed when the program is running
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  Q_ = MatrixXd(4, 4);
  H_ = MatrixXd (2, 4); // H is the measurement matrix for LIDAR measurements. The one for RADAR measurements must me re-computed for each measurement.
  H_ << 1, 0, 0, 0,
        0, 1, 0, 0;
  R_laser_ = R_laser_in;
  R_radar_ = R_radar_in;
  noise_ax_ = noise_ax_in;
  noise_ay_ = noise_ay_in;
}

void KalmanFilter::InitializeState(const MeasurementPackage &measurement_pack) {

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    const float rho = measurement_pack.raw_measurements_[0];
    const float phi = measurement_pack.raw_measurements_[1];
    x_ << rho * cos(phi), rho * sin(phi), 0.0, 0.0; // Convert polar coordinates to Cartesian coordinates
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0.0, 0.0;
  }
}

void KalmanFilter::Predict(const float dt) {

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time difference is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Set the process noise covariance matrix Q
  Q_ <<  dt_4 / 4 * noise_ax_, 0, dt_3 / 2 * noise_ax_, 0,
         0, dt_4 / 4 * noise_ay_, 0, dt_3 / 2 * noise_ay_,
         dt_3 / 2 * noise_ax_, 0, dt_2 * noise_ax_, 0,
         0, dt_3 / 2 * noise_ay_, 0, dt_2 * noise_ay_;

  // Make the prediction
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {

  // Compute y, S, and K
  VectorXd y = z - H_ * x_;
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S = H_ * PHt + R_laser_;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = PHt * Sinv;

  // Compute the new estimates for x and P
  x_ = x_ + (K * y);
  P_ -= K * H_ * P_; // This is more efficient than P = (I - K * H) * P
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

  // 0: Extract the individual state parameters for better readability
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  // 1: Compute the Jacobian matrix of h(x) (i.e. the Jacobian of h evaluated at x)
  MatrixXd Hj(3,4);
  // Pre-compute a set of terms to avoid repeated calculation
  float c1 = px * px + py * py;
  float c2 = sqrt(c1);
  float c3 = (c1 * c2);
  // Protect against division by zero
  if(fabs(c1) < 0.0001) {
    std::cout << "Error - Division by Zero: (px^2 + py^2) is either zero or close to zero." << std::endl;
    return;
  }
  // Compute the Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
       -(py / c1), (px / c1), 0, 0,
         py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  // 2: Compute h(x)
  VectorXd h(3);
  h(0) = c2;
  h(1) = atan2(py, px); // Returns values in (-Pi,Pi)
  if(h(1) > 3.13 || h(1) < -3.13) return; // Throw away this measurement if phi is too close Pi
  h(2) = (px * vx + py * vy) / c2;

  // 3: Compute y, S, and K
  VectorXd y = z - h;
  MatrixXd PHjt = P_ * Hj.transpose();
  MatrixXd S = Hj * PHjt + R_radar_;
  MatrixXd Sinv = S.inverse();
  MatrixXd K = PHjt * Sinv;

  // 4: Compute the new estimates for x and P
  x_ = x_ + K * y;
  P_ -= K * Hj * P_; // This is more efficient than P = (I - K * H) * P
}

void KalmanFilter::Print() {
  std::cout << "x_ = " << x_ << std::endl;
  std::cout << "P_ = " << P_ << std::endl;
}
