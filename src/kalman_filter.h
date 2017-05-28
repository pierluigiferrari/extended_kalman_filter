/*
 * kalman_filter.h
 *
 *  Created on: May 16, 2017
 *      Author: pierluigiferrari
 */

#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "Eigen/Dense"
#include "measurement_package.h"

class KalmanFilter {

public:

  /*
   * Default constructor
   */
  KalmanFilter();

  /*
   * Constructor
   * @param P_in Initial state covariance
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param noise_ax_in Acceleration noise coefficient in x-direction. Defaults to 9.
   * @param noise_ay_in Acceleration noise coefficient in y-direction. Defaults to 9.
   */
  KalmanFilter(Eigen::MatrixXd &P_in,
               Eigen::MatrixXd &R_laser_in,
               Eigen::MatrixXd &R_radar_in,
               const float noise_ax_in = 9,
               const float noise_ay_in = 9);

  /*
   * Initializes the state with the first measurement. For the very first measurement,
   * there are no prediction and update steps. Instead, the state position will simply
   * be set to the first measurement coordinates.
   * @param measurement_pack The first measurement coordinates
   */
  void InitializeState(const MeasurementPackage &measurement_pack);

  /*
   * Predicts the state mean vector and state covariance matrix using
   * the state transition matrix F and the process covariance matrix Q.
   * @param dt the elapsed time since the last state update
   */
  void Predict(const float dt);

  /*
   * Updates the state mean vector and state covariance matrix using the standard
   * Kalman Filter equations.
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /*
   * Updates the state mean vector and state covariance matrix using the Extended
   * Kalman Filter equations.
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

  /*
   * Prints the current state x vector and P matrix
   */
  void Print();

  // State vector
  Eigen::VectorXd x_;

  // State covariance matrix
  Eigen::MatrixXd P_;

  // State transition matrix
  Eigen::MatrixXd F_;

  // Process covariance matrix
  Eigen::MatrixXd Q_;

  // Measurement matrix
  Eigen::MatrixXd H_;

  // Measurement covariance matrix for LIDAR
  Eigen::MatrixXd R_laser_;

  // Measurement covariance matrix for RADAR
  Eigen::MatrixXd R_radar_;

  // Acceleration noise coefficients
  float noise_ax_;
  float noise_ay_;

};

#endif /* KALMAN_FILTER_H_ */
