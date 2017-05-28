/*
 * fusion_ekf.h
 *
 *  Created on: May 19, 2017
 *      Author: pierluigiferrari
 */

#ifndef FUSION_EKF_H_
#define FUSION_EKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:

  /*
   * Constructor
   */
  FusionEKF();

  /*
   * Destructor.
   */
  virtual ~FusionEKF();

  /*
   * Runs the whole Kalman Filter process
   */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  // The Kalman Filter math for prediction and update lives here
  KalmanFilter ekf_;

private:

  // True once the first measurement has been processed
  bool is_initialized_;

  // The most recent measurement time stamp
  long long previous_timestamp_;
};

#endif /* FUSION_EKF_H_ */
