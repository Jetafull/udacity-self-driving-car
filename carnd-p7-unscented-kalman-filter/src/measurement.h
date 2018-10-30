#ifndef MEASUREMENTS_H_
#define MEASUREMENTS_H_

#include "Eigen/Dense"
#include "measurement_package.h"

class Measurement {
 public:
  Eigen::MatrixXd Zsig;    // sigma points
  int n;                   // measurement dimension
  Eigen::VectorXd z_pred;  // predicted measurement
  Eigen::MatrixXd S;       // covariance matrix of measurement prediction
  Eigen::MatrixXd R;       // measurement noise covariance matrix
  MeasurementPackage::SensorType sensor_type;
};

#endif /* MEASUREMENTS_H_ */