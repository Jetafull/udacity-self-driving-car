#ifndef UKF_H
#define UKF_H

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "measurement.h"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
 public:
  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* time when the state is true, in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  const int n_x_ = 5;

  ///* Augmented state dimension
  const int n_aug_ = 7;

  ///* Sigma point spreading parameter
  const double lambda_ = 3 - n_aug_;

  VectorXd x_aug_;
  MatrixXd P_aug_;
  MatrixXd Xsig_aug_;
  MatrixXd Xsig_pred_;

  // radar measurements
  const int n_radar_ = 3;
  Measurement radar_measurement_;

  // lidar measurements
  const int n_lidar_ = 2;
  Measurement lidar_measurement_;

  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  void Predict(double delta_t);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  // void UpdateWithLidarMeasurement(MeasurementPackage meas_package);
  void UpdateWithMeasurement(MeasurementPackage, Measurement);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  // void UpdateWithRadarMeasurement(MeasurementPackage meas_package);

  void AugmentSigmaPoints();
  void MapPredictionToMeasurement(Measurement&);
  void MapPredictionToLidarMeasurement();
  void MapPredictionToRadarMeasurement();
};

#endif /* UKF_H */
