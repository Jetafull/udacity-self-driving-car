#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  is_initialized_ = false;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);
  P_.fill(0.0);
  for (int i = 0; i < n_x_; i++) P_(i, i) = 1.0;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 0.3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;

  // DO NOT MODIFY measurement noise values below these are provided by the
  // sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // DO NOT MODIFY measurement noise values above these are provided by the
  // sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
  // set vector for weights
  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_ / (lambda_ + n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  // time
  time_us_ = 0;

  // create augmented mean vector
  x_aug_ = VectorXd(n_aug_);

  // create augmented state covariance
  P_aug_ = MatrixXd(n_aug_, n_aug_);

  // create sigma point matrix
  Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // matrix with predicted points as columns
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // radar measurement prediction
  z_pred_ = VectorXd(n_radar_);

  // covariance matrix for radar prediction
  S_radar_ = MatrixXd(n_radar_, 2 * n_aug_ + 1);

  // create matrix for sigma points in measurement space
  Zsig = MatrixXd(n_radar_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // TODO: if not initialized
  //  x_.fill(0.0);
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    float px = Xsig_aug_(0, i);
    float py = Xsig_aug_(1, i);
    float v = Xsig_aug_(2, i);
    float psi = Xsig_aug_(3, i);
    float psi_dot = Xsig_aug_(4, i);
    float nu_ak = Xsig_aug_(5, i);
    float nu_yawdd = Xsig_aug_(6, i);

    float px_p, py_p;
    if (fabs(psi < 0.0001)) {
      px_p = px + v * cos(psi) * delta_t;
      py_p = py + v * sin(psi) * delta_t;
    } else {
      px_p = px + v * (sin(psi + psi_dot * delta_t) - sin(psi)) / psi_dot;
      py_p = py + v * (-cos(psi + psi_dot * delta_t) + cos(psi)) / psi_dot;
    }

    // add noise
    px_p = px_p + 0.5 * delta_t * delta_t * cos(psi) * nu_ak;
    py_p = py_p + 0.5 * delta_t * delta_t * sin(psi) * nu_ak;

    float v_p = v + delta_t * nu_ak;
    float psi_p = psi + psi_dot * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
    float psi_dot_p = psi_dot + delta_t * nu_yawdd;

    // update state
    Xsig_pred_(0, i) = px_p;
    Xsig_pred_(1, i) = py_p;
    Xsig_pred_(2, i) = v_p;
    Xsig_pred_(3, i) = psi_p;
    Xsig_pred_(4, i) = psi_dot_p;
  }

  // predicted state mean
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }

  // predicted state covariance matrix
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  // create example vector for incoming radar measurement
  VectorXd z = meas_package.raw_measurements_;

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_radar_);

  /*******************************************************************************
   * Student part begin
   ******************************************************************************/

  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred_;
    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    // angle normalization
    while (x_diff(3) > M_PI) x_diff(3) -= 2. * M_PI;
    while (x_diff(3) < -M_PI) x_diff(3) += 2. * M_PI;

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S_radar_.inverse();

  // residual
  VectorXd z_diff = z - z_pred_;

  // angle normalization
  while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
  while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S_radar_ * K.transpose();

  // calculate NIS
}

void UKF::AugmentedSigmaPoints() {
  // create augmented mean state
  x_aug_.head(n_x_) = x_;

  // create augmented covariance matrix
  P_aug_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_(n_x_, n_x_) = std_a_ * std_a_;
  P_aug_(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd A_aug = P_aug_.llt().matrixL();

  // create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i + 1) = x_aug_ + sqrt(lambda_ + n_aug_) * A_aug.col(i);
    Xsig_aug_.col(i + 1 + n_aug_) =
        x_aug_ - sqrt(lambda_ + n_aug_) * A_aug.col(i);
  }
}

void UKF::PredictRadarMeasurement() {
  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // extract values for better readibility
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                          // r
    Zsig(1, i) = atan2(p_y, p_x);                                      // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  // r_dot
  }

  // mean predicted measurement
  z_pred_.fill(0.0);
  for (int i = 0; i < 2 * n_radar_ + 1; i++) {
    z_pred_ = z_pred_ + weights_(i) * Zsig.col(i);
  }

  // innovation covariance matrix S_radar_
  S_radar_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred_;

    // angle normalization
    while (z_diff(1) > M_PI) z_diff(1) -= 2. * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2. * M_PI;

    S_radar_ = S_radar_ + weights_(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_radar_, n_radar_);
  R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0,
      std_radrd_ * std_radrd_;
  S_radar_ = S_radar_ + R;
}