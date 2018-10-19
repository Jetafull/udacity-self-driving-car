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
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);
    
    // measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
    0, 0.0225;
    
    // measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
    0, 0.0009, 0,
    0, 0, 0.09;
    
    /**
     TODO:
     * Finish initializing the FusionEKF.
     * Set the process and measurement noises
     */
    H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;
    
    Hj_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0;
    
    // initialize F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    // state covariance matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;
    
    // process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    
    
    /*****************************************************************************
     *  Initialization
     ****************************************************************************/
    if (!is_initialized_) {
        // first measurement
        cout << "Fusion Extended Kalman Filter Initialization " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;
        
        // use measurement as the previous_timestamps
        previous_timestamp_ = measurement_pack.timestamp_;
        
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
            /**
             Convert radar from polar to cartesian coordinates and initialize state.
             */
            cout << "Initializing with RADAR data" << endl;
            float rho = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            ekf_.x_ << rho*cos(phi), rho*sin(phi), 0, 0;
            ekf_.R_ = R_radar_;
            ekf_.H_ = Hj_;
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
            /**
             Initialize state.
             */
            cout << "Initializing with LASER data" << endl;
            ekf_.x_ << measurement_pack.raw_measurements_(0),
            measurement_pack.raw_measurements_(1), 0, 0;
            ekf_.R_ = R_laser_;
            ekf_.H_ = H_laser_;
        }
        
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
    
    /*****************************************************************************
     *  Prediction
     ****************************************************************************/
    // calculate elapsed time
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;    //dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;
    
    // update the state transition matrix F according to the new elapsed time dt
    ekf_.F_ << 1, 0, dt, 0,
    0, 1, 0, dt,
    0, 0, 1, 0,
    0, 0, 0, 1;
    
    // Update the process noise covariance matrix
    // define noises for process covariance matrix Q
    ekf_.Q_ << pow(dt,4)/4*noise_ax, 0, pow(dt,3)/2*noise_ax, 0,
    0, pow(dt,4)/4*noise_ay, 0, pow(dt,3)/2*noise_ay,
    pow(dt,3)/2*noise_ax, 0, pow(dt,2)*noise_ax, 0,
    0, pow(dt,3)/2*noise_ay, 0, pow(dt,2)*noise_ay;
    ekf_.Predict();
    
    /*****************************************************************************
     *  Update
     ****************************************************************************/
    
    /**
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
     */
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
        // Radar updates
        ekf_.R_ = R_radar_;
        ekf_.H_ = Hj_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
        // Laser updates
        ekf_.R_ = R_laser_;
        ekf_.H_ = H_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }
    
    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
