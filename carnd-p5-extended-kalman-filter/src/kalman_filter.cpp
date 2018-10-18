#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

void KalmanFilter::Predict() {
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_*P_*Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_*Ht;
    MatrixXd K = PHt*Si;
    
    // new estimate
    x_ = x_ + K*y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    // create Jacobian of H
    Tools tools;
    MatrixXd Hj = tools.CalculateJacobian(x_);

    // recover state parameters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    
    // map states to measurements direct using h function
    float rho = sqrt(px*px+py*py);
    VectorXd z_pred(3);
    z_pred << rho, atan2(py, px), (px*vx+py*vy)/rho;
    
    // calculate y
    // normalize phi to between -pi and pi
    // assert(-M_PI < y(1) & y(1) < M_PI);
    VectorXd y = z - z_pred;
    while (y(1) < -M_PI) {
        y(1) += 2*M_PI;
    }
    while (y(1) >+ M_PI) {
        y(1) -= 2*M_PI;
    }

    // update S, K and P with Hj
    MatrixXd Hjt = Hj.transpose();
    MatrixXd S = Hj*P_*Hjt + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHjt = P_*Hjt;
    MatrixXd K = PHjt*Si;
    
    // new estimate
    x_ = x_ + K*y;
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I-K*Hj)*P_;
}
