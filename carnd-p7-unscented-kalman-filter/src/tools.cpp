#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  int n = estimations.size();
  VectorXd rmse(n);
  rmse.fill(0.0);

  // check the validity of the following inputs:
  // * the estimation vector size should not be zero
  // * the estimation vector vector should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i = 0; i < estimations.size(); ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  // calculate mean
  rmse = rmse / estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}