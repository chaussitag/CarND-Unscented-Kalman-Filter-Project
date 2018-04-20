#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(estimations[0].size());
  rmse.fill(0.0);
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size() || estimations.size() < 1) {
    cerr << "the size of ground_truth and estimations should be same and non-zero" << endl;
    return rmse;
  }

  //accumulate squared residuals
  const int n = estimations.size();
  for(int i = 0; i < n; ++i) {
    VectorXd residual = estimations[i] - ground_truth[i];
    residual = residual.array() * residual.array();
    rmse += residual;
  }

  //calculate the mean
  rmse = rmse / n;

  //calculate the squared root
  rmse = rmse.array().sqrt();

  //return the result
  return rmse;
}