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
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check vector size
  if (estimations.size() == 0){
      std::cout << "Invalid estimation data" << std::endl;
      return rmse;
  }
  if (estimations.size() != ground_truth.size()){
      std::cout << "Invalid ground_truth data" << std::endl;
      return rmse;
  }

  // accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i){
      VectorXd residual = estimations[i] - ground_truth[i];
      // coefficient-wise multiplication
      residual = residual.array()*residual.array();
      rmse += residual;
  }

  // the mean
  rmse = rmse / estimations.size();
  // the squared root
  rmse = rmse.array().sqrt();

  return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  // state parameters
  const float px = x_state(0);
  const float py = x_state(1);
  const float vx = x_state(2);
  const float vy = x_state(3);

  const float c1 = px * px + py * py;
  const float c2 = sqrt(c1);
  const float c3 = c1 * c2;

  // check
  if (fabs(c1) < 0.0001){
      std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
      Hj << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
      return Hj;
  }

  // Jacobian matrix
  Hj << (px / c2), (py / c2), 0, 0,
      -(py / c1), (px / c1), 0, 0,
      py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

  return Hj;

}
