#include "kalman_filter.h"
#include <iostream>
#include "tools.h"
#include <math.h>

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
//  MatrixXd Ht = H_.transpose();
//  MatrixXd S = H_ * P_ * Ht + R_;
//  MatrixXd Si = S.inverse();
//  MatrixXd PHt = P_ * Ht;
//  MatrixXd K = PHt * Si;
//
//  P_ -= K * H_ * P_;
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h_x(3);
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double c1 = px * px + py * py;
  double c2 = sqrt(c1);

  // check to avoid too small
  if(fabs(c1) < 0.0001){
      c1 = 0.000001;
      c2 = sqrt(c1);
  }

  // h(x') function
  h_x << c2,
          atan2(py,px),
          (px * vx + py * vy)/c2;

  VectorXd y = z - h_x;

  // Jacobian Matrix Hj
  Tools tool;
  H_ = tool.CalculateJacobian(x_);

  NormalizeAngle(y(1));

//  MatrixXd Ht = H_.transpose();
//  MatrixXd S = H_ * P_ * Ht + R_;
//  MatrixXd Si = S.inverse();
//  MatrixXd PHt = P_ * Ht;
//  MatrixXd K = PHt * Si;
//
//  P_ -= K * H_ * P_;
  UpdateCommon(y);
}

void KalmanFilter::NormalizeAngle(double& phi)
{
  phi = atan2(sin(phi), cos(phi));
}

void KalmanFilter::UpdateCommon(const VectorXd& y)
{
  const MatrixXd PHt = P_ * H_.transpose();
  const MatrixXd S = H_ * PHt + R_;
  const MatrixXd K = PHt * S.inverse();

  x_ += K * y;
  P_ -= K * H_ * P_;
}