#include <iostream>

#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

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

  x_ = F_ * x_ ;
  P_ = F_ * P_ * F_.transpose() + Q_ ;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_prediction = H_ * x_ ;
  VectorXd y = z - z_prediction ;

  MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
  MatrixXd K = P_ * H_.transpose() * S.inverse() ;

  x_ = x_ + (K * y) ;

  MatrixXd I = MatrixXd::Identity(4, 4) ;
  this->P_ = (I - K * H_) * P_ ;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd z_prediction = H_ * x_ ;
  VectorXd y = z - z_prediction ;

  y << y[0], Tools().get_normalized_angle(y[1]), y[2] ;

  MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
  MatrixXd K = P_ * H_.transpose() * S.inverse() ;

  std::cout << "y is\n" << y << "\nS is \n" << S << "\nK is \n" << K << std::endl ;

  x_ = x_ + (K * y) ;

  MatrixXd I = MatrixXd::Identity(4, 4) ;
  this->P_ = (I - K * H_) * P_ ;
}
