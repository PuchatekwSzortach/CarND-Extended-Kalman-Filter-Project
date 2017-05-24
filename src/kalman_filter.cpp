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

  float px = this->x_[0] ;
  float py = this->x_[1] ;
  float vx = this->x_[2] ;
  float vy = this->x_[3] ;

  float squared_radial_distance = std::sqrt((px*px) + (py*py)) ;

  VectorXd radial_coordinates_prediction(3);
  radial_coordinates_prediction << squared_radial_distance, std::atan2(py, px),
    ((px*vx) + (py*vy)) / squared_radial_distance ;

  VectorXd y = z - radial_coordinates_prediction ;
  y << y[0], Tools().get_normalized_angle(y[1]), y[2] ;

  MatrixXd S = H_ * P_ * H_.transpose() + R_ ;
  MatrixXd K = P_ * H_.transpose() * S.inverse() ;

  x_ = x_ + (K * y) ;

  MatrixXd I = MatrixXd::Identity(4, 4) ;
  this->P_ = (I - K * H_) * P_ ;
}
