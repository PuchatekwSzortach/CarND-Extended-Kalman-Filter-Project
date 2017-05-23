#include <iostream>
#include "tools.h"

#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd errors(4) ;
  errors << 0, 0, 0, 0 ;

  // Sum square sums of errors between ground truth and observations so far
  for(int i = 0 ; i < estimations.size() ; ++i)
  {

    VectorXd difference = (estimations[i] - ground_truth[i]) ;
    VectorXd squared_difference = difference.array() * difference.array() ;

    errors += squared_difference ;
  }

  // Now divide by number of samples and take sqrt for a complete RMSE
  return (errors / estimations.size()).array().sqrt() ;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */

  MatrixXd Hj(3,4) ;

  float px = x_state[0] ;
  float py = x_state[1] ;
  float vx = x_state[2] ;
  float vy = x_state[3] ;

  float squared_radial_distance = (px * px) + (py * py) ;

  if(std::fabs(squared_radial_distance) < 0.0001) {

    return Hj ;

  }

  float radial_distance = std::sqrt(squared_radial_distance) ;

  float H_2_0_numerator = py * ((vx * py) - (vy * px)) ;
  float H_2_0 = H_2_0_numerator / (squared_radial_distance * radial_distance) ;

  float H_2_1_numerator = px * ((vy * px) - (vx * py)) ;
  float H_2_1 = H_2_1_numerator / (squared_radial_distance * radial_distance) ;

  Hj << px / radial_distance, py / radial_distance, 0, 0,
      -py / squared_radial_distance, px / squared_radial_distance, 0, 0,
      H_2_0, H_2_1, px / radial_distance, py / radial_distance ;

  return Hj ;
}

float Tools::get_normalized_angle(float angle)
{
  // If angle is below PI, bump it up
  while(angle < -M_PI)
  {
    angle += 2 * M_PI ;
  }

  // If angl is above PI, decrease it
  while(angle > M_PI)
  {
    angle -= 2 * M_PI ;
  }

  return angle ;

}
