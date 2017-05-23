#include <iostream>
#include "tools.h"

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
}
