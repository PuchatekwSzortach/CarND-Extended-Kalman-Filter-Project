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

  this->is_initialized_ = false;

  this->previous_timestamp_ = 0;

  // initializing matrices
  this->R_laser_ = MatrixXd(2, 2);
  this->R_radar_ = MatrixXd(3, 3);

  this->H_laser_ = MatrixXd(2, 4);
  this->H_laser_ << 1, 0, 0, 0,
                    0, 1, 0, 0 ;

  this->Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  this->R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  this->R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


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
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */

    double px = 0 ;
    double py = 0 ;
    double vx = 0 ;
    double vy = 0 ;

    if(measurement_pack.sensor_type_ == MeasurementPackage::LASER)
    {
      //      /**
      //      Initialize state.
      //      */
      px = measurement_pack.raw_measurements_[0] ;
      py = measurement_pack.raw_measurements_[1] ;
    }
    else if(measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {

      //      /**
      //      Convert radar from polar to cartesian coordinates and initialize state.
      //      */
      float r = measurement_pack.raw_measurements_[0] ;
      float theta = tools.get_normalized_angle(measurement_pack.raw_measurements_[1]) ;

      px = r * std::cos(theta) ;
      py = r * std::sin(theta) ;

    }

    this->previous_timestamp_ = measurement_pack.timestamp_ ;

    // first measurement
    cout << "EKF: " << endl;
    VectorXd initial_measurement(4);
    initial_measurement << px, py, vx, vy ;
    ekf_.x_ = initial_measurement ;

    MatrixXd P(4, 4);
    P << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000 ;

    // These matrices change at each time step, so for now their initial values don't matter.
    // We just need to initialize them to something so that Kalman Filter object grabs memory for them
    MatrixXd F(4,4), H(4,4), R(2,2), Q(4,4) ;

    this->ekf_.Init(initial_measurement, P, F, H, R, Q);

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  float time_delta = float((measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0) ;

  // Update timestamp
  this->previous_timestamp_ = measurement_pack.timestamp_ ;

  // Update state transision matrix F
  this->ekf_.F_ << 1, 0, time_delta, 0,
        0, 1, 0, time_delta,
        0, 0, 1, 0,
        0, 0, 0, 1 ;

  float half_squared_time_delta = float(time_delta * time_delta / 2.0) ;

  MatrixXd G(4,2) ;
  G << half_squared_time_delta, 0,
      0, half_squared_time_delta,
      time_delta, 0,
      0, time_delta ;

  MatrixXd Qv(2,2) ;
  Qv << 9.0, 0,
        0, 9.0 ;

  this->ekf_.Q_ = G * Qv * G.transpose() ;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    // Radar updates
    std::cout << "RADAR measurement" << std::endl ;
    std::cout << "Not handling it yet" << std::endl ;

  } else {

    // Don't use laser for a moment
    int i = 0 ;
    // Laser updates
//    this->ekf_.H_ = this->H_laser_ ;
//    this->ekf_.R_ = this->R_laser_ ;
//
//    this->ekf_.Update(measurement_pack.raw_measurements_) ;

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;

}
