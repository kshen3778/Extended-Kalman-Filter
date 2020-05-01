#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;


/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing LASER matrices
  R_laser_ = MatrixXd(2, 2);
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  H_laser_ = MatrixXd(2, 4);


  // initializing RADAR matrices
  R_radar_ = MatrixXd(3, 3);
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;


  /**
   * TODO: Initialize H_laser
   Hj for radar will be calculated later with CalculateJacobian in the Measurement Update stage.
   Technically, F, Q, and P could be initialized in a later stage
   */

  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //initialize F, Q to zero matrices, they will be properly set and updated later
  ekf_.F_ = MatrixXd::Zero(4, 4);

  ekf_.Q_ = MatrixXd::Zero(4, 4);


  //initialize P to default values in previous assignment (state covariance matrix)
  //perhaps this means at start: very certain about position (variance 1) but not sure about initial velocity values of 0 (variance 1000)
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;


}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement when we just start
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.

      //State X is always in (x,y,x_vel,y_vel) so here it's polar -> cartesian
      //However in the Measurement Update step, the radar measurement is coming in as polar,
      //so for the state X we need to convert from cartesian -> polar to compare it with z (polar sensor measurement),
      // and from cartesian -> polar you need to use Hj jacobian matrix
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rho_dot = measurement_pack.raw_measurements_(2);
      //convert polar -> cartesian
      ekf_.x_(0) = rho * cos(phi); //x position
      ekf_.x_(1) = rho * sin(phi); //y position
      //while it is possible to calculate the vx and vy using rho_dot and phi,
      //a radar measurement does not contain enough information (so not advised)


    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      //initial velocity defaulted to 1
    }

    //for tracking time
    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.

   NOTE: Because we are using a linear model for predict, there is no need for Fj (F jacobian matrix) or f(x,u) function
   Basically the only matrices we need to set are F and Q
   Instructions for how they look can be found in notes/videos
   */

  //Set F
  
  //by default time is in microseconds (so if you divide by 1 million you will get the time in seconds)
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  ekf_.F_(0, 0) = 1;
  ekf_.F_(1, 1) = 1;
  ekf_.F_(2, 2) = 1;
  ekf_.F_(3, 3) = 1;

  //Set Q
  float noise_ax = 9.0;
  float noise_ay = 9.0;
  float Q0 = (pow(dt, 4) / 4);
  float Q1 = (pow(dt, 3) / 2);
  float Q2 = pow(dt, 2);
  ekf_.Q_ << Q0*noise_ax, 0, Q1*noise_ax, 0,
        0, Q0*noise_ay, 0, Q1*noise_ay,
        Q1*noise_ax, 0, Q2*noise_ax, 0,
        0, Q1*noise_ay, 0, Q2*noise_ay;



  ekf_.Predict();

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  //Laser will call normal Update
  //Radar will call UpdateEKF will convert cartesian -> polar through h(x')
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Tools tools;
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    //measurement_pack.raw_measurements_ is z vector (polar coordinates)
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    //No need for UpdateEKF since we are working with Laser
    //measurement_pack.raw-measurements_ is z vector (cartesian coordinates)
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
