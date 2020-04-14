#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
   //we don't have to worry about using f(x,u) or Fj because
   //we are using a linear model, so no need for EKF equations here

   x_ = F_ * x_;
   P_ = F_ * P_ * F_.transpose() + Q_;
}

//Laser
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */

   VectorXd y = z - (H_ * x_);
   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   //new estimate
   x_ = x_ + (K * y);
   P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;

}

//Radar
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */

   //The only different between this and normal KF Update is
   //You have to convert the state from cartesian -> polar (h(x') function)
   //z is a polar vector
   //when you do z - h(x'): h function turns state into polar
   float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
   float phi = atan2(x_(1), x_(0));
   float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;


   VectorXd hx(3);
   hx << rho, phi, rho_dot;
   VectorXd y = z - hx;
   MatrixXd S = H_ * P_ * H_.transpose() + R_;
   MatrixXd K = P_ * H_.transpose() * S.inverse();
   //new estimate
   x_ = x_ + (K * y);
   P_ = (MatrixXd::Identity(x_.size(), x_.size()) - K * H_) * P_;
}
