#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */

   VectorXd rmse(4);
   rmse << 0,0,0,0;

   // check the validity of the following inputs:
   //* the estimation vector size should not be zero
   //* the estimation vector size should equal ground truth vector size
   if(estimations.size() == 0 || (estimations.size() != ground_truth.size())){
     cout << "Invalid size for estimation or ground truth vectors" << endl;
     return rmse;
   }

   for(int i=0; i<estimations.size(); i++){
     //get vector difference
     VectorXd diff = estimations[i] - ground_truth[i];

     //element wise multiplication for square
     diff = diff.array() * diff.array();

     rmse += diff;
   }

   rmse /= estimations.size(); //mean
   rmse = rmse.array().sqrt();
   return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   
}
