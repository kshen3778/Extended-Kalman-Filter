#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using namespace std;

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

   for(unsigned int i=0; i<estimations.size(); i++){
     //get vector difference
     VectorXd diff = estimations[i] - ground_truth[i];

     //element wise multiplication for square
     diff = diff.array() * diff.array();

     rmse += diff;
   }

   rmse = rmse / estimations.size(); //mean
   rmse = rmse.array().sqrt();
   return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */

   float px = x_state(0);
   float py = x_state(1);
   float vx = x_state(2);
   float vy = x_state(3);

   MatrixXd Hj(3,4);

   //calculate Hj based on instructions in the lesson
   //define helper variables to aboid repeat calculation
   float n1 = px*px+py*py;
   float n2 = sqrt(n1);
   float n3 = (n1*n2);
   //check if n1 is almost zero (avoid division by zero event)
   //can also check if px and py is 0
   if(fabs(n1) < 0.0001){
     cout << "Error: Division by Zero in CalculateJacobian()" << endl;
     return Hj;
   }

   Hj << (px/n2), (py/n2), 0, 0,
          -(py/n1), (px/n1), 0, 0,
          py*(vx*py - vy*px)/n3, px*(px*vy - py*vx)/n3, px/n2, py/n2;

   return Hj;

}
