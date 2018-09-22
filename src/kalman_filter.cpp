#include "kalman_filter.h"
#include <iostream>
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
	// map state vector to measurement space
	VectorXd z_predection = H_ * x_ ;

	// calculate Y "error " vector

	VectorXd y = z - z_predection ;

	MatrixXd Ht =  H_.transpose();

	//std::cout << "kalman gain done "<<std::endl;
	//calculate S matrix
	MatrixXd s = H_ * P_ * Ht + R_ ;

	MatrixXd Pht = P_ * Ht ;

	MatrixXd Si = s.inverse();
	// Kalman gain
	MatrixXd k =  Pht * Si;
	x_ = x_ + (k * y) ;

	MatrixXd I = MatrixXd::Identity(4, 4);

	P_ = (I - k * H_) * P_;


}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

	VectorXd z_predection(3);


	//calc. z_predection using h(x)

	//1. calc. the rogh "range"
	z_predection(0) = sqrt(pow((double)x_(0),2) + pow((double)x_(1),2));

	//2. calc. the bearing angle, No need for normalization as the atan2 returns angle in ragne from - pi to pi
	z_predection(1) = atan2((double)x_(1),(double)x_(0));

	//3. calc. the rate range "rogh dot "
	z_predection(2) = ( ( x_(0) * x_(2) ) + ( x_(1) * x_(3) ) )/ z_predection(0);

	// calculate Y "error " vector

	VectorXd y = z - z_predection ;

	MatrixXd Ht =  H_.transpose();

	//std::cout << "kalman gain done "<<std::endl;
	//calculate S matrix
	MatrixXd s = H_ * P_ * Ht + R_ ;

	MatrixXd Pht = P_ * Ht ;

	MatrixXd Si = s.inverse();
	// Kalman gain
	MatrixXd k =  Pht * Si;
	x_ = x_ + (k * y) ;

	MatrixXd I = MatrixXd::Identity(4, 4);

	P_ = (I - k * H_) * P_;



}
