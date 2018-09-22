#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd rmse = VectorXd(4);

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	VectorXd rmse(4);

	rmse << 0,0,0,0;


	// check the validity of the inputs:

	if((estimations.empty() != true) && (estimations.size() == ground_truth.size()))
	{
		//accumulate squared residuals

		for(int i=0; i < estimations.size(); ++i){

			VectorXd residual = estimations[i] - ground_truth[i];
			residual =  residual.array() * residual.array();
			rmse += residual;

		}

		//calculate the mean

		rmse = rmse.array()* (1.0f/estimations.size());

		//calculate the squared root

		rmse = rmse.array().sqrt();

	}

	//return the result
	return rmse;

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3,4);

	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);


	//check division by zero
	if((px == 0) || (py == 0))
	{
		cout<<"Error dividing by zero"<<std::endl;
	}
	else
	{
		Hj(0,0) = px/(sqrt(pow(px,2) + pow(py,2)));
		Hj(0,1) = py/(sqrt(pow(px,2) + pow(py,2)));
		Hj(0,2) = 0;
		Hj(0,3) = 0;

		Hj(1,0) = (py/(pow(px,2) + pow(py,2))) * -1;
		Hj(1,1) =  px/(pow(px,2) + pow(py,2));
		Hj(1,2) = 0;
		Hj(1,3) = 0;

		float temp = ((vx * py) - (vy * px))/pow((pow(px,2)+ pow(py,2)),(3/2));
		Hj(2,0) = temp * py;
		Hj(2,1) = temp * px;
		Hj(2,2) = Hj(0,0);
		Hj(2,3) = Hj(0,1);

	}

	//compute the Jacobian matrix

	return Hj;
}
