#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  //initialize H laser matrix, for initializing the H_Radar or H_j it will be done based on the
  // first measurment rfusionEKFeceived.

  H_laser_ << 1,0,0,0,
		      0,1,0,0;

  //initialize state Covariance matrix P

  ekf_.P_ = MatrixXd(4, 4);

  ekf_.P_  << 1,0,0,0,
		  	  0,1,0,0,
			  0,0,1000,0,
			  0,0,0,1000;

  // create process covariance matrix, No need to init it as it will be calculated before prediction step
  ekf_.Q_ = MatrixXd(4,4);

  // create and initialize state transition matrix
  ekf_.F_ = MatrixXd(4,4);

  ekf_.F_ << 1,0,1,0,
		  	 0,1,0,1,
			 0,0,1,0,
			 0,0,0,1;
  //init. acceleration noise in x and y
  noise_ax = 9 ;

  noise_ay = 9;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}


bool skip = false;

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   **************************Q_**************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    //Initialize the previous time stamp with the first received measurements time stamp.
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */

    	// initialize Px from the radar measurement; roh *  cos bearing angle

    	ekf_.x_(0) = measurement_pack.raw_measurements_(0) * cos((float) measurement_pack.raw_measurements_(1));

    	// initialize Py from the radar measurement; roh * sin of bearing angle

    	ekf_.x_(1) =  measurement_pack.raw_measurements_(0) * sin((float) measurement_pack.raw_measurements_(1));



    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */

    	// initialize Px from the laser measurement Px
    	ekf_.x_(0) = measurement_pack.raw_measurements_(0);

    	ekf_.x_(1) = measurement_pack.raw_measurements_(1);

    }

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

	//compute the time elapsed between the current and previous measurements
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds

	//update the previous time stamp
	previous_timestamp_ = measurement_pack.timestamp_;

	//update the state transition after dt
	ekf_.F_(0,2) = dt;
	ekf_.F_(1,3) = dt;

	//calculate Q Matrix
	ekf_.Q_ <<  ((pow(dt,4)/4)*noise_ax)  , 0                         ,((pow(dt,3)/2) * noise_ax) , 0                         ,
			    0                         , ((pow(dt,4)/4)*noise_ay)  , 0                         , ((pow(dt,3)/2) * noise_ay),
			    ((pow(dt,3)/2) * noise_ax), 0                         , (pow(dt,2)*noise_ax)      ,  0                        ,
			    0                         , ((pow(dt,3)/2) * noise_ay), 0                         , (pow(dt,2)*noise_ay)      ;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR && !skip) {
    // Radar updates

	  // check first if Px or Py = 0 , If so discard this measurement packet
	  if((ekf_.x_(0) == 0 )|| (ekf_.x_(1) == 0) )
	  {
		  //do nothing, discard this measurement packet to avoid dividing by zero in Jacobian calculation
	  }
	  else
	  {
		  // calculate Jacobian matrix
		  Hj_ = tools.CalculateJacobian(ekf_.x_);

		  // Assign Hj to ekf H
		  ekf_.H_ = Hj_ ;

		  // Assign R radar to ekf R
		  ekf_.R_ = R_radar_ ;

		  // call update extended
		  ekf_.UpdateEKF(measurement_pack.raw_measurements_);

	  }

  }
  else if(measurement_pack.sensor_type_ == MeasurementPackage::LASER)
  {
	  // Laser updates

	  //Assign H_laser to ekf H
	  ekf_.H_ = H_laser_;

	  //Assign R laser to ekf R
	  ekf_.R_ = R_laser_;

	  //call update KF
	  ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
