#include <iostream>
#include "Eigen/Dense"
#include <vector>

using namespace std;
using namespace Eigen;

//Kalman Filter variables
VectorXd x;	// object state //current location of the object  //position + velocity
MatrixXd P;	// object covariance matrix -where we predict the object to be //new state and covariance P
VectorXd u;	// external motion //
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix //convert system state estimate from state space to measurement space //set up to reduce the state estimate and error covariance to position only values rather than position and velocity.

MatrixXd R;	// measurement covariance matrix  //our measurement from the sensor
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix //process noise for system model

vector<VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);


int main() {
	
	//design the KF with 1D motion
	x = VectorXd(2);  //declaring for x
	x << 0, 0;

	P = MatrixXd(2, 2); //declaring for P
	P << 1000, 0, 0, 1000;

	u = VectorXd(2); //declaring for u with the type of matrice it is
	u << 0, 0;

	F = MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	H = MatrixXd(1, 2);
	H << 1, 0;

	R = MatrixXd(1, 1);
	R << 1;

	I = MatrixXd::Identity(2, 2);

	Q = MatrixXd(2, 2);
	Q << 0, 0, 0, 0;

	//create a list of measurements
	VectorXd single_meas(1);
	single_meas << 1;
	measurements.push_back(single_meas);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 3;
	measurements.push_back(single_meas);

	//call Kalman filter algorithm
	filter(x, P);

	return 0;

}


void filter(VectorXd &x, MatrixXd &P) {

	for (unsigned int n = 0; n < measurements.size(); ++n) {

		VectorXd z = measurements[n];
		
        //KF Measurement update step
		 
		VectorXd y = z - H * x;
		MatrixXd Ht = H.transpose();
		MatrixXd S = H * P * Ht + R;
		MatrixXd Si = S.inverse();
		MatrixXd K =  P * Ht * Si;

		//new state
		x = x + (K * y);
		P = (I - K * H) * P;

		/*
		 * KF Prediction step
		 */
		x = F * x + u;
		MatrixXd Ft = F.transpose();
		P = F * P * Ft + Q;

		std::cout << "x=" << std::endl <<  x << std::endl;
		std::cout << "P=" << std::endl <<  P << std::endl;


	}
}