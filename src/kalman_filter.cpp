#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Resources:

// Lesson 24: Extended Kalman Filters
// Concept 7: Kalman Fileter Equations in C++
// https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/1c4c9856-381c-41b8-bc40-2c2fd729d84e/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/f2081cbe-559a-4671-a8b4-04452531b875
// Notes: A lot of the code is implemented in this quiz for 1 sensor

// Lesson 24: Extended Kalman Filters
// Concept 7: Laser Measurements Part 3
// https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/1c4c9856-381c-41b8-bc40-2c2fd729d84e/lessons/ec3054b9-9ffc-45c4-8523-485e2f7022da/concepts/f1cef488-6a2d-484d-a30d-8caa4fd082fd
// Notes: Predict and Update are implemented here***

// Lesson 21: Kalman Filters
// Concept 25: Quiz: Kalman Matrices
// https://classroom.udacity.com/nanodegrees/nd013/parts/168c60f1-cc92-450a-a91b-e427c326e6a7/modules/1c4c9856-381c-41b8-bc40-2c2fd729d84e/lessons/4a76ef9b-27d7-4aa4-8f3c-fd2b1e8757a4/concepts/487244950923
// Notes: Explination of Kalman Filter implementation in Python

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; // estimate
  P_ = P_in; // covariance uncertainty
  F_ = F_in; // state transition function
  H_ = H_in; // measurement function
  R_ = R_in;  // measurement noise
  Q_ = Q_in;  // process covariance matrix
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
   

  // prediction
  x_ = (F_ * x_); //+ u; // NOTE: Don't have u (external motion vector)
  P_ = F_ * P_ * F_.transpose() + Q_; // NOTE: Added Q as per formula sheet
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */


  // Process error
  MatrixXd y = z - H_ * x_;
	
	// System error
	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	
	// Kalman Gian
	MatrixXd K = P_ * H_.transpose() * S.inverse();


  // KF Measurement update step
  // Update state
  x_ = x_ + (K * y);
      
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */


  // Calculate z vector (same as h(x'))
  float px =  x_[0];
  float py =  x_[1];
  float vx =  x_[2];
  float vy =  x_[3];

  float rho = sqrt(pow(px,2) + pow(py,2));
  float phi = atan2(py,px);

  float rho_dot;
  if (fabs(rho) < 0.00000001){
    rho_dot = 0;
  }
  else{
    rho_dot = ((px * vx) + (py * vy)) / rho;
  }


  VectorXd hx(3);
  hx << rho, phi, rho_dot;


  // Process error
  VectorXd y = z - hx;

  //Normalizing Angles between -Pi and Pi
  
  while(y(1) > M_PI)
  {
    y(1) -= 2 * M_PI;
  }
  while(y(1) < -M_PI)
  {
    y(1) += 2 * M_PI;
  }

  // System error
	MatrixXd S = H_ * P_ * H_.transpose() + R_;

  // Kalman Gian
	MatrixXd K = P_ * H_.transpose() * S.inverse();

  // Update state
  x_ = x_ + (K * y);
      
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;


}
