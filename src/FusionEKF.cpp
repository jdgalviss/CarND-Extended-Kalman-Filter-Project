#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

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

  H_laser_ << 1, 0, 0, 0,
            0, 1, 0, 0;

  Hj_ << 0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;

  // the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

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
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.P_ = MatrixXd(4,4);
    //ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      ekf_.x_ << measurement_pack.raw_measurements_[0]*cos(measurement_pack.raw_measurements_[1]),
                -measurement_pack.raw_measurements_[0]*sin(measurement_pack.raw_measurements_[1]),
                0,
                0;

      ekf_.P_ << 10, 0, 0, 0,
                  0, 10, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0],
                  measurement_pack.raw_measurements_[1],
                  0,
                  0;

      ekf_.P_ << 10, 0, 0, 0,
                  0, 10, 0, 0,
                  0, 0, 1000, 0,
                  0, 0, 0, 1000;

    }

    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */
  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;


  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_ << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
  // 2. Set the process covariance matrix Q
  MatrixXd Qv = MatrixXd(2, 2);
  Qv << 9, 0,
       0, 9;
       
  MatrixXd G = MatrixXd(4, 2);
  G << dt*dt/2, 0,
       0, dt*dt/2,
       dt, 0,
       0, dt;

  ekf_.Q_ = G*Qv*G.transpose();

  ekf_.Predict();

  /**
   * Update
   */

  /**
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    //Radar updates
    
    VectorXd measurements(3);
    measurements << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], measurement_pack.raw_measurements_[2];
    ekf_.H_ = MatrixXd(3, 4);
    ekf_.H_  = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = MatrixXd(3, 3);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurements);
  
  } else {
    // Laser updates
    VectorXd measurements(2);
    measurements << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1];
    ekf_.H_ = MatrixXd(2, 4);
    ekf_.H_  = H_laser_;
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.R_ = R_laser_;
    ekf_.Update(measurements);
    
  }
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
