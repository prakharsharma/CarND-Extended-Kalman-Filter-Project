#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

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

  noise_ax = 9;
  noise_ay = 9;

  ekf_.x_ = VectorXd(4);
  ekf_.x_ << 0, 0, 0, 0;

  ekf_.H_laser_ = MatrixXd(2, 4);
  ekf_.H_laser_ << 1, 0, 0, 0,
      0, 1, 0, 0;

  ekf_.Hj_ = MatrixXd(3, 4);
  ekf_.Hj_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;

  ekf_.R_laser_ = MatrixXd(2, 2);
  ekf_.R_laser_ << 0.0225, 0,
      0, 0.0225;

  ekf_.R_radar_ = MatrixXd(3, 3);
  ekf_.R_radar_ << 0.09, 0, 0,
      0, 0.0009, 0,
      0, 0, 0.09;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  cout << "######################################" << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "Radar measurement" << endl;
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    cout << "Laser measurement" << endl;
  }

  if (!is_initialized_) {
    ProcessFirstMeasurement(measurement_pack);
    is_initialized_ = true;
  } else {
    ProcessSubsequentMeasurement(measurement_pack);
  }

  previous_timestamp_ = measurement_pack.timestamp_;

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  cout << endl;
}

void FusionEKF::ProcessFirstMeasurement(
    const MeasurementPackage &measurement_pack) {

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    double range = measurement_pack.raw_measurements_(0);
    double bearing = measurement_pack.raw_measurements_(1);
    double range_rate = measurement_pack.raw_measurements_(2);
    double cosine = cos(bearing);
    double sine = sin(bearing);

    ekf_.x_ << range * cosine, range * sine,
        range_rate * cosine, range_rate * sine;
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.x_ << measurement_pack.raw_measurements_(0),
        measurement_pack.raw_measurements_(1), 0, 0;
  }
}

void FusionEKF::ProcessSubsequentMeasurement(
    const MeasurementPackage &measurement_pack) {
  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt;
  double dt4 = dt3 * dt;

  //1. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  //2. Set the process covariance matrix Q
  ekf_.Q_ << dt4 * noise_ax/4, 0, dt3 * noise_ax/2, 0,
      0, dt4 * noise_ay/4, 0, dt3 * noise_ay/2,
      dt3 * noise_ax/2, 0, dt2 * noise_ax, 0,
      0, dt3 * noise_ay/2, 0, dt2 * noise_ay;

  ekf_.Predict();
//  cout << "done with prediction update" << endl;

  /*****************************************************************************
   *  Update
   ****************************************************************************/

//  cout << "measurement: " << measurement_pack.raw_measurements_ << endl;

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    ekf_.Update(measurement_pack.raw_measurements_);
  }
//  cout << "done with measurement update" << endl;
}
