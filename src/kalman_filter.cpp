#include <iostream>
#include "kalman_filter.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_laser_ * x_;
//  cout << "predicted measurement: " << z_pred << endl;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_laser_.transpose();
  MatrixXd S = H_laser_ * P_ * Ht + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double c1 = px * px + py *py;
  double c2 = sqrt(c1);
  double c3 = (c1 * c2);

  if (fabs(c1) < 0.00001) {
    cerr << "Avoiding division by zero, skipping radar "
        "measurement" << endl;
    return;
  }

  double range = sqrt(c1);
  double bearing = atan2(py, px);
  double range_rate = (px * vx + py * vy)/range;
//  cout << "range: " << range << ", bearing: " << bearing
//       << ", range_rate: " << range_rate << endl;

  VectorXd z_pred(3);
  z_pred << range, bearing, range_rate;
//  cout << "predicted measurement: " << z_pred << endl;

  VectorXd y = z - z_pred;

  Hj_ << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
//  cout << "Updated Jacobian: " << Hj_ << endl;

  MatrixXd Hjt = Hj_.transpose();

  MatrixXd S = Hj_ * P_ * Hjt + R_radar_;
  MatrixXd Si = S.inverse();

  MatrixXd K = P_ * Hjt * Si;

  // new estimate
  x_ = x_ + K * y;

  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_= (I - K * Hj_) * P_;
}
