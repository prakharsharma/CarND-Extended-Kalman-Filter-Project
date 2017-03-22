#ifndef EXTENDED_KF_KALMAN_FILTER_H_
#define EXTENDED_KF_KALMAN_FILTER_H_
#include "Eigen/Dense"

class KalmanFilter {
public:

  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;

  // state transistion matrix
  Eigen::MatrixXd F_;

  // process covariance matrix
  Eigen::MatrixXd Q_;

  // laser measurement matrix
  Eigen::MatrixXd H_laser_;

  // radar measurement matrix
  Eigen::MatrixXd Hj_;

  // laser measurement covariance matrix
  Eigen::MatrixXd R_laser_;

  // radar measurement covariance matrix
  Eigen::MatrixXd R_radar_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const Eigen::VectorXd &z);

private:
  /**
   * Copy constructor
   */
  KalmanFilter(const KalmanFilter&);

  /**
   * Copy operator
   */
  KalmanFilter& operator=(const KalmanFilter&);
};

#endif /* EXTENDED_KF_KALMAN_FILTER_H_ */
