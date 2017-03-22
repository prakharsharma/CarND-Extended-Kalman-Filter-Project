#ifndef EXTENDED_KF_FusionEKF_H_
#define EXTENDED_KF_FusionEKF_H_

#include "measurement_package.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
  /**
  * Constructor.
  */
  FusionEKF();

  /**
  * Destructor.
  */
  virtual ~FusionEKF();

  /**
  * Run the whole flow of the Kalman Filter from here.
  */
  void ProcessMeasurement(const MeasurementPackage &measurement_pack);

  /**
  * Kalman Filter update and prediction math lives in here.
  */
  KalmanFilter ekf_;

private:
  // check whether the tracking toolbox was initiallized or not (first measurement)
  bool is_initialized_;

  // previous timestamp
  long previous_timestamp_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

  float noise_ax;
  float noise_ay;

  /**
   * Copy constructor
   */
  FusionEKF(const FusionEKF&);

  /**
   * Copy operator
   */
  FusionEKF& operator=(const FusionEKF&);
};

#endif /* EXTENDED_KF_FusionEKF_H_ */
