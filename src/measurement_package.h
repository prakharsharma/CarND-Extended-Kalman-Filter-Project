#ifndef EXTENDED_KF_MEASUREMENT_PACKAGE_H_
#define EXTENDED_KF_MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};

#endif /* EXTENDED_KF_MEASUREMENT_PACKAGE_H_ */
