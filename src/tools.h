#ifndef EXTENDED_KF_TOOLS_H_
#define EXTENDED_KF_TOOLS_H_

#include <vector>
#include "Eigen/Dense"

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  Eigen::VectorXd CalculateRMSE(
      const std::vector<Eigen::VectorXd> &estimations,
      const std::vector<Eigen::VectorXd> &ground_truth
  );

private:
  /**
   * Copy constructor
   */
  Tools(const Tools&);

  /**
   * Copy operator
   */
  Tools& operator=(const Tools&);
};

#endif /* EXTENDED_KF_TOOLS_H_ */
