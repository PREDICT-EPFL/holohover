#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
namespace calibration
{
class ForceTorqueCalibration
{
public:
  /*!
   * Initialize calibration process.
   */
  ForceTorqueCalibration();

  ~ForceTorqueCalibration();

  /*!
   * Add measurements to the calibration process as one data point for the LS estimate.
   * @param acc           IMU reading of the EE linear acceleration in the EE frame as Eigen::Vector3d
   * @param gravity       Direction of gravity [0, 0, -9.81] expressed in the EE frame as Eigen::Vector3d
   * @param angVel        IMU reading of the EE angular velocity in the EE frame as Eigen::Vector3d
   * @param angAcc        Angular acceleration of the EE expressed in the EE frame as Eigen::Vector3d
   * @param ftRaw         F/T sensor readings in EE frame as Eigen::VectorXd (size must be equal 6)
   */
  void addMeasurement(const Eigen::Vector3d& acc, const Eigen::Vector3d& gravity, Eigen::Vector3d angVel,
                      Eigen::Vector3d angAcc, const Eigen::VectorXd& ftRaw);

  /*!
   * Execute LS estimation based on the data points of the measurement matrix and F/T sensor readings.
   * Method to apply the Least Squares Algorithm to estimate the calibration parameters:
   * the load's mass, its center of mass(multiplied by the mass to ensure linearity) and the internal force
   * and torque sensor offsets.
   * @return          Vector of estimated calibration parameters as Eigen::VectorXd of size 10
   */
  Eigen::VectorXd getCalibParams();

  /*!
   * Reset all gathered data.
   */
  void resetCalibration();

protected:
  /*!
   * Creates a date point which gets added to the measurement matrix for the LS estimate.
   * @param acc           Linear acceleration passed to addMeasurement()
   * @param gravity       Direction of gravity passed to addMeasurement()
   * @param angVel        Angular velocity passed to addMeasurement()
   * @param angAcc        Angular acceleration passed to addMeasurement()
   * @return          Data point for measurement matrix for the LS estimate as a Eigen::MatrixXd of size (6, 10)
   */
  Eigen::MatrixXd createMeasurementMat(const Eigen::Vector3d& acc, const Eigen::Vector3d& gravity,
                                       Eigen::Vector3d angVel, Eigen::Vector3d angAcc);

  /*!
   * Creates a skew matrix as an equivalent to a cross product.
   * @param inVec         Vector of which the skew matrix is created as Eigen::Vector3d
   * @return          Skew matrix as a Eigen::Matrix3d
   */
  Eigen::Matrix3d skewMatrix(Eigen::Vector3d inVec);

  Eigen::VectorXd ftReadings_;
  Eigen::MatrixXd measurementMat_;
  int numMeasurements_;
};
}  // namespace calibration
}  // namespace rokubimini
