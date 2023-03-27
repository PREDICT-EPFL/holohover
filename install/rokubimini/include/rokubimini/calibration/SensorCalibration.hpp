#pragma once

#include <eigen3/Eigen/Core>
#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
namespace calibration
{
/**
 * @class SensorCalibration
 *
 * @brief The SensorCalibration class is used for holding the calibration information of each rokubi mini sensor.
 *
 */
class SensorCalibration
{
public:
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;
  /**
   * @fn SensorCalibration()
   *
   * @brief Default constructor.
   *
   * This method constructs a \a SensorCalibration.
   *
   */
  SensorCalibration() = default;
  ~SensorCalibration() = default;

  /**
   * @fn bool load(const std::string& key, NodeHandlePtr nh)
   * @brief Loads the calibrations parameters
   *
   * @param key The key to search for the parameter.
   * @param nh The ROS Node to access the parameter.
   * @return True if the calibrations were loaded successfully.
   */
  bool load(const std::string& key, const NodeHandlePtr& nh);

  /**
   * @fn const uint32_t getPassPhrase() const
   * @brief Gets the passphrase.
   *
   * @return The passphrase.
   */

  const uint32_t getPassPhrase() const
  {
    return passphrase_;
  }

  /**
   * @fn void setPassPhrase(const uint32_t passphrase)
   * @brief Sets the passphrase.
   *
   * @param passphrase The passphrase to be set.
   */

  void setPassPhrase(const uint32_t passphrase)
  {
    passphrase_ = passphrase;
  }

  /**
   * @fn const Eigen::Matrix<double, 6, 6> &getCalibrationMatrix() const
   * @brief Gets the calibration matrix.
   *
   * @return The calibration matrix.
   */

  const Eigen::Matrix<double, 6, 6>& getCalibrationMatrix() const
  {
    return calibrationMatrix_;
  }

  /**
   * @fn Eigen::Matrix<double, 6, 6> &getCalibrationMatrix()
   * @brief Non-const version of getCalibrationMatrix() const. Gets the calibration matrix.
   *
   * @return The calibration matrix.
   */

  Eigen::Matrix<double, 6, 6>& getCalibrationMatrix()
  {
    return calibrationMatrix_;
  }

  /**
   * @fn void setCalibrationMatrix(const Eigen::Matrix<double, 6, 6> &calibrationMatrix)
   * @brief Sets the calibration matrix.
   *
   * @param calibrationMatrix The calibration matrix.
   */

  void setCalibrationMatrix(const Eigen::Matrix<double, 6, 6>& calibrationMatrix)
  {
    calibrationMatrix_ = calibrationMatrix;
  }

private:
  /**
   * @var uint32_t passphrase_
   * @brief The passphrase.
   */
  uint32_t passphrase_{ 0x87654321 };

  /**
   * @var Eigen::Matrix<double, 6, 6> calibrationMatrix_
   * @brief The calibration matrix.
   */
  Eigen::Matrix<double, 6, 6> calibrationMatrix_;

  /**
   * @var double calibrationTemperature_
   * @brief The calibration temperature.
   *
   * @todo Not used so far.
   */
  // double calibrationTemperature_{ 0.0 };

  /**
   * @var Eigen::Matrix<double, 6, 1> calibrationOffset_
   * @brief The calibration offset matrix.
   */
  Eigen::Matrix<double, 6, 1> calibrationOffset_;

  /**
   * @var uint8_t adcRange_
   * @brief The ADC range.
   *
   * @todo Not used so far.
   */
  // uint8_t adcRange_{ 0 };

  /**
   * @var Eigen::Matrix<double, 2, 1> thermistorCalibration_
   * @brief The thermistor calibration matrix.
   */
  Eigen::Matrix<double, 2, 1> thermistorCalibration_;

  /**
   * @var Eigen::Matrix<double, 6, 1> temperatureGain_
   * @brief The temperature gain matrix.
   */

  Eigen::Matrix<double, 6, 1> temperatureGain_;

  /**
   * @var Eigen::Matrix<double, 6, 1> accelerationOffset_
   * @brief The matrix for the acceleration.
   *
   */

  Eigen::Matrix<double, 6, 1> accelerationOffset_;

  /**
   * @var Eigen::Matrix<double, 6, 1> angularRateOffset_
   * @brief The offset matrix for the angular rate.
   *
   */

  Eigen::Matrix<double, 6, 1> angularRateOffset_;

  /**
   * @var Eigen::Matrix<double, 10, 1> inertia_
   * @brief The inertia matrix.
   *
   */

  Eigen::Matrix<double, 10, 1> inertia_;

  /**
   * @var uint32_t calibrationDate_
   * @brief Last date of calibration.
   *
   */

  // uint32_t calibrationDate_{ 0 };

  /**
   * @var uint8_t serialNumber_
   * @brief The serial number of the sensor.
   *
   * @todo Not used so far.
   */

  // uint8_t serialNumber_{ 0 };

  /**
   * @var uint8_t firmwareVersion_
   * @brief The version of the firmware of the sensor.
   *
   * @todo Not used so far.
   */
  // uint8_t firmwareVersion_{ 0 };
};

}  // namespace calibration
}  // namespace rokubimini
