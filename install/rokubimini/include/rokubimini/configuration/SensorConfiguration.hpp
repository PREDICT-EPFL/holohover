#pragma once

#include <cstdint>
#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
namespace configuration
{
/**
 * @class SensorConfiguration
 *
 * @brief Class holding the sensor configuration settings.
 *
 */
class SensorConfiguration
{
public:
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;
  /**
   * @fn SensorConfiguration()
   *
   * @brief Default constructor.
   *
   */
  SensorConfiguration() = default;

  /**
   * @fn SensorConfiguration(const uint8_t calibrationMatrixActive, const uint8_t temperatureCompensationActive, const
   uint8_t imuActive,
                      const uint8_t coordinateSystemConfigurationActive, const uint8_t inertiaCompensationActive,
                      const uint8_t orientationEstimationActive)
   *
   * @brief Custom constructor which accepts custom settings and creates the object.
   *
   * @param calibrationMatrixActive Flag indicating if the calibration matrix is active.
   * @param temperatureCompensationActive Flag indicating if the temperature compensation is active.
   * @param imuActive Type of IMU activated.
   * @param coordinateSystemConfigurationActive  Flag indicating if the coordinate system configuration is active.
   * @param inertiaCompensationActive The inertia compensation.
   * @param orientationEstimationActive The orientation estimation.
  */
  SensorConfiguration(const uint8_t calibrationMatrixActive, const uint8_t temperatureCompensationActive,
                      const uint8_t imuActive, const uint8_t coordinateSystemConfigurationActive,
                      const uint8_t inertiaCompensationActive, const uint8_t orientationEstimationActive);

  ~SensorConfiguration() = default;

  /**
   * @fn bool load(const std::string& key, NodeHandlePtr nh)
   *
   * @brief Loads the sensor configuration from the parameters
   *
   * @param key The key to search for the parameters.
   * @param nh The ROS Node to access the parameters.
   * @return True if the sensor configuration was loaded successfully.
   */
  bool load(const std::string& key, const NodeHandlePtr& nh);

  /**
   * @fn uint8_t getCalibrationMatrixActive() const
   *
   * @brief Gets the \a calibrationMatrixActive variable.
   * @return The \a calibrationMatrixActive value.
   *
   */
  uint8_t getCalibrationMatrixActive() const
  {
    return calibrationMatrixActive_;
  }

  /**
   * @fn void setCalibrationMatrixActive(const uint8_t calibrationMatrixActive)
   *
   * @brief Sets the \a calibrationMatrixActive variable.
   * @param calibrationMatrixActive The value to set.
   *
   */
  void setCalibrationMatrixActive(const uint8_t calibrationMatrixActive)
  {
    calibrationMatrixActive_ = calibrationMatrixActive;
  }

  /**
   * @fn uint8_t getTemperatureCompensationActive() const
   *
   * @brief Gets the \a temperatureCompensationActive variable.
   * @return The \a temperatureCompensationActive value.
   *
   */
  uint8_t getTemperatureCompensationActive() const
  {
    return temperatureCompensationActive_;
  }

  /**
   * @fn void setTemperatureCompensationActive(const uint8_t temperatureCompensationActive)
   *
   * @brief Sets the \a temperatureCompensationActive variable.
   * @param temperatureCompensationActive The value to set.
   *
   */
  void setTemperatureCompensationActive(const uint8_t temperatureCompensationActive)
  {
    temperatureCompensationActive_ = temperatureCompensationActive;
  }

  /**
   * @fn uint8_t getImuActive() const
   *
   * @brief Gets the \a imuActive variable.
   * @return The \a imuActive value.
   *
   */
  uint8_t getImuActive() const
  {
    return imuActive_;
  }

  /**
   * @fn void setImuActive(const uint8_t imuActive)
   *
   * @brief Sets the \a imuActive variable.
   * @param imuActive The value to set.
   *
   */
  void setImuActive(const uint8_t imuActive)
  {
    imuActive_ = imuActive;
  }

  /**
   * @fn uint8_t getCoordinateSystemConfigurationActive() const
   *
   * @brief Gets the \a coordinateSystemConfigurationActive variable.
   * @return The \a coordinateSystemConfigurationActive value.
   *
   */
  uint8_t getCoordinateSystemConfigurationActive() const
  {
    return coordinateSystemConfigurationActive_;
  }

  /**
   * @fn void setCoordinateSystemConfigurationActive(const uint8_t coordinateSystemConfigurationActive)
   *
   * @brief Sets the \a coordinateSystemConfigurationActive variable.
   * @param coordinateSystemConfigurationActive The value to set.
   *
   */
  void setCoordinateSystemConfigurationActive(const uint8_t coordinateSystemConfigurationActive)
  {
    coordinateSystemConfigurationActive_ = coordinateSystemConfigurationActive;
  }

  /**
   * @fn uint8_t getInertiaCompensationActive() const
   *
   * @brief Gets the \a inertiaCompensationActive variable.
   * @return The \a inertiaCompensationActive value.
   *
   */
  uint8_t getInertiaCompensationActive() const
  {
    return inertiaCompensationActive_;
  }

  /**
   * @fn void setInertiaCompensationActive(const uint8_t inertiaCompensationActive)
   *
   * @brief Sets the \a inertiaCompensationActive variable.
   * @param inertiaCompensationActive The value to set.
   *
   */
  void setInertiaCompensationActive(const uint8_t inertiaCompensationActive)
  {
    inertiaCompensationActive_ = inertiaCompensationActive;
  }

  /**
   * @fn uint8_t getOrientationEstimationActive() const
   *
   * @brief Gets the \a orientationEstimationActive variable.
   * @return The \a orientationEstimationActive value.
   *
   */
  uint8_t getOrientationEstimationActive() const
  {
    return orientationEstimationActive_;
  }

  /**
   * @fn void setOrientationEstimationActive(const uint8_t orientationEstimationActive)
   *
   * @brief Sets the \a orientationEstimationActive variable.
   * @param orientationEstimationActive The value to set.
   *
   */
  void setOrientationEstimationActive(const uint8_t orientationEstimationActive)
  {
    orientationEstimationActive_ = orientationEstimationActive;
  }

  /**
   * @fn void print() const
   *
   * @brief Prints the existing sensor configuration settings.
   *
   */
  void print() const;

private:
  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
   */
  uint8_t calibrationMatrixActive_;

  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
   */
  uint8_t temperatureCompensationActive_;

  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
   */
  uint8_t imuActive_;

  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
   */
  uint8_t coordinateSystemConfigurationActive_;

  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
   */
  uint8_t inertiaCompensationActive_;

  /**
   * @var uint16_t sincFilterSize_
   *
   * @brief The sincFilterSize variable.
   *
   */
  uint8_t orientationEstimationActive_;
};

}  // namespace configuration
}  // namespace rokubimini
