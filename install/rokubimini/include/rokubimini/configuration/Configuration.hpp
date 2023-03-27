#pragma once

#include <climits>
#include <mutex>
#include <string>

// #define EIGEN_INITIALIZE_MATRICES_BY_NAN

#include <rokubimini/calibration/SensorCalibration.hpp>
#include <rokubimini/configuration/SensorConfiguration.hpp>
#include <rokubimini/configuration/ForceTorqueFilter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>

namespace rokubimini
{
namespace configuration
{
/**
 * @class Configuration
 *
 * @brief Class holding the configuration of the sensor.
 *
 */
class Configuration
{
public:
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;
  /**
   * @fn Configuration()
   *
   * @brief Default constructor.
   *
   */
  Configuration() = default;

  /**
   * @fn Configuration(const Configuration &other)
   *
   * @brief Copy constructor.
   * @param other The other configuration to copy from.
   */
  Configuration(const Configuration& other);
  virtual ~Configuration() = default;

  /**
   * @fn Configuration &operator=(const Configuration &other)
   *
   * @brief Assignment operator for Configuration.
   *
   */
  Configuration& operator=(const Configuration& other);

  /**
   * @fn void load(const std::string& key, NodeHandlePtr nh)
   *
   * @brief Loads the configuration from the parameters
   *
   * @param key The key to search for the parameters.
   * @param nh The ROS Node to access the parameters.
   *
   */
  void load(const std::string& key, const NodeHandlePtr& nh);

  /**
   * @fn void setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect)
   *
   * @brief Sets the \a setReadingToNanOnDisconnect variable.
   * @param setReadingToNanOnDisconnect The value to set.
   *
   */
  void setSetReadingToNanOnDisconnect(const bool setReadingToNanOnDisconnect);

  /**
   * @fn bool getSetReadingToNanOnDisconnect() const
   *
   * @brief Gets the \a setReadingToNanOnDisconnect variable.
   * @return The \a setReadingToNanOnDisconnect value.
   *
   */

  bool getSetReadingToNanOnDisconnect() const;

  /**
   * @fn void setForceTorqueFilter(const ForceTorqueFilter &forceTorqueFilter)
   *
   * @brief Sets the \a forceTorqueFilter variable.
   * @param forceTorqueFilter The value to set.
   *
   */
  void setForceTorqueFilter(const ForceTorqueFilter& forceTorqueFilter);

  /**
   * @fn const ForceTorqueFilter &getForceTorqueFilter() const
   *
   * @brief Gets the \a forceTorqueFilter variable.
   * @return The value of \a forceTorqueFilter to get.
   *
   */
  const ForceTorqueFilter& getForceTorqueFilter() const;

  /**
   * @fn void setForceTorqueOffset(const Eigen::Matrix<double, 6, 1> &forceTorqueOffset)
   *
   * @brief Sets the \a forceTorqueOffset variable.
   * @param forceTorqueOffset The value to set.
   *
   */
  void setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset);

  /**
   * @fn const Eigen::Matrix<double, 6, 1> &getForceTorqueOffset() const
   *
   * @brief Gets the \a forceTorqueOffset variable.
   * @return The value of \a forceTorqueOffset to get.
   *
   */
  const Eigen::Matrix<double, 6, 1>& getForceTorqueOffset() const;

  /**
   * @fn void setUseCustomCalibration(const bool useCustomCalibration)
   *
   * @brief Sets the \a useCustomCalibration variable.
   * @param useCustomCalibration The value to set.
   *
   */
  void setUseCustomCalibration(const bool useCustomCalibration);

  /**
   * @fn bool getUseCustomCalibration() const
   *
   * @brief Gets the \a useCustomCalibration variable.
   * @return The value of \a useCustomCalibration to get.
   *
   */
  bool getUseCustomCalibration() const;

  /**
   * @fn void setSensorConfiguration(const SensorConfiguration &sensorConfiguration)
   *
   * @brief Sets the \a sensorConfiguration variable.
   * @param sensorConfiguration The value to set.
   *
   */
  void setSensorConfiguration(const SensorConfiguration& sensorConfiguration);

  /**
   * @fn const SensorConfiguration &getSensorConfiguration() const
   *
   * @brief Gets the \a sensorConfiguration variable.
   * @return The value of \a sensorConfiguration to get.
   *
   */
  const SensorConfiguration& getSensorConfiguration() const;

  /**
   * @fn void setSensorCalibration(const calibration::SensorCalibration &sensorCalibration)
   *
   * @brief Sets the \a sensorCalibration variable.
   * @param sensorCalibration The value to set.
   *
   */
  void setSensorCalibration(const calibration::SensorCalibration& sensorCalibration);

  /**
   * @fn const calibration::SensorCalibration &getSensorCalibration() const
   *
   * @brief Gets the \a sensorCalibration variable.
   * @return The value of \a sensorCalibration to get.
   *
   */
  const calibration::SensorCalibration& getSensorCalibration() const;

  /**
   * @fn void setImuAccelerationFilter(const unsigned int imuAccelerationFilter)
   *
   * @brief Sets the \a imuAccelerationFilter variable.
   * @param imuAccelerationFilter The value to set.
   *
   */
  void setImuAccelerationFilter(const unsigned int imuAccelerationFilter);

  /**
   * @fn unsigned int getImuAngularRateFilter() const
   *
   * @brief Gets the \a imuAccelerationFilter variable.
   * @return The value of \a imuAccelerationFilter to get.
   *
   */
  unsigned int getImuAccelerationFilter() const;

  /**
   * @fn void setImuAngularRateFilter(const unsigned int imuAngularRateFilter)
   *
   * @brief Sets the \a imuAngularRateFilter variable.
   * @param imuAngularRateFilter The value to set.
   *
   */
  void setImuAngularRateFilter(const unsigned int imuAngularRateFilter);

  /**
   * @fn unsigned int getImuAngularRateFilter() const
   *
   * @brief Gets the \a imuAngularRateFilter variable.
   * @return The value of \a imuAngularRateFilter to get.
   *
   */
  unsigned int getImuAngularRateFilter() const;

  /**
   * @fn void setImuAccelerationRange(const uint8_t imuAccelerationRange)
   *
   * @brief Sets the \a imuAccelerationRange variable.
   * @param imuAccelerationRange The value to set.
   *
   */
  void setImuAccelerationRange(const uint8_t imuAccelerationRange);

  /**
   * @fn uint8_t getImuAccelerationRange() const
   *
   * @brief Gets the \a imuAccelerationRange variable.
   * @return The value of \a imuAccelerationRange to get.
   *
   */
  uint8_t getImuAccelerationRange() const;

  /**
   * @fn void setImuAngularRateRange(const uint8_t imuAngularRateRange)
   *
   * @brief Sets the \a imuAngularRateRange variable.
   * @param imuAngularRateRange The value to set.
   *
   */
  void setImuAngularRateRange(const uint8_t imuAngularRateRange);

  /**
   * @fn uint8_t getImuAngularRateRange() const
   *
   * @brief Gets the \a imuAngularRateRange variable.
   * @return The value of \a imuAngularRateRange to get.
   *
   */
  uint8_t getImuAngularRateRange() const;

  /**
   * @fn void setSaveConfiguration(const bool saveConfiguration)
   *
   * @brief Sets the \a saveConfiguration variable.
   * @param saveConfiguration The value to set.
   *
   */
  void setSaveConfiguration(const bool saveConfiguration);

  /**
   * @fn bool getSaveConfiguration() const
   *
   * @brief Gets the \a saveConfiguration variable.
   * @return The value of \a saveConfiguration to get.
   *
   */
  bool getSaveConfiguration() const;

  /**
   * @fn bool hasImuAngularRateRange() const
   *
   * @brief Checks if the value of the \a imuAngularRateRange_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */
  bool hasImuAngularRateRange() const;

  /**
   * @fn bool hasSetReadingToNanOnDisconnect() const
   *
   * @brief Checks if the value of the \a setReadingToNanOnDisconnect_ variable has been set by the user in the
   * configuration file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasSetReadingToNanOnDisconnect() const;

  /**
   * @fn bool hasSensorConfiguration() const
   *
   * @brief Checks if the value of the \a sensorConfiguration_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasSensorConfiguration() const;

  /**
   * @fn bool hasForceTorqueFilter() const
   *
   * @brief Checks if the value of the \a forceTorqueFilter_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasForceTorqueFilter() const;

  /**
   * @fn bool hasForceTorqueOffset() const
   *
   * @brief Checks if the value of the \a forceTorqueOffset_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */
  bool hasForceTorqueOffset() const;

  /**
   * @fn bool hasUseCustomCalibration() const
   *
   * @brief Checks if the value of the \a useCustomCalibration_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */
  bool hasUseCustomCalibration() const;

  /**
   * @fn bool hasSensorCalibration() const
   *
   * @brief Checks if the value of the \a sensorCalibration_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasSensorCalibration() const;

  /**
   * @fn bool hasImuAccelerationRange() const
   *
   * @brief Checks if the value of the \a imuAccelerationRange_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasImuAccelerationRange() const;

  /**
   * @fn bool hasImuAccelerationFilter() const
   *
   * @brief Checks if the value of the \a imuAccelerationFilter_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasImuAccelerationFilter() const;

  /**
   * @fn bool hasImuAngularRateFilter() const
   *
   * @brief Checks if the value of the \a imuAngularRateFilter_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasImuAngularRateFilter() const;

  /**
   * @fn bool hasSaveConfiguration() const
   *
   * @brief Checks if the value of the \a saveConfiguration_ variable has been set by the user in the configuration
   * file.
   *
   * @return True If the variable has been set by the user.
   *
   */

  bool hasSaveConfiguration() const;
  /**
   * @fn void printConfiguration() const
   *
   * @brief Prints the existing Configuration.
   *
   */
  void printConfiguration() const;

protected:
  /**
   * @var mutable std::recursive_mutex mutex_
   *
   * @brief Mutex for synchronized access on the object's private variables.
   *
   */
  mutable std::recursive_mutex mutex_;

  /**
   * @var bool setReadingToNanOnDisconnect_
   *
   * @brief The setReadingToNanOnDisconnect variable.
   *
   */
  bool setReadingToNanOnDisconnect_;

  /**
   * @var bool hasSetReadingToNanOnDisconnect_
   *
   * @brief Flag indicating if \a setReadingToNanOnDisconnect_ is set.
   *
   */
  bool hasSetReadingToNanOnDisconnect_;

  /**
   * @var ForceTorqueFilter forceTorqueFilter_
   *
   * @brief The forceTorqueFilter variable.
   *
   */
  ForceTorqueFilter forceTorqueFilter_;

  /**
   * @var bool hasForceTorqueFilter_
   *
   * @brief Flag indicating if \a forceTorqueFilter_ is set.
   *
   */
  bool hasForceTorqueFilter_;

  /**
   * @var SensorConfiguration sensorConfiguration_
   *
   * @brief The sensorConfiguration variable.
   *
   */
  SensorConfiguration sensorConfiguration_;

  /**
   * @var bool hasSensorConfiguration_
   *
   * @brief Flag indicating if \a sensorConfiguration_ is set.
   *
   */
  bool hasSensorConfiguration_;

  /**
   * @var bool useCustomCalibration_
   *
   * @brief The useCustomCalibration variable.
   *
   */
  bool useCustomCalibration_;

  /**
   * @var bool hasUseCustomCalibration_
   *
   * @brief Flag indicating if \a useCustomCalibration_ is set.
   *
   */
  bool hasUseCustomCalibration_;

  /**
   * @var calibration::SensorCalibration sensorCalibration_
   *
   * @brief The sensorCalibration variable.
   *
   */
  calibration::SensorCalibration sensorCalibration_;

  /**
   * @var bool hasSensorCalibration_
   *
   * @brief Flag indicating if \a sensorCalibration_ is set.
   *
   */
  bool hasSensorCalibration_;

  /**
   * @var unsigned int imuAccelerationFilter_
   *
   * @brief The imuAccelerationFilter variable.
   *
   */
  unsigned int imuAccelerationFilter_;

  /**
   * @var bool hasImuAccelerationFilter_
   *
   * @brief Flag indicating if \a imuAccelerationFilter_ is set.
   *
   */
  bool hasImuAccelerationFilter_;

  /**
   * @var unsigned int imuAngularRateFilter_
   *
   * @brief The imuAngularRateFilter variable.
   *
   */
  unsigned int imuAngularRateFilter_;

  /**
   * @var bool hasImuAngularRateFilter_
   *
   * @brief Flag indicating if \a imuAngularRateFilter_ is set.
   *
   */
  bool hasImuAngularRateFilter_;

  /**
   * @var unsigned int imuAccelerationRange_
   *
   * @brief The imuAccelerationRange variable.
   *
   */
  unsigned int imuAccelerationRange_;

  /**
   * @var bool hasImuAccelerationRange_
   *
   * @brief Flag indicating if \a imuAccelerationRange_ is set.
   *
   */
  bool hasImuAccelerationRange_;

  /**
   * @var unsigned int imuAngularRateRange_
   *
   * @brief The imuAngularRateRange variable.
   *
   */
  unsigned int imuAngularRateRange_;

  /**
   * @var bool hasImuAngularRateRange_
   *
   * @brief Flag indicating if \a imuAngularRateRange_ is set.
   *
   */
  bool hasImuAngularRateRange_;

  /**
   * @var Eigen::Matrix<double, 6, 1> forceTorqueOffset_
   *
   * @brief The forceTorqueOffset variable.
   *
   */
  Eigen::Matrix<double, 6, 1> forceTorqueOffset_;

  /**
   * @var bool hasForceTorqueOffset_
   *
   * @brief Flag indicating if \a forceTorqueOffset_ is set.
   *
   */
  bool hasForceTorqueOffset_;

  /**
   * @var bool saveConfiguration_
   *
   * @brief The saveConfiguration_ variable.
   *
   */
  bool saveConfiguration_;

  /**
   * @var bool hasSaveConfiguration_
   *
   * @brief Flag indicating if \a saveConfiguration_ is set.
   *
   */
  bool hasSaveConfiguration_;
};

}  // namespace configuration

}  // namespace rokubimini
