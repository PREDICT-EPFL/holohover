#pragma once

#include <rokubimini/Rokubimini.hpp>
#include <rokubimini_serial/RokubiminiSerialImpl.hpp>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <rokubimini_msgs/srv/firmware_update_serial.hpp>
#include <rokubimini_msgs/srv/reset_wrench.hpp>
#include <rokubimini_msgs/msg/reading.hpp>
#include <thread>
#include <eigen3/Eigen/Dense>

namespace rokubimini
{
namespace serial
{
/**
 *@class RokubiminiSerial
 *
 *@brief The Rokubimini Serial class.
 *
 *Inherits from the Rokubimini class. It's the interface
 *in the BRIDGE pattern used. It provides the API to be called by
 *client code and is used for interfacing with the implementation
 *class called RokubiminiSerialImpl.
 *
 */

class RokubiminiSerial : public Rokubimini
{
public:
  /**
   * @fn RokubiminiSerial()
   *
   * @brief Default constructor.
   *
   * The default constructor of the RokubiminiSerial class.
   */
  RokubiminiSerial() = default;

  /**
   * @fn RokubiminiSerial(const std::string name, NodeHandlePtr nh)
   *
   * @brief Constructor with initialization list for the name and nh
   *
   */
  RokubiminiSerial(const std::string& name, NodeHandlePtr nh) : Rokubimini(name, std::move(nh)){ /* do nothing */ };
  ~RokubiminiSerial() override = default;

  /**
   * @fn virtual void preSetupConfiguration()
   *
   * @brief Post-setup configuration hook.
   *
   */
  void preSetupConfiguration() override;

  /**
   * @fn void postSetupConfiguration()
   *
   * @brief Post-setup configuration hook.
   *
   */

  void postSetupConfiguration() override;

  /**
   * @fn bool setPublishMode(double timeStep)
   *
   * @brief Sets the publishing mode (synchronous or async) of the serial device.
   *
   * @param timeStep The time step to set for synchronous publishing, provided it's not 0. If it's 0, this means
   * asynchronous publishing.
   * @return True if the operation was successful.
   *
   */
  bool setPublishMode(double timeStep);

  /**
   * @fn bool init();
   *
   * @brief Initializes communication with a Rokubimini Serial device.
   *
   * This method is called by the
   * SerialBusManager to establish communication with the device.
   *
   */
  bool init();

  /**
   * @fn void updateProcessReading()
   *
   * @brief Updates the \a RokubiminiSerial object with new measurements.
   *
   * This method updates the internal \a Reading variable of \a
   * RokubiminiSerial, by getting the new values from its
   * implementation \a RokubiminiSerialImpl.
   */
  void updateProcessReading() override;

  /**
   * @fn bool deviceIsMissing()
   *
   * @brief Checks if the device is missing.
   *
   */

  bool deviceIsMissing() const override;

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down a Rokubimini Serial device before
   * communication has been closed.
   *
   * This method shuts down a Rokubimini Serial device before the
   * SerialBusManager has terminated communication with the device.
   *
   */

  void shutdownWithCommunication() override;

  /**
   * @fn void setImplPointer(const RokubiminiSerialImplPtr &implPtr)
   *
   * @brief Sets a pointer to the implementation.
   *
   * This method realizes the pimpl paradigm. Through it,
   * the RokubiminiSerial object holds a pointer to its
   * implementation, a RokubiminiSerialImpl object.
   *
   * @param implPtr The pointer to the RokubiminiSerialImpl
   * implementation.
   *
   */

  void setImplPointer(const RokubiminiSerialImplPtr& implPtr)
  {
    implPtr_ = implPtr;
  }

  /**
   * @fn bool getSerialNumber(unsigned int &serialNumber)
   *
   * @brief Gets the serial number of the device.
   *
   *
   * @param serialNumber The serial number to be fetched.
   * @return True if the serial number was successfully fetched.
   *
   */

  // Missing: Methods for calling SDO
  bool getSerialNumber(unsigned int& serialNumber) override;

  /**
   * @fn bool getForceTorqueSamplingRate(int &samplingRate)
   *
   * @brief Gets the force torque sampling rate of the device.
   *
   * @param samplingRate The force torque sampling rate to be
   * fetched.
   * @return True if the force torque sampling rate was
   * successfully fetched.
   *
   */

  bool getForceTorqueSamplingRate(int& samplingRate) override;

  /**
   * @fn bool setForceTorqueFilter(const
   * configuration::ForceTorqueFilter &filter)
   *
   * @brief Sets a force torque filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the force torque filter was
   * successfully set.
   *
   */

  bool setForceTorqueFilter(const configuration::ForceTorqueFilter& filter) override;

  /**
   * @fn bool setAccelerationFilter(const unsigned int filter)
   *
   * @brief Sets an acceleration filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the acceleration torque filter was
   * successfully set.
   *
   */

  bool setAccelerationFilter(const unsigned int filter) override;

  /**
   * @fn bool setAngularRateFilter (const unsigned int filter)
   *
   * @brief Sets an angular rate filter to the device.
   *
   * @param filter The filter to be set.
   * @return True if the angular rate filter was
   * successfully set.
   *
   */

  bool setAngularRateFilter(const unsigned int filter) override;

  /**
   * @fn bool setAccelerationRange(const unsigned int range)
   *
   * @brief Sets an acceleration range to the device.
   *
   * @param range The range to be set.
   * @return True if the acceleration range was
   * successfully set.
   *
   */

  bool setAccelerationRange(const unsigned int range) override;

  /**
   * @fn bool setAngularRateRange(const unsigned int range)
   *
   * @brief Sets an angular rate range to the device.
   *
   * @param range The range to be set.
   * @return True if the angular rate range was
   * successfully set.
   *
   */

  bool setAngularRateRange(const unsigned int range) override;

  /**
   * @fn bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1> &forceTorqueOffset)
   *
   * @brief Sets a force torque offset to the device.
   *
   * @param forceTorqueOffset The offset to be set.
   * @return True if the offset was
   * successfully set.
   *
   */

  bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset) override;

  /**
   * @fn bool setSensorConfiguration(const configuration::SensorConfiguration &sensorConfiguration)
   *
   * @brief Sets a sensor configuration to the device.
   *
   * @param sensorConfiguration The configuration to be set.
   * @return True if the configuration was
   * successfully set.
   *
   */

  bool setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration) override;

  /**
   * @fn bool setSensorCalibration(const calibration::SensorCalibration &sensorCalibration)
   *
   * @brief Sets a sensor calibration to the device.
   *
   * @param sensorCalibration The calibration to be set.
   * @return True if the calibration was
   * successfully set.
   *
   */

  bool setSensorCalibration(const calibration::SensorCalibration& sensorCalibration) override;

  /**
   * @fn bool setConfigMode()
   *
   * @brief Sets the device in config mode.
   *
   * @return True if the operation was successful.
   */
  bool setConfigMode();

  /**
   * @fn bool setRunMode()
   *
   * @brief Sets the device in run mode.
   *
   * @return True if the operation was successful.
   */
  bool setRunMode();

  /**
   * @fn bool setHardwareReset()
   *
   * @brief Triggers a hardware reset of the sensor.
   *
   * @return True if the operation was successful.
   */

  bool setHardwareReset();

  /**
   * @fn bool setInitMode()
   *
   * @brief Triggers a software reset of the sensor bringing it to a
   * known state.
   *
   * @return True if the operation was successful.
   */
  bool setInitMode();

  /**
   * @fn bool saveConfigParameter()
   *
   * @brief Saves the current configuration to the device.
   *
   * @return True if the configuration was
   * successfully saved in the device.
   *
   */
  bool saveConfigParameter() override;

  /**
   * @fn bool loadConfig()
   *
   * @brief Loads the configuration of the device.
   *
   * @return True if the operation was successful.
   */
  bool loadConfig();

  /**
   * @fn bool printUserConfig()
   *
   * @brief Prints all the user configurable parameters.
   *
   * @return True if the operation was successful.
   */
  bool printUserConfig();

  /**
   * @fn void createRosPublishers()
   *
   * @brief Adds ROS publishers related to the device.
   *
   */
  void createRosPublishers() override;

  /**
   * @fn void createRosDiagnostics()
   *
   * @brief Adds ROS diagnostics related to the operation of rokubimini.
   *
   */
  void createRosDiagnostics() override;

  /**
   * @fn void updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
   *
   * @brief Updates the Connection Status of the device and adds it to the diagnostics status.
   *
   */
  void updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @fn void signalShutdown()
   *
   * @brief Signals shutdown for the ROS node. It's used if a firmware update was successful.
   *
   */
  void signalShutdown();

  /**
   * @fn firmwareUpdateCallback(std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial::Request> request,
                              std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial::Response> response);
   *
   * @brief The callback for the firmware update ROS service.
   * @param request The request of the ROS service.
   * @param response The response of the ROS service.
   * @return True always.
   *
   */
  bool firmwareUpdateCallback(std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial::Request> request,
                              std::shared_ptr<rokubimini_msgs::srv::FirmwareUpdateSerial::Response> response);

  /**
   * @fn resetWrenchCallback(std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Request> request,
                           std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Response> response)
   *
   * @brief The callback for the reset wrench ROS service.
   * @param request The request of the ROS service.
   * @param response The response of the ROS service.
   * @return True always.
   *
   */
  bool resetWrenchCallback(std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Request> request,
                           std::shared_ptr<rokubimini_msgs::srv::ResetWrench::Response> response);
  /**
   * @fn void createRosServices()
   *
   * @brief Adds ROS services related to the device.
   *
   */
  void createRosServices() override;

  /**
   * @fn void publishRosMessages()
   *
   * @brief Publishes ROS messages with data from the
   * rokubimini device.
   *
   */
  void publishRosMessages() override;

  /**
   * @fn void update()
   *
   * @brief Updates the internal Reading with new measurements and publishes them to ROS.
   *
   *
   */
  void update();

  /**
   * @fn void parseCommunicationMsgs()
   *
   * @brief Parses the incoming communication msgs from the device.
   *
   *
   */

  void parseCommunicationMsgs();

  /**
   * @fn void publishDataFlagsDiagnostics()
   *
   * @brief Publishes the Data Flags Diagnostics to the "/diagnostics" topic.
   *
   *
   */

  void publishDataFlagsDiagnostics();

protected:
  /**
   * @var RokubiminiSerialImplPtr implPtr_
   *
   * @brief The pointer to implementation.
   *
   */

  RokubiminiSerialImplPtr implPtr_{ nullptr };

  /**
   * @var RosPublisherPtr readingPublisher_
   *
   * @brief The rokubimini_msgs::Reading publisher.
   *
   */
  rclcpp::Publisher<rokubimini_msgs::msg::Reading>::SharedPtr readingPublisher_;

  /**
   * @var RosPublisherPtr wrenchPublisher_
   *
   * @brief The sensor_msgs::Wrench publisher.
   *
   */
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrenchPublisher_;

  /**
   * @var RosPublisherPtr temperaturePublisher_
   *
   * @brief The sensor_msgs::Temperature publisher.
   *
   */
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperaturePublisher_;

  /**
   * @var ros::ServiceServer firmwareUpdateService_
   *
   * @brief The service for firmware updates.
   *
   */
  rclcpp::Service<rokubimini_msgs::srv::FirmwareUpdateSerial>::SharedPtr firmwareUpdateService_;

  /**
   * @var ros::ServiceServer resetWrenchService_
   *
   * @brief The service for resetting the sensor wrench measurements.
   *
   */
  rclcpp::Service<rokubimini_msgs::srv::ResetWrench>::SharedPtr resetWrenchService_;

  /**
   * @var uint64_t noFrameSyncCounter_
   *
   * @brief The counter for measuring failed frame synchronizations.
   *
   */
  uint64_t noFrameSyncCounter_{ 0 };

  /**
   * @var std::atomic<bool> computeMeanWrenchFlag_
   *
   * @brief The flag for enabling the computation of the mean of the wrench measurements. Used withing the "reset
   * service" callback.
   *
   */
  std::atomic<bool> computeMeanWrenchFlag_{ false };

  /**
   * @var std::atomic<uint32_t> wrenchMessageCount_
   *
   * @brief The counter that is used for the computation of the mean of the wrench measurements. Used for the "reset
   * service" callback.
   *
   */
  std::atomic<uint32_t> wrenchMessageCount_{ 0 };

  /**
   * @var mutable std::recursive_mutex meanWrenchOffsetMutex_
   *
   * @brief The mutex for accessing the mean of the wrench measurements. Used for the "reset service" callback.
   *
   */
  mutable std::recursive_mutex meanWrenchOffsetMutex_;

  /**
   * @var const static uint32_t totalNumberOfWrenchMessages_ = 100;
   *
   * @brief The total number of wrench messages to receive for computing the mean wrench offset. Used for the "reset
   * service" callback.
   *
   */
  const static uint32_t TOTAL_NUMBER_OF_WRENCH_MESSAGES = 50;

  /**
   * @var eigen::Matrix<double, 6, Dynamic> resetServiceWrenchSamples_
   *
   * @brief The wrench samples used by the "reset service" callback for calculating the mean wrench offset.
   *
   */
  Eigen::Matrix<double, 6, Eigen::Dynamic> resetServiceWrenchSamples_;

  /**
   * @var std::thread publishingThread_
   *
   * @brief The thread that publishes the sensor messages to ROS.
   *
   */
  std::thread publishingThread_;

  /**
   * @var std::atomic<bool> publishFlag_
   *
   * @brief The flag for starting publishing. Set after initializing the publishers
   *
   */
  std::atomic<bool> publishFlag_{ false };
};

}  // namespace serial
}  // namespace rokubimini
