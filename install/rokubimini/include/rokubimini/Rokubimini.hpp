#pragma once

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

#include <rokubimini/Reading.hpp>
#include <rokubimini/Statusword.hpp>
#include <rokubimini/configuration/Configuration.hpp>
#include <rokubimini/configuration/SensorConfiguration.hpp>
#include <rokubimini/configuration/ForceTorqueFilter.hpp>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace rokubimini
{
/**
 * @class Rokubimini
 *
 * @brief Class representing a rokubi mini device.
 *
 */
class Rokubimini
{
public:
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;
  using DiagnosticsUpdaterPtr = std::shared_ptr<diagnostic_updater::Updater>;
  using RosPublisherPtr = rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr;
  using TimerPtr = rclcpp::TimerBase::SharedPtr;
  using ReadingCb = std::function<void(const std::string&, const Reading&)>;
  using ErrorCb = std::function<void(const std::string&)>;
  using ErrorRecoveredCb = std::function<void(const std::string&)>;
  using FatalCb = std::function<void(const std::string&)>;
  using FatalRecoveredCb = std::function<void(const std::string&)>;
  using DeviceDisconnectedCb = std::function<void(const std::string&)>;
  using DeviceReconnectedCb = std::function<void(const std::string&)>;

  // Anonymous callbacks do not include the name of the device.
  using AnonymousReadingCb = std::function<void(const Reading&)>;
  using AnonymousErrorCb = std::function<void(void)>;

  /**
   * @fn Rokubimini()
   *
   * @brief Default constructor.
   *
   */
  Rokubimini() = default;

  /**
   * @fn Rokubimini(const std::string name, NodeHandlePtr nh)
   *
   * @brief Constructor with initialization list for the name and nh
   *
   */
  Rokubimini(const std::string& name, NodeHandlePtr nh) : name_(name), nh_(std::move(nh)){ /* do nothing */ };
  virtual ~Rokubimini() = default;

  /**
   * @fn std::string getName() const
   *
   * @brief Gets the \a name of the device.
   * @return The \a name value.
   *
   */
  std::string getName() const
  {
    return name_;
  }

  /**
   * @fn configuration::Configuration &getConfiguration()
   *
   * @brief Non-const version of getConfiguration() const. Gets the \a Configuration of the device.
   * @return The \a Configuration value.
   *
   */
  configuration::Configuration& getConfiguration()
  {
    return configuration_;
  }

  /**
   * @fn const configuration::Configuration &getConfiguration() const
   *
   * @brief Gets the \a Configuration of the device.
   * @return The \a Configuration value.
   *
   */
  const configuration::Configuration& getConfiguration() const
  {
    return configuration_;
  }

  /**
   * @fn void addReadingCb(const ReadingCb &readingCb, const int priority = 0)
   *
   * @brief Adds a reading callback to a list.
   * @param readingCb The reading callback.
   * @param priority The priority of the callback.
   *
   */
  void addReadingCb(const ReadingCb& readingCb, const int priority = 0)
  {
    readingCbs_.insert({ priority, readingCb });
  }

  /**
   * @fn void addErrorCb(const ErrorCb &errorCb, const int priority = 0)
   *
   * @brief Adds an error callback to a list.
   * @param errorCb The error callback.
   * @param priority The priority of the callback.
   *
   */
  void addErrorCb(const ErrorCb& errorCb, const int priority = 0)
  {
    errorCbs_.insert({ priority, errorCb });
  }

  /**
   * @fn void addErrorRecoveredCb(const ErrorRecoveredCb &errorRecoveredCb, const int priority = 0)
   *
   * @brief Adds an error recovered callback to a list.
   * @param errorRecoveredCb The error recovered callback.
   * @param priority The priority of the callback.
   *
   */
  void addErrorRecoveredCb(const ErrorRecoveredCb& errorRecoveredCb, const int priority = 0)
  {
    errorRecoveredCbs_.insert({ priority, errorRecoveredCb });
  }

  /**
   * @fn void addFatalCb(const FatalCb &fatalCb, const int priority = 0)
   *
   * @brief Adds an fatal callback to a list.
   * @param fatalCb The fatal callback.
   * @param priority The priority of the callback.
   *
   */
  void addFatalCb(const FatalCb& fatalCb, const int priority = 0)
  {
    fatalCbs_.insert({ priority, fatalCb });
  }

  /**
   * @fn void addFatalRecoveredCb(const FatalRecoveredCb &fatalRecoveredCb, const int priority = 0)
   *
   * @brief Adds an fatal recovered callback to a list.
   * @param fatalRecoveredCb The fatal recovered callback.
   * @param priority The priority of the callback.
   *
   */
  void addFatalRecoveredCb(const FatalRecoveredCb& fatalRecoveredCb, const int priority = 0)
  {
    fatalRecoveredCbs_.insert({ priority, fatalRecoveredCb });
  }

  /**
   * @fn void addDeviceDisconnectedCb(const DeviceDisconnectedCb &deviceDisconnectedCb, const int priority = 0)
   *
   * @brief Adds a device disconnencted callback to a list.
   * @param DeviceDisconnectedCb The device disconnencted callback.
   * @param priority The priority of the callback.
   *
   */
  void addDeviceDisconnectedCb(const DeviceDisconnectedCb& deviceDisconnectedCb, const int priority = 0)
  {
    deviceDisconnectedCbs_.insert({ priority, deviceDisconnectedCb });
  }

  /**
   * @fn void addDeviceReconnectedCb(const DeviceReconnectedCb &deviceReconnectedCb, const int priority = 0)
   *
   * @brief Adds a device reconnected callback to a list.
   * @param DeviceReconnectedCb The device reconnected callback.
   * @param priority The priority of the callback.
   *
   */
  void addDeviceReconnectedCb(const DeviceReconnectedCb& deviceReconnectedCb, const int priority = 0)
  {
    deviceReconnectedCbs_.insert({ priority, deviceReconnectedCb });
  }

  /**
   * @fn void startupWithoutCommunication()
   *
   * @brief Starts up a Rokubimini device before communication has been established by the BusManager.
   *
   *
   */
  void startupWithoutCommunication();

  /**
   * @fn virtual void startupWithCommunication()
   *
   * @brief Starts up a Rokubimini device after communication has been established.
   *
   * This method is virtual because it's implementation-specific.
   *
   */
  void startupWithCommunication();

  /**
   * @fn virtual void updateProcessReading()
   *
   * @brief Updates the \a Rokubimini object with new measurements.
   *
   * This method updates the internal \a Reading variable of \a
   * Rokubimini, by getting the new values from its
   * implementation. It's virtual since it's implementation-specific.
   */
  virtual void updateProcessReading() = 0;

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down a Rokubimini device before
   * communication has been closed.
   *
   * This method shuts down a Rokubimini device before the
   * BusManager has terminated communication with the device. It's virtual since it's implementation-specific.
   *
   */
  virtual void shutdownWithCommunication() = 0;

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down a Rokubimini device after
   * communication has been closed.
   *
   * This method shuts down a Rokubimini device after the
   * BusManager has terminated communication with the device. It's virtual since it's implementation-specific.
   *
   */
  virtual void shutdownWithoutCommunication(){ /* do nothing */ };

  /**
   * @fn void clearGoalStateEnum()
   *
   * @brief Clears the goal state enumeration.
   *
   * @todo Fix me.
   *
   */
  void clearGoalStateEnum();

  /**
   * @fn void errorCb()
   *
   * @brief Calls the callbacks in the error callbacks list.
   *
   */
  void errorCb();

  /**
   * @fn void errorRecoveredCb()
   *
   * @brief Calls the callbacks in the error recovered callbacks list.
   *
   */
  void errorRecoveredCb();

  /**
   * @fn bool deviceIsInErrorState()
   *
   * @brief Checks if the device is in error state.
   * @return True if the device is in error state.
   *
   */
  bool deviceIsInErrorState();

  /**
   * @fn void fatalCb()
   *
   * @brief Calls the callbacks in the fatal callbacks list.
   *
   */
  void fatalCb();

  /**
   * @fn void fatalRecoveredCb()
   *
   * @brief Calls the callbacks in the fatal recovered callbacks list.
   *
   */
  void fatalRecoveredCb();

  /**
   * @fn bool deviceIsInFatalState()
   *
   * @brief Checks if the device is in fatal state.
   * @return True if the device is in fatal state.
   *
   */
  bool deviceIsInFatalState();

  /**
   * @fn void deviceDisconnectedCb()
   *
   * @brief Calls the callbacks in the device disconnected callbacks list.
   *
   */
  void deviceDisconnectedCb();

  /**
   * @fn void deviceReconnectedCb()
   *
   * @brief Calls the callbacks in the device reconnected callbacks list.
   *
   */
  void deviceReconnectedCb();

  /**
   * @fn virtual bool deviceIsMissing() const
   *
   * @brief Checks if the device is missing.
   * @return True if the device is missing.
   */
  virtual bool deviceIsMissing() const = 0;

  /**
   * @fn Statusword getStatusword() const
   *
   * @brief Gets the \a statusword of the device.
   *
   * @return The \a statusword of the device.
   *
   */
  Statusword getStatusword() const
  {
    return statusword_;
  }

  /**
   * @fn virtual bool getSerialNumber(unsigned int &serialNumber)
   *
   * @brief Gets the serial number of the device. It's virtual since it's implementation-specific.
   *
   *
   * @param serialNumber The serial number to be fetched.
   * @return True if the serial number was successfully fetched.
   *
   */
  virtual bool getSerialNumber(unsigned int& serialNumber) = 0;

  /**
   * @fn virtual bool getForceTorqueSamplingRate(int &samplingRate)
   *
   * @brief Gets the force torque sampling rate of the device. It's virtual since it's implementation-specific.
   *
   * @param samplingRate The force torque sampling rate to be
   * fetched.
   * @return True if the force torque sampling rate was
   * successfully fetched.
   *
   */
  virtual bool getForceTorqueSamplingRate(int& samplingRate) = 0;

  /**
   * @fn virtual bool setForceTorqueFilter(const
   * configuration::ForceTorqueFilter &filter)
   *
   * @brief Sets a force torque filter to the device. It's virtual since it's implementation-specific.
   *
   * @param filter The filter to be set.
   * @return True if the force torque filter was
   * successfully set.
   *
   */
  virtual bool setForceTorqueFilter(const configuration::ForceTorqueFilter& filter) = 0;

  /**
   * @fn virtual bool setAccelerationFilter(const unsigned int filter)
   *
   * @brief Sets an acceleration filter to the device. It's virtual since it's implementation-specific.
   *
   * @param filter The filter to be set.
   * @return True if the acceleration torque filter was
   * successfully set.
   *
   */
  virtual bool setAccelerationFilter(const unsigned int filter) = 0;

  /**
   * @fn virtual bool setAngularRateFilter (const unsigned int filter)
   *
   * @brief Sets an angular rate filter to the device. It's virtual since it's implementation-specific.
   *
   * @param filter The filter to be set.
   * @return True if the angular rate filter was
   * successfully set.
   *
   */
  virtual bool setAngularRateFilter(const unsigned int filter) = 0;

  /**
   * @fn virtual bool setAccelerationRange(const unsigned int range)
   *
   * @brief Sets an acceleration range to the device. It's virtual since it's implementation-specific.
   *
   * @param range The range to be set.
   * @return True if the acceleration range was
   * successfully set.
   *
   */
  virtual bool setAccelerationRange(const unsigned int range) = 0;

  /**
   * @fn virtual bool setAngularRateRange(const unsigned int range)
   *
   * @brief Sets an angular rate range to the device. It's virtual since it's implementation-specific.
   *
   * @param range The range to be set.
   * @return True if the angular rate range was
   * successfully set.
   *
   */
  virtual bool setAngularRateRange(const unsigned int range) = 0;

  /**
   * @fn virtual bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1> &forceTorqueOffset)
   *
   * @brief Sets a force torque offset to the device. It's virtual since it's implementation-specific.
   *
   * @param forceTorqueOffset The offset to be set.
   * @return True if the offset was
   * successfully set.
   *
   */
  virtual bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset) = 0;

  /**
   * @fn virtual bool setSensorConfiguration(const configuration::SensorConfiguration &sensorConfiguration)
   *
   * @brief Sets a sensor configuration to the device. It's virtual since it's implementation-specific.
   *
   * @param sensorConfiguration The configuration to be set.
   * @return True if the configuration was
   * successfully set.
   *
   */
  virtual bool setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration) = 0;

  /**
   * @fn virtual bool setSensorCalibration(const calibration::SensorCalibration &sensorCalibration)
   *
   * @brief Sets a sensor calibration to the device. It's virtual since it's implementation-specific.
   *
   * @param sensorCalibration The calibration to be set.
   * @return True if the calibration was
   * successfully set.
   *
   */
  virtual bool setSensorCalibration(const calibration::SensorCalibration& sensorCalibration) = 0;

  /**
   * @fn virtual bool saveConfigParameter()
   *
   * @brief Saves the current configuration to the device. It's virtual since it's implementation-specific.
   *
   * @return True if the configuration was
   * successfully saved in the device.
   *
   */
  virtual bool saveConfigParameter() = 0;

  /**
   * @fn Reading getReading() const
   *
   * @brief Gets the \a reading from the Rokubimini device.
   * @return The \a reading value.
   *
   */
  Reading getReading() const;

  /**
   * @fn void getReading(Reading &reading) const
   *
   * @brief Gets the internal variable reading into the provided parameter.
   *
   * @param reading The reading to assign the internal \a reading variable.
   *
   */
  void getReading(Reading& reading) const;

  /**
   * @todo Methods for logging.
   *
   */

  /**
   * @fn void setNodeHandle(NodeHandlePtr& nh) const;
   *
   * @brief Sets the nodeHandle of the device.
   *
   * @param nh The nodeHanlde of the device.
   *
   */
  void setNodeHandle(const NodeHandlePtr& nh)
  {
    nh_ = nh;
  }

  /**
   * @fn void createRosPublishers()
   *
   * @brief Creates ROS publishers for the rokubimini device. It's virtual because it's implementation specific.
   *
   *
   */
  virtual void createRosPublishers() = 0;

  /**
   * @fn void createRosServices()
   *
   * @brief Creates ROS services for the rokubimini device. It's virtual because it's implementation specific.
   *
   */
  virtual void createRosServices() = 0;

  /**
   * @fn void publishRosMessages()
   *
   * @brief Publishes ROS messages with data from the rokubimini device. It's virtual because it's implementation
   * specific.
   *
   *
   */
  virtual void publishRosMessages() = 0;

  /**
   * @fn void load(const std::string& busName, NodeHandlePtr nh)
   *
   * @brief Loads the rokubimini configuration from the parameters.
   *
   *
   */
  virtual void load();

  /**
   * @fn std::string getProductName() const
   *
   * @brief Gets the product name of the device.
   * @return The product name.
   *
   */
  std::string getProductName() const
  {
    return productName_;
  }

  /**
   * @fn void createRosDiagnostics()
   *
   * @brief Creates ROS diagnostics for the rokubimini device. It's virtual because it's implementation specific.
   *
   *
   */
  virtual void createRosDiagnostics() = 0;

protected:
  /**
   * @fn virtual void preSetupConfiguration()
   *
   * @brief Pre-setup configuration hook.
   *
   * This method is virtual because it's implementation-specific.
   *
   */
  virtual void preSetupConfiguration() = 0;

  /**
   * @fn virtual void postSetupConfiguration()
   *
   * @brief Post-setup configuration hook.
   *
   * This method is virtual because it's implementation-specific.
   *
   */
  virtual void postSetupConfiguration() = 0;

  /**
   * @fn void setStatusword(Statusword &statusword)
   *
   * @brief Sets the \a statusword of the device.
   *
   * @todo Check if needed.
   * @param statusword The \a statusword to set.
   *
   */

  void setStatusword(Statusword& statusword);

  /**
   * @var static constexpr double NAN_VALUE
   *
   * @brief NaN abbreviation for convenience.
   *
   */
  static constexpr double NAN_VALUE = std::numeric_limits<double>::quiet_NaN();

  /**
   * @var std::string name_
   *
   * @brief The name of the device.
   *
   */
  std::string name_;

  /**
   * @var configuration::Configuration configuration_
   *
   * @brief The Configuration of the device.
   *
   */
  configuration::Configuration configuration_;

  /**
   * @var Statusword statusword_
   *
   * @brief The current statusword.
   *
   */
  Statusword statusword_;

  /**
   * @var std::atomic<bool> statuswordRequested_
   *
   * @brief Flag indicating if a statusword has been actively requested.
   *
   */
  std::atomic<bool> statuswordRequested_{ false };

  /**
   * @var mutable std::recursive_mutex readingMutex_
   *
   * @brief The mutex for accessing the \a reading.
   *
   */
  mutable std::recursive_mutex readingMutex_;

  /**
   * @var Reading reading_
   *
   * @brief The reading of the device.
   *
   */
  Reading reading_;

  /**
   * @var std::shared_ptr<ros::NodeHandle> nh_
   *
   * @brief The node handle of the ROS node, used by the publishers.
   *
   */
  NodeHandlePtr nh_;

  /**
   * @var std::string productName_
   *
   * @brief The product name of the rokubimini.
   *
   */
  std::string productName_;

  /**
   * @var std::multimap<int, ReadingCb, std::greater<int>> readingCbs_
   *
   * @brief The list with the reading callbacks.
   *
   */
  std::multimap<int, ReadingCb, std::greater<int>> readingCbs_;

  /**
   * @var std::multimap<int, ErrorCb, std::greater<int>> errorCbs_
   *
   * @brief The list with the error callbacks.
   *
   */
  std::multimap<int, ErrorCb, std::greater<int>> errorCbs_;

  /**
   * @var std::multimap<int, ErrorCb, std::greater<int>> errorCbs_
   *
   * @brief The list with the error recovered callbacks.
   *
   */
  std::multimap<int, ErrorRecoveredCb, std::greater<int>> errorRecoveredCbs_;

  /**
   * @var std::multimap<int, FatalCb, std::greater<int>> fatalCbs_
   *
   * @brief The list with the fatal callbacks.
   *
   */
  std::multimap<int, FatalCb, std::greater<int>> fatalCbs_;

  /**
   * @var std::multimap<int, FatalRecoveredCb, std::greater<int>> fatalRecoveredCbs_
   *
   * @brief The list with the fatal recovered callbacks.
   *
   */
  std::multimap<int, FatalRecoveredCb, std::greater<int>> fatalRecoveredCbs_;

  /**
   * @var std::multimap<int, DeviceDisconnectedCb, std::greater<int>> deviceDisconnectedCbs_
   *
   * @brief The list with the device disconnected callbacks.
   *
   */
  std::multimap<int, DeviceDisconnectedCb, std::greater<int>> deviceDisconnectedCbs_;

  /**
   * @var std::multimap<int, DeviceReconnectedCb, std::greater<int>> deviceReconnectedCbs_
   *
   * @brief The list with the device reconnected callbacks.
   *
   */
  std::multimap<int, DeviceReconnectedCb, std::greater<int>> deviceReconnectedCbs_;

  /**
   * @var DiagnosticsUpdaterPtr connectionStatusUpdater_
   *
   * @brief The Connection Status diagnostics updater
   *
   */
  DiagnosticsUpdaterPtr connectionStatusUpdater_;

  /**
   * @var TimerPtr dataFlagsDiagnosticsTimer_
   *
   * @brief A timer for running the Data Flags Diagnostics update callback.
   *
   */
  TimerPtr dataFlagsDiagnosticsTimer_;

  /**
   * @var RosPublisherPtr dataFlagsDiagnosticsPublisher_
   *
   * @brief The Publisher for publishing Data Flags Diagnostics.
   *
   */
  RosPublisherPtr dataFlagsDiagnosticsPublisher_;
};

}  // namespace rokubimini
