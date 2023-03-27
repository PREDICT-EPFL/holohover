#pragma once

#include <diagnostic_updater/diagnostic_updater.hpp>
// soem_interface
#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>
#include <rokubimini_ethercat/soem_interface/EthercatSlaveBase.hpp>

#include <rokubimini/Reading.hpp>
#include <rokubimini/calibration/SensorCalibration.hpp>
#include <rokubimini/configuration/SensorConfiguration.hpp>
#include <rokubimini/configuration/ForceTorqueFilter.hpp>

#include <rokubimini_ethercat/PdoTypeEnum.hpp>
#include <rokubimini_ethercat/RxPdo.hpp>

#define MAX_FILE_SIZE_FIRMWARE (0x100000)

namespace rokubimini
{
namespace ethercat
{
/**
 * @class RokubiminiEthercatSlave
 *
 * @brief The Rokubimini Ethercat Implementation (Slave) class.
 *
 * It's the implementation in the BRIDGE pattern used. It provides
 * the implementation to be called by the interface
 * (RokubiminiEthercat) in order to communicate with the EtherCAT Device.
 *
 */

const static std::map<uint8_t, std::string> STATE_TO_STRING = { { EC_STATE_INIT, "Initialization state" },
                                                                { EC_STATE_PRE_OP, "Pre-Operational state" },
                                                                { EC_STATE_SAFE_OP, "Safe-Operational state" },
                                                                { EC_STATE_OPERATIONAL, "Operational state" } };
class RokubiminiEthercatSlave : public rokubimini::soem_interface::EthercatSlaveBase
{
public:
  /**
   * @fn RokubiminiEthercatSlave()
   *
   * @brief Default constructor is deleted.
   *
   *
   */

  RokubiminiEthercatSlave() = delete;

  /**
   * @fn RokubiminiEthercatSlave(const std::string &name, rokubimini::soem_interface::EthercatBusBase *bus, const
   * uint32_t address,const PdoTypeEnum pdoTypeEnum)
   *
   * @brief Custom constructor for the RokubiminiEthercatSlave class.
   *
   * @param name The name of the device.
   * @param bus The bus to which the RokubiminiEthercat will be attached.
   * @param address The address of the RokubiminiEthercatSlave in the Ethercat bus.
   * @param pdoTypeEnum The PDO type of the RokubiminiEthercatSlave.
   *
   */

  RokubiminiEthercatSlave(const std::string& name, rokubimini::soem_interface::EthercatBusBase* bus,
                          const uint32_t address, const PdoTypeEnum pdoTypeEnum);

  ~RokubiminiEthercatSlave() override = default;

  /**
   * @fn bool startup()
   *
   * @brief This method starts up and initializes the device.
   *
   * @return True if the startup was
   * successful.
   *
   */

  bool startup() override;

  /**
   * @fn void preSetupConfiguration()
   *
   * @brief Pre-setup configuration hook.
   *
   *
   */
  void preSetupConfiguration();

  /**
   * @fn void updateRead()
   *
   * @brief This method is called by the EthercatBusManager. Each device attached to this bus reads its data from the
   * buffer.
   *
   *
   */
  void updateRead() override;

  /**
   * @fn void updateWrite()
   *
   * @brief This method is called by the EthercatBusManager. Each device attached to the bus writes its data to the
   * buffer.
   *
   *
   */

  void updateWrite() override;

  /**
   * @fn void shutdown()
   *
   * @brief Shuts down the device.
   *
   *
   */

  void shutdown() override;

  /**
   * @fn void setState(const uint16_t state)
   *
   * @brief Sets the desired EtherCAT state machine state of the device in the bus.
   *
   *
   * @param state Desired state.
   *
   */

  void setState(const uint16_t state);

  /**
   * @fn bool waitForState(const uint16_t state)
   *
   * @brief Wait for an EtherCAT state machine state to be reached.
   *
   *
   * @param state Desired state.
   * @return True if the state has been reached within the timeout.
   */

  bool waitForState(const uint16_t state);

  /**
   * @fn PdoInfo getCurrentPdoInfo() const
   *
   * @brief Accessor for the current PDO info.
   *
   * @return The current PDO info.
   *
   */

  PdoInfo getCurrentPdoInfo() const override;

  /**
   * @fn std::string getName() const
   *
   * @brief Accessor for device name.
   *
   * @return The name of the device.
   *
   */

  std::string getName() const override
  {
    return name_;
  }

  /**
   * @fn PdoTypeEnum getCurrentPdoTypeEnum() const
   *
   * @brief Accessor for the current PDO type.
   *
   * @return The current PDO type.
   *
   */

  PdoTypeEnum getCurrentPdoTypeEnum() const
  {
    return currentPdoTypeEnum_;
  }

  /**
   * @fn bool getSerialNumber(unsigned int &serialNumber)
   *
   * @brief Accessor for device serial number.
   *
   * @param serialNumber The serial number to get.
   * @return True if the serial number could be
   * successfully fetched.
   *
   */

  bool getSerialNumber(unsigned int& serialNumber) override;

  /**
   * @fn unsigned int fetchSerialNumber()
   *
   * @brief Accessor for device serial number.
   *
   * @return The fetched serial number.
   *
   */

  inline unsigned int fetchSerialNumber() const
  {
    return serialNumber_;
  }

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

  bool getForceTorqueSamplingRate(int& samplingRate);

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

  bool setForceTorqueFilter(const configuration::ForceTorqueFilter& filter);

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

  bool setAccelerationFilter(const unsigned int filter);

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

  bool setAngularRateFilter(const unsigned int filter);

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

  bool setAccelerationRange(const unsigned int range);

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

  bool setAngularRateRange(const unsigned int range);

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

  bool setForceTorqueOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset);

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

  bool setSensorConfiguration(const configuration::SensorConfiguration& sensorConfiguration);

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

  bool setSensorCalibration(const calibration::SensorCalibration& sensorCalibration);

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
   * @fn bool saveConfigParameter()
   *
   * @brief Saves the current configuration to the device.
   *
   * @return True if the configuration was
   * successfully saved in the device.
   *
   */
  bool saveConfigParameter();

  /**
   * @fn void getReading(rokubimini::Reading &reading)
   *
   * @brief Gets the internal reading variable.
   *
   * @param reading The variable to store the reading.
   *
   */

  void getReading(rokubimini::Reading& reading) const;

  /**
   * @fn bool firmwareUpdate(const std::string& filePath, const std::string& fileName, const uint32_t& password)
   *
   * @brief Updates the firmware of the device.
   * @param filePath The path to find the firmware file
   * @param fileName The name of the file that will be written in the slave.
   * @param password The password for authorization.
   * @return True if the transfer (FoE) was successful.
   */
  bool firmwareUpdate(const std::string& filePath, const std::string& fileName, const uint32_t& password);

  /**
   * @fn bool isRunning()
   *
   * @brief Returns if the instance is running.
   * @return True if the instance is running.
   */
  bool isRunning()
  {
    return isRunning_;
  }

  /**
   * @fn void createDataFlagsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
   *
   * @brief Creates the Data Flags Diagnostics status
   * @param stat The status to be filled with the new diagnostics
   *
   */

  void createDataFlagsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @fn void updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
   *
   * @brief Fills the Device Connection Status (ROS diagnostics).
   * @param stat The status to be filled with the new diagnostics.
   *
   */

  void updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @fn bool runsAsync()
   *
   * @brief Returns if the driver runs asynchronously.
   * @return True if there is.
   */

  bool runsAsync() const
  {
    return runsAsync_;
  }

  /**
   * @fn void setRunsAsync(bool runsAsync)
   *
   * @brief Sets the runsAsync_ variable.
   * @param runsAsync The variable to set.
   */

  void setRunsAsync(bool runsAsync)
  {
    runsAsync_ = runsAsync;
  }

  /**
   * @fn void resetDataFlags()
   *
   * @brief Resets the Data Flags variable `currentDataFlagsDiagnostics_`.
   *
   */

  void resetDataFlags();

  /**
   * @fn void updateDataFlags()
   *
   * @brief Updates the Data Flags variable `currentDataFlagsDiagnostics_`.
   *
   */

  void updateDataFlags();

private:
  /**
   * @fn bool configurePdo(const PdoTypeEnum pdoTypeEnum)
   *
   * @brief Configures a PDO in the Ethercat device.
   *
   * @param pdoTypeEnum The type of the PDO.
   * @return True if the PDO was configured successfully.
   *
   */

  bool configurePdo(const PdoTypeEnum pdoTypeEnum);

  /**
   * @fn bool sendCalibrationMatrixEntry(const uint8_t subId, const double entry)
   *
   * @brief Sends a calibration matrix entry to device.
   *
   * @param subId The sub-index of the SDO to write to.
   * @param entry The entry on the matrix.
   * @return True if the operation was successful.
   *
   */

  bool sendCalibrationMatrixEntry(const uint8_t subId, const double entry);

  /**
   * @fn bool readFileToBuffer(const std::string& filePath)
   *
   * @brief Reads the contents of a file to an internal buffer.
   * @param filePath The path of the file.
   * @return True if operation was successful.
   */

  bool readFileToBuffer(const std::string& filePath);

  /**
   * @fn uint8_t getState()
   *
   * @brief Returns the EtherCAT State of the device.
   * @return The EtherCAT State.
   */

  uint8_t getState();

  /**
   * @fn uint16_t getALStatusCode()
   *
   * @brief Returns the AL Status Code of the slave.
   * @return The AL Status Code.
   */
  uint16_t getALStatusCode();

  /**
   * @fn std::string getSlaveStateString()
   *
   * @brief Returns the EtherCAT State of the device in a string.
   * @param state The state to look up.
   * @return A string containing the EtherCAT State.
   */

  std::string getSlaveStateString(uint8_t state);

  using PdoInfos = std::map<PdoTypeEnum, PdoInfo>;

  /**
   * @var std::string name_
   *
   * @brief Name of the sensor.
   *
   */

  const std::string name_;

  /**
   * @var PdoInfos pdoInfos_
   *
   * @brief Map between PDO types and their infos.
   *
   */
  PdoInfos pdoInfos_;

  /**
   * @var std::atomic<PdoTypeEnum> PdoTypeEnum_
   *
   * @brief PDO type.
   *
   */

  std::atomic<PdoTypeEnum> pdoTypeEnum_;

  /**
   * @var std::atomic<PdoTypeEnum> currentPdoTypeEnum_
   *
   * @brief Current PDO type.
   *
   */

  std::atomic<PdoTypeEnum> currentPdoTypeEnum_;

  /**
   * @var Reading reading_
   *
   * @brief The internal reading variable. It's used for providing
   * to client code the sensor readings, through the \a getReading
   * () function.
   *
   */

  Reading reading_;

  /**
   * @var int fileSize_
   *
   * @brief The size of the file.
   *
   */
  int fileSize_;

  /**
   * @var char* fileBuffer_
   *
   * @brief The buffer to save the contents of the file.
   *
   */
  char* fileBuffer_;

  /**
   * @var std::atomic<bool> isRunning_
   *
   * @brief Internal flag to indicate if the instance is running.
   *
   */
  std::atomic<bool> isRunning_;

  /**
   * @var std::atomic<bool> isRunning_
   *
   * @brief Internal flag to indicate if the instance is running asynchronously or not.
   *
   */
  std::atomic<bool> runsAsync_;

  /**
   * @var Statusword currentDataFlagsDiagnostics_
   *
   * @brief The current Data Flags Diagnostics (Data Status) which is updated when a new frame is published and is
   * published by the DataFlagsDiagnosticsPublisher.
   *
   */
  Statusword currentDataFlagsDiagnostics_;

  /**
   * @var mutable std::recursive_mutex dataFlagsMutex_
   *
   * @brief Mutex prohibiting simultaneous access to the Data Flags structure.
   *
   */

  mutable std::recursive_mutex dataFlagsMutex_;
};

using RokubiminiEthercatSlavePtr = std::shared_ptr<RokubiminiEthercatSlave>;

}  // namespace ethercat
}  // namespace rokubimini