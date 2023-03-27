/*
 * RokubiminiSerialImpl.hpp
 *  Created on: April, 2020
 *  Author(s):  Mike Karamousadakis, Ilias Patsiaouras
 *
 *  and is based on:
 *
 * ForceTorqueSensor.hpp
 *  Created on:   Dec 19, 2016
 *  Author(s):    Christian Gehring, Vassilios Tsounis, Klajd Lika
 */

#pragma once

//! System dependencies
#include <fcntl.h>
#include <sys/types.h>
#include <termios.h>
#include <boost/thread.hpp>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>
#include <string>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <diagnostic_updater/diagnostic_updater.hpp>

//! Device driver specific dependencies
// #include "rokubimini_serial/RokubiminiSerialImplOptions.hpp"
#include <rokubimini_serial/states.hpp>

#include <rokubimini/Reading.hpp>
#include <rokubimini/configuration/Configuration.hpp>
#include <rokubimini/calibration/SensorCalibration.hpp>
#include <rokubimini/configuration/SensorConfiguration.hpp>
#include <rokubimini/configuration/ForceTorqueFilter.hpp>
#include <rokubimini_serial/RokubiminiSerialCommunication.hpp>

namespace rokubimini
{
namespace serial
{
/**
 * @union DataStatus
 *
 * @brief The status of the sensor data.
 *
 *
 */

union DataStatus
{
  struct __attribute__((__packed__))
  {
    uint16_t app_took_too_long : 1;
    uint16_t overrange : 1;
    uint16_t invalid_measurements : 1;  // saturation, short, open circuit
    uint16_t raw_measurements : 1;      // gages instead of forces
    uint16_t : 12;                      // reserved
  };
  uint16_t byte;
};

/**
 * @union AppOutput
 *
 * @brief The main output from the sensors in the device.
 *
 *
 */

union AppOutput
{
  struct __attribute__((__packed__))
  {
    DataStatus status;
    float forces[6];
    uint32_t timestamp;
    float temperature;
    // float volts;
  };
  uint8_t bytes[1];
};

/**
 * @union RxFrame
 *
 * @brief The frame transmitted and received via the serial bus.
 *
 *
 */

union RxFrame
{
  struct __attribute__((__packed__))
  {
    uint8_t header;
    AppOutput data;
    uint16_t crc16_ccitt;
  };
  uint8_t bytes[1];
};
struct BaudRateStruct
{
  uint32_t value; /* The baud rate of the serial communication, as a normal integer (e.g. 115200 etc) */
  uint32_t mask;  /* The baud rate of the serial communication, as a termios bit mask. */
};
const static std::map<uint32_t, BaudRateStruct> CODE_TO_BAUD_RATE_MAP = {
  { 0, { 9600, B9600 } },
  { 1, { 57600, B57600 } },
  { 2, { 115200, B115200 } },
  // WARNING: The option no. 3, is used in the code as the default option. Be careful if you change this struct.
  { 3, { 230400, B230400 } },
  // WARNING: The option no. 4, is used in the code as the max option. Be careful if you change this struct.
  { 4, { 460800, B460800 } }

};
/**
 *@class RokubiminiSerialImpl
 *
 *@brief The Rokubimini Serial Implementation class.
 *
 *It's the implementation in the BRIDGE pattern used. It provides
 *the implementation to be called by the interface
 *(RokubiminiSerial) in order to communicate with the Serial Device.
 *
 */

class RokubiminiSerialImpl
{
public:
  /**
   * @fn RokubiminiSerialImpl()
   *
   * @brief Default constructor is deleted.
   *
   *
   */

  RokubiminiSerialImpl() = delete;

  /**
   * @fn RokubiminiSerialImpl(const std::string &name, const std::string &port, const std::uint32_t &baudRate)
   *
   * @brief Custom constructor for the RokubiminiSerialImpl class.
   *
   * @param name The name of the device.
   * @param port The port to connect to.
   * @param baudRate The baud rate of the serial communication.
   *
   */

  RokubiminiSerialImpl(const std::string& name, const std::string& port);

  ~RokubiminiSerialImpl() = default;

  /**
   * @fn bool init()
   *
   * @brief This method initializes internal variables and the device for communication. It connects to the serial-port.
   *
   * @return True if the initialization was successful.
   *
   */

  bool init();

  /**
   * @fn bool startup()
   *
   * @brief This method starts up communication with the device.
   *
   * @return True if the startup was
   * successful.
   *
   */

  bool startup();

  /**
   * @fn void updateRead()
   *
   * @brief This method is called by the BusManager. Each device attached to this bus reads its data from the buffer
   * (not used).
   *
   *
   */

  void updateRead(){ /* do nothing */ };

  /**
   * @fn void updateWrite()
   *
   * @brief This method is called by the BusManager. Each device attached to the bus writes its data to the buffer (not
   * used).
   *
   *
   */

  void updateWrite(){ /* do nothing */ };

  /**
   * @fn void closeSerialPort()
   *
   * @brief Closes the serial port.
   *
   *
   */

  void closeSerialPort();

  /**
   * @fn void shutdown()
   *
   * @brief Shuts down the device. It automatically shuts-down
   * threads and disconnects from serial-port.
   *
   *
   */

  void shutdown();

  /**
   * @fn std::string getName() const
   *
   * @brief Accessor for device name.
   *
   * @return The name of the device.
   *
   */

  std::string getName() const
  {
    return name_;
  }

  std::string getModeStateString() const
  {
    switch (modeState_)
    {
      case ModeState::CONFIG_MODE:
        return "Config Mode";
      case ModeState::RUN_MODE:
        return "Run Mode";
      case ModeState::INIT_MODE:
        return "Init Mode";
      default:
        return "ERROR: Unrecognized mode state";
    }
  }

  std::string getConnectionStateString() const
  {
    switch (connectionState_)
    {
      case ConnectionState::DISCONNECTED:
        return "Disconnected";
      case ConnectionState::ISCONNECTING:
        return "Is Connecting";
      case ConnectionState::CONNECTED:
        return "Connected";
      default:
        return "ERROR: Unrecognized connection state";
    }
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

  bool getSerialNumber(unsigned int& serialNumber)
  {
    serialNumber = serialNumber_;
    return true;
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

  bool getForceTorqueSamplingRate(int& samplingRate)
  {
    return false;
  }

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

  bool setAccelerationFilter(const unsigned int filter)
  {
    return false;
  }

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

  bool setAngularRateFilter(const unsigned int filter)
  {
    return false;
  }

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

  bool setAccelerationRange(const unsigned int range)
  {
    return false;
  }

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

  bool setAngularRateRange(const unsigned int range)
  {
    return false;
  }

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
   * @fn void getReading(rokubimini::Reading &reading)
   *
   * @brief Gets the internal reading variable.
   *
   * @param reading The variable to store the reading.
   *
   */

  void getReading(rokubimini::Reading& reading);

  /**
   * @fn bool setConfigMode();
   *
   * @brief Sets the device in config mode.
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
   * @fn bool setCommunicationSetup()
   *
   * @brief Sets communication setup for the device. This includes setting the temperature compensation, the matrix
   * calibration, the data format and the baud rate.
   *
   * @param sensorConfiguration The sensor configuration to be set.
   * @param dataFormat The data format (binary = 0, CSV = 1).
   * @param baudRate The desired baud rate (see user manual).
   *
   * @return True if the operation was successful.
   */
  bool setCommunicationSetup(const configuration::SensorConfiguration& sensorConfiguration, const uint8_t& dataFormat,
                             const uint8_t& baudRate);
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
   * @fn bool firmwareUpdate(const std::string& filePath)
   *
   * @brief Updates the firmware of the device.
   * @param filePath The path to find the firmware file
   * @return True if the flashing of firware was successful.
   */
  bool firmwareUpdate(const std::string& filePath);

  /**
   * @fn bool isRunning()
   *
   * @brief Returns if the serial driver is running.
   * @return True if the serial driver is running.
   */
  bool isRunning()
  {
    return isRunning_;
  }

  /**
   * @fn void setPollingTimeStep(double timeStep)
   *
   * @brief Sets the time step of the polling thread in seconds.
   * @parameter timeStep The time step in seconds.
   */
  void setPollingTimeStep(double timeStep)
  {
    pollingThreadTimeStep_ = timeStep;
    setRunsAsync(false);
  }

  /**
   * @fn bool hasFrameSync()
   *
   * @brief Returns if there is frame synchronization with the device.
   * @return True if there is.
   */
  bool hasFrameSync()
  {
    return frameSync_;
  }

  /**
   * @fn bool parseCommunicationMsgs(const double& timeout)
   *
   * @brief Parses the Communication Messages of the serial sensor.
   * @param timeout The timeout in seconds to wait. By default this is set to 1 seconds.
   * @return True if the operation was successful.
   */
  bool parseCommunicationMsgs(const double& timeout = 1.0);

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
   * @fn bool runsAsync()
   *
   * @brief Returns if the driver runs asynchronously.
   * @return True if there is.
   */
  bool runsAsync()
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
   * @fn std::string getErrorStrings() const
   *
   * @brief Retrieves detailed error indication.
   *
   * @return The error string.
   *
   */

  std::vector<std::string> getErrorStrings() const;

  /**
   * @fn bool hasError() const
   *
   * @brief Checks the connection has errors.
   *
   * @return True if the connection has errors.
   *
   */

  bool hasError() const;

  /**
   * @fn void updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
   *
   * @brief Fills the Device Connection Status (ROS diagnostics).
   * @param stat The status to be filled with the new diagnostics.
   *
   */

  void updateConnectionStatus(diagnostic_updater::DiagnosticStatusWrapper& stat);

  /**
   * @fn void getFrameDataStatus(DataStatus& status)
   *
   * @brief Gets the latest frame's Data Status.
   * @param status The status to be filled with the latest Data Status.
   */

  void getFrameDataStatus(DataStatus& status);

  /**
   * @fn void createDataFlagsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
   *
   * @brief Creates the Data Flags Diagnostics status
   * @param stat The status to be filled with the new diagnostics.
   *
   */

  void createDataFlagsDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

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
   * @fn bool startPollingThread()
   *
   * @brief This method starts up the polling thread.
   *
   * @return True if the operation was
   * successful.
   *
   */

  bool startPollingThread();

  /**
   * @fn bool writeSerial(const std::string& str)
   *
   * @brief Writes an Ascii string to the serial device.
   *
   * @param str The string to write.
   *
   * @return True if the operation was successful.
   */
  bool writeSerial(const std::string& str);

  /**
   * @fn bool readSerialNoWait(const uint32_t& numChars, std::string& str)
   *
   * @brief Reads a string from the serial device without waiting. The number of characters of the string is not
   * guaranteed to be equal with the number of characters given as parameter.
   * @param numChars The number of chars to read.
   * @param str The string to read.
   * @return True if new characters were read from the serial device.
   */
  bool readSerialNoWait(const uint32_t& numChars, std::string& str);

  /**
   * @fn bool readSerialWaitTimeout(const uint32_t& numChars, std::string& str, double timeout)
   *
   * @brief Reads a string from the serial device waiting a timeout duration. The number of characters of the string is
   * not guaranteed to be equal with the number of characters given as parameter.
   * @param numChars The number of chars to read.
   * @param str The string to read.
   * @param timeout The timeout in seconds to wait. By default this is set to 1 seconds.
   * @return True if new characters were read from the serial device.
   */
  bool readSerialWaitTimeout(const uint32_t& numChars, std::string& str, const double& timeout = 1.0);

  /**
   * @fn bool parseAcknowledgement(const char& command_code)
   *
   * @brief Parses the acknowledgement from the serial device.
   * @param command_code The command code to parse the ACK for.
   * @param timeout The timeout in seconds to wait before parsing the acknowledgement. By default this is set to 2
   * seconds.
   * @return True if the acknowledgement was received successfully.
   */
  bool parseAcknowledgement(const char& command_code, const double& timeout = 2.0);

  /**
   * @fn bool clearReadBuffer()
   *
   * @brief Clears the Read Serial buffer by extracting every character available.
   *
   * @return True if the acknowledgement was received successfully.
   */
  bool clearReadBuffer();

  /**
   * @fn bool sendCommand(const std::string& command)
   *
   * @brief Sends a command to the serial device.
   *
   * @param command The command to send.
   *
   * @return True if the operation was successful.
   */
  bool sendCommand(const std::string& command);

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

  bool sendCalibrationMatrixEntry(const uint8_t subId, const double entry)
  {
    return false;
  }

  /**
   * @fn bool parseRegexWaitTimeout(RokubiminiSerialResponseRegex& reg, const double& timeout = 1.0)
   *
   * @brief Parses a regex from the serial sensor input stream.
   * @param regex The regex to be parsed.
   * @param timeout The timeout in seconds to wait before parsing the regex. By default this is set to 1 seconds.
   * @return True if the operation was successful.
   */
  bool parseRegexWaitTimeout(RokubiminiSerialResponseRegex& reg, const double& timeout = 1.0);

  using timespec = struct timespec;

  /**
   * @fn inline double diffSec(timespec a, timespec b)
   *
   * @brief Calculates the difference (b-a) of two timespecs in seconds.
   *
   * @param a The first timespec.
   * @param b The second timespec.
   * @return The difference in seconds.
   *
   */
  inline double diffSec(timespec a, timespec b)
  {
    return (static_cast<double>(b.tv_sec - a.tv_sec) + static_cast<double>(b.tv_nsec - a.tv_nsec) / NSEC_PER_SEC);
  }

  /**
   * @fn inline double timespecToSec(timespec t)
   *
   * @brief Converts a timespec to seconds.
   *
   * @param t The timespec.
   * @return The seconds.
   *
   */
  inline double timespecToSec(timespec t)
  {
    return (static_cast<double>(t.tv_sec) + static_cast<double>(t.tv_nsec) / NSEC_PER_SEC);
  }

  /**
   * @fn inline timespec timespecAdd(timespec a, timespec b)
   *
   * @brief Calculates the sum of two timespecs.
   *
   * @param a The first timespec.
   * @param b The second timespec.
   * @return The timespec after adding the two input timespecs.
   *
   */
  inline timespec timespecAdd(timespec a, timespec b)
  {
    timespec result;
    if (static_cast<uint64_t>(a.tv_nsec + b.tv_nsec) >= NSEC_PER_SEC)
    {
      result.tv_sec = a.tv_sec + b.tv_sec + 1;
      result.tv_nsec = a.tv_nsec + b.tv_nsec - NSEC_PER_SEC;
    }
    else
    {
      result.tv_sec = a.tv_sec + b.tv_sec;
      result.tv_nsec = a.tv_nsec + b.tv_nsec;
    }
    return result;
  }

  /**
   * @fn bool connect()
   *
   * @brief Connects to the Serial-over-USB port.
   *
   * @return True if connection was
   * successful.
   *
   */

  bool connect();

  /**
   * @fn bool connect(const std::string &port)
   *
   * @brief Connects to the Serial-over-USB port.
   *
   * @param port The port to connect to.
   * @return True if connection was
   * successful.
   *
   */

  bool connect(const std::string& port);

  /**
   * @fn bool readDevice(RxFrame &frame)
   *
   * @brief Reads a raw measurement frame from the serial-port
   *
   * @param frame The raw measurement frame to read.
   * @return True if the reading was
   * successful.
   *
   */

  bool readDevice(RxFrame& frame);

  /**
   * @fn bool isConnected() const
   *
   * @brief Checks if the device is connected.
   *
   * @return True if the device is connected.
   *
   */

  bool isConnected() const;

  /**
   * @fn bool isConnecting() const
   *
   * @brief Checks if device is already in the process of connecting.
   *
   * @return True if device is in the process of connecting.
   *
   */

  bool isConnecting() const;

  /**
   * @fn bool isInConfigMode() const
   *
   * @brief Checks if the ModeState is Config Mode.
   *
   * @return True if the ModeState is Config Mode.
   *
   */

  bool isInConfigMode() const;

  /**
   * @fn ConnectionState getConnectionState() const
   *
   * @brief Gets the current connection status.
   *
   * @return The current connection status.
   *
   */

  ConnectionState getConnectionState() const;

  /**
   * @fn ErrorFlags getErrorFlags() const
   *
   * @brief Gets the current error status.
   *
   * @return The current error status.
   *
   */

  ErrorFlags getErrorFlags() const;

  /**
   * @fn bool openSerialPort()
   *
   * @brief Opens the serial port.
   * @param keepOpening Set if there will be multiple attempts
   * or only one.
   * @return True if the serial port was openned successfully.
   *
   */
  bool openSerialPort(bool keepOpening);
  /**
   * @fn bool initSensorCommunication(bool keepOpening)
   *
   * @brief Initializes communication with the sensor.
   *
   * @param keepOpening Set if there will be multiple attempts
   * or only one.
   * @return True if initialization of the communication was
   * successful.
   *
   */

  bool initSensorCommunication(bool keepOpening);

  /**
   * @fn bool initSerialPort(const std::string &port)
   *
   * @brief Sets up and initializes the serial port for
   * communication.
   *
   * @param port The port to initialize.
   * @return True if the port was initialized
   * successfully.
   *
   */

  bool initSerialPort(const std::string& port);

  /**
   * @fn uint16_t calcCrc16_x25(uint8_t *data, int len)
   *
   * @brief Calculates the CRC16 X25 checksum for the input data.
   *
   * @param data The input data.
   * @param len The length of the input data in bytes.
   * @return The checksum calculated.
   *
   */

  uint16_t calcCrc16X25(uint8_t* data, int len);

  /**
   * @fn uint16_t crcCcittUpdate(uint16_t crc, uint8_t data)
   *
   * @brief Implementation function of the CRC16 X25 checksum for the input data.
   *
   * @param data The input data.
   * @param crc The current checksum.
   * @return The new checksum calculated.
   *
   */

  uint16_t crcCcittUpdate(uint16_t crc, uint8_t data);

  /**
   * @fn void connectionWorker()
   *
   * @brief Worker threads for managing sensor connections.
   *
   */
  void connectionWorker();

  /**
   * @fn void pollingWorker()
   *
   * @brief Worker threads for polling the sensors.
   *
   */
  void pollingWorker();

  /**
   * @fn void increaseAndCheckTimeoutCounter()
   *
   * @brief Increases the timeout counter and checks if it has passed the maximum available timeouts.
   *
   */
  void increaseAndCheckTimeoutCounter();

  /**
   * @var std::string name_
   *
   * @brief Name of the sensor.
   *
   */

  std::string name_;

  /**
   * @var Reading serialImplReading_
   *
   * @brief The internal reading variable. It's used for providing
   * to client code the sensor readings, through the \a getReading
   * () function.
   *
   */

  Reading serialImplReading_;

  /**
   * @var std::string port_
   *
   * @brief The serial port to connect to.
   *
   */

  std::string port_;

  /**
   * @var BaudRateStruct baudRate_
   *
   * @brief The baud rate of the serial communication, as a termios bit mask.
   *
   */

  BaudRateStruct baudRate_;

  /**
   * @var std:string productName_
   *
   * @brief The product name of the device.
   *
   */

  std::string productName_;

  /**
   * @var uint32_t serialNumber_
   *
   * @brief The serial number of the device.
   *
   */

  uint32_t serialNumber_;

  /**
   * @var mutable std::recursive_mutex readingMutex_
   *
   * @brief Mutex prohibiting simultaneous access the internal Reading variable.
   *
   */

  mutable std::mutex readingMutex_;

  /**
   * @var mutable std::recursive_mutex serialMutex_
   *
   * @brief Mutex prohibiting simultaneous access to Serial device.
   *
   */

  mutable std::recursive_mutex serialMutex_;

  /**
   * @var boost::atomic<int> usbFileDescriptor_
   *
   * @brief The USB file descriptor.
   *
   */

  boost::atomic<int> usbFileDescriptor_;

  /**
   * @var boost::atomic<bool> frameSync_
   *
   * @brief Flag that indicates if the frame is synced.
   *
   */

  boost::atomic<bool> frameSync_;

  /**
   * @var std::ifstream usbStreamIn_
   *
   * @brief Input stream for the USB file descriptor.
   *
   */

  std::ifstream usbStreamIn_;

  /**
   * @var std::ofstream usbStreamOut_
   *
   * @brief Output stream for the USB file descriptor.
   *
   */

  std::ofstream usbStreamOut_;

  /**
   * @var uint8_t frameHeader
   *
   * @brief The frame header value.
   *
   */

  uint8_t frameHeader = 0xAA;

  /**
   * @var RxFrame frame_
   *
   * @brief The internal variable for the receiving frame. This variable represents the raw data received from the
   * sensor.
   *
   */

  RxFrame frame_;

  /*
   *  Device measurements and data buffers
   */

  /**
   * @var bool runInThreadedMode_
   *
   * @brief Flag to indicate whether the driver should setup worker
   * threads at startup.
   *
   */
  bool runInThreadedMode_;

  /**
   * @var double pollingThreadTimeStep_
   *
   * @brief If setup, the sensor polling thread will poll at this
   * rate.
   *
   */

  double pollingThreadTimeStep_;

  /**
   * @var boost::thread connectionThread_
   *
   * @brief Connection thread handle.
   *
   */

  boost::thread connectionThread_;

  /**
   * @var boost::thread pollingThread_
   *
   * @brief Polling thread handle.
   *
   */

  boost::thread pollingThread_;

  /**
   * @var boost::atomic<ConnectionState> connectionState_
   *
   * @brief Internal connection state.
   *
   */

  boost::atomic<ConnectionState> connectionState_;

  /**
   * @var ErrorFlags errorFlags_
   *
   * @brief Flags to indicate errors in serial communication.
   *
   */
  ErrorFlags errorFlags_;
  /**
   * @var boost::atomic<ModeState> modeState_
   *
   * @brief Mode state of the sensor.
   *
   */

  boost::atomic<ModeState> modeState_;
  /**
   * @var boost::atomic<bool> isRunning_
   *
   * @brief Internal flag to indicate if the threads are running.
   *
   */
  //! Internal flags/indicators
  boost::atomic<bool> isRunning_;

  /**
   * @var unsigned long pollingSyncErrorCounter_
   *
   * @brief Synchronization error counter.
   *
   */
  //! Internal statistics and error counters
  unsigned long pollingSyncErrorCounter_;

  /**
   * @var unsigned long frameReceivedCounter_
   *
   * @brief Received frame counter.
   *
   */
  unsigned long frameReceivedCounter_;

  /**
   * @var unsigned long frameSuccessCounter_
   *
   * @brief Correct frames counter.
   *
   */

  unsigned long frameSuccessCounter_;

  /**
   * @var unsigned long frameCrcErrorCounter_
   *
   * @brief Counter for frames with CRC errors.
   *
   */

  unsigned long frameCrcErrorCounter_;

  /**
   * @var unsigned int frameSyncErrorCounter_
   *
   * @brief Frame sync errors.
   *
   */
  unsigned int frameSyncErrorCounter_;

  /**
   * @var unsigned int maxFrameSyncErrorCounts_
   *
   * @brief Maximum acceptable frame sync errors.
   *
   */
  unsigned int maxFrameSyncErrorCounts_;
  /**
   * @var unsigned int maxCountOpenSerialPort_
   *
   * @brief Maximum attempts to open serial port.
   *
   */
  unsigned int maxCountOpenSerialPort_;
  /**
   * @var static uint64_t nsecPerSec_
   *
   * @brief The number of nanoseconds in a second.
   *
   */
  const static uint64_t NSEC_PER_SEC = 1000000000;
  /**
   * @var const static uint8_t DEFAULT_BAUD_RATE_OPTION = 3
   *
   * @brief The default baud rate option.
   *
   */
  const static uint8_t DEFAULT_BAUD_RATE_OPTION = 3;
  /**
   * @var const static uint8_t MAX_BAUD_RATE_OPTION = 4
   *
   * @brief The max baud rate option.
   *
   */
  const static uint8_t MAX_BAUD_RATE_OPTION = 4;

  /**
   * @var uint32_t frameOffset_
   *
   * @brief The frame offset to start reading (in bytes).
   *
   */
  uint32_t frameOffset_;

  /**
   * @var RxFrame placeholder_
   *
   * @brief A placeholder to save unfinished frames.
   *
   */
  RxFrame placeholder_;

  /**
   * @var constexpr static double FTDI_DRIVER_LATENCY = 0.001
   *
   * @brief The ftdi driver latency for reading a frame from the device (in seconds).
   *
   * This latency has been set to 1ms because that is the ftdi's latency timer resolution, since we set the
   * "ASYNC_LOW_LATENCY" flag.
   *
   *
   */
  constexpr static double FTDI_DRIVER_LATENCY = 0.001;

  /**
   * @var constexpr static double TIMEOUT_MARGIN = 0.1;
   *
   * @brief The timeout margin to read from device (in seconds).
   *
   */
  constexpr static double TIMEOUT_MARGIN = 0.15;

  /**
   * @var constexpr static double MAXIMUM_ACCEPTABLE_TIMEOUT = 1.77;
   *
   * @brief The maximum acceptable timeout to read from device (in seconds). This upper-bound timeout was computed with
   * the following settings: fir_disabled: false, sinc length: 4096, chop_enabled: true
   *
   */
  constexpr static double MAXIMUM_ACCEPTABLE_TIMEOUT = 1.77;

  /**
   * @var double readTimeout_
   *
   * @brief The timeout to read from device (in seconds).
   *
   */
  double readTimeout_;

  /**
   * @var unsigned int timeoutCounter_
   *
   * @brief Timeout counter.
   *
   */
  unsigned int timeoutCounter_;

  /**
   * @var std::condition_variable newFrameIsAvailable_
   *
   * @brief The condition variable that is used between the publishing and the polling thread to synchronize on new
   * data.
   *
   */
  std::condition_variable newFrameIsAvailable_;

  /**
   * @var bool dataReady_
   *
   * @brief This flag is on when new data are ready to be published.
   *
   */
  bool dataReady_;

  /**
   * @var std::atomic<bool> runsAsync_
   *
   * @brief The flag that represents whether the driver runs asynchronously or not.
   *
   */
  std::atomic<bool> runsAsync_;

  /**
   * @var DataStatus frameDataStatus_
   *
   * @brief The current frame's Data Status.
   *
   */
  DataStatus frameDataStatus_;

  /**
   * @var std::atomic<DataStatus> currentDataFlagsDiagnostics_
   *
   * @brief The current Data Flags Diagnostics (Data Status) which is updated when a new frame is published and is
   * published by the DataFlagsDiagnosticsPublisher.
   *
   */
  std::atomic<DataStatus> currentDataFlagsDiagnostics_;
  /**
   * @var rclcpp::Time frameTimestamp_
   */
  rclcpp::Time frameTimestamp_;

  /**
   * @var rclcpp::Clock clock
   *
   * @brief The ros clock.
   *
   */
  rclcpp::Clock clock_;
};

using RokubiminiSerialImplPtr = std::shared_ptr<RokubiminiSerialImpl>;

}  // namespace serial
}  // namespace rokubimini
