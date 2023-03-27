
#pragma once
#include <regex>
#include <rokubimini/configuration/Configuration.hpp>
#include <rokubimini/calibration/SensorCalibration.hpp>
#include <rokubimini/configuration/SensorConfiguration.hpp>
#include <rokubimini/configuration/ForceTorqueFilter.hpp>

namespace rokubimini
{
namespace serial
{
class RokubiminiSerialResponseRegex
{
public:
  RokubiminiSerialResponseRegex() = default;

  virtual ~RokubiminiSerialResponseRegex() = default;

  virtual bool matchInString(const std::string& str) = 0;
};

class RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommand() = default;
  virtual ~RokubiminiSerialCommand() = default;
  virtual bool formatCommand(std::string& formattedString) = 0;
};

class RokubiminiSerialCommandCommSetup : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandCommSetup() = delete;
  RokubiminiSerialCommandCommSetup(const configuration::SensorConfiguration& sensorConfiguration,
                                   const uint8_t& dataFormat, const uint8_t& baudRate);

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "c,%u,%u,%u,%u";
  configuration::SensorConfiguration sensorConfiguration_;
  uint8_t dataFormat_;
  uint8_t baudRate_;
};

class RokubiminiSerialCommandFilter : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandFilter() = delete;
  RokubiminiSerialCommandFilter(const configuration::ForceTorqueFilter& filter);

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "f,%u,%u,%u,%u";
  configuration::ForceTorqueFilter filter_;
};

class RokubiminiSerialCommandOffset : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandOffset() = delete;
  RokubiminiSerialCommandOffset(const Eigen::Matrix<double, 6, 1>& forceTorqueOffset);

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "b,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f";
  const Eigen::Matrix<double, 6, 1> forceTorqueOffset_;
};

class RokubiminiSerialCommandSensorCalibrationRow : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandSensorCalibrationRow() = delete;
  RokubiminiSerialCommandSensorCalibrationRow(const double& sensorCalibration1, const double& sensorCalibration2,
                                              const double& sensorCalibration3, const double& sensorCalibration4,
                                              const double& sensorCalibration5, const double& sensorCalibration6,
                                              const uint32_t& row);

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "m%u,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f";
  double sensorCalibration1_;
  double sensorCalibration2_;
  double sensorCalibration3_;
  double sensorCalibration4_;
  double sensorCalibration5_;
  double sensorCalibration6_;
  uint32_t row_;
};

class RokubiminiSerialCommandHardReset : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandHardReset() = default;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "#";
};

class RokubiminiSerialCommandSoftReset : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandSoftReset() = default;
  ;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "I";
};

class RokubiminiSerialCommandConfig : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandConfig() = default;
  ;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "C";
};

class RokubiminiSerialCommandRun : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandRun() = default;
  ;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "R";
};

class RokubiminiSerialCommandSave : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandSave() = default;
  ;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "s";
};

class RokubiminiSerialCommandLoad : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandLoad() = default;
  ;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "l";
};

class RokubiminiSerialCommandPrint : public RokubiminiSerialCommand
{
public:
  RokubiminiSerialCommandPrint() = default;
  ;

  bool formatCommand(std::string& formattedString) override;

private:
  const std::string formatString_ = "w";
};

class RokubiminiSerialResponseRegexCommSetup : public RokubiminiSerialResponseRegex
{
public:
  bool matchInString(const std::string& str) override;

  inline const uint8_t& getTempComp()
  {
    return tempComp_;
  }
  inline const uint8_t& getDataFormat()
  {
    return dataFormat_;
  }
  inline const uint8_t& getBaudRate()
  {
    return baudRate_;
  }
  inline const uint8_t& getCalibration()
  {
    return calibration_;
  }
  inline void setBaudRate(uint8_t& baudRate)
  {
    baudRate_ = baudRate;
  }

private:
  const std::regex stringRegex_ = std::regex("\\bc(,\\d){4}\\b");
  const std::string format_ = "c,%c,%c,%c,%c";
  uint8_t tempComp_;
  uint8_t dataFormat_;
  uint8_t baudRate_;
  uint8_t calibration_;
};

class RokubiminiSerialResponseRegexProductName : public RokubiminiSerialResponseRegex
{
public:
  bool matchInString(const std::string& str) override;
  inline const std::string& getProductName()
  {
    return productName_;
  }

private:
  const std::regex stringRegex_ = std::regex("name:\\s(.*)\\r");
  const std::string format_ = "name: %s\r";
  std::string productName_;
};

class RokubiminiSerialResponseRegexAck : public RokubiminiSerialResponseRegex
{
public:
  bool matchInString(const std::string& str) override;

  inline const uint8_t& getFirstNumber()
  {
    return firstNumber_;
  }
  inline const char& getCommand()
  {
    return command_;
  }
  inline const uint8_t& getReturnCode()
  {
    return returnCode_;
  }

private:
  const std::regex stringRegex_ = std::regex("r,\\d,\\w,\\d");
  const std::string format_ = "r,%c,%c,%c";
  uint8_t firstNumber_;
  char command_;
  std::uint8_t returnCode_;
};

class RokubiminiSerialResponseRegexSerialNumber : public RokubiminiSerialResponseRegex
{
public:
  bool matchInString(const std::string& str) override;
  inline const uint32_t& getSerialNumber()
  {
    return serialNumber_;
  }

private:
  const std::regex stringRegex_ = std::regex("S/N:\\s(.*)\\r");
  const std::string format_ = "S/N: %u\r";
  uint32_t serialNumber_;
};

class RokubiminiSerialResponseRegexFirmwareVersion : public RokubiminiSerialResponseRegex
{
public:
  bool matchInString(const std::string& str) override;
  inline const std::string& getFirmwareVersion()
  {
    return firmwareVersion_;
  }

private:
  const std::regex stringRegex_ = std::regex("FW version:\\s(.*)\\r");
  const std::string format_ = "FW version: %s\r";
  std::string firmwareVersion_;
};

class RokubiminiSerialResponseRegexBoot : public RokubiminiSerialResponseRegex
{
public:
  bool matchInString(const std::string& str) override;

private:
  const std::regex stringRegex_ = std::regex("App Init");
};
}  // namespace serial
}  // namespace rokubimini