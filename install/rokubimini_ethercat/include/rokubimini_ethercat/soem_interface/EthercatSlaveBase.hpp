#pragma once

// std
#include <cstdint>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
namespace soem_interface
{
class EthercatBusBase;

/**
 * @brief      Base class for generic ethercat slaves using soem
 */
class EthercatSlaveBase
{
public:
  /**
   * @brief      Struct defining the Pdo characteristic
   */
  struct PdoInfo
  {
    // The id of the rx pdo
    uint16_t rxPdoId_ = 0;
    // The id of the tx pdo
    uint16_t txPdoId_ = 0;
    // The size of the rx pdo
    uint16_t rxPdoSize_ = 0;
    // The size of the tx pdo
    uint16_t txPdoSize_ = 0;
    // The value referencing the current pdo type on slave side
    uint32_t moduleId_ = 0;
  };

  EthercatSlaveBase(EthercatBusBase* bus, const uint32_t address);
  virtual ~EthercatSlaveBase() = default;

  /**
   * @brief      Returns the name of the slave.
   *
   * @return     Name of the ethercat bus
   */
  virtual std::string getName() const = 0;

  /**
   * @brief      Startup non-ethercat specific objects for the slave
   *
   * @return     True if succesful
   */
  virtual bool startup() = 0;

  /**
   * @brief      Called during reading the ethercat bus. Use this method to
   *             extract readings from the ethercat bus buffer
   */
  virtual void updateRead() = 0;

  /**
   * @brief      Called during writing to the ethercat bus. Use this method to
   *             stage a command for the slave
   */
  virtual void updateWrite() = 0;

  /**
   * @brief      Used to shutdown slave specific objects
   */
  virtual void shutdown() = 0;

  /**
   * @brief      Gets the current pdo information.
   *
   * @return     The current pdo information.
   */
  virtual PdoInfo getCurrentPdoInfo() const = 0;

  /**
   * @brief      Returns the bus address of the slave
   *
   * @return     Bus address.
   */
  uint32_t getAddress() const
  {
    return address_;
  }

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
   * @fn virtual bool getSerialNumber(unsigned int& serialNumber) = 0;
   *
   * @brief Accessor for device serial number. It's abstract because it's implementation specific.
   *
   * @param serialNumber The serial number to get.
   * @return True if the serial number could be
   * successfully fetched.
   *
   */

  virtual bool getSerialNumber(unsigned int& serialNumber) = 0;

  /**
   * @fn bool setSerialNumber(unsigned int &serialNumber)
   *
   * @brief Accessor for device serial number.
   *
   * @param serialNumber The serial number to set.
   * @return True if the serial number could be
   * successfully saved.
   *
   */

  inline bool setSerialNumber(unsigned int& serialNumber)
  {
    serialNumber_ = serialNumber;
    return true;
  }

  /**
   * @fn void setProductName(const std::string& productName)
   *
   * @brief Sets the product name of the device.
   * @param productName The product name to set.
   *
   */
  void setProductName(const std::string& productName)
  {
    productName_ = productName;
  }
  /*!
   * Send a writing SDO.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-inidices at once.
   * @param value          Value to write.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t index, const uint8_t subindex, const bool completeAccess, const Value value);

  /*!
   * Send a reading SDO.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-inidices at once.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t index, const uint8_t subindex, const bool completeAccess, Value& value);

  // Send SDOs.
  virtual bool sendSdoReadInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess, int8_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess, int16_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess, int32_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess, int64_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess, uint8_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 uint16_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 uint32_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadUInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 uint64_t& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadFloat(const uint16_t index, const uint8_t subindex, const bool completeAccess, float& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoReadDouble(const uint16_t index, const uint8_t subindex, const bool completeAccess, double& value)
  {
    return sendSdoRead(index, subindex, completeAccess, value);
  }

  virtual bool sendSdoWriteInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                const int8_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 const int16_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 const int32_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 const int64_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt8(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 const uint8_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt16(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                  const uint16_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt32(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                  const uint32_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteUInt64(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                  const uint64_t value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteFloat(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                 const float value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoWriteDouble(const uint16_t index, const uint8_t subindex, const bool completeAccess,
                                  const double value)
  {
    return sendSdoWrite(index, subindex, false, value);
  }

  virtual bool sendSdoReadGeneric(const std::string& indexString, const std::string& subindexString,
                                  const std::string& valueTypeString, std::string& valueString);
  virtual bool sendSdoWriteGeneric(const std::string& indexString, const std::string& subindexString,
                                   const std::string& valueTypeString, const std::string& valueString);

protected:
  /**
   * @brief      Prints a warning. Use this method to suppress compiler warnings
   */
  void printWarnNotImplemented()
  {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("EtherCAT bus"), "Functionality is not implemented.");
  }

  // Mutex prohibiting simultaneous access to EtherCAT slave.
  mutable std::recursive_mutex mutex_;
  // Non owning pointer to the ethercat bus
  EthercatBusBase* bus_;
  // The bus address
  const uint32_t address_{ 0 };
  /**
   * @var std::string productName_
   *
   * @brief The product name of the slave.
   *
   */
  std::string productName_{ "" };

  /**
   * @var unsigned int serialNumber_
   *
   * @brief The serial number of the device.
   *
   */
  unsigned int serialNumber_{ 0 };
};

using EthercatSlaveBasePtr = std::shared_ptr<EthercatSlaveBase>;

}  // namespace soem_interface
}  // namespace rokubimini
