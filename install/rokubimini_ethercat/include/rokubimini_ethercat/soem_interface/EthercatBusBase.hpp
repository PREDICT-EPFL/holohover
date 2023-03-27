#pragma once

#include <cassert>
// std
#include <atomic>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <iomanip>

// soem
#include <soem/ethercat.h>

// soem_interface
#include <rokubimini_ethercat/soem_interface/EthercatSlaveBase.hpp>

#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
namespace soem_interface
{
/**
 * @brief      Class for managing an ethercat bus containing one or multiple
 *             slaves
 */
class EthercatBusBase
{
public:
  using PdoSizePair = std::pair<uint16_t, uint16_t>;
  using PdoSizeMap = std::unordered_map<std::string, PdoSizePair>;

  EthercatBusBase() = delete;
  /*!
   * Constructor.
   * @param name Name of the bus, e.g. "eth0".
   */
  explicit EthercatBusBase(const std::string& name);

  /*!
   * Destructor.
   */
  ~EthercatBusBase() = default;

  /*!
   * Get the name of the bus.
   * @return Name of the bus.
   */
  const std::string& getName() const
  {
    return name_;
  }

  /*!
   * Get the time of the last successful PDO reading.
   * @return Stamp.
   */
  const rclcpp::Time& getUpdateReadStamp() const
  {
    return updateReadStamp_;
  }

  /*!
   * Get the time of the last successful PDO writing.
   * @return Stamp.
   */
  const rclcpp::Time& getUpdateWriteStamp() const
  {
    return updateWriteStamp_;
  }

  /*!
   * Check if a bus is available.
   * @param name Name of the bus.
   * @return True if available.
   */
  static bool busIsAvailable(const std::string& name);

  /*!
   * Print all available busses.
   */
  static void printAvailableBusses();

  /*!
   * Check if this bus is available.
   * @return True if available.
   */
  bool busIsAvailable() const;

  /*!
   * Get the number of slaves which were detected on this bus.
   * @return Number of slaves.
   */
  int getNumberOfSlaves() const;

  /*!
   * Add an EtherCAT slave.
   * @slave EtherCAT slave.
   * @return True if successful.
   */
  bool addSlave(const EthercatSlaveBasePtr& slave);

  /*!
   * Return the state of a slave.
   * @slave EtherCAT slave.
   * @return The EtherCAT state of the slave.
   */
  uint8_t getSlaveState(uint16_t slave);

  /**
   * @brief  Returns the AL Status Code of a slave
   *
   * @param[in]  slave  Address of the slave.
   * @return The AL Status code of the slave.
   */
  uint16_t getSlaveALStatusCode(uint16_t slave);

  /*!
   * Startup the bus communication.
   * @param sizeCheck	perform a check of the Rx and Tx Pdo sizes defined in the PdoInfo oject of the slaves
   * @return True if successful.
   */
  bool startup(const bool sizeCheck = true);

  /*!
   * Update step 1: Read all PDOs.
   */
  void updateRead();

  /*!
   * Update step 2: Write all PDOs.
   */
  void updateWrite();

  /*!
   * Shutdown the bus communication.
   */
  void shutdown();

  /*!
   * Set the desired EtherCAT state machine state.
   * @param state Desired state.
   * @param slave Address of the slave, 0 for all slaves.
   */
  void setState(const uint16_t state, const uint16_t slave = 0);

  /*!
   * Wait for an EtherCAT state machine state to be reached.
   * @param state      Desired state.
   * @param slave      Address of the slave, 0 for all slaves.
   * @param maxRetries Maximum number of retries.
   * @param retrySleep Duration to sleep between the retries.
   * @return True if the state has been reached within the timeout.
   */
  bool waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 40,
                    const double retrySleep = 0.001);

  /*!
   * Generate and return the error string.
   * @param error EtherCAT error object.
   * @return The error string.
   */
  std::string getErrorString(ec_errort error);

  /**
   * @brief      Prints application layer status
   *
   * @param[in]  slave  Address of the slave, 0 for all slaves.
   */
  void printALStatus(const uint16_t slave = 0);

  /*!
   * Check if an error for the SDO index of the slave exists.
   * @param slave   Address of the slave.
   * @param index   Index of the SDO.
   * @return True if an error for the index exists.
   */
  bool checkForSdoErrors(const uint16_t slave, const uint16_t index);

  /*!
   * Synchronize the distributed clocks.
   *
   * @param      slave     Address of the slave.
   * @param      activate  True to activate the distr. clock, false to
   *                       deactivate.
   * @param[in]  timeStep  The time step
   */
  void syncDistributedClock0(const uint16_t slave, const bool activate, const double cycleTime,
                             const double cycleShift);

  /*!
   * Returns a map of the actually requested PDO sizes (Rx & Tx) This is useful
   * for slaves where the PDO size at startup is unknown This method shall be
   * used after adding the slaves and after executing the "startup" method
   *
   * @return     std::unordered_map with the addresses and the corresponding Pdo
   *             sizes
   */
  PdoSizeMap getHardwarePdoSizes();

  /*!
   * Returns a pair with the TxPdo and RxPdo sizes for the requested address
   * Overloads the "PdoSizeMap getHardwarePdoSizes()" method.
   *
   * @param      slave  Address of the slave
   *
   * @return     std::pair with the rx (first) and tx (second) Pdo sizes
   */
  PdoSizePair getHardwarePdoSizes(const uint16_t slave);

  /*!
   * Send a writing SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Value to write.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoWrite(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess,
                    const Value value)
  {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    const int size = sizeof(Value);
    Value value_copy = value;  // copy value to make it modifiable
    int wkc = 0;
    {
      std::lock_guard<std::recursive_mutex> guard(contextMutex_);
      wkc = ecx_SDOwrite(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), size, &value_copy,
                         EC_TIMEOUTRXM);
    }
    if (wkc <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                          "Slave " << slave << ": Working counter too low (" << wkc << ") for writing SDO (ID: 0x"
                                   << std::setfill('0') << std::setw(4) << std::hex << index << ", SID 0x"
                                   << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex)
                                   << ").");
      return false;
    }
    return true;
  }

  /*!
   * Send a reading SDO.
   * @param slave          Address of the slave.
   * @param index          Index of the SDO.
   * @param subindex       Sub-index of the SDO.
   * @param completeAccess Access all sub-indices at once.
   * @param value          Return argument, will contain the value which was read.
   * @return True if successful.
   */
  template <typename Value>
  bool sendSdoRead(const uint16_t slave, const uint16_t index, const uint8_t subindex, const bool completeAccess,
                   Value& value)
  {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    int size = sizeof(Value);
    int wkc = 0;
    {
      std::lock_guard<std::recursive_mutex> guard(contextMutex_);
      wkc = ecx_SDOread(&ecatContext_, slave, index, subindex, static_cast<boolean>(completeAccess), &size, &value,
                        EC_TIMEOUTRXM);
    }
    if (wkc <= 0)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                          "Slave " << slave << ": Working counter too low (" << wkc << ") for reading SDO (ID: 0x"
                                   << std::setfill('0') << std::setw(4) << std::hex << index << ", SID 0x"
                                   << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint16_t>(subindex)
                                   << ").");
      return false;
    }
    if (size != sizeof(Value))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("EtherCAT bus"),
                          "Slave " << slave << ": Size mismatch (expected " << sizeof(Value) << " bytes, read " << size
                                   << " bytes) for reading SDO (ID: 0x" << std::setfill('0') << std::setw(4) << std::hex
                                   << index << ", SID 0x" << std::setfill('0') << std::setw(2) << std::hex
                                   << static_cast<uint16_t>(subindex) << ").");
      return false;
    }
    return true;
  }

  /*!
   * Get the PDO expected working counter.
   * @param slave Address of the slave, 0 for all slaves.
   * @return Expected working counter.
   */
  int getExpectedWorkingCounter(const uint16_t slave = 0) const;

  /*!
   * Check if the current working counter for all slaves is high enough.
   * @return True if the working counter is equal or higher than expected.
   */
  bool workingCounterIsOk() const;

  /*!
   * Return the device name of the provided slave.
   * @return The device name.
   */
  std::string getDeviceName(const uint16_t& slave) const
  {
    return std::string(ecatContext_.slavelist[slave].name);
  }

  /*!
   * Check whether the provided device name matches that of the provided slave.
   * @return True if the provided device name matches the name of the given slave.
   */
  bool isCorrectDeviceName(const uint16_t& slave, const std::string& deviceName) const
  {
    return getDeviceName(slave) == deviceName;
  }

  /*!
   * Read a TxPDO from the buffer.
   * @param slave Address of the slave.
   * @param txPdo Return argument, TxPDO container.
   */
  template <typename TxPdo>
  void readTxPdo(const uint16_t slave, TxPdo& txPdo) const
  {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    assert(sizeof(TxPdo) == ecatContext_.slavelist[slave].Ibytes);
    memcpy(&txPdo, ecatContext_.slavelist[slave].inputs, sizeof(TxPdo));
  }

  /*!
   * Write an RxPDO to the buffer.
   * @param slave Address of the slave.
   * @param rxPdo RxPDO container.
   */
  template <typename RxPdo>
  void writeRxPdo(const uint16_t slave, const RxPdo& rxPdo)
  {
    assert(static_cast<int>(slave) <= getNumberOfSlaves());
    std::lock_guard<std::recursive_mutex> guard(contextMutex_);
    assert(sizeof(RxPdo) == ecatContext_.slavelist[slave].Obytes);
    memcpy(ecatContext_.slavelist[slave].outputs, &rxPdo, sizeof(RxPdo));
  }

  /*!
   * Write file via FoE. It's blocking.
   * @param slave Address of the slave.
   * @param filename The name of the file that will be written in the slave.
   * @param password The password for authorization.
   * @param fileSize The size of the file to write.
   * @param fileBuffer The buffer having the contents of the file.
   * @param timeout Timeout per mailbox cycle in us, default is EC_TIMEOUTSTATE * 10
   * @return Workcounter from last slave response
   */
  int writeFile(const uint16_t slave, const std::string& fileName, const uint32_t& password, const int fileSize,
                char* fileBuffer, int timeout = EC_TIMEOUTSTATE * 10);
  /*!
   * Writes firmware to a slave.
   * @param slave Address of the slave.
   * @param fileName The name of the file that will be written in the slave.
   * @param password The password for authorization.
   * @param fileSize The size of the file to write.
   * @param fileBuffer The buffer having the contents of the file.
   * @return True if the transfer (FoE) was successful.
   */
  bool writeFirmware(const uint16_t slave, const std::string& fileName, const uint32_t& password, const int fileSize,
                     char* fileBuffer);

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

protected:
  //! Name of the bus.
  std::string name_;

  //! List of slaves.
  std::vector<EthercatSlaveBasePtr> slaves_;

  //! Bool indicating whether PDO data has been sent and not read yet.
  bool sentProcessData_{ false };

  //! Working counter of the most recent PDO.
  std::atomic<int> wkc_;

  //! Internal flag to indicate if the instance is running.
  std::atomic<bool> isRunning_;

  //! Clock.
  rclcpp::Clock clock_;
  //! Time of the last successful PDO reading.
  rclcpp::Time updateReadStamp_;
  //! Time of the last successful PDO writing.
  rclcpp::Time updateWriteStamp_;

  //! Maximal number of retries to configure the EtherCAT bus.
  const unsigned int ecatConfigMaxRetries_{ 5 };
  //! Time to sleep between the retries.
  const double ecatConfigRetrySleep_{ 1.0 };

  // EtherCAT input/output mapping of the slaves within the datagrams.
  char ioMap_[4096];

  // EtherCAT context data elements:

  // Port reference.
  ecx_portt ecatPort_;
  // List of slave data. Index 0 is reserved for the master, higher indices for the slaves.
  ec_slavet ecatSlavelist_[EC_MAXSLAVE];
  // Number of slaves found in the network.
  int ecatSlavecount_{ 0 };
  // Slave group structure.
  ec_groupt ecatGrouplist_[EC_MAXGROUP];
  // Internal, reference to EEPROM cache buffer.
  uint8 ecatEsiBuf_[EC_MAXEEPBUF];
  // Internal, reference to EEPROM cache map.
  uint32 ecatEsiMap_[EC_MAXEEPBITMAP];
  // Internal, reference to error list.
  ec_eringt ecatEList_;
  // Internal, reference to processdata stack buffer info.
  ec_idxstackT ecatIdxStack_;
  // Boolean indicating if an error is available in error stack.
  boolean ecatError_{ FALSE };
  // Reference to last DC time from slaves.
  int64 ecatDcTime_{ 0 };
  // Internal, SM buffer.
  ec_SMcommtypet ecatSmCommtype_[EC_MAX_MAPT];
  // Internal, PDO assign list.
  ec_PDOassignt ecatPdoAssign_[EC_MAX_MAPT];
  // Internal, PDO description list.
  ec_PDOdesct ecatPdoDesc_[EC_MAX_MAPT];
  // Internal, SM list from EEPROM.
  ec_eepromSMt ecatSm_;
  // Internal, FMMU list from EEPROM.
  ec_eepromFMMUt ecatFmmu_;

  mutable std::recursive_mutex contextMutex_;
  // EtherCAT context data.
  // Note: SOEM does not use dynamic memory allocation (new/delete). Therefore
  // all context pointers must be null or point to an existing member.
  ecx_contextt ecatContext_ = { &ecatPort_,
                                &ecatSlavelist_[0],
                                &ecatSlavecount_,
                                EC_MAXSLAVE,
                                &ecatGrouplist_[0],
                                EC_MAXGROUP,
                                &ecatEsiBuf_[0],
                                &ecatEsiMap_[0],
                                0,
                                &ecatEList_,
                                &ecatIdxStack_,
                                &ecatError_,
                                0,
                                0,
                                &ecatDcTime_,
                                &ecatSmCommtype_[0],
                                &ecatPdoAssign_[0],
                                &ecatPdoDesc_[0],
                                &ecatSm_,
                                &ecatFmmu_,
                                nullptr };
};

using EthercatBusBasePtr = std::shared_ptr<EthercatBusBase>;

}  // namespace soem_interface
}  // namespace rokubimini
