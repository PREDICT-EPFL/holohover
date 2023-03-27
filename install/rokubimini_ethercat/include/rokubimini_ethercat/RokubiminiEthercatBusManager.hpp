#pragma once

// std
#include <memory>
#include <mutex>

// rokubimini
#include <rokubimini_ethercat/soem_interface/EthercatBusBase.hpp>

#include <rokubimini_ethercat/RokubiminiEthercat.hpp>

#include <rokubimini_bus_manager/BusManager.hpp>
#include <utility>
namespace rokubimini
{
namespace ethercat
{
using namespace soem_interface;

/**
 * @class RokubiminiEthercatBusManager
 *
 * @brief Inherits from RokubiminiBusManager. It's used for managing an Ethercat bus.
 *
 */

class RokubiminiEthercatBusManager : public RokubiminiBusManager
{
public:
  /**
   * @fn RokubiminiEthercatBusManager()
   *
   * @brief Default constructor of RokubiminiEthercatBusManager.
   *
   */
  RokubiminiEthercatBusManager() = delete;

  /**
   * @fn RokubiminiEthercatBusManager(const std::string& busName, NodeHandlePtr nh)
   *
   * @brief Constructor with initialization list for the name and NodeHandle
   *
   */
  explicit RokubiminiEthercatBusManager(const std::string& busName, NodeHandlePtr nh)
    : RokubiminiBusManager(busName, std::move(nh)){ /* do nothing */ };

  /**
   * @fn RokubiminiSerialBusManager(const NodeHandlePtr& nh)
   *
   * @brief Constructor with initialization list for the NodeHandle
   *
   */
  explicit RokubiminiEthercatBusManager(const NodeHandlePtr& nh)
    : RokubiminiBusManager(nh){
      /* do nothing */
    };
  ~RokubiminiEthercatBusManager() override = default;

  /**
   *@brief      Starts up the bus and puts them in operational mode
   *
   *@return     True if successful
   */
  bool startupBus();

  /**
   *@brief      Starts up  the bus
   *
   *@return     True if successful
   */
  bool startupCommunication() override;

  /**
   *@brief      Sets bus to safe operational state
   */
  void setBusSafeOperational();

  /**
   *@brief      Sets bus to pre operational state
   */
  void setBusPreOperational();

  /**
   *@brief      Sets bus to operational state
   */
  void setBusOperational();

  /**
   *@brief      Waits for the slave to reach a state
   *
   *@param[in]  state       Ethercat state
   *@param[in]  slave       Slave address, 0 = all slaves
   *@param[in]  maxRetries  The maximum retries
   *@param[in]  retrySleep  The retry sleep
   */
  void waitForState(const uint16_t state, const uint16_t slave = 0, const unsigned int maxRetries = 40,
                    const double retrySleep = 0.001);

  /**
   *@brief      Calls update read on the bus
   */
  void readBus() override;

  /**
   *@brief      Calls update write on the bus
   */
  void writeToBus() override;

  /**
   *@brief      Calls shutdown on the bus
   */
  void shutdownBus() override;

  using RokubiminiBusManager::addRokubiminiToBus;

  /**
  * @fn bool addRokubiminiToBus(
    RokubiminiEthercat *rokubimini,
    rokubimini::soem_interface::EthercatBusBase *bus,
    const std::shared_ptr<setup::RokubiminiEthercat> rokubiminiEthercatSetup) const
  *
  * @brief Adds a Rokubimini to Bus.
  *
  * This method is used for adding a Rokubimini Ethercat instance to
  * the Ethercat Bus. This method also adds the EthercatSlave pointer
  * to the Rokubimini Ethercat implementation.
  *
  * @param rokubimini The RokubiminiEthercat instance.
  * @param bus The bus to which the RokubiminiEthercat will be attached.
*/
  bool addRokubiminiToBus(const std::shared_ptr<RokubiminiEthercat>& rokubimini,
                          rokubimini::soem_interface::EthercatBusBase* bus) const;

  /**
   *@fn virtual void doPreStartupActions()
   *
   *@brief Allows for Pre-Startup Actions from the EthercatBusManager.
   *This method allows the EthercatBusManager to do some startup
   *actions before the \a startup() method is called for
   *every RokubiminiEthercat instance.
   *
   */
  void setConfigMode() override;

  /**
   *@fn virtual void doPostStartupActions()
   *
   *@brief Allows for Post-Startup Actions from the EthercatBusManager.
   *
   *This method allows the EthercatBusManager to do some startup
   *actions after the \a startup() method is called for
   *every RokubiminiEthercat instance.
   *
   */

  void setRunMode() override;

  /**
   * @fn virtual bool createRokubimini(const std::string& rokubiminiName)
   *
   * @brief Creates an RokubiminiEthercat instance
   *
   * This function creates a RokubiminiEthercat instance based on a given name and attaches it to the bus.
   *
   * @param rokubiminiName The name of the Rokubimini.
   * @return True if the operation succeeded.
   *
   */
  bool createRokubimini(const std::string& rokubiminiName) override;

  /**
   * @fn bool loadBusParameters()
   *
   * @brief Loads ethercat bus parameters.
   *
   */
  bool loadBusParameters() override;

  /**
   * @fn double loadTimeStep()
   *
   * @brief Loads the time_step from the Parameters.
   *
   */
  double loadTimeStep() override;

protected:
  /**
   *@var std::recursive_mutex busMutex_
   *
   *@brief Mutex prohibiting simultaneous access to EtherCAT bus
   *manager.
   */
  std::recursive_mutex busMutex_;

  /**
   *@var std::unique_ptr<rokubimini::soem_interface::EthercatBusBase>  bus_
   *
   *@brief The Ethercat bus instance.
   */
  std::unique_ptr<rokubimini::soem_interface::EthercatBusBase> bus_;

  /**
   *@var std::string ethercatBusName_
   *
   *@brief The name of the Ethercat bus.
   */

  std::string ethercatBusName_;
};

using RokubiminiEthercatBusManagerPtr = std::shared_ptr<RokubiminiEthercatBusManager>;

}  // namespace ethercat
}  // namespace rokubimini
