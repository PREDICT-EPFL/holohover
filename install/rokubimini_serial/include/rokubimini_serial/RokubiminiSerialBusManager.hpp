#pragma once

// std
#include <memory>
#include <mutex>

// rokubimini
#include <rokubimini_serial/RokubiminiSerial.hpp>

#include <rokubimini_bus_manager/BusManager.hpp>
#include <utility>
namespace rokubimini
{
namespace serial
{
/**
 * @class RokubiminiSerialBusManager
 *
 * @brief Inherits from RokubiminiBusManager. It's used for managing the serial bus.
 *
 * Although there isn't such a thing called "serial bus", it's used
 * for compliance with the inheritance scheme.
 *
 */
class RokubiminiSerialBusManager : public RokubiminiBusManager
{
public:
  /**
   * @fn RokubiminiSerialBusManager()
   *
   * @brief Default constructor of RokubiminiSerialBusManager.
   *
   */
  RokubiminiSerialBusManager() = delete;

  /**
   * @fn RokubiminiSerialBusManager(const std::string& busName, NodeHandlePtr nh)
   *
   * @brief Constructor with initialization list for the name and NodeHandle
   *
   */
  explicit RokubiminiSerialBusManager(const std::string& busName, NodeHandlePtr nh)
    : RokubiminiBusManager(busName, std::move(nh)){ /* do nothing */ };

  /**
   * @fn RokubiminiSerialBusManager(const NodeHandlePtr& nh)
   *
   * @brief Constructor with initialization list for the NodeHandle
   *
   */
  explicit RokubiminiSerialBusManager(const NodeHandlePtr& nh)
    : RokubiminiBusManager(nh){
      /* do nothing */
    };

  /**
   * @fn ~RokubiminiSerialBusManager()
   *
   * @brief Default Destructor.
   *
   */
  ~RokubiminiSerialBusManager() override = default;

  /**
   * @fn bool addRokubiminiToBus(
      std::shared_ptr<RokubiminiSerial> rokubimini) const
   *
   * @brief Creates a RokubiminiSerial Implementation object for
   * the @param rokubimini given.
   *
   * @param rokubimini The Rokubimini Serial instance for which an
   * implementation will be created.
   * @return True if the creation of the
   * RokubiminiSerialImpl object is successful.
   *
  */
  using RokubiminiBusManager::addRokubiminiToBus;
  bool addRokubiminiToBus(const std::shared_ptr<RokubiminiSerial>& rokubimini) const;

  /**
   * @fn void setConfigMode()
   *
   * @brief Sets all the serial devices in config mode.
   *
   */
  void setConfigMode() override;

  /**
   * @fn void setRunMode()
   *
   * @brief Sets all the serial devices in run mode.
   *
   */
  void setRunMode() override;

  /**
   * @fn bool startupCommunication()
   *
   * @brief Initializes the communication with the serial devices.
   *
   * @return True if the operation was successful.
   */
  bool startupCommunication() override;

  /**
   * @fn bool createRokubimini(const std::string& rokubiminiName)
   *
   * @brief Creates an RokubiminiSerial instance
   *
   * This function creates a RokubiminiSerial instance based on a given name and attaches it to the bus.
   *
   * @param rokubiminiName The name of the Rokubimini.
   * @return True if the operation succeeded.
   *
   */
  bool createRokubimini(const std::string& rokubiminiName) override;

  /**
   * @fn bool loadBusParameters()
   *
   * @brief Loads serial bus parameters from parameter server.
   *
   */
  bool loadBusParameters() override;

  /**
   * @fn double loadTimeStep()
   *
   * @brief Loads the internal timeStep_ variable.
   *
   */
  double loadTimeStep() override;

protected:
  /**
   *@var std::string serialPort_
   *
   *@brief The name of serial port.
   */
  std::string serialPort_;

  /**
   *@var uint32_t baudRate_
   *
   *@brief The baud rate of the bus.
   */
  uint32_t baudRate_;

  /**
   *@var double timeStep_
   *
   *@brief The time step of publishing loop.
   */
  double timeStep_;

private:
  /**
   * @fn void fetchTimeStep()
   *
   * @brief Fetches the 'time_step' from the Parameter Server and stores it in the protected 'timeStep_' variable.
   *
   */
  void fetchTimeStep();
};

using RokubiminiSerialBusManagerPtr = std::shared_ptr<RokubiminiSerialBusManager>;

}  // namespace serial
}  // namespace rokubimini