#pragma once

// std
#include <memory>
#include <mutex>
#include <bota_node/bota_node.hpp>
#include <rokubimini/Rokubimini.hpp>
#include <utility>
#include <rclcpp/rclcpp.hpp>

namespace rokubimini
{
/**
 * @class RokubiminiBusManager
 *
 * @brief The Rokubimini Bus Manager class.
 *
 * Base class which provides an extendable API. It's useful
 * for creating a bus manager with an existing
 * communication protocol.
 *
 */
class RokubiminiBusManager : public bota_node::Node
{
public:
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;
  using DiagnosticsUpdaterPtr = std::shared_ptr<diagnostic_updater::Updater>;
  /**
   * @fn RokubiminiBusManager()
   *
   * @brief Default constructor.
   *
   */

  RokubiminiBusManager() = delete;

  /**
   * @fn RokubiminiBusManager(const std::string& busName, NodeHandlePtr nh)
   *
   * @brief Constructor with initialization list for the name and NodeHandle
   *
   * This method constructs a \a RokubiminiBusManager and clears
   * the contents of the protected vector \a rokubiminis_.
   */
  explicit RokubiminiBusManager(const std::string& busName, NodeHandlePtr nh)
    : bota_node::Node(nh), name_(busName), nh_(std::move(nh))
  {
    rokubiminis_.clear();
  };

  /**
   * @fn RokubiminiBusManager(const NodeHandlePtr& nh)
   *
   * @brief Constructor with initialization list for the NodeHandle
   *
   * This method constructs a \a RokubiminiBusManager and clears
   * the contents of the protected vector \a rokubiminis_.
   */
  explicit RokubiminiBusManager(const NodeHandlePtr& nh) : bota_node::Node(nh), nh_(nh)
  {
    rokubiminis_.clear();
  };

  /**
   * @fn virtual ~RokubiminiBusManager()
   *
   * @brief Default Destructor.
   *
   */
  ~RokubiminiBusManager() override = default;

  /**
   * @fn virtual bool addRokubiminiToBus(Rokubimini *rokubimini,
   * const std::shared_ptr<setup::Rokubimini> rokubiminiSetup) const
   *
   * @brief Adds a Rokubimini to Bus.
   *
   * This method is used for adding a Rokubimini (implementation-specific)
   * to the Bus (implementation-specific). This method is bound to each
   * implementation and the reason that it's here, is because of runtime
   * polymorphism through inheritance. It should be pointed that the methods
   * in the children of this class, add the implementation pointer to
   * the Rokubimini implementation (e.g. RokubiminiSerialImplPtr to the
   * RokubiminiSerial instance).
   *
   * @param rokubimini The Rokubimini instance.
   */

  virtual bool addRokubiminiToBus(const std::shared_ptr<Rokubimini>& rokubimini) const;

  /**
   * @fn virtual bool startupCommunication()
   *
   * @brief Starts the communication through the bus.
   *
   * This method starts the communication with each attached Rokubimini
   * through the bus.
   *
   */

  virtual bool startupCommunication()
  {
    return true;
  };

  /**
   * @fn bool init()
   *
   * @brief Initializes the Bus Manager Node.
   *
   * Implements the initialization phase of the Bus Manager.
   *
   */
  bool init() override;

  /**
   * @fn void cleanup()
   *
   * @brief Cleans up the Bus Manager Node.
   *
   * Used for shutting down the Bus Manager.
   *
   */
  void cleanup() override;

  /**
   * @fn bool startup()
   *
   * @brief Starts the communication with all the Rokubimini devices.
   *
   * This method starts the communication with all the Rokubimini
   * devices.
   *
   */

  virtual bool startup();

  /**
   * @fn bool update(const bota_worker::WorkerEvent& event);
   *
   * @brief Updates with new values from/to the Rokubiminis.
   *
   * This method updates the Manager with new values
   * (\a Readings) from the Rokubimini devices.
   *
   * @param event The worker event.
   */

  virtual bool update(const bota_worker::WorkerEvent& event);

  /**
   * @fn void shutdown()
   *
   * @brief Shuts down everything.
   *
   * Shuts down all the Rokubimini devices.
   * Also stops the \a updateWorker thread that is
   * running in parallel.
   *
   */

  virtual void shutdown();

  /**
   * @fn virtual void shutdownBus()
   *
   * @brief Shuts down the bus.
   *
   * This method shuts down the bus
   * which has been created by the BusManager.
   *
   */

  virtual void shutdownBus(){ /* do nothing */ };

  /**
   * @fn virtual void writeToBus()
   *
   * @brief Writes to the buses.
   *
   * This method writes to the bus
   * which has been created by the BusManager.
   *
   */

  virtual void writeToBus(){ /* do nothing */ };

  /**
   * @fn virtual void readBus()
   *
   * @brief Reads the bus.
   *
   * This method reads the bus
   * which has been created by the BusManager.
   */

  virtual void readBus(){ /* do nothing */ };

  /**
   * @fn virtual void setConfigMode()
   *
   * @brief Sets the devices controlled from the BusManager to config mode.
   *
   * This method allows the BusManager to set the devices to configuration mode before the \a startup() method is called
   * for every Rokubimini instance.
   *
   */

  virtual void setConfigMode(){ /* do nothing */ };

  /**
   * @fn virtual void setRunMode()
   *
   * @brief Sets the devices controlled from the BusManager to run mode.
   *
   * This method allows the BusManager to set the devices to run mode after the \a startup() method is called for every
   * Rokubimini instance.
   *
   */

  virtual void setRunMode(){ /* do nothing */ };

  /**
   * @fn virtual bool createRokubimini(const std::string& rokubiminiName)
   *
   * @brief Creates an implementation-specific Rokubimini instance
   *
   * This function creates a Rokubimini instance based on a given name.
   * It's virtual because it's implementation-specific.
   *
   * @param rokubiminiName The name of the Rokubimini.
   * @return True if the operation succeeded.
   *
   */
  virtual bool createRokubimini(const std::string& rokubiminiName) = 0;

  /**
   * @fn virtual bool createRokubiminisFromParamServer()
   *
   * @brief Creates rokubiminis from the parameters.
   *
   */
  virtual bool createRokubiminisFromParamServer();

  /**
   * @fn virtual bool loadBusParameters()
   *
   * @brief Loads bus-specific parameters.
   * It's pure virtual since it's implementation-specific.
   *
   */
  virtual bool loadBusParameters() = 0;

  /**
   * @fn virtual bool load()
   *
   * @brief Loads the configuration of the bus (bus parameters and rokubiminis).
   *
   */
  virtual bool load();

  /**
   * @fn std::string getName() const
   *
   * @brief Gets the \a name of the bus.
   * @return The \a name value.
   *
   */
  inline std::string getName() const
  {
    return name_;
  }

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
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->setNodeHandle(nh);
    }
  }

  /**
   * @fn inline bool hasRokubimini(const std::string& name)
   * @brief Returns true if the rokubimini with \a name is found.
   *
   * @param name The name of the Rokubimini to be found.
   * @return True if the rokubimini with \a name is found.
   */

  inline bool hasRokubimini(const std::string& name)
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      if (rokubimini->getName() == name)
      {
        return true;
      }
    }
    return false;
  }

  /**
   * @fn std::shared_ptr<Rokubimini> getRokubimini(const std::string &name) const
   * @brief Returns the Rokubimini instance with name \a name.
   *
   * This method is used for getting a Rokubimini instance based on
   * its name. If there isn't such a Rokubimini in the Manager's list,
   * a \a nullptr is returned.
   *
   * @param name The name of the Rokubimini to be found.
   */

  inline std::shared_ptr<Rokubimini> getRokubimini(const std::string& name) const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      if (rokubimini->getName() == name)
      {
        return rokubimini;
      }
    }
    return nullptr;
  }

  /**
   * @fn std::vector<std::shared_ptr<Rokubimini>> getRokubiminis() const
   *
   * @brief Returns all the Rokubimini instances located in the
   * Bus Manager's list.
   *
   * This getter method is used for getting all the Rokubimini
   * instances of the Bus Manager.
   *
   */
  inline std::vector<std::shared_ptr<Rokubimini>> getRokubiminis() const
  {
    return rokubiminis_;
  }

  /**
   * @fn void createRokubiminiRosPublishers() const
   *
   * @brief Creates ROS Publishers for each attached rokubimini.
   *
   */
  inline void createRokubiminiRosPublishers() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->createRosPublishers();
    }
  }

  /**
   * @fn void createRokubiminiRosServices() const
   *
   * @brief Creates ROS Services for each attached rokubimini.
   *
   */
  inline void createRokubiminiRosServices() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->createRosServices();
    }
  }

  /**
   * @fn void createRokubiminiRosDiagnostics() const
   *
   * @brief Creates ROS Diagnostics for each attached rokubimini.
   *
   */
  inline void createRokubiminiRosDiagnostics() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->createRosDiagnostics();
    }
  }
  /**
   * @fn void publishRosMessages() const
   *
   * @brief Publishes the ROS messages for each attached rokubimini.
   *
   */
  inline void publishRosMessages() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->publishRosMessages();
    }
  }

  /**
   * @fn void startupWithoutCommunication() const
   *
   * @brief Starts up all Rokubimini devices before communication has been established by the Bus Manager.
   *
   */
  inline void startupWithoutCommunication() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->startupWithoutCommunication();
    }
  }

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down every Rokubimini device after
   * communication has been closed.
   *
   * This method shuts down every Rokubimini device after the
   * BusManager has terminated communication with the device. It's virtual since it's implementation-specific.
   *
   */
  inline void shutdownWithoutCommunication() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->shutdownWithoutCommunication();
    }
  }

  /**
   * @fn void shutdownWithCommunication()
   *
   * @brief Shuts down every Rokubimini device before
   * communication has been closed.
   *
   * This method shuts down every Rokubimini device before the
   * BusManager has terminated communication with the device. It's virtual since it's implementation-specific.
   *
   */
  inline void shutdownWithCommunication() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->shutdownWithCommunication();
    }
  }

  /**
   * @fn virtual void startupWithCommunication()
   *
   * @brief Starts up all Rokubimini devices after communication has been established from the Bus Manager.
   *
   * This method is virtual because it's implementation-specific.
   *
   */
  inline void startupWithCommunication() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->startupWithCommunication();
    }
  }

  /**
   * @fn virtual void updateProcessReading()
   *
   * @brief Updates all \a Rokubimini instances with new measurements.
   *
   * This method updates the internal \a Reading variable of all \a Rokubiminis, by getting the new values from its
   * implementation.
   */
  inline void updateProcessReading() const
  {
    for (const auto& rokubimini : rokubiminis_)
    {
      rokubimini->updateProcessReading();
    }
  }

  /**
   * @fn double loadTimeStep()
   *
   * @brief Loads the time_step parameter.
   *
   * It's virtual since it's implementation-specific.
   */
  virtual double loadTimeStep() = 0;

protected:
  /**
   * @var std::vector<std::shared_ptr<Rokubimini>> rokubiminis_
   *
   * @brief List of attached Rokubiminis.
   *
   */
  std::vector<std::shared_ptr<Rokubimini>> rokubiminis_;

  /**
   * @var std::string name_
   *
   * @brief The name of the bus.
   *
   */
  std::string name_;
  /**
   * @var NodeHandlePtr nh_
   *
   * @brief The internal NodeHandle variable.
   *
   */
  NodeHandlePtr nh_;

  /**
   * @var std::atomic<bool> isRunning_
   *
   * @brief Boolean that specifies if the Manager is already running.
   *
   *
   */

  std::atomic<bool> isRunning_{ false };
};

}  // namespace rokubimini
