/*!
 * @file	Node.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <unistd.h>  // for getpid()
#include <memory>    // for std::shared_ptr

#include "bota_worker/WorkerManager.hpp"
#include "bota_worker/WorkerOptions.hpp"

namespace bota_node
{
bool setProcessPriority(int priority);

class Node
{
public:
  using NodeHandlePtr = std::shared_ptr<rclcpp::Node>;

  Node() = delete;
  explicit Node(NodeHandlePtr nh);
  virtual ~Node() = default;

  /*
   * (abstract) interface functions
   */

  /*!
   * Init function, used to initialize all members and starting workers (if any).
   * @return      True if successful. Returning false indicates that the node shall shut down.
   */
  virtual bool init() = 0;
  /*!
   * Pre-Cleanup function, which is called by Nodewrap _before_ stopping workers. (Thread safety up to the user!).
   * This function is called even if init() returned false.
   */
  virtual void preCleanup()
  {
  }
  /*!
   * Cleanup function, called by Nodewrap _after_ stopping workers.
   * This function is called even if init() returned false.
   */
  virtual void cleanup() = 0;

  /*
   * general
   */

  /*!
   * Method to signal nodewrap to shutdown the node.
   */
  void shutdown();

  /*!
   * Helper functions to add Workers to the WorkerManager
   */
  template <class T>
  inline bool addWorker(const std::string& name, const double timestep, bool (T::*fp)(const bota_worker::WorkerEvent&),
                        T* obj, const int priority = 0)
  {
    return workerManager_.addWorker(name, timestep, fp, obj, priority);
  }

  inline bool addWorker(const bota_worker::WorkerOptions& options)
  {
    return workerManager_.addWorker(options);
  }

  /*!
   * Check if WorkerManager is managing a Worker with given name
   * @param name  Name of the worker
   * @return      True if worker was found
   */
  inline bool hasWorker(const std::string& name)
  {
    return workerManager_.hasWorker(name);
  }

  /*!
   * Stop a worker managed by the WorkerManager
   * @param name  Name of the worker
   * @param wait  Whether to wait until the worker has finished or return immediately
   */
  inline void cancelWorker(const std::string& name, const bool wait = true)
  {
    workerManager_.cancelWorker(name, wait);
  }

  /*!
   * Method to stop all workers managed by the WorkerManager
   */
  inline void stopAllWorkers()
  {
    stopAllWorkers(true);
  }
  inline void stopAllWorkers(bool wait)
  {
    workerManager_.cancelWorkers(wait);
  }

  /*
   * accessors
   */
  inline rclcpp::Node& getNodeHandle() const
  {
    return *nh_;
  }

protected:
  NodeHandlePtr nh_;

private:
  bota_worker::WorkerManager workerManager_;
};

}  // namespace bota_node
