/*!
 * @file	Nodewrap.hpp
 * @author	Philipp Leemann
 * @date	July, 2016
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>

#include "bota_worker/WorkerOptions.hpp"
#include "bota_signal_handler/SignalHandler.hpp"

namespace bota_node
{
template <class NodeImpl>
class Nodewrap
{
public:
  Nodewrap() = delete;
  /*!
   * @param argc
   * @param argv
   * @param nodeName                name of the node
   * @param numSpinners             number of async ros spinners. Set to -1 to get value from ros params. A value of 0
   * means to use the
   * number of processor cores.
   * @param installSignalHandler    set to False to use the ros internal signal handler instead
   */
  Nodewrap(int argc, char** argv, const std::string& nodeName, int numSpinners = -1,
           const bool installSignalHandler = true)
    : nh_(nullptr)
    , executor_(nullptr)
    , executorThread_(nullptr)
    , impl_(nullptr)
    , signalHandlerInstalled_(installSignalHandler)
    , running_(false)
    , cvRunning_()
    , mutexRunning_()
  {
    if (signalHandlerInstalled_)
    {
      rclcpp::InitOptions options{};
      options.shutdown_on_signal = false;
      rclcpp::init(argc, argv, options);
    }
    else
    {
      rclcpp::init(argc, argv);
    }

    nh_ = std::make_shared<rclcpp::Node>("bota_node");

    if (numSpinners == -1)
    {
      nh_->declare_parameter<int>("num_spinners", 2);
      if (!nh_->get_parameter("num_spinners", numSpinners))
      {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not acquire parameter '"
                                                  << nh_->get_name() << "/num_spinners"
                                                  << "' from server. Parameter still contains '" << numSpinners
                                                  << "'.");
      }
    }

    impl_.reset(new NodeImpl(nh_));
    executor_.reset(new rclcpp::executors::MultiThreadedExecutor(rclcpp::ExecutorOptions(), numSpinners));

    checkSteadyClock();
  }

  // not necessary to call ros::shutdown in the destructor, this is done as soon as the last nodeHandle
  // is destructed
  virtual ~Nodewrap() = default;

  /*!
   * blocking call, executes init, run (if init() was successful) and cleanup (independent of the success of init()).
   */
  bool execute()
  {
    bool init_success = init();
    if (init_success)
    {
      run();
    }
    cleanup();
    return init_success;
  }

  /*!
   * Initializes the node
   */
  bool init()
  {
    if (signalHandlerInstalled_)
    {
      signal_handler::SignalHandler::bindAll(&Nodewrap::signalHandler, this);
    }

    executor_->add_node(nh_);
    executorThread_.reset(new std::thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, executor_.get())));
    if (!impl_->init())
    {
      RCLCPP_ERROR(nh_->get_logger(), "Failed to init Node %s!", nh_->get_name());
      return false;
    }

    running_ = true;
    return true;
  }

  /*!
   * blocking call, returns when the program should shut down
   */
  void run()
  {
    // returns if running_ is false
    std::unique_lock<std::mutex> lk(mutexRunning_);
    cvRunning_.wait(lk, [this] { return !running_; });
  }

  /*!
   * Stops the workers, ros spinners and calls cleanup of the underlying instance of bota_node::Node
   */
  void cleanup()
  {
    if (signalHandlerInstalled_)
    {
      signal_handler::SignalHandler::unbindAll(&Nodewrap::signalHandler, this);
    }

    impl_->preCleanup();
    impl_->stopAllWorkers();
    // Stop executor thread
    executor_->cancel();
    executorThread_->join();
    impl_->cleanup();
  }

  /*!
   * Stops execution of the run(..) function.
   */
  void stop()
  {
    std::lock_guard<std::mutex> lk(mutexRunning_);
    running_ = false;
    cvRunning_.notify_all();
  }

public:  /// INTERNAL FUNCTIONS
  void signalHandler(const int signum)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Signal: " << signum);
    stop();

    if (signum == SIGSEGV)
    {
      signal(signum, SIG_DFL);
      kill(getpid(), signum);
    }
  }

  static void checkSteadyClock()
  {
    if (std::chrono::steady_clock::period::num != 1 || std::chrono::steady_clock::period::den != 1000000000)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Bota Node"), "std::chrono::steady_clock does not have a nanosecond resolution!");
    }
    if (std::chrono::system_clock::period::num != 1 || std::chrono::system_clock::period::den != 1000000000)
    {
      RCLCPP_ERROR(rclcpp::get_logger("Bota Node"), "std::chrono::system_clock does not have a nanosecond resolution!");
    }
  }

  NodeImpl* getImplPtr()
  {
    return impl_.get();
  }

protected:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::unique_ptr<std::thread> executorThread_;
  std::unique_ptr<NodeImpl> impl_;

  bool signalHandlerInstalled_;

  std::atomic<bool> running_;
  std::condition_variable cvRunning_;
  std::mutex mutexRunning_;
};

}  // namespace bota_node
