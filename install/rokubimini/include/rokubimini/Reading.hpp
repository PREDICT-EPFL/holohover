#pragma once

#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include <rokubimini/Statusword.hpp>
#include <boost/math/constants/constants.hpp>

namespace rokubimini
{
/**
 * @var static constexpr double G_TO_METERS_PER_SECOND_SQUARED
 *
 * @brief g to m/s^2 abbreviation for convenience.
 *
 */
static constexpr double G_TO_METERS_PER_SECOND_SQUARED = 9.80665;

/**
 * @var static constexpr double DEG_TO_RAD
 *
 * @brief degrees to rad abbreviation for convenience.
 *
 */
static constexpr double DEG_TO_RAD = boost::math::constants::pi<double>() / 180;

/**
 * @class Reading
 *
 * @brief Class representing the readings received from the
 * rokubi mini devices.
 *
 */
class Reading
{
public:
  using ImuType = sensor_msgs::msg::Imu;
  using WrenchType = geometry_msgs::msg::WrenchStamped;
  using TempType = sensor_msgs::msg::Temperature;

  /**
   * @fn Reading()
   *
   * @brief Default constructor.
   *
   */
  Reading() = default;
  virtual ~Reading() = default;

  /**
   * @fn const ImuType &getImu() const
   *
   * @brief Gets the \a imu variable.
   * @return The \a imu values.
   *
   */
  const ImuType& getImu() const
  {
    return imu_;
  }

  /**
   * @fn ImuType &getImu()
   *
   * @brief Non-const version of getImu() const. Gets the \a imu variable.
   * @return The \a imu values.
   *
   */
  ImuType& getImu()
  {
    return imu_;
  }

  /**
   * @fn const WrenchType &getWrench() const
   *
   * @brief Gets the \a wrench variable.
   * @return The \a wrench values.
   *
   */

  const WrenchType& getWrench() const
  {
    return wrench_;
  }

  /**
   * @fn WrenchType &getWrench()
   *
   * @brief Non-const version of getWrench() const. Gets the \a wrench variable.
   * @return The \a wrench values.
   *
   */
  WrenchType& getWrench()
  {
    return wrench_;
  }

  /**
   * @fn const ImuType &getExternalImu() const
   *
   * @brief Gets the \a externalImu variable.
   * @return The \a externalImu values.
   *
   */
  const ImuType& getExternalImu() const
  {
    return externalImu_;
  }

  /**
   * @fn ImuType &getExternalImu()
   *
   * @brief Non-const version of getExternalImu() const. Gets the \a externalImu variable.
   * @return The \a externalImu values.
   *
   */
  ImuType& getExternalImu()
  {
    return externalImu_;
  }

  /**
   * @fn const Statusword &getStatusword() const
   *
   * @brief Gets the \a statusword variable.
   * @return The \a statusword value.
   *
   */
  const Statusword& getStatusword() const
  {
    return statusword_;
  }

  /**
   * @fn void setStatusword(const Statusword &statusword)
   *
   * @brief Sets the \a statusword variable.
   * @param statusword The value to set.
   *
   */
  void setStatusword(const Statusword& statusword)
  {
    statusword_ = statusword;
  }

  /**
   * @fn bool isForceTorqueSaturated() const
   *
   * @brief Gets the \a isForceTorqueSaturated flag.
   * @return The \a isForceTorqueSaturated value.
   *
   */
  bool isForceTorqueSaturated() const
  {
    return isForceTorqueSaturated_;
  }

  /**
   * @fn void setForceTorqueSaturated(const bool isForceTorqueSaturated)
   *
   * @brief Sets the \a isForceTorqueSaturated variable.
   * @param isForceTorqueSaturated The value to set.
   *
   */
  void setForceTorqueSaturated(const bool isForceTorqueSaturated)
  {
    isForceTorqueSaturated_ = isForceTorqueSaturated;
  }

  /**
   * @fn const TempType& getTemperature() const
   *
   * @brief Gets the \a temperature flag.
   * @return The \a temperature value.
   *
   */
  const TempType& getTemperature() const
  {
    return temperature_;
  }

  /**
   * @fn TempType getTemperature() const
   *
   * @brief Gets the \a temperature flag.
   * @return The \a temperature value.
   *
   */
  TempType& getTemperature()
  {
    return temperature_;
  }

  /**
   * @fn void setTemperature(const TempType& temperature)
   *
   * @brief Sets the \a temperature variable.
   * @param temperature The value to set.
   *
   */
  void setTemperature(const TempType& temperature)
  {
    temperature_ = temperature;
  }

protected:
  /**
   * @var ImuType imu_
   *
   * @brief The imu variable.
   *
   */
  ImuType imu_;

  /**
   * @var WrenchType wrench_
   *
   * @brief The wrench variable.
   *
   */
  WrenchType wrench_;

  /**
   * @var ImuType externalImu_
   *
   * @brief The externalImu variable.
   *
   */
  ImuType externalImu_;

  /**
   * @var bool isForceTorqueSaturated_
   *
   * @brief The isForceTorqueSaturated variable.
   *
   */
  bool isForceTorqueSaturated_{ false };

  /**
   * @var TempType temperature_
   *
   * @brief The temperature variable.
   *
   */
  TempType temperature_;

  /**
   * @var float statusword_
   *
   * @brief The statusword variable.
   *
   */
  Statusword statusword_;
};

}  // namespace rokubimini
