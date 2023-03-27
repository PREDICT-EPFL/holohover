#pragma once

// std
#include <cstdint>

namespace rokubimini
{
namespace ethercat
{
// This PDO should only be used as long as the firmware doesn't provide the single PDO
struct TxPdoZ
{
  // PDO A, 0x6000
  uint8_t status_;
  uint32_t warningsErrorsFatals_;
  uint64_t timeStamp_;
  float forceX_;
  float forceY_;
  float forceZ_;
  float torqueX_;
  float torqueY_;
  float torqueZ_;
  uint8_t forceTorqueSaturated_;
  float accelerationX_;
  float accelerationY_;
  float accelerationZ_;
  uint8_t accelerationSaturated_;
  float angularRateX_;
  float angularRateY_;
  float angularRateZ_;
  uint8_t angularRateSaturated_;
  float temperature_;

  // PDO B, 0x6001
  uint8_t statusB_;
  uint32_t warningsErrorsFatalsB_;
  uint64_t timeStampB_;
  float forceXB_;
  float forceYB_;
  float forceZB_;
  float torqueXB_;
  float torqueYB_;
  float torqueZB_;
  uint8_t forceTorqueSaturatedB_;
  float accelerationXB_;
  float accelerationYB_;
  float accelerationZB_;
  uint8_t accelerationSaturatedB_;
  float angularRateXB_;
  float angularRateYB_;
  float angularRateZB_;
  uint8_t angularRateSaturatedB_;
  float temperatureB_;

  float estimatedOrientationX_;
  float estimatedOrientationY_;
  float estimatedOrientationZ_;
  float estimatedOrientationW_;
  float forceInWorldX_;
  float forceInWorldY_;
  float forceInWorldZ_;
  float torqueInWorldX_;
  float torqueInWorldY_;
  float torqueInWorldZ_;

  // PDO C, 0x6002
  uint8_t statusC_;
  uint32_t warningsErrorsFatalsC_;
  uint64_t timeStampC_;
  float forceXC_;
  float forceYC_;
  float forceZC_;
  float torqueXC_;
  float torqueYC_;
  float torqueZC_;
  uint8_t forceTorqueSaturatedC_;
  float accelerationXC_;
  float accelerationYC_;
  float accelerationZC_;
  uint8_t accelerationSaturatedC_;
  float angularRateXC_;
  float angularRateYC_;
  float angularRateZC_;
  uint8_t angularRateSaturatedC_;
  float temperatureC_;

  float estimatedOrientationXC_;
  float estimatedOrientationYC_;
  float estimatedOrientationZC_;
  float estimatedOrientationWC_;
  float forceInWorldXC_;
  float forceInWorldYC_;
  float forceInWorldZC_;
  float torqueInWorldXC_;
  float torqueInWorldYC_;
  float torqueInWorldZC_;

  float externalAccelerationX_;
  float externalAccelerationY_;
  float externalAccelerationZ_;
  uint8_t externalAccelerationSaturated_;
  float externalAngularRateX_;
  float externalAngularRateY_;
  float externalAngularRateZ_;
  uint8_t externalAngularRateSaturated_;
  float externalEstimatedOrientationX_;
  float externalEstimatedOrientationY_;
  float externalEstimatedOrientationZ_;
  float externalEstimatedOrientationW_;
  float relativeOrientationX_;
  float relativeOrientationY_;
  float relativeOrientationZ_;

} __attribute__((packed));

}  // namespace ethercat
}  // namespace rokubimini
