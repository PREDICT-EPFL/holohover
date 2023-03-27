#pragma once

// std
#include <cstdint>

namespace rokubimini
{
namespace ethercat
{
struct TxPdoB
{
  // TxPdoA
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

} __attribute__((packed));

}  // namespace ethercat
}  // namespace rokubimini
