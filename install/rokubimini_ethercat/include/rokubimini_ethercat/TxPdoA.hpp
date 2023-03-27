#pragma once

// std
#include <cstdint>

namespace rokubimini
{
namespace ethercat
{
struct TxPdoA
{
  uint8_t status_;
  uint32_t warningsErrorsFatals_;
  float forceX_;
  float forceY_;
  float forceZ_;
  float torqueX_;
  float torqueY_;
  float torqueZ_;
  uint16_t forceTorqueSaturated_;
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
} __attribute__((packed));

}  // namespace ethercat
}  // namespace rokubimini
