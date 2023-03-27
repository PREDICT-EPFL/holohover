#pragma once

// std
#include <cstdint>

namespace rokubimini
{
namespace ethercat
{
// Not sure if needed
struct RxPdo
{
  uint8_t digitalOutput_ = 0;
} __attribute__((packed));

}  // namespace ethercat
}  // namespace rokubimini
