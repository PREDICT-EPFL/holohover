#pragma once

namespace rokubimini
{
namespace ethercat
{
// Todo fill with available PDOs
enum class PdoTypeEnum : int8_t
{
  NA = 0,
  A,  // Normal usage
  B,  // Advanced Measurements??
  C,
  Z,  // We use z for now, since the slave outputs all the pdo at the same time
  EXTIMU
};

}  // namespace ethercat
}  // namespace rokubimini
