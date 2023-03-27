/*
 * states.hpp
 *
 *  Created on:   Dec 19, 2016
 *  Author(s):    Christian Gehring
 */

#pragma once
#include <stdint.h>

namespace rokubimini
{
namespace serial
{
enum class ConnectionState : unsigned int
{
  DISCONNECTED = 0,
  ISCONNECTING,
  CONNECTED
};

enum class ModeState : unsigned int
{
  RUN_MODE = 0,
  CONFIG_MODE,
  INIT_MODE
};

struct ErrorFlags
{
  uint8_t frame_sync;
  uint8_t crc;
  uint8_t timeout;
  uint8_t polling_sync;
};

}  // namespace serial
}  // namespace rokubimini
