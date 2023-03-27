#pragma once

// std
#include <cstdint>
#include <iostream>
#include <string>

namespace rokubimini
{
namespace fsm
{
// FSM state IDs.
// NOTE: This mapping must be equal to the mapping in the firmware.
// For now I'm using random numbers
#define ROKUBIMINI_STATE_ID_CALIBRATE (0)
#define ROKUBIMINI_STATE_ID_DEVICE_MISSING (1)
#define ROKUBIMINI_STATE_ID_ERROR (2)
#define ROKUBIMINI_STATE_ID_FATAL (3)
#define ROKUBIMINI_STATE_ID_OPERATIONAL (4)
#define ROKUBIMINI_STATE_ID_NA (5)
#define ROKUBIMINI_STATE_ID_STANDBY (6)

// FSM state names.
#define ROKUBIMINI_STATE_NAME_CALIBRATE ("Calibrate")
#define ROKUBIMINI_STATE_NAME_DEVICE_MISSING ("DeviceMissing")
#define ROKUBIMINI_STATE_NAME_ERROR ("Error")
#define ROKUBIMINI_STATE_NAME_FATAL ("Fatal")
#define ROKUBIMINI_STATE_NAME_OPERATIONAL ("Operational")
#define ROKUBIMINI_STATE_NAME_NA ("N/A")
#define ROKUBIMINI_STATE_NAME_STANDBY ("Standby")

/**
 * @enum class StateEnum
 * @brief  FSM state enumerators.
 */
enum class StateEnum
{
  CALIBRATE,
  DEVICE_MISSING,
  ERROR,
  FATAL,
  OPERATIONAL,
  NA,
  STANDBY,
};

/**
 * @fn uint16_t stateEnumToId(const StateEnum stateEnum)
 * @brief Gets an ID matching the state enumeration.
 *
 * @param stateEnum The state enumeration.
 * @return The ID matching the state enumeration.
 */
uint16_t stateEnumToId(const StateEnum stateEnum);

/**
 * @fn StateEnum stateIdToEnum(uint16_t stateId)
 * @brief Gets a state enumeration matching the state ID.
 *
 * @param stateId The state ID.
 * @return The state enumeration matching the state ID.
 */
StateEnum stateIdToEnum(uint16_t stateId);

/**
 * @fn std::string stateEnumToName(const StateEnum stateEnum)
 * @brief Gets a name matching the state enumeration.
 *
 * @param stateEnum The state enumeration.
 * @return The name matching the state enumeration.
 */
std::string stateEnumToName(const StateEnum stateEnum);

/**
 * @fn StateEnum stateNameToEnum(const std::string &string)
 * @brief Gets a state enumeration matching the state name.
 *
 * @param string The state name.
 * @return The state enumeration matching the state name.
 */
StateEnum stateNameToEnum(const std::string& string);

/**
 * @fn std::ostream &operator<<(std::ostream &out, const StateEnum stateEnum)
 * @brief Outputs the name of the state enumeration to an output stream.
 *
 * @param out The output stream.
 * @param stateEnum The state enumeration.
 * @return The output stream with the state enumeration streamed to it.
 */
std::ostream& operator<<(std::ostream& out, const StateEnum stateEnum);

}  // namespace fsm
}  // namespace rokubimini
