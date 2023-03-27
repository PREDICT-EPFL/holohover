#pragma once

// std
#include <cstdint>

namespace rokubimini
{
namespace ethercat
{
namespace sdo
{
// Not sure if needed
struct RxSdoCalibration
{
  uint32_t passphrase_{ 0 };
  float Force_Torque_Calibration_Matrix_0_0_{ 0.0 };
  float Force_Torque_Calibration_Matrix_0_1_{ 0.0 };
  float Force_Torque_Calibration_Matrix_0_2_{ 0.0 };
  float Force_Torque_Calibration_Matrix_0_3_{ 0.0 };
  float Force_Torque_Calibration_Matrix_0_4_{ 0.0 };
  float Force_Torque_Calibration_Matrix_0_5_{ 0.0 };
  float Force_Torque_Calibration_Matrix_1_0_{ 0.0 };
  float Force_Torque_Calibration_Matrix_1_1_{ 0.0 };
  float Force_Torque_Calibration_Matrix_1_2_{ 0.0 };
  float Force_Torque_Calibration_Matrix_1_3_{ 0.0 };
  float Force_Torque_Calibration_Matrix_1_4_{ 0.0 };
  float Force_Torque_Calibration_Matrix_1_5_{ 0.0 };
  float Force_Torque_Calibration_Matrix_2_0_{ 0.0 };
  float Force_Torque_Calibration_Matrix_2_1_{ 0.0 };
  float Force_Torque_Calibration_Matrix_2_2_{ 0.0 };
  float Force_Torque_Calibration_Matrix_2_3_{ 0.0 };
  float Force_Torque_Calibration_Matrix_2_4_{ 0.0 };
  float Force_Torque_Calibration_Matrix_2_5_{ 0.0 };
  float Force_Torque_Calibration_Matrix_3_0_{ 0.0 };
  float Force_Torque_Calibration_Matrix_3_1_{ 0.0 };
  float Force_Torque_Calibration_Matrix_3_2_{ 0.0 };
  float Force_Torque_Calibration_Matrix_3_3_{ 0.0 };
  float Force_Torque_Calibration_Matrix_3_4_{ 0.0 };
  float Force_Torque_Calibration_Matrix_3_5_{ 0.0 };
  float Force_Torque_Calibration_Matrix_4_0_{ 0.0 };
  float Force_Torque_Calibration_Matrix_4_1_{ 0.0 };
  float Force_Torque_Calibration_Matrix_4_2_{ 0.0 };
  float Force_Torque_Calibration_Matrix_4_3_{ 0.0 };
  float Force_Torque_Calibration_Matrix_4_4_{ 0.0 };
  float Force_Torque_Calibration_Matrix_4_5_{ 0.0 };
  float Force_Torque_Calibration_Matrix_5_0_{ 0.0 };
  float Force_Torque_Calibration_Matrix_5_1_{ 0.0 };
  float Force_Torque_Calibration_Matrix_5_2_{ 0.0 };
  float Force_Torque_Calibration_Matrix_5_3_{ 0.0 };
  float Force_Torque_Calibration_Matrix_5_4_{ 0.0 };
  float Force_Torque_Calibration_Matrix_5_5_{ 0.0 };
  float Force_Torque_Calibration_Temperature_{ 0.0 };
  float Force_Torque_Calibration_Offset_Fx_{ 0.0 };
  float Force_Torque_Calibration_Offset_Fy_{ 0.0 };
  float Force_Torque_Calibration_Offset_Fz_{ 0.0 };
  float Force_Torque_Calibration_Offset_Tx_{ 0.0 };
  float Force_Torque_Calibration_Offset_Ty_{ 0.0 };
  float Force_Torque_Calibration_Offset_Tz_{ 0.0 };
  uint8_t ADC_Range{ 0 };
  float Thermistor_Calibration_0{ 0.0 };
  float Thermistor_Calibration_1{ 0.0 };
  float Force_Torque_Temperature_Gain_0{ 0.0 };
  float Force_Torque_Temperature_Gain_1{ 0.0 };
  float Force_Torque_Temperature_Gain_2{ 0.0 };
  float Force_Torque_Temperature_Gain_3{ 0.0 };
  float Force_Torque_Temperature_Gain_4{ 0.0 };
  float Force_Torque_Temperature_Gain_5{ 0.0 };
  float Acceleration_Calibration_Offset_x{ 0.0 };
  float Acceleration_Calibration_Offset_y{ 0.0 };
  float Acceleration_Calibration_Offset_z{ 0.0 };
  float External_Acceleration_Calibration_Offset_x{ 0.0 };
  float External_Acceleration_Calibration_Offset_y{ 0.0 };
  float External_Acceleration_Calibration_Offset_z{ 0.0 };
  float Angular_Rate_Calibration_Offset_x{ 0.0 };
  float Angular_Rate_Calibration_Offset_y{ 0.0 };
  float Angular_Rate_Calibration_Offset_z{ 0.0 };
  float External_Angular_Rate_Calibration_Offset_x{ 0.0 };
  float External_Angular_Rate_Calibration_Offset_y{ 0.0 };
  float External_Angular_Rate_Calibration_Offset_z{ 0.0 };
  float Sensor_Inertia_Mass{ 0.0 };
  float Sensor_Inertia_Dx{ 0.0 };
  float Sensor_Inertia_Dy{ 0.0 };
  float Sensor_Inertia_Dz{ 0.0 };
  float Sensor_Inertia_Ixx{ 0.0 };
  float Sensor_Inertia_Iyy{ 0.0 };
  float Sensor_Inertia_Izz{ 0.0 };
  float Sensor_Inertia_Ixy{ 0.0 };
  float Sensor_Inertia_Iyz{ 0.0 };
  float Sensor_Inertia_Ixz{ 0.0 };
  uint32_t Calibration_Date{ 0 };
  uint8_t Serial_Number[12]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  uint8_t Firmware_Version[12]{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

} __attribute__((packed));
}  // namespace sdo
}  // namespace ethercat
}  // namespace rokubimini
