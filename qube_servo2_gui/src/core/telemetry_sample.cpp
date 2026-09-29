#include "qube_servo2_gui/core/models/telemetry_sample.hpp"
namespace qube_servo2::gui::models {
namespace { constexpr double kPi = 3.14159265358979323846; }
double TelemetrySample::position_deg() const noexcept { return position_rad * 180.0 / kPi; }
double TelemetrySample::velocity_rpm() const noexcept { return velocity_rad_s * 60.0 / (2.0 * kPi); }
double TelemetrySample::electrical_power_w() const noexcept { return applied_voltage_v * current_a; }
double TelemetrySample::mechanical_power_w() const noexcept { return motor_torque_nm * velocity_rad_s; }
}
