#pragma once

#include <QString>
#include <QMetaType>

namespace qube_servo2::gui::models {

struct TelemetrySample {
    double ros_time_s{0.0};
    double command_voltage{0.0};
    double position_rad{0.0};
    double velocity_rad_s{0.0};
    double current_a{0.0};
    double applied_voltage_v{0.0};
    double back_emf_v{0.0};
    double motor_torque_nm{0.0};
    bool valid{false};

    double position_deg() const noexcept;
    double velocity_rpm() const noexcept;
    double electrical_power_w() const noexcept;
    double mechanical_power_w() const noexcept;
};

}  // namespace qube_servo2::gui::models

Q_DECLARE_METATYPE(qube_servo2::gui::models::TelemetrySample)
