#pragma once

#include <stdexcept>
namespace qube_servo2::actuators {

struct MotorParameters {

    double resistance{0.0};
    double inductance{0.0};
    double torque_constant{0.0};
    double back_emf_constant{0.0};
    double voltage_limit{0.0};
    double peak_current_limit{0.0};
    double viscous_friction{0.0};
    double coulomb_friction{0.0};
    double friction_smoothing{0.0};

    void validate() const {

        if( resistance <= 0.0 ) throw std::invalid_argument("motor.resistance must be grater than 0.0");
        if( inductance <= 0.0 ) throw std::invalid_argument("motor.inductance must be grater than 0.0");
        if( torque_constant <= 0.0 ) throw std::invalid_argument("motor.torque_constant must be grater than 0.0");
        if( back_emf_constant <= 0.0 ) throw std::invalid_argument("motor.back_emf_constant must be grater than 0.0");
        if( voltage_limit <= 0.0 ) throw std::invalid_argument("motor.voltage_limit must be grater than 0.0");
        if( peak_current_limit <= 0.0 ) throw std::invalid_argument("motor.peak_current_limit must be grater than 0.0");
        if( viscous_friction < 0.0 ) throw std::invalid_argument("motor.viscous_friction must be grater than 0.0");
        if( coulomb_friction < 0.0 ) throw std::invalid_argument("motor.coulomb_friction must be grater than 0.0");
        if( friction_smoothing <= 0.0 ) throw std::invalid_argument("motor.friction_smoothing must be grater than 0.0");

    }
};

} // namespace qube_servo2::actuators
