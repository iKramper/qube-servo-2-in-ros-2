#pragma once

#include "qube_servo2_actuators/motor_parameters.hpp"
#include "qube_servo2_actuators/motor_states.hpp"

namespace qube_servo2::actuators {

class DcMotorModel final {

private:
    const MotorParameters parameters_;
    MotorState state_;

public:
    explicit DcMotorModel( MotorParameters parameters );
    const MotorParameters & parameters() const noexcept;
    const MotorState & state() const noexcept;
    MotorState step( double voltage_cmd, double angular_velocity, double dt );
    void reset() noexcept;
};

} // namespace qube_servo2::actuators