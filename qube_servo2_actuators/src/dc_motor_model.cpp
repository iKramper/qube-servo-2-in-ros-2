#include "qube_servo2_actuators/dc_motor_model.hpp"
#include "qube_servo2_actuators/motor_parameters.hpp"
#include "qube_servo2_actuators/motor_states.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace qube_servo2::actuators {

DcMotorModel::DcMotorModel( MotorParameters parameters ) : parameters_( std::move( parameters )) {}

const MotorParameters & DcMotorModel::parameters() const noexcept {
    return parameters_;
}

const MotorState & DcMotorModel::state() const noexcept {
    return state_;
}

MotorState DcMotorModel::step( double voltage_cmd, double angular_velocity, double dt ) {

    if( !std::isfinite( voltage_cmd ) || !std::isfinite( angular_velocity ) ||  !std::isfinite( dt ) ) {
        throw std::invalid_argument("Motor inputs must be finite.");
    }

    if( dt <= 0.0 ) {
        return state_;
    }

    // 1. Apply voltage saturation:
    state_.applied_voltage = std::clamp( voltage_cmd, -parameters_.voltage_limit, parameters_.voltage_limit );

    // 2. Compute back emf ( e_b = k_e * omega ):
    state_.back_emf = parameters_.back_emf_constant * angular_velocity;

    // 3. Solve electrical dynamics ( L di/dt + R i = V - e_b -> (discrete solution) -> i[k+1] = i_inf + (i[k] - i_inf) exp(-R dt / L) ):
    const double steady_state_current = ( state_.applied_voltage - state_.back_emf ) / parameters_.resistance;
    const double decay = std::exp( -parameters_.resistance * dt / parameters_.inductance );
    state_.current = steady_state_current + ( state_.current - steady_state_current ) * decay;

    // 4. Apply peak-current saturation:
    state_.current = std::clamp( state_.current, -parameters_.peak_current_limit, parameters_.peak_current_limit );

    // 5. Compute electromagnetic torque ( tau_e = k_t * i ):
    state_.electromagnetic_torque = parameters_.torque_constant * state_.current;

    // 6. Friction:
    const double smooth_sign = std::tanh( angular_velocity / parameters_.friction_smoothing );
    state_.friction_torque = parameters_.viscous_friction * angular_velocity + parameters_.coulomb_friction * smooth_sign;

    // 7. Torque delivered to gz:
    state_.output_torque = state_.electromagnetic_torque - state_.friction_torque;

    return state_;
}

void DcMotorModel::reset() noexcept {
    state_ = {};
}

} // namespace qube_servo2::actuators