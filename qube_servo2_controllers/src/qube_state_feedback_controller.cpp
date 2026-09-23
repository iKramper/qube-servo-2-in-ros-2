#include <algorithm>

namespace controller::pid {

void pid_update () {
    double error = reference - position;
    integral += error * dt;
    double derivate = (error - previous_error) / dt;
    double voltage = (kp * error) + (ki * integral) + (kd * derivate);
    voltage = std::clamp (voltage, -voltage_limit, voltage_limit);
    previous_error = error;
}

void pp_update () {
    const double theta = position_state;
    const double omega = velocity_state;
    const double ref = reference;

    double voltage = -k_theta * theta - k_omega * omega + n_ref * ref;

    volage = std::clamp (voltage, -voltage_limit, voltage_limit);
}

} // namespace controller::pid
