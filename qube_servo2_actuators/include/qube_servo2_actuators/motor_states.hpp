#pragma once

namespace qube_servo2::actuators {

struct MotorState {

    double current{0.0};
    double applied_voltage{0.0};
    double back_emf{0.0};
    double electromagnetic_torque{0.0};
    double friction_torque{0.0};
    double output_torque{0.0};

};

} // namespace qube_servo2::actuators