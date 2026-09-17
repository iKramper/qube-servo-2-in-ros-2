#include "qube_servo2_actuators/qube_servo2_gz_system.hpp"

// Gazebo
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>

// Gazebo components:
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

// ros2_control:
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// Pluginlib:
#include "pluginlib/class_list_macros.hpp"

// QUBE Servo 2:
#include "qube_servo2_actuators/dc_motor_model.hpp"
#include "qube_servo2_actuators/motor_parameters.hpp"


namespace qube_servo2::actuators {

namespace
{

double getRequiredParameter(
    const hardware_interface::HardwareInfo & hardware_info,
    const std::string & parameter_name)
{
    const auto iterator =
        hardware_info.hardware_parameters.find(parameter_name);

    if (iterator == hardware_info.hardware_parameters.end())
    {
        throw std::runtime_error(
            "Missing required motor parameter: '" +
            parameter_name +
            "'");
    }

    try
    {
        return std::stod(iterator->second);
    }
    catch (const std::exception &)
    {
        throw std::runtime_error(
            "Motor parameter '" +
            parameter_name +
            "' does not contain a valid numeric value: '" +
            iterator->second +
            "'");
    }
}

}  // namespace


struct QubeServo2GazeboSystem::Impl {

    sim::EntityComponentManager *ecm{ nullptr };
    sim::Entity motor_joint{ sim::kNullEntity };
    std::string motor_joint_name{};
    unsigned int update_rate{ 0 };

    MotorParameters motor_parameters{};
    std::unique_ptr<DcMotorModel> motor_model{ nullptr };

    double voltage_cmd{ 0.0 };
    double position{ 0.0 };
    double velocity{ 0.0 };
    double current{0.0};
    double applied_voltage{0.0};
    double back_emf{0.0};
    double motor_torque{0.0};

    std::vector<hardware_interface::StateInterface> state_interfaces{};
    std::vector<hardware_interface::CommandInterface> command_interfaces{};

};

bool QubeServo2GazeboSystem::initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & ecm,
    unsigned int update_rate)
{
    nh_ = model_nh;
    impl_->ecm = &ecm;
    impl_->update_rate = update_rate;

    // Validate hardware definition
    if (hardware_info.joints.size() != 1)
    {
        RCLCPP_ERROR(
            nh_->get_logger(),
            "QUBE Servo 2 expects exactly one ros2_control joint, "
            "but %zu were provided.",
            hardware_info.joints.size());
        return false;
    }

    const auto & joint_info =
        hardware_info.joints.front();
    impl_->motor_joint_name = joint_info.name;

    // Find Gazebo joint entity
    const auto joint_iterator = joints.find(impl_->motor_joint_name);

    if( joint_iterator == joints.end() ) {
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Joint '%s' exists in ros2_control but was not found in Gazebo.",
            impl_->motor_joint_name.c_str());
        return false;
    }
    impl_->motor_joint = joint_iterator->second;
    const sim::Joint joint{impl_->motor_joint};
    if ( !joint.Valid(ecm) ) {
        RCLCPP_ERROR( nh_->get_logger(), "Gazebo returned an invalid entity for joint '%s'.", impl_->motor_joint_name.c_str() );
        return false;
    }

    joint.EnablePositionCheck(ecm);
    joint.EnableVelocityCheck(ecm);

    try
    {
        impl_->motor_parameters.resistance =
            getRequiredParameter(
                hardware_info,
                "resistance");

        impl_->motor_parameters.inductance =
            getRequiredParameter(
                hardware_info,
                "inductance");

        impl_->motor_parameters.torque_constant =
            getRequiredParameter(
                hardware_info,
                "torque_constant");

        impl_->motor_parameters.back_emf_constant =
            getRequiredParameter(
                hardware_info,
                "back_emf_constant");

        impl_->motor_parameters.voltage_limit =
            getRequiredParameter(
                hardware_info,
                "voltage_limit");

        impl_->motor_parameters.peak_current_limit =
            getRequiredParameter(
                hardware_info,
                "peak_current_limit");

        impl_->motor_parameters.viscous_friction =
            getRequiredParameter(
                hardware_info,
                "viscous_friction");

        impl_->motor_parameters.coulomb_friction =
            getRequiredParameter(
                hardware_info,
                "coulomb_friction");

        impl_->motor_parameters.friction_smoothing =
            getRequiredParameter(
                hardware_info,
                "friction_smoothing");

        // Physical validation
        impl_->motor_parameters.validate();

        // Motor model construction
        impl_->motor_model = std::make_unique<DcMotorModel>( impl_->motor_parameters );
    }
    catch (const std::exception & exception) {
        RCLCPP_ERROR(
            nh_->get_logger(),
            "Unable to initialize QUBE Servo 2 motor parameters: %s",
            exception.what());
        return false;
    }

    // State interfaces
    for (const auto & state_interface : joint_info.state_interfaces)
    {
        if (
            state_interface.name ==
            hardware_interface::HW_IF_POSITION)
        {
            impl_->state_interfaces.emplace_back(
                impl_->motor_joint_name,
                hardware_interface::HW_IF_POSITION,
                &impl_->position);
        }

        else if (
            state_interface.name ==
            hardware_interface::HW_IF_VELOCITY)
        {
            impl_->state_interfaces.emplace_back(
                impl_->motor_joint_name,
                hardware_interface::HW_IF_VELOCITY,
                &impl_->velocity);
        }

        else if (state_interface.name == "current")
        {
            impl_->state_interfaces.emplace_back(
                impl_->motor_joint_name,
                "current",
                &impl_->current);
        }

        else if (state_interface.name == "applied_voltage")
        {
            impl_->state_interfaces.emplace_back(
                impl_->motor_joint_name,
                "applied_voltage",
                &impl_->applied_voltage);
        }

        else if (state_interface.name == "back_emf")
        {
            impl_->state_interfaces.emplace_back(
                impl_->motor_joint_name,
                "back_emf",
                &impl_->back_emf);
        }

        else if (state_interface.name == "motor_torque")
        {
            impl_->state_interfaces.emplace_back(
                impl_->motor_joint_name,
                "motor_torque",
                &impl_->motor_torque);
        }

        else
        {
            RCLCPP_ERROR(
                nh_->get_logger(),
                "Unsupported state interface '%s' "
                "for QUBE joint '%s'.",
                state_interface.name.c_str(),
                impl_->motor_joint_name.c_str());

            return false;
        }
    }

    // Command interfaces
    for (const auto & command_interface : joint_info.command_interfaces)
    {
        if (command_interface.name == "voltage")
        {
            impl_->command_interfaces.emplace_back(
                impl_->motor_joint_name,
                "voltage",
                &impl_->voltage_cmd);
        }
        else
        {
            RCLCPP_ERROR(
                nh_->get_logger(),
                "Unsupported command interface '%s' "
                "for QUBE joint '%s'. Expected 'voltage'.",
                command_interface.name.c_str(),
                impl_->motor_joint_name.c_str());

            return false;
        }
    }

    // Required interface validation
    if (impl_->command_interfaces.empty())
    {
        RCLCPP_ERROR(
            nh_->get_logger(),
            "QUBE joint '%s' has no voltage command interface.",
            impl_->motor_joint_name.c_str());

        return false;
    }

    // Initialization report
    RCLCPP_INFO(
        nh_->get_logger(),
        "QUBE Servo 2 Gazebo actuator initialized.");

    RCLCPP_INFO(
        nh_->get_logger(),
        "Joint: %s",
        impl_->motor_joint_name.c_str());

    RCLCPP_INFO(
        nh_->get_logger(),
        "ros2_control update rate: %u Hz",
        impl_->update_rate);

    RCLCPP_INFO(
        nh_->get_logger(),
        "Motor resistance: %.6f Ohm",
        impl_->motor_parameters.resistance);

    RCLCPP_INFO(
        nh_->get_logger(),
        "Motor inductance: %.9f H",
        impl_->motor_parameters.inductance);

    RCLCPP_INFO(
        nh_->get_logger(),
        "Torque constant: %.6f N*m/A",
        impl_->motor_parameters.torque_constant);

    RCLCPP_INFO(
        nh_->get_logger(),
        "Back-EMF constant: %.6f V/(rad/s)",
        impl_->motor_parameters.back_emf_constant);

    RCLCPP_INFO(
        nh_->get_logger(),
        "Voltage limit: +/- %.3f V",
        impl_->motor_parameters.voltage_limit);

    RCLCPP_INFO(
        nh_->get_logger(),
        "Peak current limit: +/- %.3f A",
        impl_->motor_parameters.peak_current_limit);

    return true;
}

QubeServo2GazeboSystem::QubeServo2GazeboSystem() : impl_{std::make_unique<Impl>()} {}

QubeServo2GazeboSystem::~QubeServo2GazeboSystem() = default;

CallbackReturn QubeServo2GazeboSystem::on_init( const hardware_interface::HardwareComponentInterfaceParams & params ) {

    if ( hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS ) return CallbackReturn::ERROR;
    return CallbackReturn::SUCCESS;

}

CallbackReturn QubeServo2GazeboSystem::on_configure( const rclcpp_lifecycle::State & /* previous_state */ ) {

    if (impl_->motor_joint == sim::kNullEntity) {
        RCLCPP_ERROR( nh_->get_logger(), "QUBE Servo 2 motor joint has not been initialized." );
        return CallbackReturn::ERROR;
    }

    if (impl_->motor_model == nullptr) {
        RCLCPP_ERROR( nh_->get_logger(), "QUBE Servo 2 motor model has not been initialized." );
        return CallbackReturn::ERROR;
    }

    RCLCPP_INFO( nh_->get_logger(), "QUBE Servo 2 actuator successfully configured." );

    return CallbackReturn::SUCCESS;
}

CallbackReturn QubeServo2GazeboSystem::on_activate( const rclcpp_lifecycle::State & /* previous_state */ ) {

    // Inicio seguro:
    impl_->voltage_cmd = 0.0;
    impl_->current = 0.0;
    impl_->applied_voltage = 0.0;
    impl_->back_emf = 0.0;
    impl_->motor_torque = 0.0;

    if (impl_->motor_model != nullptr) impl_->motor_model->reset();

    if ( impl_->ecm != nullptr && impl_->motor_joint != sim::kNullEntity )
        impl_->ecm->SetComponentData<sim::components::JointForceCmd>( impl_->motor_joint, {0.0} );

    RCLCPP_INFO( nh_->get_logger(), "QUBE Servo 2 actuator successfully activated." );

    return CallbackReturn::SUCCESS;
}

CallbackReturn QubeServo2GazeboSystem::on_deactivate( const rclcpp_lifecycle::State & /* previous_state */ ) {

    // Apago seguro:
    impl_->voltage_cmd = 0.0;
    if ( impl_->ecm != nullptr && impl_->motor_joint != sim::kNullEntity )
        impl_->ecm->SetComponentData<sim::components::JointForceCmd>( impl_->motor_joint, {0.0} );

    RCLCPP_INFO( nh_->get_logger(), "QUBE Servo 2 actuator successfully deactivated." );

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> QubeServo2GazeboSystem::export_state_interfaces() {
    return std::move(impl_->state_interfaces);
}

std::vector<hardware_interface::CommandInterface> QubeServo2GazeboSystem::export_command_interfaces() {
    return std::move(impl_->command_interfaces);
}

hardware_interface::return_type QubeServo2GazeboSystem::read( const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */ ) {

    if ( impl_->ecm == nullptr || impl_->motor_joint == sim::kNullEntity ) return hardware_interface::return_type::ERROR;

    // Posicion:
    const auto * joint_position = impl_->ecm->Component<sim::components::JointPosition>( impl_->motor_joint );

    if ( joint_position == nullptr || joint_position->Data().empty() ) {
        RCLCPP_ERROR( nh_->get_logger(), "Unable to read position from joint '%s'.", impl_->motor_joint_name.c_str() );
        return hardware_interface::return_type::ERROR;
    }

    // Velocidad:
    const auto * joint_velocity = impl_->ecm->Component<sim::components::JointVelocity>( impl_->motor_joint );
    if ( joint_velocity == nullptr || joint_velocity->Data().empty() ) {
        RCLCPP_ERROR( nh_->get_logger(), "Unable to read velocity from joint '%s'.", impl_->motor_joint_name.c_str() );
        return hardware_interface::return_type::ERROR;
    }

    // Actualizacion estados mecanicos:
    impl_->position = joint_position->Data().front();
    impl_->velocity = joint_velocity->Data().front();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type
QubeServo2GazeboSystem::write(
    const rclcpp::Time & /* time */,
    const rclcpp::Duration & period)
{
    if (
        impl_->ecm == nullptr ||
        impl_->motor_joint == sim::kNullEntity ||
        impl_->motor_model == nullptr)
    {
        return hardware_interface::return_type::ERROR;
    }

    // Control period
    const double dt =
        period.seconds();

    // The first control iteration can have dt == 0.
    if ( dt <= 0.0 ) {
        return hardware_interface::return_type::OK;
    }

    if (!std::isfinite(dt)) {
        RCLCPP_ERROR( nh_->get_logger(), "Invalid control period received by QUBE actuator." );
        return hardware_interface::return_type::ERROR;
    }

    try
    {
        const MotorState motor_state =
            impl_->motor_model->step(
                impl_->voltage_cmd,
                impl_->velocity,
                dt);

        // Expose electrical states through ros2_control
        impl_->current = motor_state.current;
        impl_->applied_voltage = motor_state.applied_voltage;
        impl_->back_emf = motor_state.back_emf;
        impl_->motor_torque = motor_state.output_torque;

        // Apply calculated motor torque to Gazebo
        impl_->ecm->SetComponentData<sim::components::JointForceCmd>( impl_->motor_joint, {impl_->motor_torque} );
    }
    catch (const std::exception & exception)
    {
        RCLCPP_ERROR(
            nh_->get_logger(),
            "QUBE motor model error: %s",
            exception.what());

        // Fail-safe: remove motor torque
        impl_->ecm->SetComponentData<sim::components::JointForceCmd>(
            impl_->motor_joint,
        {0.0});

        return hardware_interface::return_type::ERROR;
    }


    return hardware_interface::return_type::OK;
}

} // namespace qube_servo2::actuators

PLUGINLIB_EXPORT_CLASS( qube_servo2::actuators::QubeServo2GazeboSystem, gz_ros2_control::GazeboSimSystemInterface )