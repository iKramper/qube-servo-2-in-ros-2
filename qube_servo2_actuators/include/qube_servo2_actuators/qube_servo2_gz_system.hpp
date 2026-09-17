#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace qube_servo2::actuators {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class QubeServo2GazeboSystem final : public gz_ros2_control::GazeboSimSystemInterface {

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;

public:
    QubeServo2GazeboSystem();
    ~QubeServo2GazeboSystem() override;

    // Gazebo bridge: conecta el plugin con las entidades y el estado de la simulacion.
    bool initSim(
        rclcpp::Node::SharedPtr & model_nh,
        std::map<std::string, sim::Entity> & joints,
        const hardware_interface::HardwareInfo & hardware_info,
        sim::EntityComponentManager & ecm,
        unsigned int update_rate)
    override;

    // Lifecycle interface: callbacks de la maquina de estados del hardware
    CallbackReturn on_init( const hardware_interface::HardwareComponentInterfaceParams & params ) override;
    CallbackReturn on_configure( const rclcpp_lifecycle::State & previous_state ) override;
    CallbackReturn on_activate( const rclcpp_lifecycle::State & previous_state ) override;
    CallbackReturn on_deactivate( const rclcpp_lifecycle::State & previous_state ) override;

    // Interface declaration: que datos se exponen (state) y que datos se aceptan (command)
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // I/O cycle: sincronizacion de esos datos en cada paso de control:
    hardware_interface::return_type read( const rclcpp::Time & time, const rclcpp::Duration & period ) override;
    hardware_interface::return_type write( const rclcpp::Time & time, const rclcpp::Duration & period ) override;

};

} // namespace qube_servo2::actuators