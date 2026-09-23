#pragma once

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

#include "std_msgs/msg/float64.hpp"

namespace qube_servo2::controllers {

class QubeVoltageController : public controller_interface::ControllerInterface {
	private:
		std::string joint_name_;
		std::string interface_name_;
		std::string reference_topic_;

		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reference_subscriber_;
		realtime_tools::RealtimeBuffer<double> reference_buffer_;

	public:
		controller_interface::InterfaceConfiguration command_interface_configuration () const override;
		controller_interface::InterfaceConfiguration state_interface_configuration () const override;
		controller_interface::CallbackReturn on_init () override;
		controller_interface::CallbackReturn on_configure ( const rclcpp_lifecycle::State &previous_state ) override;
		controller_interface::CallbackReturn on_activate ( const rclcpp_lifecycle::State &previous_state ) override;
		controller_interface::CallbackReturn on_deactivate ( const rclcpp_lifecycle::State &previous_state ) override;
		controller_interface::return_type update ( const rclcpp::Time &time, const rclcpp::Duration &period ) override;
};

} // namespace qube_servo2::controllers