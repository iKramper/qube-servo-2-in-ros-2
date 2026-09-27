#include "qube_servo2_controllers/qube_state_feedback_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <numbers>

#include <controller_interface/controller_interface.hpp>

#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/logging.hpp>

namespace qube_servo2::controllers {

controller_interface::InterfaceConfiguration
QubeStateFeedbackController::command_interface_configuration () const {
	controller_interface::InterfaceConfiguration config;

	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	config.names = { joint_name_ + "/" + interface_name_ };

	return config;
}

controller_interface::InterfaceConfiguration
QubeStateFeedbackController::state_interface_configuration () const {
	controller_interface::InterfaceConfiguration config;

	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	config.names = { joint_name_ + "/position", joint_name_ + "/velocity" };

	return config;
}

controller_interface::CallbackReturn
QubeStateFeedbackController::on_init () {
	try {

		auto_declare<std::string> ( "joint_name", "motor_hub_link_joint" );
		auto_declare<std::string> ( "interface_name", "voltage" );
		auto_declare<std::string> ( "reference_topic", "~/reference" );
		auto_declare<std::string> ( "mode", "position" );
		auto_declare<double> ( "gains.k_position", 0.0 );
		auto_declare<double> ( "gains.k_velocity", 0.0 );
		auto_declare<double> ( "feedforward.velocity_gain", 1.0 / 22.4 );
		auto_declare<double> ( "limits.min_output", -10.0 );
		auto_declare<double> ( "limits.max_output", 10.0 );
		auto_declare<bool> ( "angle_wraparound", true );
		reference_buffer_.writeFromNonRT ( 0.0 );
	} catch ( const std::exception &exception ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (), "Failed to initialize QUBE state-feedback controller: %s", exception.what () );

		return controller_interface::CallbackReturn::ERROR;
	}

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
QubeStateFeedbackController::on_configure ( const rclcpp_lifecycle::State & /* previous_state */ ) {

	joint_name_ = get_node ()->get_parameter ( "joint_name" ).as_string ();
	interface_name_ = get_node ()->get_parameter ( "interface_name" ).as_string ();
	reference_topic_ = get_node ()->get_parameter ( "reference_topic" ).as_string ();
	mode_ = get_node ()->get_parameter ( "mode" ).as_string ();
	angle_wraparound_ = get_node ()->get_parameter ( "angle_wraparound" ).as_bool ();
	k_position_ = get_node ()->get_parameter ( "gains.k_position" ).as_double ();
	k_velocity_ = get_node ()->get_parameter ( "gains.k_velocity" ).as_double ();
	k_velocity_feedforward_ = get_node ()->get_parameter ( "feedforward.velocity_gain" ).as_double ();
	min_output_ = get_node ()->get_parameter ( "limits.min_output" ).as_double ();
	max_output_ = get_node ()->get_parameter ( "limits.max_output" ).as_double ();

	if ( joint_name_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'joint_name' cannot be empty." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( interface_name_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'interface_name' cannot be empty." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( interface_name_ != "voltage" ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (),
			"QUBE state-feedback controller expects command interface "
			"'voltage', but received '%s'.",
			interface_name_.c_str () );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( reference_topic_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'reference_topic' cannot be empty." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( mode_ != "position" && mode_ != "velocity" ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (), "Parameter 'mode' must be either "
										"'position' or 'velocity'." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( !std::isfinite ( k_position_ ) || !std::isfinite ( k_velocity_ ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "State-feedback gains must be finite." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( !std::isfinite ( k_velocity_feedforward_ ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Velocity feedforward gain must be finite." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( !std::isfinite ( min_output_ ) || !std::isfinite ( max_output_ ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Output limits must be finite." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( min_output_ >= max_output_ ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (), "Parameter 'limits.min_output' must be lower than "
										"'limits.max_output'." );

		return controller_interface::CallbackReturn::FAILURE;
	}

	reference_subscriber_ = get_node ()->create_subscription<std_msgs::msg::Float64> (
		reference_topic_, 10, [this] ( const std_msgs::msg::Float64::SharedPtr message ) {
			reference_buffer_.writeFromNonRT ( message->data );
		} );

	reference_buffer_.writeFromNonRT ( 0.0 );

	reference_position_ = 0.0;

	reference_position_initialized_ = false;

	RCLCPP_INFO (
		get_node ()->get_logger (),
		"QUBE state-feedback controller configured: "
		"mode='%s', K=[%.6f, %.6f], velocity feedforward=%.6f.",
		mode_.c_str (), k_position_, k_velocity_, k_velocity_feedforward_ );

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
QubeStateFeedbackController::on_activate ( const rclcpp_lifecycle::State & /* previous_state */ ) {

	if ( command_interfaces_.size () != 1U ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (),
			"Expected exactly one command interface, "
			"but got %zu.",
			command_interfaces_.size () );

		return controller_interface::CallbackReturn::ERROR;
	}

	if ( state_interfaces_.size () != 2U ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (),
			"Expected exactly two state interfaces "
			"(position and velocity), but got %zu.",
			state_interfaces_.size () );

		return controller_interface::CallbackReturn::ERROR;
	}

	reference_position_ = 0.0;
	reference_position_initialized_ = false;
	reference_buffer_.writeFromNonRT ( 0.0 );

	if ( !command_interfaces_[0].set_value ( 0.0 ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to initialize motor voltage command to 0 V." );

		return controller_interface::CallbackReturn::ERROR;
	}

	RCLCPP_INFO (
		get_node ()->get_logger (), "QUBE state-feedback controller activated in '%s' mode.", mode_.c_str () );

	return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
QubeStateFeedbackController::update ( const rclcpp::Time & /* time */, const rclcpp::Duration &period ) {

	const double *reference = reference_buffer_.readFromRT ();

	if ( reference == nullptr || !std::isfinite ( *reference ) ) {
		return controller_interface::return_type::ERROR;
	}

	const auto position = state_interfaces_[0].get_optional<double> ();
	const auto velocity = state_interfaces_[1].get_optional<double> ();

	if ( !position || !velocity ) {
		return controller_interface::return_type::OK;
	}

	if ( !std::isfinite ( *position ) || !std::isfinite ( *velocity ) ) {
		return controller_interface::return_type::ERROR;
	}

	double position_reference = 0.0;
	double velocity_reference = 0.0;
	double feedforward_voltage = 0.0;

	if ( mode_ == "position" ) {
		position_reference = *reference;

		velocity_reference = 0.0;

		feedforward_voltage = 0.0;
	}

	else {
		const double dt = period.seconds ();

		if ( !std::isfinite ( dt ) ) {
			return controller_interface::return_type::ERROR;
		}

		// A non-positive sampling period is not considered
		// a fatal controller failure.
		if ( dt <= 0.0 ) {
			return controller_interface::return_type::OK;
		}
		if ( !reference_position_initialized_ ) {
			reference_position_ = *position;

			reference_position_initialized_ = true;
		}

		velocity_reference = *reference;

		reference_position_ += dt * velocity_reference;

		position_reference = reference_position_;

		feedforward_voltage = k_velocity_feedforward_ * velocity_reference;
	}

	double position_error = position_reference - *position;

	if ( mode_ == "position" && angle_wraparound_ ) {
		position_error = std::remainder ( position_error, 2.0 * std::numbers::pi_v<double> );
	}

	const double velocity_error = velocity_reference - *velocity;

	const double voltage_unsaturated =
		feedforward_voltage + k_position_ * position_error + k_velocity_ * velocity_error;

	const double voltage_command = std::clamp ( voltage_unsaturated, min_output_, max_output_ );

	if ( !command_interfaces_[0].set_value ( voltage_command ) ) {
		// Temporary command-interface contention.
		// Skip this sample instead of killing the controller.

		return controller_interface::return_type::OK;
	}

	return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
QubeStateFeedbackController::on_deactivate ( const rclcpp_lifecycle::State & /* previous_state */ ) {
	if ( command_interfaces_.size () == 1U ) {
		if ( !command_interfaces_[0].set_value ( 0.0 ) ) {
			RCLCPP_ERROR (
				get_node ()->get_logger (), "Failed to set motor voltage to 0 V "
											"during deactivation." );

			return controller_interface::CallbackReturn::ERROR;
		}
	}
	reference_buffer_.writeFromNonRT ( 0.0 );
	reference_position_ = 0.0;
	reference_position_initialized_ = false;
	return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace qube_servo2::controllers

PLUGINLIB_EXPORT_CLASS (
	qube_servo2::controllers::QubeStateFeedbackController, controller_interface::ControllerInterface )