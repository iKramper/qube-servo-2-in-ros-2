#include "qube_servo2_controllers/qube_pid_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/logging.hpp>

namespace qube_servo2::controllers {

// Funcion que declara que variable fisica puede comandar el controlador.
controller_interface::InterfaceConfiguration
QubePIDController::command_interface_configuration () const {

	controller_interface::InterfaceConfiguration config;

	// No quiero todas las interfaces del hardware, voy a especificar cuales quiero:
	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	config.names = { joint_name_ + "/" + interface_name_ };

	return config;
};

// Funcion que declara que variables de estado necesita leer el controlador.
controller_interface::InterfaceConfiguration
QubePIDController::state_interface_configuration () const {

	controller_interface::InterfaceConfiguration config;

	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

	if ( mode_ == "velocity" ) {
		config.names = { joint_name_ + "/velocity" };
	} else if ( mode_ == "position" ) {
		config.names = { joint_name_ + "/position", joint_name_ + "/velocity" };
	} else {
		config.type = controller_interface::interface_configuration_type::NONE;
	}

	return config;
}

controller_interface::CallbackReturn
QubePIDController::on_init () {

	try {
		// Controlador:
		auto_declare<std::string> ( "joint_name", "motor_hub_link_joint" );
		auto_declare<std::string> ( "interface_name", "voltage" );
		auto_declare<std::string> ( "mode", "velocity" );
		auto_declare<std::string> ( "reference_topic", "~/reference" );

		// Ganancias PID:
		auto_declare<double> ( "gains.kp", 0.0 );
		auto_declare<double> ( "gains.ki", 0.0 );
		auto_declare<double> ( "gains.kd", 0.0 );

		// Limites de voltage:
		auto_declare<double> ( "limits.min_output", -10.0 );
		auto_declare<double> ( "limits.max_output", 10.0 );

		// Configuraciones adicionales:
		auto_declare<bool> ( "anti_windup", true );
		auto_declare<bool> ( "angle_wraparound", true );

		// Segura y conocida referencia inicial:
		reference_buffer_.writeFromNonRT ( 0.0 );
	} catch ( const std::exception &exception ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to initialize QUBE PID controller: %s", exception.what () );
		return controller_interface::CallbackReturn::ERROR;
	}

	return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
QubePIDController::on_configure ( const rclcpp_lifecycle::State & /* previous_state */ ) {

	// Lectura de la configuracion del controlador:
	joint_name_ = get_node ()->get_parameter ( "joint_name" ).as_string ();
	interface_name_ = get_node ()->get_parameter ( "interface_name" ).as_string ();
	mode_ = get_node ()->get_parameter ( "mode" ).as_string ();
	reference_topic_ = get_node ()->get_parameter ( "reference_topic" ).as_string ();
	angle_wraparound_ = get_node ()->get_parameter ( "angle_wraparound" ).as_bool ();

	// Validar configuracion:
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
			"QUBE PID controller expects command interface "
			"'voltage', but received '%s'.",
			interface_name_.c_str () );
		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( mode_ != "velocity" && mode_ != "position" ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (), "Parameter 'mode' must be either "
										"'velocity' or 'position'." );
		return controller_interface::CallbackReturn::FAILURE;
	}

	if ( reference_topic_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'reference_topic' cannot be empty." );
		return controller_interface::CallbackReturn::FAILURE;
	}

	// Construccion de parametros del PID:
	const PIDgains gains{ .kp = get_node ()->get_parameter ( "gains.kp" ).as_double (),
						  .ki = get_node ()->get_parameter ( "gains.ki" ).as_double (),
						  .kd = get_node ()->get_parameter ( "gains.kd" ).as_double () };

	const PIDlimits limits{ .min_output = get_node ()->get_parameter ( "limits.min_output" ).as_double (),
							.max_output = get_node ()->get_parameter ( "limits.max_output" ).as_double () };

	const bool anti_windup = get_node ()->get_parameter ( "anti_windup" ).as_bool ();

	// Configuracion del PID:
	if ( !pid_.configure ( gains, limits, anti_windup ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Invalid PID configuration." );
		return controller_interface::CallbackReturn::FAILURE;
	}

	// Creacion del subscriptor de referencia:
	reference_subscriber_ = get_node ()->create_subscription<std_msgs::msg::Float64> (
		reference_topic_, 10, [this] ( const std_msgs::msg::Float64::SharedPtr message ) {
			reference_buffer_.writeFromNonRT ( message->data );
		} );

	// Inicializacion de la referencia segura:
	reference_buffer_.writeFromNonRT ( 0.0 );
	return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
QubePIDController::on_activate ( const rclcpp_lifecycle::State & /* previous_state */ ) {

	// Validar las interfaces de comando
	if ( command_interfaces_.size () != 1U ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (),
			"Expected exactly one command interface, "
			"but got %zu.",
			command_interfaces_.size () );
		return controller_interface::CallbackReturn::ERROR;
	}

	// Validar estado de los estados
	const std::size_t expected_state_interfaces = mode_ == "velocity" ? 1U : 2U;

	if ( state_interfaces_.size () != expected_state_interfaces ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (),
			"Expected %zu state interfaces for mode '%s', "
			"but got %zu.",
			expected_state_interfaces, mode_.c_str (), state_interfaces_.size () );

		return controller_interface::CallbackReturn::ERROR;
	}

	// Reset del PID:
	pid_.reset ();

	// Referencia segura:
	reference_buffer_.writeFromNonRT ( 0.0 );

	// Comando seguro al actuador:
	if ( !command_interfaces_[0].set_value ( 0.0 ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to initialize motor voltage command to 0 V." );
		return controller_interface::CallbackReturn::ERROR;
	}

	return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::return_type
QubePIDController::update ( const rclcpp::Time & /* time */, const rclcpp::Duration &period ) {

	// Muestra:
	const double dt = period.seconds ();
	if ( !std::isfinite ( dt ) )
		return controller_interface::return_type::ERROR;
	if ( dt <= 0.0 )
		return controller_interface::return_type::OK;

	// Obtenemos nuestra referencia de velocidad:
	const double *reference = reference_buffer_.readFromRT ();
	if ( reference == nullptr )
		return controller_interface::return_type::ERROR;
	double error = 0.0;
	double error_derivative = 0.0;

	// Obtenemos la referencia actual del modo de control:
	if ( mode_ == "velocity" ) {

		const auto velocity = state_interfaces_[0].get_optional<double> ();
		if ( !velocity )
			return controller_interface::return_type::ERROR;

		// e_w[k] = w_ref[k] - w[k]
		error = *reference - *velocity;
		error_derivative = 0.0;

	} else {

		const auto position = state_interfaces_[0].get_optional<double> ();
		const auto velocity = state_interfaces_[1].get_optional<double> ();
		if ( !position || !velocity )
			return controller_interface::return_type::ERROR;
		// e_theta[k] = theta_ref[k] - theta[k]
		error = *reference - *position;
		if ( angle_wraparound_ ) {
			error = std::remainder ( error, 2.0 * std::numbers::pi_v<double> );
		}
		error_derivative = -*velocity;
	}

	// PID:
	const double voltage_command = pid_.update ( error, error_derivative, dt );
	if ( !command_interfaces_[0].set_value ( voltage_command ) ) {
		return controller_interface::return_type::ERROR;
	}

	return controller_interface::return_type::OK;
};

controller_interface::CallbackReturn
QubePIDController::on_deactivate ( const rclcpp_lifecycle::State & /* time */ ) {

	if ( command_interfaces_.size () == 1U ) {
		if ( !command_interfaces_[0].set_value ( 0.0 ) ) {
			RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to set voltage command to 0 V during deactivation." );
			return controller_interface::CallbackReturn::ERROR;
		}
	}

	pid_.reset ();
	reference_buffer_.writeFromNonRT ( 0.0 );
	return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace qube_servo2::controllers

PLUGINLIB_EXPORT_CLASS ( qube_servo2::controllers::QubePIDController, controller_interface::ControllerInterface )