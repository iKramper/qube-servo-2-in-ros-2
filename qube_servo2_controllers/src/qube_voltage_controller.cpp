#include "qube_servo2_controllers/qube_voltage_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/float64.hpp"
#include <controller_interface/controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/logging.hpp>

namespace qube_servo2::controllers {

// Funcion que declara que variable fisica puede comandar el controlador.
controller_interface::InterfaceConfiguration
QubeVoltageController::command_interface_configuration () const {

	controller_interface::InterfaceConfiguration config;

	// No quiero todas las interfaces del hardware, voy a especificar cuales quiero:
	config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
	config.names = { joint_name_ + "/" + interface_name_ };

	return config;
};

// Funcion que declara que variables de estado necesita leer el controlador.
controller_interface::InterfaceConfiguration
QubeVoltageController::state_interface_configuration () const {

	controller_interface::InterfaceConfiguration config;

	// Para este caso en especifico de controlar el voltage, dado que V[k] = r_v[k],
	// no aparece ningun estado que queramos observar.
	config.type = controller_interface::interface_configuration_type::NONE;

	return config;
}

controller_interface::CallbackReturn
QubeVoltageController::on_init () {

	// Recuperamos los parametros actuales en el sistema de ROS, declarando un valor predefinido para luego llamarlo si
	// se modifica.
	joint_name_ = auto_declare<std::string> ( "joint_name", "motor_hub_link_joint" );
	interface_name_ = auto_declare<std::string> ( "interface_name", "voltage" );
	reference_topic_ = auto_declare<std::string> ( "reference_topic", "~/referene" );

	// Voltage inicial fuera de la malla de control para 1khz.
	reference_buffer_.writeFromNonRT ( 0.0 );
	return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
QubeVoltageController::on_configure ( const rclcpp_lifecycle::State & /* previous_state */ ) {

	// 1. Validar configuracion:
	if ( joint_name_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'joint_name' cannot be empty." );
		return controller_interface::CallbackReturn::ERROR;
	}

	if ( interface_name_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'interface_name' cannot be empty." );
		return controller_interface::CallbackReturn::ERROR;
	}

	if ( reference_topic_.empty () ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Parameter 'reference_topic' cannot be empty." );
		return controller_interface::CallbackReturn::ERROR;
	}

	// 2. Crear subscriber de referencia:
	reference_subscriber_ = get_node ()->create_subscription<std_msgs::msg::Float64> (
		reference_topic_, 10,
		[this] ( const std_msgs::msg::Float64::SharedPtr msg ) { reference_buffer_.writeFromNonRT ( msg->data ); } );

	// 3. Dejar la referencia en un estado conocido / seguro:
	reference_buffer_.writeFromNonRT ( 0.0 );

	return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::CallbackReturn
QubeVoltageController::on_activate ( const rclcpp_lifecycle::State & /* previous_state */ ) {

	if ( command_interfaces_.size () != 1 ) {
		RCLCPP_ERROR (
			get_node ()->get_logger (), "Expected exactly one command interface, but got %zu.",
			command_interfaces_.size () );
		return controller_interface::CallbackReturn::ERROR;
	}

	reference_buffer_.writeFromNonRT ( 0.0 );

	if ( !command_interfaces_[0].set_value ( 0.0 ) ) {
		RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to initialize voltage command to 0 V." );
		return controller_interface::CallbackReturn::ERROR;
	}

	return controller_interface::CallbackReturn::SUCCESS;
};

controller_interface::return_type
QubeVoltageController::update ( const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */ ) {

	const double *reference = reference_buffer_.readFromNonRT ();

	if ( reference == nullptr )
		return controller_interface::return_type::ERROR;

	const double voltage_command = *reference;

	if ( !command_interfaces_[0].set_value ( voltage_command ) ) {
		return controller_interface::return_type::ERROR;
	}

	return controller_interface::return_type::OK;
};

controller_interface::CallbackReturn
QubeVoltageController::on_deactivate ( const rclcpp_lifecycle::State & /* time */ ) {

	if ( command_interfaces_.size () == 1 ) {
		if ( !command_interfaces_[0].set_value ( 0.0 ) ) {
			RCLCPP_ERROR ( get_node ()->get_logger (), "Failed to set voltage command to 0 V during deactivation." );
			return controller_interface::CallbackReturn::ERROR;
		}
	}

	reference_buffer_.writeFromNonRT ( 0.0 );
	return controller_interface::CallbackReturn::ERROR;
}

} // namespace qube_servo2::controllers

PLUGINLIB_EXPORT_CLASS ( qube_servo2::controllers::QubeVoltageController, controller_interface::ControllerInterface )