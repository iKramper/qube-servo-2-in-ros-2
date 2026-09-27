#include "qube_servo2_controllers/discrete_pid.hpp"
#include <algorithm>
#include <cmath>

namespace qube_servo2::controllers {

bool
DiscretePID::configure ( const PIDgains gains, const PIDlimits limits, const bool anti_windup ) noexcept {

	if ( !std::isfinite ( gains.kp ) || !std::isfinite ( gains.ki ) || !std::isfinite ( gains.kd ) ) {
		return false;
	}

	if ( !std::isfinite ( limits.min_output ) || !std::isfinite ( limits.max_output ) ) {
		return false;
	}

	if ( limits.min_output >= limits.max_output ) {
		return false;
	}

	gains_ = gains;
	limits_ = limits;
	anti_windup_ = anti_windup;

	reset ();

	return true;
};

double
DiscretePID::update ( double error, double error_derivative, double dt ) {

	if ( !std::isfinite ( error ) || !std::isfinite ( error_derivative ) || !std::isfinite ( dt ) || dt <= 0.0 ) {
		return 0.0;
	}

	if ( !initialized_ ) {
		previous_error_ = error;
		initialized_ = true;
	}

	// Accion proporcional ( ) - P[k] = Kp * e[k]:
	const double proportional = gains_.kp * error;

	// Accion derivativa () - D[k] = Kd * de/dt:
	const double derivative = gains_.kd * error_derivative;

	// Accion integral - regla trapezoidal ( )  -I[k] = I[k-1] + dt/2 * (e[k] + e[k-1]):
	const double candidate_integral = integral_ + 0.5 * dt * ( error + previous_error_ );
	const double candidate_output = proportional + gains_.ki * candidate_integral + derivative;

	// Integral condicional - anti-windup:
	const bool saturated_high = candidate_output > limits_.max_output;
	const bool saturated_low = candidate_output < limits_.min_output;
	const bool error_drives_back_from_high = saturated_high && error < 0.0;
	const bool error_drives_back_from_low = saturated_low && error > 0.0;
	if ( !anti_windup_ || ( !saturated_high && !saturated_low ) || error_drives_back_from_high ||
		 error_drives_back_from_low ) {
		integral_ = candidate_integral;
	}

	// Salida:
	const double output = proportional + gains_.ki * integral_ + derivative;
	previous_error_ = error;
	return std::clamp ( output, limits_.min_output, limits_.max_output );
};

void
DiscretePID::reset () noexcept {
	integral_ = 0.0;
	previous_error_ = 0.0;
	initialized_ = false;
};

} // namespace qube_servo2::controllers