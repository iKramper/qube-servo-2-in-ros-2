#pragma once

namespace qube_servo2::controllers {

struct PIDgains {
		double kp{ 0.0 };
		double ki{ 0.0 };
		double kd{ 0.0 };
};

struct PIDlimits {
		double min_output{ -10.0 };
		double max_output{ 10.0 };
};

class DiscretePID {
	private:
		PIDgains gains_;
		PIDlimits limits_;

		double integral_{ 0.0 };
		double previous_error_{ 0.0 };

		bool anti_windup_{ true };
		bool initialized_{ false };

	public:
		DiscretePID () = default;
		bool configure ( const PIDgains gains, const PIDlimits limits, const bool anti_windup ) noexcept;
		double update ( double error, double error_derivative, double dt );
		void reset () noexcept;
};

} // namespace qube_servo2::controllers