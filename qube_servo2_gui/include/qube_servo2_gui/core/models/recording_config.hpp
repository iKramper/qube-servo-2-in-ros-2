#pragma once

#include <QString>

namespace qube_servo2::gui::models {

enum class RecordingMode {
    Manual = 0,
    TimeWindow,
    PeriodCount,
    PeakCount
};

enum class PeakSignal {
    Velocity = 0,
    Current,
    Torque,
    AppliedVoltage,
    Position,
    ElectricalPower,
    MechanicalPower
};

struct RecordingConfig {
    RecordingMode mode{RecordingMode::Manual};

    // 0 = save every received telemetry sample.
    int sample_interval_ms{0};

    // Shared capture delay for time-window / period-count modes.
    double start_delay_s{0.0};
    double duration_s{10.0};
    int period_count{10};

    // Peak-trigger / peak-count options.
    PeakSignal peak_signal{PeakSignal::Velocity};
    double peak_threshold{1.0};
    double min_peak_distance_s{0.10};
    int peak_target_count{5};
    bool absolute_peaks{true};

    QString modeName() const {
        switch (mode) {
        case RecordingMode::TimeWindow: return "Time window";
        case RecordingMode::PeriodCount: return "N periods";
        case RecordingMode::PeakCount: return "Peak count";
        case RecordingMode::Manual:
        default: return "Manual";
        }
    }

    QString peakSignalName() const {
        switch (peak_signal) {
        case PeakSignal::Current: return "current_a";
        case PeakSignal::Torque: return "motor_torque_nm";
        case PeakSignal::AppliedVoltage: return "applied_voltage_v";
        case PeakSignal::Position: return "position_rad";
        case PeakSignal::ElectricalPower: return "electrical_power_w";
        case PeakSignal::MechanicalPower: return "mechanical_power_w";
        case PeakSignal::Velocity:
        default: return "velocity_rad_s";
        }
    }
};

}  // namespace qube_servo2::gui::models
