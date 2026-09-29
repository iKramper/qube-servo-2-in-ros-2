#pragma once

#include <QString>
#include <QMetaType>

namespace qube_servo2::gui::models {

enum class WaveformType { Constant = 0, Step, Ramp, Pulse, Square, Triangle, Sine, Exponential, Chirp };

struct WaveformConfig {
    WaveformType type{WaveformType::Step};
    double amplitude{1.0};
    double offset{0.0};
    double frequency_hz{1.0};
    double chirp_end_frequency_hz{10.0};
    double duty_cycle_percent{50.0};
    double phase_deg{0.0};
    double pulse_width_s{0.5};
    double delay_s{0.5};
    double duration_s{5.0};
    double initial_value{0.0};
    double final_value{1.0};
    double rise_time_s{1.0};
    double tau_s{1.0};
    bool repeat{false};
    int publish_rate_hz{100};
    QString name() const;
    QString summary() const;
};

}  // namespace qube_servo2::gui::models
Q_DECLARE_METATYPE(qube_servo2::gui::models::WaveformConfig)
