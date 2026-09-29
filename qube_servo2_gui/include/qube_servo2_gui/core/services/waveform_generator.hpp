#pragma once
#include "qube_servo2_gui/core/models/waveform_config.hpp"
namespace qube_servo2::gui::services {
class WaveformGenerator {
public:
    void configure(const models::WaveformConfig& config) noexcept;
    const models::WaveformConfig& config() const noexcept;
    double value(double t_s) const noexcept;
private:
    double localTime_(double t_s) const noexcept;
    models::WaveformConfig config_{};
};
}
