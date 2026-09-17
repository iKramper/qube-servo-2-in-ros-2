#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"
#include "qube_servo2_gui/core/models/waveform_config.hpp"
#include "qube_servo2_gui/core/services/waveform_generator.hpp"

#include <QElapsedTimer>
#include <QWidget>

#include <array>
#include <deque>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QLabel;
class QPushButton;
class QSpinBox;
class QTimer;

namespace qube_servo2::gui::ui::widgets {
class ScopePlot;
}

namespace qube_servo2::gui::ui::pages {

class ControlPage final : public QWidget {
    Q_OBJECT

public:
    explicit ControlPage(double voltage_limit, QWidget* parent = nullptr);
    ~ControlPage() override;

    models::WaveformConfig waveformConfig() const;
    bool isRunning() const noexcept;
    bool autoRecordEnabled() const noexcept;

public slots:
    void setActive(bool active);
    void shutdown();
    void updateTelemetry(const qube_servo2::gui::models::TelemetrySample& sample);
    void stopOutput();
    void setRecordingState(bool active);

signals:
    void commandRequested(double voltage);
    void stopRequested();
    void waveformStarted(const qube_servo2::gui::models::WaveformConfig& config);
    void waveformStopped();
    void recordingDialogRequested();

private slots:
    void startWaveform_();
    void tickWaveform_();
    void refreshPreview_();
    void updateControlsForWaveform_();
    void updateMonitorMode_();

private:
    QDoubleSpinBox* makeDoubleSpin_(double min, double max, double value, int decimals, double step);
    models::WaveformType selectedType_() const;
    void updateFieldVisibility_();
    double wrappedRadians_(double radians) const noexcept;
    void appendMonitorHistory_(const qube_servo2::gui::models::TelemetrySample& sample, double t);
    void pruneMonitorHistory_(double newest_t);
    void rebuildLivePlot_();
    void updateLiveMetrics_(const qube_servo2::gui::models::TelemetrySample& sample);

    struct MonitorHistorySample {
        double t{0.0};
        std::array<double, 3> reference{};
        std::array<double, 3> response{};
    };

    double voltage_limit_{10.0};
    services::WaveformGenerator generator_;
    QElapsedTimer elapsed_;
    QTimer* waveform_timer_{nullptr};
    bool running_{false};

    QComboBox* waveform_type_{nullptr};
    QComboBox* monitor_mode_{nullptr};
    QDoubleSpinBox* amplitude_{nullptr};
    QDoubleSpinBox* offset_{nullptr};
    QDoubleSpinBox* frequency_{nullptr};
    QDoubleSpinBox* chirp_end_frequency_{nullptr};
    QDoubleSpinBox* duty_cycle_{nullptr};
    QDoubleSpinBox* phase_{nullptr};
    QDoubleSpinBox* pulse_width_{nullptr};
    QDoubleSpinBox* delay_{nullptr};
    QDoubleSpinBox* duration_{nullptr};
    QDoubleSpinBox* initial_value_{nullptr};
    QDoubleSpinBox* final_value_{nullptr};
    QDoubleSpinBox* rise_time_{nullptr};
    QDoubleSpinBox* tau_{nullptr};
    QSpinBox* publish_rate_{nullptr};
    QCheckBox* repeat_{nullptr};
    QCheckBox* auto_record_{nullptr};
    QCheckBox* output_enabled_{nullptr};

    QFormLayout* form_left_{nullptr};
    QFormLayout* form_right_{nullptr};

    QPushButton* start_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QPushButton* emergency_btn_{nullptr};
    QPushButton* recording_btn_{nullptr};

    QLabel* command_value_{nullptr};
    QLabel* applied_value_{nullptr};
    QLabel* current_value_{nullptr};
    QLabel* waveform_help_{nullptr};

    widgets::ScopePlot* preview_plot_{nullptr};
    widgets::ScopePlot* live_plot_{nullptr};

    double live_t0_{-1.0};
    std::array<double, 3> reference_by_mode_{};
    std::deque<MonitorHistorySample> monitor_history_;
    double monitor_history_retention_s_{120.0};
    std::size_t monitor_history_max_points_{6000};
    bool active_{false};
    bool live_plot_synced_{false};
    bool shutting_down_{false};
};

}  // namespace qube_servo2::gui::ui::pages
