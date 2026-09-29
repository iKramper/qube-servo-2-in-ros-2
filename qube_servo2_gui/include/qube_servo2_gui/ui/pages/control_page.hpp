#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"
#include "qube_servo2_gui/core/models/waveform_config.hpp"
#include "qube_servo2_gui/core/services/waveform_generator.hpp"

#include <QElapsedTimer>
#include <QWidget>

#include <array>
#include <deque>

class QButtonGroup;
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFormLayout;
class QGridLayout;
class QLabel;
class QPushButton;
class QSlider;
class QSpinBox;
class QStackedWidget;
class QTabWidget;
class QTimer;

namespace qube_servo2::gui::ui::widgets {
class ControlTopologyWidget;
class DialReferenceWidget;
class ScopePlot;
class ThrottleLever;
}

namespace qube_servo2::gui::ui::pages {

class ControlPage final : public QWidget {
    Q_OBJECT

public:
    explicit ControlPage(double voltage_limit,
                         double position_reference_limit,
                         double velocity_reference_limit,
                         QWidget* parent = nullptr);
    ~ControlPage() override;

    models::WaveformConfig waveformConfig() const;
    bool isRunning() const noexcept;
    bool autoRecordEnabled() const noexcept;
    bool manualReferenceSelected() const noexcept;
    QString selectedController() const;
    QString selectedMode() const;

public slots:
    void setActive(bool active);
    void shutdown();
    void updateTelemetry(const qube_servo2::gui::models::TelemetrySample& sample);
    void stopOutput();
    void setRecordingState(bool active);
    void setControllerState(const QString& controller, const QString& state, const QString& mode);
    void setControllerOperationResult(bool success, const QString& message);

signals:
    void referenceRequested(const QString& controller, const QString& mode, double value);
    void controllerActivationRequested(const QString& controller, const QString& mode);
    void controllersDeactivateRequested();
    void emergencyStopRequested();
    void waveformStarted(const qube_servo2::gui::models::WaveformConfig& config);
    void waveformStopped();
    void recordingDialogRequested();

private slots:
    void startWaveform_();
    void tickWaveform_();
    void refreshPreview_();
    void updateControlsForWaveform_();
    void updateMonitorMode_();
    void updateControllerSelection_();
    void activateSelectedController_();
    void setAutomaticSource_();
    void setManualSource_();
    void onManualNumericChanged_(double value);
    void onVoltageSliderChanged_(int raw);
    void onManualDialChanged_(double value);
    void onThrottleChanged_(double value);
    void captureCurrentPosition_();
    void flushManualReference_();

private:
    QDoubleSpinBox* makeDoubleSpin_(double min, double max, double value, int decimals, double step);
    models::WaveformType selectedType_() const;
    void updateFieldVisibility_();
    void updateReferenceSemantics_();
    void updateReferenceSourceUi_();
    void updateManualWidget_();
    void setManualReference_(double value, bool publish_if_armed = true);
    void syncManualEditors_(double value, QObject* origin = nullptr);
    bool controllerMatchesSelection_() const noexcept;
    double currentReferenceLimit_() const noexcept;
    QString currentReferenceUnit_() const;
    int currentMonitorIndex_() const noexcept;
    double safeStopReference_() const noexcept;
    double wrappedRadians_(double radians) const noexcept;
    void appendMonitorHistory_(const qube_servo2::gui::models::TelemetrySample& sample, double t);
    void pruneMonitorHistory_(double newest_t);
    void rebuildLivePlot_();
    void updateLiveMetrics_(const qube_servo2::gui::models::TelemetrySample& sample);
    void refreshSourceReadout_();
    void updatePreviewSummary_();

    struct MonitorHistorySample {
        double t{0.0};
        std::array<double, 3> reference{};
        std::array<double, 3> response{};
    };

    double voltage_limit_{10.0};
    double position_reference_limit_{6.283185307179586};
    double velocity_reference_limit_{30.0};

    services::WaveformGenerator generator_;
    QElapsedTimer elapsed_;
    QTimer* waveform_timer_{nullptr};
    QTimer* manual_publish_timer_{nullptr};
    bool running_{false};
    bool manual_source_{false};

    QComboBox* controller_combo_{nullptr};
    QComboBox* control_mode_{nullptr};
    QPushButton* activate_controller_btn_{nullptr};
    QPushButton* deactivate_controller_btn_{nullptr};
    QLabel* controller_state_{nullptr};
    QLabel* controller_hint_{nullptr};

    QButtonGroup* source_group_{nullptr};
    QPushButton* automatic_source_btn_{nullptr};
    QPushButton* manual_source_btn_{nullptr};
    QLabel* source_readout_{nullptr};
    QStackedWidget* reference_source_stack_{nullptr};
    QWidget* reference_panel_{nullptr};
    QTabWidget* left_tabs_{nullptr};
    QGridLayout* body_layout_{nullptr};
    std::array<QLabel*, 8> preview_config_labels_{};

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

    widgets::DialReferenceWidget* position_dial_{nullptr};
    widgets::ThrottleLever* velocity_throttle_{nullptr};
    QSlider* voltage_slider_{nullptr};
    QStackedWidget* manual_widget_stack_{nullptr};
    QDoubleSpinBox* manual_numeric_{nullptr};
    QLabel* manual_value_label_{nullptr};
    QLabel* manual_hint_{nullptr};
    QPushButton* manual_zero_btn_{nullptr};
    QPushButton* capture_position_btn_{nullptr};

    QPushButton* start_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QPushButton* emergency_btn_{nullptr};
    QPushButton* manual_emergency_btn_{nullptr};
    QPushButton* recording_btn_{nullptr};

    QLabel* command_value_{nullptr};
    QLabel* applied_value_{nullptr};
    QLabel* current_value_{nullptr};
    QLabel* waveform_help_{nullptr};
    QLabel* active_source_metric_{nullptr};

    widgets::ScopePlot* preview_plot_{nullptr};
    widgets::ScopePlot* live_plot_{nullptr};
    widgets::ControlTopologyWidget* topology_widget_{nullptr};

    QString active_controller_{};
    QString active_mode_{};
    QString active_state_{"inactive"};
    double last_reference_{0.0};
    double manual_reference_{0.0};
    double last_position_{0.0};
    bool have_telemetry_{false};

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
