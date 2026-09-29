#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"

#include <QElapsedTimer>
#include <QWidget>

class QLabel;
class QProgressBar;

namespace qube_servo2::gui::ui::pages {

class SystemPage final : public QWidget {
    Q_OBJECT

public:
    explicit SystemPage(QWidget* parent = nullptr);

    void setConfiguration(const QString& joint,
                          const QString& command_topic,
                          const QString& joint_topic,
                          const QString& dynamic_topic,
                          double voltage_limit);

public slots:
    void setConnected(bool connected);
    void setCommand(double voltage);
    void updateTelemetry(const qube_servo2::gui::models::TelemetrySample& sample);
    void setControllerState(const QString& controller, const QString& state, const QString& mode);
    void setReferencePublished(const QString& controller, const QString& mode, double value);

signals:
    void sendZeroRequested();

private:
    void refreshLiveReadouts_(const qube_servo2::gui::models::TelemetrySample& sample);
    void refreshSessionReadouts_();
    static QString prettyController_(const QString& controller);
    static QString referenceUnit_(const QString& controller, const QString& mode);

    QLabel* connection_label_{nullptr};
    QLabel* joint_label_{nullptr};
    QLabel* command_topic_label_{nullptr};
    QLabel* joint_topic_label_{nullptr};
    QLabel* dynamic_topic_label_{nullptr};

    QLabel* controller_hero_{nullptr};
    QLabel* telemetry_rate_hero_{nullptr};
    QLabel* sample_count_hero_{nullptr};
    QLabel* uptime_hero_{nullptr};

    QLabel* controller_state_label_{nullptr};
    QLabel* controller_mode_label_{nullptr};
    QLabel* reference_label_{nullptr};
    QLabel* reference_route_label_{nullptr};

    QLabel* position_label_{nullptr};
    QLabel* velocity_label_{nullptr};
    QLabel* current_label_{nullptr};
    QLabel* applied_voltage_label_{nullptr};
    QLabel* back_emf_label_{nullptr};
    QLabel* torque_label_{nullptr};
    QLabel* electrical_power_label_{nullptr};
    QLabel* mechanical_power_label_{nullptr};

    QLabel* max_velocity_label_{nullptr};
    QLabel* max_current_label_{nullptr};
    QLabel* max_voltage_label_{nullptr};
    QLabel* max_torque_label_{nullptr};

    QLabel* voltage_limit_label_{nullptr};
    QLabel* safety_voltage_label_{nullptr};
    QLabel* safety_current_label_{nullptr};
    QLabel* safety_state_label_{nullptr};
    QProgressBar* voltage_utilization_bar_{nullptr};

    double voltage_limit_{10.0};
    double max_abs_velocity_{0.0};
    double max_abs_current_{0.0};
    double max_abs_voltage_{0.0};
    double max_abs_torque_{0.0};
    double last_reference_{0.0};

    QString active_controller_{};
    QString active_state_{"inactive"};
    QString active_mode_{};

    qint64 sample_count_{0};
    qint64 rate_window_samples_{0};
    double telemetry_rate_hz_{0.0};
    bool connected_{false};

    QElapsedTimer session_timer_;
    QElapsedTimer rate_timer_;
    QElapsedTimer ui_throttle_;
};

}  // namespace qube_servo2::gui::ui::pages
