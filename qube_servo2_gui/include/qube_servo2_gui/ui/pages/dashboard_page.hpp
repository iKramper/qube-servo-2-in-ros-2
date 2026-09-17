#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"

#include <QWidget>

class QLabel;

namespace qube_servo2::gui::ui::widgets {
class ScopePlot;
}

namespace qube_servo2::gui::ui::pages {

class DashboardPage final : public QWidget {
    Q_OBJECT

public:
    explicit DashboardPage(QWidget* parent = nullptr);
    ~DashboardPage() override;

public slots:
    void setActive(bool active);
    void shutdown();
    void updateTelemetry(const qube_servo2::gui::models::TelemetrySample& sample);
    void clearCharts();

private:
    QLabel* position_value_{nullptr};
    QLabel* velocity_value_{nullptr};
    QLabel* rpm_value_{nullptr};
    QLabel* command_value_{nullptr};
    QLabel* applied_value_{nullptr};
    QLabel* emf_value_{nullptr};
    QLabel* current_value_{nullptr};
    QLabel* torque_value_{nullptr};
    QLabel* electrical_power_value_{nullptr};
    QLabel* mechanical_power_value_{nullptr};

    widgets::ScopePlot* position_plot_{nullptr};
    widgets::ScopePlot* speed_plot_{nullptr};
    widgets::ScopePlot* electrical_plot_{nullptr};
    widgets::ScopePlot* current_plot_{nullptr};
    widgets::ScopePlot* torque_plot_{nullptr};
    widgets::ScopePlot* power_plot_{nullptr};

    double t0_{-1.0};
    bool shutting_down_{false};
};

}  // namespace qube_servo2::gui::ui::pages
