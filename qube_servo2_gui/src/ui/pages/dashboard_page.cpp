#include "qube_servo2_gui/ui/pages/dashboard_page.hpp"
#include "qube_servo2_gui/ui/widgets/scope_plot.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QVBoxLayout>

#include <cmath>

namespace qube_servo2::gui::ui::pages {

namespace {

double clampSmall(double value, double eps = 1e-6) {
    return std::abs(value) < eps ? 0.0 : value;
}

double wrapRadians0To2Pi(double radians) {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;

    double wrapped = std::fmod(radians, kTwoPi);
    if (wrapped < 0.0) wrapped += kTwoPi;

    // Avoid visual / textual -0 and numerical residue exactly at a turn.
    if (std::abs(wrapped) < 1e-9 || std::abs(wrapped - kTwoPi) < 1e-9) {
        return 0.0;
    }
    return wrapped;
}

QString fixedSmart(double value, int decimals, double eps = 1e-6) {
    return QString::number(clampSmall(value, eps), 'f', decimals);
}

QString preciseZeroed(double value, int decimals, double eps) {
    return QString::number(clampSmall(value, eps), 'f', decimals);
}

QLabel* makeMetric(const QString& text, QWidget* parent) {
    auto* label = new QLabel(text, parent);
    label->setObjectName("metricValue");
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return label;
}

QFrame* makePlotPanel(const QString& title,
                      const QList<QLabel*>& metrics,
                      widgets::ScopePlot* plot,
                      QWidget* parent) {
    auto* panel = new QFrame(parent);
    panel->setObjectName("panel");
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(6, 4, 6, 6);
    layout->setSpacing(2);

    auto* header = new QHBoxLayout();
    header->setContentsMargins(0, 0, 0, 0);
    header->setSpacing(8);
    auto* label = new QLabel(title, panel);
    label->setObjectName("panelTitle");
    header->addWidget(label);
    header->addStretch(1);
    for (auto* metric : metrics) header->addWidget(metric);
    layout->addLayout(header);
    layout->addWidget(plot, 1);
    return panel;
}

}  // namespace

DashboardPage::DashboardPage(QWidget* parent)
    : QWidget(parent) {
    auto* page_root = new QVBoxLayout(this);
    page_root->setContentsMargins(0, 0, 0, 0);
    page_root->setSpacing(0);

    auto* scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    page_root->addWidget(scroll);

    auto* container = new QWidget(scroll);
    scroll->setWidget(container);

    auto* root = new QVBoxLayout(container);
    root->setContentsMargins(10, 8, 10, 8);
    root->setSpacing(8);

    auto* header = new QHBoxLayout();
    auto* title = new QLabel("QUBE-SERVO 2 / LIVE TELEMETRY", this);
    title->setObjectName("title");
    auto* hint = new QLabel("Hover: cursor values  •  Wheel: time zoom  •  Y/FIT: vertical scale", this);
    hint->setObjectName("subtitle");
    auto* clear_btn = new QPushButton("Clear", this);
    clear_btn->setObjectName("secondary");
    clear_btn->setToolTip("Clear plot history without changing the selected scales.");
    connect(clear_btn, &QPushButton::clicked, this, &DashboardPage::clearCharts);
    header->addWidget(title);
    header->addSpacing(14);
    header->addWidget(hint);
    header->addStretch(1);
    header->addWidget(clear_btn);
    root->addLayout(header);

    auto* grid = new QGridLayout();
    grid->setContentsMargins(0, 0, 0, 0);
    grid->setHorizontalSpacing(6);
    grid->setVerticalSpacing(6);

    position_value_ = makeMetric("θ  -- rad", this);
    velocity_value_ = makeMetric("ω  -- rad/s", this);
    rpm_value_ = makeMetric("-- rpm", this);
    command_value_ = makeMetric("Cmd  -- V", this);
    applied_value_ = makeMetric("Vm  -- V", this);
    emf_value_ = makeMetric("Eb  -- V", this);
    current_value_ = makeMetric("i  -- A", this);
    torque_value_ = makeMetric("τ  -- N·m", this);
    electrical_power_value_ = makeMetric("Pe  -- W", this);
    mechanical_power_value_ = makeMetric("Pm  -- W", this);

    position_plot_ = new widgets::ScopePlot("", "rad", {"Position"}, this);
    position_plot_->setMinimumHeight(340);
    position_plot_->setPiRadiansYAxis();
    position_plot_->setWindowSeconds(15.0);

    speed_plot_ = new widgets::ScopePlot("", "rad/s", {"Angular velocity"}, this);
    speed_plot_->setMinimumHeight(340);
    speed_plot_->setInitialYSpan(10.0);
    speed_plot_->setWindowSeconds(15.0);

    electrical_plot_ = new widgets::ScopePlot("", "V", {"Command", "Applied", "Back EMF"}, this);
    electrical_plot_->setMinimumHeight(340);
    electrical_plot_->setYRange(-11.0, 11.0);
    electrical_plot_->setSeriesDashed(0, true);
    electrical_plot_->setWindowSeconds(15.0);

    current_plot_ = new widgets::ScopePlot("", "A", {"Current"}, this);
    current_plot_->setMinimumHeight(340);
    current_plot_->setYRange(-2.2, 2.2);
    current_plot_->setWindowSeconds(15.0);

    torque_plot_ = new widgets::ScopePlot("", "N·m", {"Motor torque"}, this);
    torque_plot_->setMinimumHeight(340);
    torque_plot_->setYRange(-0.10, 0.10);
    torque_plot_->setWindowSeconds(15.0);

    power_plot_ = new widgets::ScopePlot("", "W", {"Electrical power", "Mechanical power"}, this);
    power_plot_->setMinimumHeight(340);
    power_plot_->setInitialYSpan(5.0);
    power_plot_->setWindowSeconds(15.0);

    grid->addWidget(makePlotPanel("POSITION  [0, 2π)", {position_value_}, position_plot_, this), 0, 0);
    grid->addWidget(makePlotPanel("SPEED", {velocity_value_, rpm_value_}, speed_plot_, this), 0, 1);
    grid->addWidget(makePlotPanel("MOTOR VOLTAGES", {command_value_, applied_value_, emf_value_}, electrical_plot_, this), 1, 0);
    grid->addWidget(makePlotPanel("MOTOR CURRENT", {current_value_}, current_plot_, this), 1, 1);

    // Same visual hierarchy as the four primary scopes. The third row simply falls
    // below the initial viewport because every scope keeps the same full height.
    grid->addWidget(makePlotPanel("MOTOR TORQUE", {torque_value_}, torque_plot_, this), 2, 0);
    grid->addWidget(makePlotPanel("POWER", {electrical_power_value_, mechanical_power_value_}, power_plot_, this), 2, 1);

    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    grid->setRowStretch(2, 1);
    root->addLayout(grid, 1);
}


DashboardPage::~DashboardPage() {
    shutdown();
}

void DashboardPage::setActive(bool active) {
    if (shutting_down_) return;
    position_plot_->setActive(active);
    speed_plot_->setActive(active);
    electrical_plot_->setActive(active);
    current_plot_->setActive(active);
    torque_plot_->setActive(active);
    power_plot_->setActive(active);
}

void DashboardPage::shutdown() {
    if (shutting_down_) return;
    shutting_down_ = true;

    position_plot_->shutdown();
    speed_plot_->shutdown();
    electrical_plot_->shutdown();
    current_plot_->shutdown();
    torque_plot_->shutdown();
    power_plot_->shutdown();
    t0_ = -1.0;
}

void DashboardPage::updateTelemetry(const models::TelemetrySample& sample) {
    if (shutting_down_ || !sample.valid) return;

    const double wrapped_position_rad = wrapRadians0To2Pi(sample.position_rad);
    position_value_->setText(QString("θ  %1 rad").arg(fixedSmart(wrapped_position_rad, 3)));
    velocity_value_->setText(QString("ω  %1 rad/s").arg(fixedSmart(sample.velocity_rad_s, 3)));
    rpm_value_->setText(QString("%1 rpm").arg(fixedSmart(sample.velocity_rpm(), 2)));
    command_value_->setText(QString("Cmd  %1 V").arg(fixedSmart(sample.command_voltage, 3)));
    applied_value_->setText(QString("Vm  %1 V").arg(fixedSmart(sample.applied_voltage_v, 3)));
    emf_value_->setText(QString("Eb  %1 V").arg(fixedSmart(sample.back_emf_v, 3)));
    current_value_->setText(QString("i  %1 A").arg(fixedSmart(sample.current_a, 4)));
    torque_value_->setText(QString("τ  %1 N·m").arg(preciseZeroed(sample.motor_torque_nm, 6, 1e-8)));
    electrical_power_value_->setText(QString("Pe  %1 W").arg(preciseZeroed(sample.electrical_power_w(), 5, 1e-7)));
    mechanical_power_value_->setText(QString("Pm  %1 W").arg(preciseZeroed(sample.mechanical_power_w(), 5, 1e-7)));

    if (t0_ < 0.0) t0_ = sample.ros_time_s;
    const double t = sample.ros_time_s - t0_;

    position_plot_->append(0, t, wrapped_position_rad);
    speed_plot_->append(0, t, clampSmall(sample.velocity_rad_s, 1e-7));

    electrical_plot_->append(0, t, clampSmall(sample.command_voltage, 1e-7));
    electrical_plot_->append(1, t, clampSmall(sample.applied_voltage_v, 1e-7));
    electrical_plot_->append(2, t, clampSmall(sample.back_emf_v, 1e-7));

    current_plot_->append(0, t, clampSmall(sample.current_a, 1e-7));
    torque_plot_->append(0, t, clampSmall(sample.motor_torque_nm, 1e-8));
    power_plot_->append(0, t, clampSmall(sample.electrical_power_w(), 1e-7));
    power_plot_->append(1, t, clampSmall(sample.mechanical_power_w(), 1e-7));
}

void DashboardPage::clearCharts() {
    position_plot_->clear();
    speed_plot_->clear();
    electrical_plot_->clear();
    current_plot_->clear();
    torque_plot_->clear();
    power_plot_->clear();
    t0_ = -1.0;
}

}  // namespace qube_servo2::gui::ui::pages
