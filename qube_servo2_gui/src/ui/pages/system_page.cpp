#include "qube_servo2_gui/ui/pages/system_page.hpp"
#include "qube_servo2_gui/ui/widgets/hud_panel.hpp"

#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QStyle>
#include <QStringList>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::ui::pages {

namespace {

QLabel* makeEyebrow(const QString& text, QWidget* parent) {
    auto* label = new QLabel(text, parent);
    label->setObjectName("eyebrow");
    return label;
}

QFrame* makeHeroCard(const QString& key, QLabel*& value, QWidget* parent) {
    auto* frame = new QFrame(parent);
    frame->setObjectName("systemHeroCard");
    auto* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(2);
    auto* key_label = new QLabel(key, frame);
    key_label->setObjectName("systemHeroLabel");
    value = new QLabel("--", frame);
    value->setObjectName("systemHeroValue");
    layout->addWidget(key_label);
    layout->addWidget(value);
    return frame;
}

QFrame* makeTelemetryCard(const QString& key,
                          const QString& unit,
                          QLabel*& value,
                          QWidget* parent) {
    auto* frame = new QFrame(parent);
    frame->setObjectName("systemTelemetryCard");
    auto* layout = new QVBoxLayout(frame);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(1);

    auto* key_label = new QLabel(key, frame);
    key_label->setObjectName("systemTelemetryLabel");
    value = new QLabel("--", frame);
    value->setObjectName("systemTelemetryValue");
    auto* unit_label = new QLabel(unit, frame);
    unit_label->setObjectName("systemTelemetryUnit");

    layout->addWidget(key_label);
    layout->addWidget(value);
    layout->addWidget(unit_label);
    return frame;
}

QLabel* makePathValue(QWidget* parent) {
    auto* label = new QLabel("--", parent);
    label->setObjectName("systemPathValue");
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return label;
}

QLabel* makeRuntimeValue(QWidget* parent) {
    auto* label = new QLabel("--", parent);
    label->setObjectName("systemRuntimeValue");
    return label;
}

QLabel* makePeakValue(QWidget* parent) {
    auto* label = new QLabel("--", parent);
    label->setObjectName("systemPeakValue");
    return label;
}

}  // namespace

SystemPage::SystemPage(QWidget* parent)
    : QWidget(parent) {
    session_timer_.start();
    rate_timer_.start();
    ui_throttle_.start();

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
    root->setContentsMargins(12, 8, 12, 12);
    root->setSpacing(8);

    auto* header = new QVBoxLayout();
    header->setSpacing(0);
    auto* eyebrow = new QLabel("QUBE-SERVO 2 / SYSTEM DECK", container);
    eyebrow->setObjectName("eyebrow");
    auto* title = new QLabel("ROS 2 & ACTUATOR INTELLIGENCE", container);
    title->setObjectName("pageTitle");
    auto* subtitle = new QLabel(
        "Controller runtime · transport health · live plant telemetry · session peaks · safety envelope",
        container);
    subtitle->setObjectName("pageSubtitle");
    header->addWidget(eyebrow);
    header->addWidget(title);
    header->addWidget(subtitle);
    root->addLayout(header);

    // ------------------------------------------------------------------
    // High-level system strip
    // ------------------------------------------------------------------
    auto* hero = new QGridLayout();
    hero->setHorizontalSpacing(7);
    hero->setVerticalSpacing(7);

    hero->addWidget(makeHeroCard("ROS LINK", connection_label_, container), 0, 0);
    hero->addWidget(makeHeroCard("ACTIVE CONTROLLER", controller_hero_, container), 0, 1);
    hero->addWidget(makeHeroCard("TELEMETRY RATE", telemetry_rate_hero_, container), 0, 2);
    hero->addWidget(makeHeroCard("SESSION SAMPLES", sample_count_hero_, container), 0, 3);
    hero->addWidget(makeHeroCard("SESSION UPTIME", uptime_hero_, container), 0, 4);
    for (int c = 0; c < 5; ++c) hero->setColumnStretch(c, 1);
    root->addLayout(hero);

    // ------------------------------------------------------------------
    // Routing + controller runtime
    // ------------------------------------------------------------------
    auto* upper = new QGridLayout();
    upper->setHorizontalSpacing(8);
    upper->setVerticalSpacing(8);

    auto* ros_panel = new widgets::HudPanel("ROS 2 ROUTING", container);
    auto* ros_grid = new QGridLayout(ros_panel);
    ros_grid->setContentsMargins(12, 34, 12, 12);
    ros_grid->setHorizontalSpacing(10);
    ros_grid->setVerticalSpacing(7);

    joint_label_ = makePathValue(ros_panel);
    command_topic_label_ = makePathValue(ros_panel);
    joint_topic_label_ = makePathValue(ros_panel);
    dynamic_topic_label_ = makePathValue(ros_panel);

    const QStringList route_keys{
        "CONTROLLED JOINT", "CONTROLLER MANAGER", "JOINT STATES", "DYNAMIC STATES"};
    const QList<QLabel*> route_values{
        joint_label_, command_topic_label_, joint_topic_label_, dynamic_topic_label_};
    for (int row = 0; row < route_keys.size(); ++row) {
        ros_grid->addWidget(makeEyebrow(route_keys[row], ros_panel), row, 0);
        ros_grid->addWidget(route_values[row], row, 1);
    }
    ros_grid->setColumnStretch(1, 1);

    auto* runtime_panel = new widgets::HudPanel("CONTROL RUNTIME", container);
    auto* runtime_grid = new QGridLayout(runtime_panel);
    runtime_grid->setContentsMargins(12, 34, 12, 12);
    runtime_grid->setHorizontalSpacing(10);
    runtime_grid->setVerticalSpacing(7);

    controller_state_label_ = makeRuntimeValue(runtime_panel);
    controller_mode_label_ = makeRuntimeValue(runtime_panel);
    reference_label_ = makeRuntimeValue(runtime_panel);
    reference_route_label_ = makeRuntimeValue(runtime_panel);

    const QStringList runtime_keys{"STATE", "MODE", "LAST REFERENCE", "REFERENCE ROUTE"};
    const QList<QLabel*> runtime_values{
        controller_state_label_, controller_mode_label_, reference_label_, reference_route_label_};
    for (int row = 0; row < runtime_keys.size(); ++row) {
        runtime_grid->addWidget(makeEyebrow(runtime_keys[row], runtime_panel), row, 0);
        runtime_grid->addWidget(runtime_values[row], row, 1);
    }
    runtime_grid->setColumnStretch(1, 1);

    upper->addWidget(ros_panel, 0, 0);
    upper->addWidget(runtime_panel, 0, 1);
    upper->setColumnStretch(0, 6);
    upper->setColumnStretch(1, 5);
    root->addLayout(upper);

    // ------------------------------------------------------------------
    // Live plant telemetry
    // ------------------------------------------------------------------
    auto* live_panel = new widgets::HudPanel("LIVE PLANT TELEMETRY", container);
    auto* live_grid = new QGridLayout(live_panel);
    live_grid->setContentsMargins(12, 34, 12, 12);
    live_grid->setHorizontalSpacing(7);
    live_grid->setVerticalSpacing(7);

    live_grid->addWidget(makeTelemetryCard("POSITION", "rad", position_label_, live_panel), 0, 0);
    live_grid->addWidget(makeTelemetryCard("ANGULAR SPEED", "rad/s", velocity_label_, live_panel), 0, 1);
    live_grid->addWidget(makeTelemetryCard("MOTOR CURRENT", "A", current_label_, live_panel), 0, 2);
    live_grid->addWidget(makeTelemetryCard("APPLIED VOLTAGE", "V", applied_voltage_label_, live_panel), 0, 3);
    live_grid->addWidget(makeTelemetryCard("BACK EMF", "V", back_emf_label_, live_panel), 1, 0);
    live_grid->addWidget(makeTelemetryCard("MOTOR TORQUE", "N·m", torque_label_, live_panel), 1, 1);
    live_grid->addWidget(makeTelemetryCard("ELECTRICAL POWER", "W", electrical_power_label_, live_panel), 1, 2);
    live_grid->addWidget(makeTelemetryCard("MECHANICAL POWER", "W", mechanical_power_label_, live_panel), 1, 3);
    for (int c = 0; c < 4; ++c) live_grid->setColumnStretch(c, 1);
    root->addWidget(live_panel);

    // ------------------------------------------------------------------
    // Session peaks + safety envelope
    // ------------------------------------------------------------------
    auto* lower = new QGridLayout();
    lower->setHorizontalSpacing(8);
    lower->setVerticalSpacing(8);

    auto* peaks_panel = new widgets::HudPanel("SESSION PEAKS", container);
    auto* peaks_grid = new QGridLayout(peaks_panel);
    peaks_grid->setContentsMargins(12, 34, 12, 12);
    peaks_grid->setHorizontalSpacing(12);
    peaks_grid->setVerticalSpacing(9);

    max_velocity_label_ = makePeakValue(peaks_panel);
    max_current_label_ = makePeakValue(peaks_panel);
    max_voltage_label_ = makePeakValue(peaks_panel);
    max_torque_label_ = makePeakValue(peaks_panel);

    const QStringList peak_keys{"MAX |ω|", "MAX |i|", "MAX |V|", "MAX |τ|"};
    const QList<QLabel*> peak_values{
        max_velocity_label_, max_current_label_, max_voltage_label_, max_torque_label_};
    for (int row = 0; row < peak_keys.size(); ++row) {
        peaks_grid->addWidget(makeEyebrow(peak_keys[row], peaks_panel), row, 0);
        peaks_grid->addWidget(peak_values[row], row, 1);
    }
    peaks_grid->setColumnStretch(1, 1);

    auto* safety_panel = new widgets::HudPanel("SAFETY ENVELOPE", container);
    auto* safety_layout = new QVBoxLayout(safety_panel);
    safety_layout->setContentsMargins(12, 34, 12, 12);
    safety_layout->setSpacing(7);

    auto* safety_grid = new QGridLayout();
    safety_grid->setHorizontalSpacing(10);
    safety_grid->setVerticalSpacing(7);
    voltage_limit_label_ = makeRuntimeValue(safety_panel);
    safety_voltage_label_ = makeRuntimeValue(safety_panel);
    safety_current_label_ = makeRuntimeValue(safety_panel);
    safety_state_label_ = makeRuntimeValue(safety_panel);

    safety_grid->addWidget(makeEyebrow("VOLTAGE LIMIT", safety_panel), 0, 0);
    safety_grid->addWidget(voltage_limit_label_, 0, 1);
    safety_grid->addWidget(makeEyebrow("VOLTAGE UTILIZATION", safety_panel), 1, 0);
    safety_grid->addWidget(safety_voltage_label_, 1, 1);
    safety_grid->addWidget(makeEyebrow("CURRENT DRAW", safety_panel), 2, 0);
    safety_grid->addWidget(safety_current_label_, 2, 1);
    safety_grid->addWidget(makeEyebrow("INTERLOCK STATE", safety_panel), 3, 0);
    safety_grid->addWidget(safety_state_label_, 3, 1);
    safety_grid->setColumnStretch(1, 1);
    safety_layout->addLayout(safety_grid);

    voltage_utilization_bar_ = new QProgressBar(safety_panel);
    voltage_utilization_bar_->setObjectName("aresSafetyBar");
    voltage_utilization_bar_->setRange(0, 100);
    voltage_utilization_bar_->setValue(0);
    voltage_utilization_bar_->setTextVisible(true);
    voltage_utilization_bar_->setFormat("V BUS  %p%");
    safety_layout->addWidget(voltage_utilization_bar_);

    auto* note = new QLabel(
        "Emergency zero publishes the safe reference and requests controller deactivation. "
        "Final physical voltage/current saturation remains enforced by the actuator/hardware layer.",
        safety_panel);
    note->setObjectName("pageSubtitle");
    note->setWordWrap(true);
    safety_layout->addWidget(note);

    auto* zero_btn = new QPushButton("EMERGENCY ZERO / DEACTIVATE", safety_panel);
    zero_btn->setObjectName("dangerAction");
    zero_btn->setMinimumHeight(38);
    connect(zero_btn, &QPushButton::clicked, this, &SystemPage::sendZeroRequested);
    safety_layout->addWidget(zero_btn);

    lower->addWidget(peaks_panel, 0, 0);
    lower->addWidget(safety_panel, 0, 1);
    lower->setColumnStretch(0, 5);
    lower->setColumnStretch(1, 6);
    root->addLayout(lower);
    root->addStretch(1);

    setConnected(false);
    refreshSessionReadouts_();
}

void SystemPage::setConfiguration(const QString& joint,
                                  const QString& command_topic,
                                  const QString& joint_topic,
                                  const QString& dynamic_topic,
                                  double voltage_limit) {
    joint_label_->setText(joint);
    command_topic_label_->setText(command_topic);
    joint_topic_label_->setText(joint_topic);
    dynamic_topic_label_->setText(dynamic_topic);
    voltage_limit_ = std::max(1e-9, std::abs(voltage_limit));
    voltage_limit_label_->setText(QString("±%1 V").arg(voltage_limit_, 0, 'f', 3));
}

QString SystemPage::prettyController_(const QString& controller) {
    if (controller == "motor_voltage_controller") return "OPEN-LOOP VOLTAGE";
    if (controller == "qube_pid_controller") return "PID";
    if (controller == "qube_state_feedback_controller") return "STATE FEEDBACK";
    return controller.isEmpty() ? QString("NONE") : controller.toUpper();
}

QString SystemPage::referenceUnit_(const QString& controller, const QString& mode) {
    if (controller == "motor_voltage_controller") return "V";
    return mode == "velocity" ? "rad/s" : "rad";
}

void SystemPage::setConnected(bool connected) {
    connected_ = connected;
    connection_label_->setText(connected ? "LIVE" : "DISCONNECTED");
    connection_label_->setProperty("connected", connected);
    connection_label_->style()->unpolish(connection_label_);
    connection_label_->style()->polish(connection_label_);

    safety_state_label_->setText(connected ? "ROS LINK NOMINAL" : "TELEMETRY STALE / OFFLINE");
}

void SystemPage::setCommand(double voltage) {
    if (applied_voltage_label_) applied_voltage_label_->setText(QString::number(voltage, 'f', 3));
}

void SystemPage::setControllerState(const QString& controller,
                                    const QString& state,
                                    const QString& mode) {
    active_controller_ = controller;
    active_state_ = state;
    active_mode_ = mode;

    controller_hero_->setText(prettyController_(controller));
    controller_state_label_->setText(state.isEmpty() ? "UNKNOWN" : state.toUpper());
    controller_mode_label_->setText(
        controller == "motor_voltage_controller" ? "DIRECT VOLTAGE" :
        (mode.isEmpty() ? "--" : mode.toUpper()));

    const bool active = state.compare("active", Qt::CaseInsensitive) == 0;
    controller_hero_->setProperty("active", active);
    controller_hero_->style()->unpolish(controller_hero_);
    controller_hero_->style()->polish(controller_hero_);

    safety_state_label_->setText(
        !connected_ ? "TELEMETRY STALE / OFFLINE" :
        (active ? "CONTROLLER ACTIVE · LINK NOMINAL" : "NO ACTIVE COMMAND CONTROLLER"));
}

void SystemPage::setReferencePublished(const QString& controller,
                                       const QString& mode,
                                       double value) {
    last_reference_ = value;
    const QString unit = referenceUnit_(controller, mode);
    reference_label_->setText(QString("%1 %2").arg(value, 0, 'f', 4).arg(unit));
    reference_route_label_->setText(
        QString("%1 / %2").arg(prettyController_(controller),
            controller == "motor_voltage_controller" ? QString("VOLTAGE") : mode.toUpper()));
}

void SystemPage::updateTelemetry(const models::TelemetrySample& sample) {
    if (!sample.valid) return;

    ++sample_count_;
    ++rate_window_samples_;

    max_abs_velocity_ = std::max(max_abs_velocity_, std::abs(sample.velocity_rad_s));
    max_abs_current_ = std::max(max_abs_current_, std::abs(sample.current_a));
    max_abs_voltage_ = std::max(max_abs_voltage_, std::abs(sample.applied_voltage_v));
    max_abs_torque_ = std::max(max_abs_torque_, std::abs(sample.motor_torque_nm));

    if (rate_timer_.elapsed() >= 1000) {
        telemetry_rate_hz_ = static_cast<double>(rate_window_samples_) * 1000.0 /
                             static_cast<double>(std::max<qint64>(1, rate_timer_.elapsed()));
        rate_window_samples_ = 0;
        rate_timer_.restart();
    }

    // The System page is diagnostic, not a control loop. Ten UI updates per
    // second are plenty and avoid unnecessary label/layout work at 50-100 Hz.
    if (ui_throttle_.elapsed() < 100) return;
    ui_throttle_.restart();

    refreshLiveReadouts_(sample);
    refreshSessionReadouts_();
}

void SystemPage::refreshLiveReadouts_(const models::TelemetrySample& sample) {
    position_label_->setText(QString::number(sample.position_rad, 'f', 4));
    velocity_label_->setText(QString::number(sample.velocity_rad_s, 'f', 4));
    current_label_->setText(QString::number(sample.current_a, 'f', 4));
    applied_voltage_label_->setText(QString::number(sample.applied_voltage_v, 'f', 4));
    back_emf_label_->setText(QString::number(sample.back_emf_v, 'f', 4));
    torque_label_->setText(QString::number(sample.motor_torque_nm, 'f', 6));
    electrical_power_label_->setText(QString::number(sample.electrical_power_w(), 'f', 4));
    mechanical_power_label_->setText(QString::number(sample.mechanical_power_w(), 'f', 4));

    max_velocity_label_->setText(QString("%1 rad/s").arg(max_abs_velocity_, 0, 'f', 3));
    max_current_label_->setText(QString("%1 A").arg(max_abs_current_, 0, 'f', 3));
    max_voltage_label_->setText(QString("%1 V").arg(max_abs_voltage_, 0, 'f', 3));
    max_torque_label_->setText(QString("%1 N·m").arg(max_abs_torque_, 0, 'f', 5));

    const double utilization = std::clamp(
        std::abs(sample.applied_voltage_v) / std::max(1e-9, voltage_limit_), 0.0, 1.0);
    const int percent = static_cast<int>(std::lround(utilization * 100.0));
    voltage_utilization_bar_->setValue(percent);
    safety_voltage_label_->setText(
        QString("%1 V · %2 %").arg(sample.applied_voltage_v, 0, 'f', 3).arg(percent));
    safety_current_label_->setText(QString("%1 A").arg(sample.current_a, 0, 'f', 4));
}

void SystemPage::refreshSessionReadouts_() {
    telemetry_rate_hero_->setText(QString("%1 Hz").arg(telemetry_rate_hz_, 0, 'f', 1));
    sample_count_hero_->setText(QString::number(sample_count_));

    const qint64 total_seconds = session_timer_.elapsed() / 1000;
    const qint64 hours = total_seconds / 3600;
    const qint64 minutes = (total_seconds % 3600) / 60;
    const qint64 seconds = total_seconds % 60;
    uptime_hero_->setText(QString("%1:%2:%3")
        .arg(hours, 2, 10, QLatin1Char('0'))
        .arg(minutes, 2, 10, QLatin1Char('0'))
        .arg(seconds, 2, 10, QLatin1Char('0')));

    if (controller_hero_->text().isEmpty()) controller_hero_->setText("NONE");
}

}  // namespace qube_servo2::gui::ui::pages
