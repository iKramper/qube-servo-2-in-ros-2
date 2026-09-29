#include "qube_servo2_gui/ui/pages/control_page.hpp"

#include "qube_servo2_gui/ui/widgets/control_topology_widget.hpp"
#include "qube_servo2_gui/ui/widgets/dial_reference_widget.hpp"
#include "qube_servo2_gui/ui/widgets/hud_panel.hpp"
#include "qube_servo2_gui/ui/widgets/scope_plot.hpp"
#include "qube_servo2_gui/ui/widgets/throttle_lever.hpp"

#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QSpinBox>
#include <QStackedWidget>
#include <QStyle>
#include <QTabBar>
#include <QTabWidget>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::ui::pages {

namespace {
constexpr double kPi = 3.14159265358979323846;

double clampSmall(double value, double eps = 1e-6) {
    return std::abs(value) < eps ? 0.0 : value;
}

QString fixedSmart(double value, int decimals, double eps = 1e-6) {
    return QString::number(clampSmall(value, eps), 'f', decimals);
}

QLabel* makeMetric(const QString& text, QWidget* parent) {
    auto* label = new QLabel(text, parent);
    label->setObjectName("metricValue");
    label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    return label;
}

QLabel* makeEyebrow(const QString& text, QWidget* parent) {
    auto* label = new QLabel(text, parent);
    label->setObjectName("eyebrow");
    return label;
}

QFrame* makeSeparator(QWidget* parent) {
    auto* line = new QFrame(parent);
    line->setObjectName("hudSeparator");
    line->setFrameShape(QFrame::HLine);
    line->setFrameShadow(QFrame::Plain);
    return line;
}

}  // namespace

ControlPage::ControlPage(double voltage_limit,
                         double position_reference_limit,
                         double velocity_reference_limit,
                         QWidget* parent)
    : QWidget(parent),
      voltage_limit_(voltage_limit),
      position_reference_limit_(position_reference_limit),
      velocity_reference_limit_(velocity_reference_limit) {

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 8, 12, 10);
    root->setSpacing(8);

    // =====================================================================
    // Page header
    // =====================================================================
    auto* title_row = new QHBoxLayout();
    title_row->setSpacing(12);

    auto* title_block = new QVBoxLayout();
    title_block->setSpacing(0);
    auto* eyebrow = makeEyebrow("QUBE-SERVO 2 / CONTROL DECK", this);
    auto* title = new QLabel("REFERENCE & CONTROL", this);
    title->setObjectName("pageTitle");
    auto* hint = new QLabel(
        "ROS 2 controller orchestration · automatic excitation · manual setpoint command · live response",
        this);
    hint->setObjectName("pageSubtitle");
    title_block->addWidget(eyebrow);
    title_block->addWidget(title);
    title_block->addWidget(hint);

    recording_btn_ = new QPushButton("DATA RECORDER", this);
    recording_btn_->setObjectName("recordButton");
    recording_btn_->setMinimumWidth(140);
    connect(recording_btn_, &QPushButton::clicked, this, &ControlPage::recordingDialogRequested);

    title_row->addLayout(title_block);
    title_row->addStretch(1);
    title_row->addWidget(recording_btn_, 0, Qt::AlignBottom);
    root->addLayout(title_row);

    body_layout_ = new QGridLayout();
    auto* body = body_layout_;
    body->setContentsMargins(0, 0, 0, 0);
    body->setHorizontalSpacing(8);
    body->setVerticalSpacing(8);

    // =====================================================================
    // CONTROL ARCHITECTURE
    // =====================================================================
    auto* architecture_panel = new widgets::HudPanel("CONTROL ARCHITECTURE", this);
    auto* architecture_layout = new QVBoxLayout(architecture_panel);
    architecture_layout->setContentsMargins(12, 32, 12, 10);
    architecture_layout->setSpacing(7);

    auto* architecture_grid = new QGridLayout();
    architecture_grid->setHorizontalSpacing(8);
    architecture_grid->setVerticalSpacing(6);

    controller_combo_ = new QComboBox(architecture_panel);
    controller_combo_->setObjectName("controllerSelector");
    controller_combo_->addItem("OPEN-LOOP VOLTAGE", "motor_voltage_controller");
    controller_combo_->addItem("PID", "qube_pid_controller");
    controller_combo_->addItem("STATE FEEDBACK", "qube_state_feedback_controller");

    control_mode_ = new QComboBox(architecture_panel);
    control_mode_->setObjectName("modeSelector");
    control_mode_->addItem("POSITION", "position");
    control_mode_->addItem("VELOCITY", "velocity");

    architecture_grid->addWidget(makeEyebrow("CONTROLLER", architecture_panel), 0, 0);
    architecture_grid->addWidget(controller_combo_, 1, 0);
    architecture_grid->addWidget(makeEyebrow("CONTROL MODE", architecture_panel), 0, 1);
    architecture_grid->addWidget(control_mode_, 1, 1);
    architecture_grid->setColumnStretch(0, 3);
    architecture_grid->setColumnStretch(1, 2);
    architecture_layout->addLayout(architecture_grid);

    auto* source_row = new QHBoxLayout();
    source_row->setSpacing(6);
    source_group_ = new QButtonGroup(this);
    source_group_->setExclusive(true);
    automatic_source_btn_ = new QPushButton("AUTOMATIC", architecture_panel);
    manual_source_btn_ = new QPushButton("MANUAL", architecture_panel);
    for (auto* button : {automatic_source_btn_, manual_source_btn_}) {
        button->setCheckable(true);
        button->setObjectName("sourceModeButton");
        button->setMinimumHeight(30);
        source_group_->addButton(button);
        source_row->addWidget(button, 1);
    }
    automatic_source_btn_->setChecked(true);
    source_readout_ = makeMetric("SOURCE · AUTO", architecture_panel);
    source_readout_->setObjectName("sourceReadout");
    source_row->addWidget(source_readout_, 2);
    architecture_layout->addLayout(source_row);

    auto* architecture_actions = new QHBoxLayout();
    architecture_actions->setSpacing(6);
    activate_controller_btn_ = new QPushButton("APPLY / ACTIVATE", architecture_panel);
    activate_controller_btn_->setObjectName("primaryAction");
    deactivate_controller_btn_ = new QPushButton("DEACTIVATE", architecture_panel);
    deactivate_controller_btn_->setObjectName("secondary");
    architecture_actions->addWidget(activate_controller_btn_, 3);
    architecture_actions->addWidget(deactivate_controller_btn_, 2);
    architecture_layout->addLayout(architecture_actions);

    controller_state_ = makeMetric("CONTROLLER · --", architecture_panel);
    controller_state_->setObjectName("controllerStateReadout");
    architecture_layout->addWidget(controller_state_);

    controller_hint_ = new QLabel(
        "The selected controller owns motor_hub_link_joint/voltage exclusively through ros2_control.",
        architecture_panel);
    controller_hint_->setObjectName("pageSubtitle");
    controller_hint_->setWordWrap(true);
    architecture_layout->addWidget(controller_hint_);

    // =====================================================================
    // REFERENCE SOURCE PANEL
    // =====================================================================
    reference_panel_ = new widgets::HudPanel("REFERENCE COMMAND", this);
    auto* reference_panel = reference_panel_;
    auto* reference_layout = new QVBoxLayout(reference_panel);
    reference_layout->setContentsMargins(12, 32, 12, 10);
    reference_layout->setSpacing(7);

    auto* arm_row = new QHBoxLayout();
    output_enabled_ = new QCheckBox("ARM REFERENCE OUTPUT", reference_panel);
    output_enabled_->setObjectName("armSwitch");
    active_source_metric_ = makeMetric("AUTO · --", reference_panel);
    active_source_metric_->setObjectName("sourceStatusMetric");
    arm_row->addWidget(output_enabled_);
    arm_row->addStretch(1);
    arm_row->addWidget(active_source_metric_);
    reference_layout->addLayout(arm_row);
    reference_layout->addWidget(makeSeparator(reference_panel));

    reference_source_stack_ = new QStackedWidget(reference_panel);

    // ---------------------------------------------------------------------
    // Automatic source page
    // ---------------------------------------------------------------------
    auto* automatic_page = new QWidget(reference_source_stack_);
    auto* automatic_layout = new QVBoxLayout(automatic_page);
    automatic_layout->setContentsMargins(0, 0, 0, 0);
    automatic_layout->setSpacing(0);

    // Automatic mode is a self-contained command workstation. Parameters,
    // waveform preview and control topology share the same REFERENCE COMMAND
    // panel so the lower-left deck never feels fragmented or cramped.
    left_tabs_ = new QTabWidget(automatic_page);
    auto* left_tabs = left_tabs_;
    left_tabs->setObjectName("referenceCommandTabs");
    left_tabs->setDocumentMode(true);
    left_tabs->setUsesScrollButtons(false);
    left_tabs->tabBar()->setExpanding(true);

    // ========================= PARAMETERS TAB =========================
    auto* parameters_page = new QWidget(left_tabs);
    auto* parameters_layout = new QVBoxLayout(parameters_page);
    parameters_layout->setContentsMargins(8, 8, 8, 8);
    parameters_layout->setSpacing(8);

    auto* waveform_row = new QHBoxLayout();
    waveform_row->setSpacing(8);
    waveform_row->addWidget(makeEyebrow("WAVEFORM", parameters_page));
    waveform_type_ = new QComboBox(parameters_page);
    waveform_type_->addItems({"Constant", "Step", "Ramp", "Pulse", "Square", "Triangular",
                              "Sinusoidal", "Exponential", "Chirp"});
    waveform_row->addWidget(waveform_type_, 1);
    parameters_layout->addLayout(waveform_row);

    const double widest = std::max({voltage_limit_, position_reference_limit_, velocity_reference_limit_});
    amplitude_ = makeDoubleSpin_(-widest, widest, 1.0, 3, 0.1);
    offset_ = makeDoubleSpin_(-widest, widest, 0.0, 3, 0.1);
    frequency_ = makeDoubleSpin_(0.001, 100.0, 1.0, 3, 0.1);
    chirp_end_frequency_ = makeDoubleSpin_(0.001, 200.0, 10.0, 3, 0.5);
    duty_cycle_ = makeDoubleSpin_(0.1, 99.9, 50.0, 1, 1.0);
    phase_ = makeDoubleSpin_(-360.0, 360.0, 0.0, 1, 5.0);
    pulse_width_ = makeDoubleSpin_(0.001, 3600.0, 0.5, 3, 0.05);
    delay_ = makeDoubleSpin_(0.0, 3600.0, 0.5, 3, 0.1);
    duration_ = makeDoubleSpin_(0.05, 3600.0, 5.0, 3, 0.5);
    initial_value_ = makeDoubleSpin_(-widest, widest, 0.0, 3, 0.1);
    final_value_ = makeDoubleSpin_(-widest, widest, 1.0, 3, 0.1);
    rise_time_ = makeDoubleSpin_(0.001, 3600.0, 1.0, 3, 0.1);
    tau_ = makeDoubleSpin_(0.001, 3600.0, 1.0, 3, 0.1);

    publish_rate_ = new QSpinBox(parameters_page);
    publish_rate_->setRange(10, 500);
    publish_rate_->setValue(100);
    publish_rate_->setSuffix(" Hz");
    publish_rate_->setKeyboardTracking(false);

    auto* forms = new QHBoxLayout();
    forms->setSpacing(12);
    form_left_ = new QFormLayout();
    form_right_ = new QFormLayout();
    form_left_->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form_right_->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
    form_left_->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form_right_->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form_left_->setHorizontalSpacing(8);
    form_right_->setHorizontalSpacing(8);
    form_left_->setVerticalSpacing(7);
    form_right_->setVerticalSpacing(7);

    form_left_->addRow("Amplitude", amplitude_);
    form_left_->addRow("Frequency [Hz]", frequency_);
    form_left_->addRow("Duty cycle [%]", duty_cycle_);
    form_left_->addRow("Pulse width [s]", pulse_width_);
    form_left_->addRow("Duration [s]", duration_);
    form_left_->addRow("Final value", final_value_);
    form_left_->addRow("Tau [s]", tau_);

    form_right_->addRow("Offset", offset_);
    form_right_->addRow("Phase [deg]", phase_);
    form_right_->addRow("Chirp f2 [Hz]", chirp_end_frequency_);
    form_right_->addRow("Delay [s]", delay_);
    form_right_->addRow("Initial value", initial_value_);
    form_right_->addRow("Rise time [s]", rise_time_);
    form_right_->addRow("Publish rate", publish_rate_);

    forms->addLayout(form_left_, 1);
    forms->addLayout(form_right_, 1);
    parameters_layout->addLayout(forms);

    auto* automatic_options = new QHBoxLayout();
    repeat_ = new QCheckBox("REPEAT", parameters_page);
    auto_record_ = new QCheckBox("RECORD WITH EXPERIMENT", parameters_page);
    automatic_options->addWidget(repeat_);
    automatic_options->addWidget(auto_record_);
    automatic_options->addStretch(1);
    parameters_layout->addLayout(automatic_options);

    waveform_help_ = new QLabel(parameters_page);
    waveform_help_->setObjectName("pageSubtitle");
    waveform_help_->setWordWrap(true);
    waveform_help_->setMaximumHeight(44);
    parameters_layout->addWidget(waveform_help_);
    parameters_layout->addStretch(1);

    auto* auto_actions = new QHBoxLayout();
    auto_actions->setSpacing(6);
    start_btn_ = new QPushButton("RUN AUTOMATIC", parameters_page);
    start_btn_->setObjectName("primaryAction");
    stop_btn_ = new QPushButton("SAFE STOP / HOLD", parameters_page);
    stop_btn_->setObjectName("secondary");
    emergency_btn_ = new QPushButton("EMERGENCY ZERO / DEACTIVATE", parameters_page);
    emergency_btn_->setObjectName("dangerAction");
    auto_actions->addWidget(start_btn_, 3);
    auto_actions->addWidget(stop_btn_, 2);
    auto_actions->addWidget(emergency_btn_, 3);
    parameters_layout->addLayout(auto_actions);
    left_tabs->addTab(parameters_page, "PARAMETERS");

    // ====================== REFERENCE PREVIEW TAB =====================
    auto* preview_page = new QWidget(left_tabs);
    auto* preview_layout = new QVBoxLayout(preview_page);
    preview_layout->setContentsMargins(8, 8, 8, 8);
    preview_layout->setSpacing(7);

    auto* preview_config = new QGridLayout();
    preview_config->setContentsMargins(0, 0, 0, 0);
    preview_config->setHorizontalSpacing(5);
    preview_config->setVerticalSpacing(5);
    for (std::size_t i = 0; i < preview_config_labels_.size(); ++i) {
        auto* chip = new QLabel("--", preview_page);
        chip->setObjectName("previewConfigChip");
        chip->setAlignment(Qt::AlignCenter);
        chip->setMinimumHeight(26);
        preview_config_labels_[i] = chip;
        preview_config->addWidget(chip, static_cast<int>(i / 4), static_cast<int>(i % 4));
        preview_config->setColumnStretch(static_cast<int>(i % 4), 1);
    }
    preview_layout->addLayout(preview_config);

    preview_plot_ = new widgets::ScopePlot("", "V", {"Reference"}, preview_page);
    preview_plot_->setMinimumHeight(280);
    preview_plot_->setFollowLatest(false);
    preview_layout->addWidget(preview_plot_, 1);
    left_tabs->addTab(preview_page, "REFERENCE PREVIEW");

    // ======================= CONTROL TOPOLOGY TAB =====================
    auto* topology_page = new QWidget(left_tabs);
    auto* topology_layout = new QVBoxLayout(topology_page);
    topology_layout->setContentsMargins(8, 8, 8, 8);
    topology_widget_ = new widgets::ControlTopologyWidget(topology_page);
    topology_widget_->setMinimumHeight(300);
    topology_layout->addWidget(topology_widget_, 1);
    left_tabs->addTab(topology_page, "CONTROL TOPOLOGY");

    automatic_layout->addWidget(left_tabs, 1);
    reference_source_stack_->addWidget(automatic_page);

    // ---------------------------------------------------------------------
    // Manual source page
    // ---------------------------------------------------------------------
    auto* manual_page = new QWidget(reference_source_stack_);
    auto* manual_layout = new QVBoxLayout(manual_page);
    manual_layout->setContentsMargins(0, 0, 0, 0);
    manual_layout->setSpacing(7);

    auto* manual_header = new QHBoxLayout();
    manual_header->addWidget(makeEyebrow("MANUAL SETPOINT", manual_page));
    manual_header->addStretch(1);
    manual_value_label_ = makeMetric("0.000", manual_page);
    manual_value_label_->setObjectName("manualValueReadout");
    manual_header->addWidget(manual_value_label_);
    manual_layout->addLayout(manual_header);

    manual_widget_stack_ = new QStackedWidget(manual_page);
    manual_widget_stack_->setObjectName("manualInstrumentStack");

    // Manual voltage slider -------------------------------------------------
    auto* voltage_manual = new QWidget(manual_widget_stack_);
    auto* voltage_layout = new QVBoxLayout(voltage_manual);
    voltage_layout->setContentsMargins(20, 18, 20, 12);
    voltage_layout->setSpacing(10);
    auto* voltage_caption = new QLabel("DIRECT MOTOR VOLTAGE", voltage_manual);
    voltage_caption->setObjectName("manualInstrumentTitle");
    voltage_caption->setAlignment(Qt::AlignCenter);
    voltage_slider_ = new QSlider(Qt::Horizontal, voltage_manual);
    voltage_slider_->setObjectName("voltageReferenceSlider");
    voltage_slider_->setRange(-1000, 1000);
    voltage_slider_->setValue(0);
    voltage_slider_->setMinimumHeight(58);
    auto* voltage_scale = new QHBoxLayout();
    auto* vmin = new QLabel(QString("-%1 V").arg(voltage_limit_, 0, 'f', 1), voltage_manual);
    auto* vzero = new QLabel("0 V", voltage_manual);
    auto* vmax = new QLabel(QString("+%1 V").arg(voltage_limit_, 0, 'f', 1), voltage_manual);
    for (auto* l : {vmin, vzero, vmax}) l->setObjectName("instrumentScale");
    voltage_scale->addWidget(vmin);
    voltage_scale->addStretch(1);
    voltage_scale->addWidget(vzero);
    voltage_scale->addStretch(1);
    voltage_scale->addWidget(vmax);
    voltage_layout->addStretch(1);
    voltage_layout->addWidget(voltage_caption);
    voltage_layout->addWidget(voltage_slider_);
    voltage_layout->addLayout(voltage_scale);
    voltage_layout->addStretch(1);
    manual_widget_stack_->addWidget(voltage_manual);

    // Manual position dial --------------------------------------------------
    auto* position_page = new QWidget(manual_widget_stack_);
    auto* position_page_layout = new QHBoxLayout(position_page);
    position_page_layout->setContentsMargins(10, 6, 10, 6);
    position_page_layout->addStretch(1);
    position_dial_ = new widgets::DialReferenceWidget(position_page);
    position_dial_->setRange(0.0, std::min(position_reference_limit_, 2.0 * kPi));
    position_dial_->setValue(0.0);
    position_dial_->setMinimumSize(300, 300);
    position_dial_->setMaximumWidth(440);
    position_page_layout->addWidget(position_dial_, 0, Qt::AlignCenter);
    position_page_layout->addStretch(1);
    manual_widget_stack_->addWidget(position_page);

    // Manual velocity throttle ----------------------------------------------
    // Wrap the instrument in a centered page. QStackedWidget otherwise forces
    // the throttle itself to fill the whole horizontal deck, which makes a
    // vertical lever look like a wide slider.
    auto* velocity_page = new QWidget(manual_widget_stack_);
    auto* velocity_page_layout = new QHBoxLayout(velocity_page);
    velocity_page_layout->setContentsMargins(12, 6, 12, 6);
    velocity_page_layout->addStretch(1);
    velocity_throttle_ = new widgets::ThrottleLever(velocity_page);
    velocity_throttle_->setRange(-velocity_reference_limit_, velocity_reference_limit_);
    velocity_throttle_->setValue(0.0);
    velocity_throttle_->setMinimumSize(220, 330);
    velocity_throttle_->setMaximumWidth(260);
    velocity_throttle_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    velocity_page_layout->addWidget(velocity_throttle_, 0, Qt::AlignHCenter | Qt::AlignVCenter);
    velocity_page_layout->addStretch(1);
    manual_widget_stack_->addWidget(velocity_page);

    manual_widget_stack_->setMinimumHeight(365);
    manual_layout->addWidget(manual_widget_stack_, 1);

    // Safety commands sit immediately beneath the physical/manual instrument.
    // This keeps the operator's command and stop controls in one visual block.
    auto* manual_actions = new QHBoxLayout();
    manual_actions->setSpacing(6);
    auto* manual_safe_stop = new QPushButton("SAFE STOP / HOLD", manual_page);
    manual_safe_stop->setObjectName("secondary");
    manual_emergency_btn_ = new QPushButton("EMERGENCY ZERO / DEACTIVATE", manual_page);
    manual_emergency_btn_->setObjectName("dangerAction");
    manual_actions->addWidget(manual_safe_stop, 2);
    manual_actions->addWidget(manual_emergency_btn_, 3);
    manual_layout->addLayout(manual_actions);
    connect(manual_safe_stop, &QPushButton::clicked, this, &ControlPage::stopOutput);

    manual_layout->addWidget(makeSeparator(manual_page));

    auto* exact_row = new QHBoxLayout();
    exact_row->setSpacing(6);
    exact_row->addWidget(makeEyebrow("EXACT SETPOINT", manual_page));
    manual_numeric_ = makeDoubleSpin_(-widest, widest, 0.0, 4, 0.05);
    manual_numeric_->setObjectName("manualNumeric");
    exact_row->addWidget(manual_numeric_, 1);
    manual_zero_btn_ = new QPushButton("ZERO", manual_page);
    manual_zero_btn_->setObjectName("secondary");
    capture_position_btn_ = new QPushButton("CAPTURE POSITION", manual_page);
    capture_position_btn_->setObjectName("secondary");
    exact_row->addWidget(manual_zero_btn_);
    exact_row->addWidget(capture_position_btn_);
    manual_layout->addLayout(exact_row);

    manual_hint_ = new QLabel(
        "Manual changes are event-driven and publish only while the selected controller is ACTIVE and the reference is ARMED.",
        manual_page);
    manual_hint_->setObjectName("pageSubtitle");
    manual_hint_->setWordWrap(true);
    manual_layout->addWidget(manual_hint_);

    reference_source_stack_->addWidget(manual_page);
    reference_layout->addWidget(reference_source_stack_, 1);

    // =====================================================================
    // Live response monitor
    // =====================================================================
    auto* monitor_panel = new widgets::HudPanel("LIVE REFERENCE / RESPONSE", this);
    auto* monitor_layout = new QVBoxLayout(monitor_panel);
    monitor_layout->setContentsMargins(10, 32, 10, 10);
    monitor_layout->setSpacing(5);

    auto* monitor_header = new QHBoxLayout();
    monitor_header->setSpacing(6);
    monitor_mode_ = new QComboBox(monitor_panel);
    monitor_mode_->addItem("VOLTAGE", 0);
    monitor_mode_->addItem("VELOCITY", 1);
    monitor_mode_->addItem("POSITION", 2);
    monitor_header->addWidget(makeEyebrow("VIEW", monitor_panel));
    monitor_header->addWidget(monitor_mode_);
    monitor_header->addStretch(1);
    command_value_ = makeMetric("REF · --", monitor_panel);
    applied_value_ = makeMetric("RESP · --", monitor_panel);
    current_value_ = makeMetric("i · -- A", monitor_panel);
    monitor_header->addWidget(command_value_);
    monitor_header->addWidget(applied_value_);
    monitor_header->addWidget(current_value_);
    monitor_layout->addLayout(monitor_header);

    live_plot_ = new widgets::ScopePlot("", "V", {"Reference", "Response"}, monitor_panel);
    live_plot_->setMinimumHeight(470);
    live_plot_->setSeriesDashed(0, true);
    live_plot_->setWindowSeconds(15.0);
    monitor_layout->addWidget(live_plot_, 1);

    auto* monitor_footer = new QHBoxLayout();
    auto* hover_hint = new QLabel(
        "HOVER · interpolated cursor values   |   WHEEL · time zoom   |   Y/FIT · vertical scale",
        monitor_panel);
    hover_hint->setObjectName("scopeHint");
    monitor_footer->addWidget(hover_hint);
    monitor_footer->addStretch(1);
    monitor_layout->addLayout(monitor_footer);

    body->addWidget(architecture_panel, 0, 0);
    body->addWidget(reference_panel, 1, 0);
    body->addWidget(monitor_panel, 0, 1, 2, 1);
    body->setColumnStretch(0, 5);
    body->setColumnStretch(1, 8);
    body->setRowStretch(0, 2);
    body->setRowStretch(1, 9);
    root->addLayout(body, 1);

    // =====================================================================
    // Connections
    // =====================================================================
    waveform_timer_ = new QTimer(this);
    waveform_timer_->setTimerType(Qt::PreciseTimer);
    connect(waveform_timer_, &QTimer::timeout, this, &ControlPage::tickWaveform_);

    // Manual input can generate many mouse-move events. Coalesce them to a
    // maximum of ~50 Hz so Qt remains responsive while the controller still
    // receives a smooth command stream. The timer always publishes the newest
    // setpoint, including the final value after a drag gesture.
    manual_publish_timer_ = new QTimer(this);
    manual_publish_timer_->setSingleShot(true);
    manual_publish_timer_->setInterval(20);
    manual_publish_timer_->setTimerType(Qt::PreciseTimer);
    connect(manual_publish_timer_, &QTimer::timeout, this, &ControlPage::flushManualReference_);

    connect(start_btn_, &QPushButton::clicked, this, &ControlPage::startWaveform_);
    connect(stop_btn_, &QPushButton::clicked, this, &ControlPage::stopOutput);
    const auto emergency_action = [this]() {
        output_enabled_->setChecked(false);
        if (waveform_timer_) waveform_timer_->stop();
        if (manual_publish_timer_) manual_publish_timer_->stop();
        const bool was_running = running_;
        running_ = false;
        emit emergencyStopRequested();
        if (was_running) emit waveformStopped();
        refreshSourceReadout_();
    };
    connect(emergency_btn_, &QPushButton::clicked, this, emergency_action);
    connect(manual_emergency_btn_, &QPushButton::clicked, this, emergency_action);

    connect(activate_controller_btn_, &QPushButton::clicked, this, &ControlPage::activateSelectedController_);
    connect(deactivate_controller_btn_, &QPushButton::clicked, this, [this]() {
        stopOutput();
        output_enabled_->setChecked(false);
        emit controllersDeactivateRequested();
    });

    connect(controller_combo_, &QComboBox::currentIndexChanged, this, &ControlPage::updateControllerSelection_);
    connect(control_mode_, &QComboBox::currentIndexChanged, this, &ControlPage::updateControllerSelection_);
    connect(automatic_source_btn_, &QPushButton::clicked, this, &ControlPage::setAutomaticSource_);
    connect(manual_source_btn_, &QPushButton::clicked, this, &ControlPage::setManualSource_);
    connect(waveform_type_, &QComboBox::currentIndexChanged, this, &ControlPage::updateControlsForWaveform_);
    connect(monitor_mode_, &QComboBox::currentIndexChanged, this, &ControlPage::updateMonitorMode_);
    connect(output_enabled_, &QCheckBox::toggled, this, [this](bool armed) {
        refreshSourceReadout_();
        if (armed && manual_source_ && controllerMatchesSelection_()) {
            setManualReference_(manual_reference_, true);
        }
    });

    connect(manual_numeric_, &QDoubleSpinBox::valueChanged, this, &ControlPage::onManualNumericChanged_);
    connect(voltage_slider_, &QSlider::valueChanged, this, &ControlPage::onVoltageSliderChanged_);
    connect(position_dial_, &widgets::DialReferenceWidget::valueChanged, this, &ControlPage::onManualDialChanged_);
    connect(velocity_throttle_, &widgets::ThrottleLever::valueChanged, this, &ControlPage::onThrottleChanged_);
    connect(manual_zero_btn_, &QPushButton::clicked, this, [this]() { setManualReference_(0.0, true); });
    connect(capture_position_btn_, &QPushButton::clicked, this, &ControlPage::captureCurrentPosition_);

    const QList<QObject*> preview_sources{
        amplitude_, offset_, frequency_, chirp_end_frequency_, duty_cycle_, phase_, pulse_width_, delay_, duration_,
        initial_value_, final_value_, rise_time_, tau_, publish_rate_, repeat_};
    for (QObject* object : preview_sources) {
        if (auto* ds = qobject_cast<QDoubleSpinBox*>(object)) {
            connect(ds, &QDoubleSpinBox::valueChanged, this, &ControlPage::refreshPreview_);
        } else if (auto* is = qobject_cast<QSpinBox*>(object)) {
            connect(is, &QSpinBox::valueChanged, this, &ControlPage::refreshPreview_);
        } else if (auto* cb = qobject_cast<QCheckBox*>(object)) {
            connect(cb, &QCheckBox::toggled, this, &ControlPage::refreshPreview_);
        }
    }

    updateControllerSelection_();
    updateControlsForWaveform_();
    updateReferenceSourceUi_();
    updateMonitorMode_();
    refreshPreview_();
}

ControlPage::~ControlPage() {
    shutdown();
}

QString ControlPage::selectedController() const {
    return controller_combo_->currentData().toString();
}

QString ControlPage::selectedMode() const {
    if (selectedController() == "motor_voltage_controller") return "voltage";
    return control_mode_->currentData().toString();
}

bool ControlPage::manualReferenceSelected() const noexcept {
    return manual_source_;
}

void ControlPage::setActive(bool active) {
    if (shutting_down_) return;
    active_ = active;
    if (preview_plot_) preview_plot_->setActive(active);
    if (live_plot_) live_plot_->setActive(active);
    if (active_ && !live_plot_synced_) rebuildLivePlot_();
}

void ControlPage::shutdown() {
    if (shutting_down_) return;
    shutting_down_ = true;
    if (waveform_timer_) waveform_timer_->stop();
    if (manual_publish_timer_) manual_publish_timer_->stop();
    running_ = false;
    reference_by_mode_.fill(0.0);
    monitor_history_.clear();
    if (preview_plot_) preview_plot_->shutdown();
    if (live_plot_) live_plot_->shutdown();
}

QDoubleSpinBox* ControlPage::makeDoubleSpin_(double min,
                                             double max,
                                             double value,
                                             int decimals,
                                             double step) {
    auto* spin = new QDoubleSpinBox(this);
    spin->setRange(min, max);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    spin->setValue(value);
    spin->setKeyboardTracking(false);
    return spin;
}

models::WaveformType ControlPage::selectedType_() const {
    return static_cast<models::WaveformType>(waveform_type_->currentIndex());
}

models::WaveformConfig ControlPage::waveformConfig() const {
    models::WaveformConfig c;
    c.type = selectedType_();
    c.amplitude = amplitude_->value();
    c.offset = offset_->value();
    c.frequency_hz = frequency_->value();
    c.chirp_end_frequency_hz = chirp_end_frequency_->value();
    c.duty_cycle_percent = duty_cycle_->value();
    c.phase_deg = phase_->value();
    c.pulse_width_s = pulse_width_->value();
    c.delay_s = delay_->value();
    c.duration_s = duration_->value();
    c.initial_value = initial_value_->value();
    c.final_value = final_value_->value();
    c.rise_time_s = rise_time_->value();
    c.tau_s = tau_->value();
    c.repeat = repeat_->isChecked();
    c.publish_rate_hz = publish_rate_->value();
    return c;
}

bool ControlPage::isRunning() const noexcept {
    return running_;
}

bool ControlPage::autoRecordEnabled() const noexcept {
    return !manual_source_ && auto_record_->isChecked();
}

void ControlPage::activateSelectedController_() {
    stopOutput();
    output_enabled_->setChecked(false);
    controller_state_->setText("CONTROLLER · CONFIGURING…");
    activate_controller_btn_->setEnabled(false);
    emit controllerActivationRequested(selectedController(), selectedMode());
}

void ControlPage::setControllerOperationResult(bool success, const QString& message) {
    activate_controller_btn_->setEnabled(true);
    controller_hint_->setText((success ? "✓ " : "⚠ ") + message);
}

void ControlPage::setControllerState(const QString& controller,
                                     const QString& state,
                                     const QString& mode) {
    active_controller_ = controller;
    active_state_ = state;
    if (!mode.isEmpty()) active_mode_ = mode;

    if (controller.isEmpty()) {
        active_mode_.clear();
        controller_state_->setText("CONTROLLER · NONE ACTIVE");
        refreshSourceReadout_();
        return;
    }

    QString display = controller;
    if (controller == "motor_voltage_controller") display = "OPEN-LOOP VOLTAGE";
    else if (controller == "qube_pid_controller") display = "PID";
    else if (controller == "qube_state_feedback_controller") display = "STATE FEEDBACK";

    controller_state_->setText(QString("%1 · %2 · %3")
        .arg(display,
             state.toUpper(),
             active_mode_.isEmpty() ? QString("MODE ?") : active_mode_.toUpper()));
    refreshSourceReadout_();
}

bool ControlPage::controllerMatchesSelection_() const noexcept {
    if (active_state_ != "active" || active_controller_ != selectedController()) return false;
    if (selectedController() == "motor_voltage_controller") return true;
    return active_mode_ == selectedMode();
}

void ControlPage::updateControllerSelection_() {
    if (waveform_timer_) waveform_timer_->stop();
    if (running_) emit waveformStopped();
    running_ = false;
    output_enabled_->setChecked(false);

    const bool closed_loop = selectedController() != "motor_voltage_controller";
    control_mode_->setEnabled(closed_loop);
    if (!closed_loop) control_mode_->setCurrentIndex(0);

    if (topology_widget_) topology_widget_->setConfiguration(selectedController(), selectedMode());

    monitor_mode_->blockSignals(true);
    monitor_mode_->setCurrentIndex(currentMonitorIndex_());
    monitor_mode_->blockSignals(false);

    updateReferenceSemantics_();
    updateManualWidget_();
    updateMonitorMode_();
    refreshPreview_();
    refreshSourceReadout_();
}

void ControlPage::setAutomaticSource_() {
    if (!manual_source_) return;
    manual_source_ = false;
    if (waveform_timer_) waveform_timer_->stop();
    running_ = false;
    output_enabled_->setChecked(false);
    updateReferenceSourceUi_();
    if (left_tabs_) left_tabs_->setCurrentIndex(0);
    refreshPreview_();
}

void ControlPage::setManualSource_() {
    if (manual_source_) return;
    const bool was_running = running_;
    if (waveform_timer_) waveform_timer_->stop();
    running_ = false;
    if (was_running) emit waveformStopped();
    manual_source_ = true;
    output_enabled_->setChecked(false);
    updateReferenceSourceUi_();
    updateManualWidget_();
    refreshPreview_();
}

void ControlPage::updateReferenceSourceUi_() {
    automatic_source_btn_->setChecked(!manual_source_);
    manual_source_btn_->setChecked(manual_source_);
    reference_source_stack_->setCurrentIndex(manual_source_ ? 1 : 0);

    // The REFERENCE COMMAND panel now owns the complete lower-left deck in
    // both modes. AUTOMATIC switches between PARAMETERS / REFERENCE PREVIEW /
    // CONTROL TOPOLOGY inside that panel. MANUAL replaces the whole deck with
    // the physical operator instrument and its safety controls.
    if (manual_widget_stack_) {
        manual_widget_stack_->setMinimumHeight(manual_source_ ? 365 : 300);
    }

    refreshSourceReadout_();
    updatePreviewSummary_();
}

int ControlPage::currentMonitorIndex_() const noexcept {
    if (selectedController() == "motor_voltage_controller") return 0;
    return selectedMode() == "velocity" ? 1 : 2;
}

double ControlPage::currentReferenceLimit_() const noexcept {
    if (selectedController() == "motor_voltage_controller") return voltage_limit_;
    if (selectedMode() == "velocity") return velocity_reference_limit_;
    return position_reference_limit_;
}

QString ControlPage::currentReferenceUnit_() const {
    if (selectedController() == "motor_voltage_controller") return "V";
    return selectedMode() == "velocity" ? "rad/s" : "rad";
}

void ControlPage::updateReferenceSemantics_() {
    const double limit = currentReferenceLimit_();
    const QString unit = currentReferenceUnit_();

    for (auto* s : {amplitude_, offset_, initial_value_, final_value_}) {
        s->setRange(-limit, limit);
    }

    if (auto* l = qobject_cast<QLabel*>(form_left_->labelForField(amplitude_))) {
        l->setText(QString("Amplitude [%1]").arg(unit));
    }
    if (auto* l = qobject_cast<QLabel*>(form_left_->labelForField(final_value_))) {
        l->setText(QString("Final value [%1]").arg(unit));
    }
    if (auto* l = qobject_cast<QLabel*>(form_right_->labelForField(offset_))) {
        l->setText(QString("Offset [%1]").arg(unit));
    }
    if (auto* l = qobject_cast<QLabel*>(form_right_->labelForField(initial_value_))) {
        l->setText(QString("Initial value [%1]").arg(unit));
    }

    output_enabled_->setText(QString("ARM REFERENCE OUTPUT  [%1]").arg(unit));
    preview_plot_->setYLabel(unit);
    preview_plot_->setYRange(-limit * 1.05, limit * 1.05);

    manual_numeric_->setRange(
        selectedMode() == "position" && selectedController() != "motor_voltage_controller" ? 0.0 : -limit,
        limit);
    manual_numeric_->setSuffix(" " + unit);
}

void ControlPage::updateManualWidget_() {
    int index = 0;
    if (selectedController() != "motor_voltage_controller") {
        index = selectedMode() == "velocity" ? 2 : 1;
    }
    manual_widget_stack_->setCurrentIndex(index);

    const double limit = currentReferenceLimit_();
    const QString unit = currentReferenceUnit_();

    capture_position_btn_->setVisible(index == 1);
    manual_zero_btn_->setText(index == 1 ? "ZERO ANGLE" : "ZERO");

    if (index == 0) {
        manual_hint_->setText(
            "Direct voltage reference. Drag the rail or type an exact value. Commands are clamped to the configured ±V limit.");
    } else if (index == 1) {
        position_dial_->setRange(0.0, std::min(limit, 2.0 * kPi));
        position_dial_->setUnit(unit);
        manual_hint_->setText(
            "Position potentiometer. CAPTURE POSITION copies the measured shaft angle into the setpoint for a bumpless manual handoff.");
    } else {
        velocity_throttle_->setRange(-limit, limit);
        velocity_throttle_->setUnit(unit);
        manual_hint_->setText(
            "Bipolar velocity throttle. The center detent is 0 rad/s; above center commands positive speed and below center negative speed.");
    }

    const double min_value = (index == 1) ? 0.0 : -limit;
    manual_reference_ = std::clamp(manual_reference_, min_value, limit);
    reference_by_mode_[static_cast<std::size_t>(currentMonitorIndex_())] = manual_reference_;
    syncManualEditors_(manual_reference_);
}

void ControlPage::syncManualEditors_(double value, QObject* origin) {
    const double limit = currentReferenceLimit_();
    const int index = manual_widget_stack_->currentIndex();
    const double minimum = index == 1 ? 0.0 : -limit;
    const double clamped = std::clamp(value, minimum, limit);

    if (origin != manual_numeric_) {
        const QSignalBlocker blocker(manual_numeric_);
        manual_numeric_->setValue(clamped);
    }

    if (origin != voltage_slider_) {
        const QSignalBlocker blocker(voltage_slider_);
        const int raw = voltage_limit_ > 0.0
            ? static_cast<int>(std::round(std::clamp(clamped / voltage_limit_, -1.0, 1.0) * 1000.0))
            : 0;
        voltage_slider_->setValue(raw);
    }

    if (origin != position_dial_) {
        const QSignalBlocker blocker(position_dial_);
        position_dial_->setValue(std::clamp(clamped, 0.0, std::min(position_reference_limit_, 2.0 * kPi)));
    }

    if (origin != velocity_throttle_) {
        const QSignalBlocker blocker(velocity_throttle_);
        velocity_throttle_->setValue(std::clamp(clamped, -velocity_reference_limit_, velocity_reference_limit_));
    }

    manual_value_label_->setText(QString("%1  %2")
        .arg(QString::number(clamped, 'f', 3), currentReferenceUnit_()));
}

void ControlPage::setManualReference_(double value, bool publish_if_armed) {
    const double limit = currentReferenceLimit_();
    const double minimum = (selectedController() != "motor_voltage_controller" && selectedMode() == "position")
        ? 0.0 : -limit;
    manual_reference_ = std::clamp(value, minimum, limit);
    last_reference_ = clampSmall(manual_reference_, 1e-7);
    reference_by_mode_[static_cast<std::size_t>(currentMonitorIndex_())] = last_reference_;
    syncManualEditors_(manual_reference_);
    refreshPreview_();

    if (publish_if_armed && manual_source_ && output_enabled_->isChecked() && controllerMatchesSelection_()) {
        if (manual_publish_timer_ && !manual_publish_timer_->isActive()) manual_publish_timer_->start();
    }
    refreshSourceReadout_();
}

void ControlPage::onManualNumericChanged_(double value) {
    setManualReference_(value, true);
}

void ControlPage::onVoltageSliderChanged_(int raw) {
    const double value = voltage_limit_ * static_cast<double>(raw) / 1000.0;
    manual_reference_ = value;
    last_reference_ = clampSmall(value, 1e-7);
    reference_by_mode_[0] = last_reference_;
    syncManualEditors_(value, voltage_slider_);
    refreshPreview_();
    if (manual_source_ && output_enabled_->isChecked() && controllerMatchesSelection_()) {
        if (manual_publish_timer_ && !manual_publish_timer_->isActive()) manual_publish_timer_->start();
    }
    refreshSourceReadout_();
}

void ControlPage::onManualDialChanged_(double value) {
    manual_reference_ = value;
    last_reference_ = clampSmall(value, 1e-7);
    reference_by_mode_[2] = last_reference_;
    syncManualEditors_(value, position_dial_);
    refreshPreview_();
    if (manual_source_ && output_enabled_->isChecked() && controllerMatchesSelection_()) {
        if (manual_publish_timer_ && !manual_publish_timer_->isActive()) manual_publish_timer_->start();
    }
    refreshSourceReadout_();
}

void ControlPage::onThrottleChanged_(double value) {
    manual_reference_ = value;
    last_reference_ = clampSmall(value, 1e-7);
    reference_by_mode_[1] = last_reference_;
    syncManualEditors_(value, velocity_throttle_);
    refreshPreview_();
    if (manual_source_ && output_enabled_->isChecked() && controllerMatchesSelection_()) {
        if (manual_publish_timer_ && !manual_publish_timer_->isActive()) manual_publish_timer_->start();
    }
    refreshSourceReadout_();
}

void ControlPage::flushManualReference_() {
    if (shutting_down_ || !manual_source_ || !output_enabled_->isChecked() || !controllerMatchesSelection_()) return;
    emit referenceRequested(selectedController(), selectedMode(), last_reference_);
}

void ControlPage::captureCurrentPosition_() {
    if (!have_telemetry_) {
        manual_hint_->setText("No valid position telemetry is available yet.");
        return;
    }
    setManualReference_(wrappedRadians_(last_position_), true);
}

void ControlPage::refreshSourceReadout_() {
    const QString source = manual_source_ ? "MANUAL" : "AUTO";
    const QString armed = output_enabled_ && output_enabled_->isChecked() ? "ARMED" : "SAFE";
    const QString active = controllerMatchesSelection_() ? "LINKED" : "NOT LINKED";
    source_readout_->setText(QString("SOURCE · %1 · %2").arg(source, armed));
    active_source_metric_->setText(QString("%1 · %2 · %3")
        .arg(source, currentReferenceUnit_(), active));
}

void ControlPage::startWaveform_() {
    if (shutting_down_ || manual_source_) return;
    if (!controllerMatchesSelection_()) {
        waveform_help_->setText("Activate the selected controller/mode before sending its reference.");
        return;
    }
    if (!output_enabled_->isChecked()) {
        waveform_help_->setText("Reference output is SAFE. Enable ARM REFERENCE OUTPUT before running the waveform.");
        return;
    }

    generator_.configure(waveformConfig());
    elapsed_.restart();
    running_ = true;
    waveform_timer_->start(std::max(2, 1000 / generator_.config().publish_rate_hz));
    emit waveformStarted(generator_.config());
    tickWaveform_();
    refreshSourceReadout_();
}

void ControlPage::tickWaveform_() {
    if (shutting_down_ || !running_ || manual_source_) return;
    const double t = static_cast<double>(elapsed_.elapsed()) / 1000.0;
    const auto& cfg = generator_.config();
    if (!cfg.repeat && t > (cfg.delay_s + cfg.duration_s)) {
        stopOutput();
        return;
    }

    const double ref = std::clamp(generator_.value(t), -currentReferenceLimit_(), currentReferenceLimit_());
    last_reference_ = clampSmall(ref, 1e-7);
    reference_by_mode_[static_cast<std::size_t>(currentMonitorIndex_())] = last_reference_;
    emit referenceRequested(selectedController(), selectedMode(), last_reference_);
}

double ControlPage::safeStopReference_() const noexcept {
    if (selectedController() == "motor_voltage_controller") return 0.0;
    if (selectedMode() == "velocity") return 0.0;
    return have_telemetry_ ? wrappedRadians_(last_position_) : last_reference_;
}

void ControlPage::stopOutput() {
    if (waveform_timer_) waveform_timer_->stop();
    const bool was_running = running_;
    running_ = false;

    const double safe = safeStopReference_();
    last_reference_ = safe;
    manual_reference_ = safe;
    reference_by_mode_[static_cast<std::size_t>(currentMonitorIndex_())] = safe;
    syncManualEditors_(safe);

    if (controllerMatchesSelection_()) {
        emit referenceRequested(selectedController(), selectedMode(), safe);
    }
    if (was_running) emit waveformStopped();
    refreshPreview_();
    refreshSourceReadout_();
}

void ControlPage::setRecordingState(bool active) {
    recording_btn_->setText(active ? "● RECORDING" : "DATA RECORDER");
    recording_btn_->setProperty("recording", active);
    recording_btn_->style()->unpolish(recording_btn_);
    recording_btn_->style()->polish(recording_btn_);
}

void ControlPage::updateTelemetry(const models::TelemetrySample& sample) {
    if (shutting_down_ || !sample.valid) return;
    last_position_ = sample.position_rad;
    have_telemetry_ = true;

    if (live_t0_ < 0.0) live_t0_ = sample.ros_time_s;
    const double t = sample.ros_time_s - live_t0_;
    appendMonitorHistory_(sample, t);
    updateLiveMetrics_(sample);

    if (active_ && live_plot_) {
        const int mode = std::clamp(monitor_mode_->currentIndex(), 0, 2);
        const auto& h = monitor_history_.back();
        live_plot_->append(0, h.t, h.reference[static_cast<std::size_t>(mode)]);
        live_plot_->append(1, h.t, h.response[static_cast<std::size_t>(mode)]);
        live_plot_synced_ = true;
    } else {
        live_plot_synced_ = false;
    }
}

void ControlPage::appendMonitorHistory_(const models::TelemetrySample& sample, double t) {
    MonitorHistorySample h;
    h.t = t;
    h.reference[0] = selectedController() == "motor_voltage_controller"
        ? reference_by_mode_[0] : sample.applied_voltage_v;
    h.reference[1] = reference_by_mode_[1];
    h.reference[2] = wrappedRadians_(reference_by_mode_[2]);
    h.response[0] = clampSmall(sample.applied_voltage_v, 1e-7);
    h.response[1] = clampSmall(sample.velocity_rad_s, 1e-7);
    h.response[2] = wrappedRadians_(sample.position_rad);
    monitor_history_.push_back(h);
    pruneMonitorHistory_(t);
}

void ControlPage::pruneMonitorHistory_(double newest_t) {
    const double min_t = newest_t - monitor_history_retention_s_;
    while (!monitor_history_.empty() && monitor_history_.front().t < min_t) {
        monitor_history_.pop_front();
    }
    while (monitor_history_.size() > monitor_history_max_points_) {
        monitor_history_.pop_front();
    }
}

void ControlPage::rebuildLivePlot_() {
    if (!live_plot_) return;
    live_plot_->clear();
    const int mode = std::clamp(monitor_mode_->currentIndex(), 0, 2);
    QList<QPointF> ref;
    QList<QPointF> response;
    ref.reserve(static_cast<qsizetype>(monitor_history_.size()));
    response.reserve(static_cast<qsizetype>(monitor_history_.size()));
    for (const auto& h : monitor_history_) {
        ref.append(QPointF(h.t, h.reference[static_cast<std::size_t>(mode)]));
        response.append(QPointF(h.t, h.response[static_cast<std::size_t>(mode)]));
    }
    live_plot_->setSeriesData(0, ref);
    live_plot_->setSeriesData(1, response);
    live_plot_synced_ = true;
}

void ControlPage::updateLiveMetrics_(const models::TelemetrySample& sample) {
    const int mode = std::clamp(monitor_mode_->currentIndex(), 0, 2);
    double reference = 0.0;
    double response = 0.0;
    QString unit;

    if (mode == 1) {
        reference = reference_by_mode_[1];
        response = sample.velocity_rad_s;
        unit = "rad/s";
    } else if (mode == 2) {
        reference = wrappedRadians_(reference_by_mode_[2]);
        response = wrappedRadians_(sample.position_rad);
        unit = "rad";
    } else {
        reference = selectedController() == "motor_voltage_controller"
            ? reference_by_mode_[0] : sample.applied_voltage_v;
        response = sample.applied_voltage_v;
        unit = "V";
    }

    command_value_->setText(QString("REF · %1 %2").arg(fixedSmart(reference, 3), unit));
    applied_value_->setText(QString("RESP · %1 %2").arg(fixedSmart(response, 3), unit));
    current_value_->setText(QString("i · %1 A").arg(fixedSmart(sample.current_a, 4)));
}

double ControlPage::wrappedRadians_(double radians) const noexcept {
    const double two_pi = 2.0 * kPi;
    double wrapped = std::fmod(radians, two_pi);
    if (wrapped < 0.0) wrapped += two_pi;
    return std::abs(wrapped) < 1e-8 || std::abs(wrapped - two_pi) < 1e-8 ? 0.0 : wrapped;
}

void ControlPage::updateMonitorMode_() {
    if (!live_plot_ || !monitor_mode_) return;
    live_plot_->setSeriesName(0, "Reference");
    live_plot_->setSeriesName(1, "Response");
    live_plot_->setSeriesDashed(0, true);

    switch (monitor_mode_->currentIndex()) {
    case 1:
        live_plot_->setYLabel("rad/s");
        live_plot_->setYRange(-velocity_reference_limit_ * 1.05, velocity_reference_limit_ * 1.05);
        break;
    case 2:
        live_plot_->setYLabel("rad");
        live_plot_->setYRange(0.0, 2.0 * kPi);
        break;
    default:
        live_plot_->setYLabel("V");
        live_plot_->setYRange(-voltage_limit_ * 1.05, voltage_limit_ * 1.05);
        break;
    }

    live_plot_synced_ = false;
    if (active_) rebuildLivePlot_();
}

void ControlPage::updatePreviewSummary_() {
    if (!preview_config_labels_[0]) return;

    const auto cfg = waveformConfig();
    const QString unit = currentReferenceUnit_();

    QString p1_key = "P1";
    QString p1_value = QString::number(cfg.amplitude, 'f', 3);
    QString p2_key = "P2";
    QString p2_value = QString::number(cfg.offset, 'f', 3);
    QString timing_key = "FREQ";
    QString timing_value = QString("%1 Hz").arg(cfg.frequency_hz, 0, 'f', 3);

    switch (cfg.type) {
    case models::WaveformType::Constant:
        p1_key = "LEVEL";
        p1_value = QString::number(cfg.offset + cfg.amplitude, 'f', 3);
        p2_key = "OFFSET";
        p2_value = QString::number(cfg.offset, 'f', 3);
        timing_key = "DELAY";
        timing_value = QString("%1 s").arg(cfg.delay_s, 0, 'f', 3);
        break;
    case models::WaveformType::Step:
        p1_key = "INITIAL";
        p1_value = QString::number(cfg.initial_value, 'f', 3);
        p2_key = "FINAL";
        p2_value = QString::number(cfg.final_value, 'f', 3);
        timing_key = "DELAY";
        timing_value = QString("%1 s").arg(cfg.delay_s, 0, 'f', 3);
        break;
    case models::WaveformType::Ramp:
        p1_key = "INITIAL";
        p1_value = QString::number(cfg.initial_value, 'f', 3);
        p2_key = "FINAL";
        p2_value = QString::number(cfg.final_value, 'f', 3);
        timing_key = "RISE";
        timing_value = QString("%1 s").arg(cfg.rise_time_s, 0, 'f', 3);
        break;
    case models::WaveformType::Pulse:
        p1_key = "AMP";
        p1_value = QString::number(cfg.amplitude, 'f', 3);
        p2_key = "OFFSET";
        p2_value = QString::number(cfg.offset, 'f', 3);
        timing_key = "WIDTH";
        timing_value = QString("%1 s").arg(cfg.pulse_width_s, 0, 'f', 3);
        break;
    case models::WaveformType::Square:
        p1_key = "AMP";
        p1_value = QString::number(cfg.amplitude, 'f', 3);
        p2_key = "DUTY";
        p2_value = QString("%1 %").arg(cfg.duty_cycle_percent, 0, 'f', 1);
        break;
    case models::WaveformType::Triangle:
    case models::WaveformType::Sine:
        p1_key = "AMP";
        p1_value = QString::number(cfg.amplitude, 'f', 3);
        p2_key = "OFFSET";
        p2_value = QString::number(cfg.offset, 'f', 3);
        break;
    case models::WaveformType::Exponential:
        p1_key = "INITIAL";
        p1_value = QString::number(cfg.initial_value, 'f', 3);
        p2_key = "FINAL";
        p2_value = QString::number(cfg.final_value, 'f', 3);
        timing_key = "TAU";
        timing_value = QString("%1 s").arg(cfg.tau_s, 0, 'f', 3);
        break;
    case models::WaveformType::Chirp:
        p1_key = "AMP";
        p1_value = QString::number(cfg.amplitude, 'f', 3);
        p2_key = "F2";
        p2_value = QString("%1 Hz").arg(cfg.chirp_end_frequency_hz, 0, 'f', 3);
        timing_key = "F1";
        timing_value = QString("%1 Hz").arg(cfg.frequency_hz, 0, 'f', 3);
        break;
    }

    const std::array<QString, 8> values{
        QString("TYPE · %1").arg(cfg.name().toUpper()),
        QString("UNIT · %1").arg(unit),
        QString("%1 · %2 %3").arg(p1_key, p1_value, unit),
        QString("%1 · %2%3").arg(p2_key, p2_value,
            (p2_key == "DUTY" || p2_key == "F2") ? QString() : QString(" ") + unit),
        QString("%1 · %2").arg(timing_key, timing_value),
        QString("DURATION · %1 s").arg(cfg.duration_s, 0, 'f', 3),
        QString("RATE · %1 Hz").arg(cfg.publish_rate_hz),
        QString("LOOP · %1").arg(cfg.repeat ? "ON" : "OFF")
    };

    for (std::size_t i = 0; i < values.size(); ++i) {
        if (preview_config_labels_[i]) preview_config_labels_[i]->setText(values[i]);
    }
}

void ControlPage::refreshPreview_() {
    if (!preview_plot_) return;
    updatePreviewSummary_();

    // Preview/topology are intentionally hidden in MANUAL mode. Avoid chart
    // rebuilds while the operator drags a dial/throttle; this keeps the manual
    // path extremely responsive.
    if (manual_source_) return;

    preview_plot_->clear();

    const double limit = currentReferenceLimit_();
    preview_plot_->setYLabel(currentReferenceUnit_());
    const bool manual_position = manual_source_ &&
        selectedController() != "motor_voltage_controller" && selectedMode() == "position";
    preview_plot_->setYRange(manual_position ? 0.0 : -limit * 1.05, limit * 1.05);

    if (manual_source_) {
        const double horizon = 5.0;
        preview_plot_->setWindowSeconds(horizon);
        QList<QPointF> points;
        points.reserve(2);
        points.append(QPointF(0.0, manual_reference_));
        points.append(QPointF(horizon, manual_reference_));
        preview_plot_->setSeriesData(0, points);
        return;
    }

    services::WaveformGenerator preview;
    const auto cfg = waveformConfig();
    preview.configure(cfg);
    const double horizon = std::clamp(std::max(2.0, cfg.delay_s + cfg.duration_s), 2.0, 60.0);
    preview_plot_->setWindowSeconds(horizon);
    constexpr int points = 320;
    for (int i = 0; i < points; ++i) {
        const double t = horizon * static_cast<double>(i) / static_cast<double>(points - 1);
        const double value = std::clamp(preview.value(t), -limit, limit);
        preview_plot_->append(0, t, value);
    }
}

void ControlPage::updateFieldVisibility_() {
    const auto type = selectedType_();
    auto set_row = [](QFormLayout* form, QWidget* field, bool visible) {
        field->setVisible(visible);
        if (auto* label = form->labelForField(field)) label->setVisible(visible);
    };
    auto left = [this, &set_row](QWidget* w, bool v) { set_row(form_left_, w, v); };
    auto right = [this, &set_row](QWidget* w, bool v) { set_row(form_right_, w, v); };

    const QList<QWidget*> left_fields{
        amplitude_, frequency_, duty_cycle_, pulse_width_, duration_, final_value_, tau_};
    const QList<QWidget*> right_fields{
        offset_, phase_, chirp_end_frequency_, delay_, initial_value_, rise_time_, publish_rate_};

    for (QWidget* w : left_fields) left(w, false);
    for (QWidget* w : right_fields) right(w, false);

    right(publish_rate_, true);
    left(duration_, true);

    switch (type) {
    case models::WaveformType::Constant:
        left(amplitude_, true); right(offset_, true); break;
    case models::WaveformType::Step:
        right(initial_value_, true); left(final_value_, true); right(delay_, true); break;
    case models::WaveformType::Ramp:
        right(initial_value_, true); left(final_value_, true); right(delay_, true); right(rise_time_, true); break;
    case models::WaveformType::Pulse:
        left(amplitude_, true); right(offset_, true); left(pulse_width_, true); right(delay_, true); break;
    case models::WaveformType::Square:
        left(amplitude_, true); right(offset_, true); left(frequency_, true); left(duty_cycle_, true); right(phase_, true); break;
    case models::WaveformType::Triangle:
        left(amplitude_, true); right(offset_, true); left(frequency_, true); right(phase_, true); break;
    case models::WaveformType::Sine:
        left(amplitude_, true); right(offset_, true); left(frequency_, true); right(phase_, true); break;
    case models::WaveformType::Exponential:
        right(initial_value_, true); left(final_value_, true); right(delay_, true); left(tau_, true); break;
    case models::WaveformType::Chirp:
        left(amplitude_, true); right(offset_, true); left(frequency_, true); right(chirp_end_frequency_, true); right(phase_, true); break;
    }
}

void ControlPage::updateControlsForWaveform_() {
    updateFieldVisibility_();
    QString help;
    switch (selectedType_()) {
    case models::WaveformType::Constant:
        help = "Constant reference: offset + amplitude."; break;
    case models::WaveformType::Step:
        help = "Step reference: initial value until delay, then final value."; break;
    case models::WaveformType::Ramp:
        help = "Ramp reference: linear transition during rise time."; break;
    case models::WaveformType::Pulse:
        help = "Pulse reference: one-shot amplitude around offset after delay."; break;
    case models::WaveformType::Square:
        help = "Square reference: bipolar amplitude around offset."; break;
    case models::WaveformType::Triangle:
        help = "Triangular reference around offset."; break;
    case models::WaveformType::Sine:
        help = "Sinusoidal reference: offset + amplitude·sin(2πft + phase)."; break;
    case models::WaveformType::Exponential:
        help = "Exponential first-order reference transition."; break;
    case models::WaveformType::Chirp:
        help = "Chirp reference: linear frequency sweep from f1 to f2."; break;
    }
    waveform_help_->setText(help + "  Current units: " + currentReferenceUnit_() + ".");
    refreshPreview_();
}

}  // namespace qube_servo2::gui::ui::pages
