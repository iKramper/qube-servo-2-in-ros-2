#include "qube_servo2_gui/ui/pages/control_page.hpp"
#include "qube_servo2_gui/ui/widgets/scope_plot.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QStyle>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::ui::pages {

namespace {
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
}

ControlPage::ControlPage(double voltage_limit, QWidget* parent)
    : QWidget(parent), voltage_limit_(voltage_limit) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(12, 10, 12, 10);
    root->setSpacing(8);

    auto* title_row = new QHBoxLayout();
    auto* title = new QLabel("QUBE-SERVO 2 / SIGNAL GENERATOR", this);
    title->setObjectName("title");
    auto* hint = new QLabel("Waveform excitation, live tracking and experiment acquisition", this);
    hint->setObjectName("subtitle");
    recording_btn_ = new QPushButton("Data Recorder", this);
    recording_btn_->setObjectName("recordButton");
    recording_btn_->setToolTip("Open data acquisition configuration and recorder status.");
    connect(recording_btn_, &QPushButton::clicked, this, &ControlPage::recordingDialogRequested);
    title_row->addWidget(title);
    title_row->addSpacing(14);
    title_row->addWidget(hint);
    title_row->addStretch(1);
    title_row->addWidget(recording_btn_);
    root->addLayout(title_row);

    auto* body = new QGridLayout();
    body->setContentsMargins(0, 0, 0, 0);
    body->setHorizontalSpacing(8);
    body->setVerticalSpacing(8);

    auto* config_panel = new QFrame(this);
    config_panel->setObjectName("panel");
    auto* config_layout = new QVBoxLayout(config_panel);
    config_layout->setContentsMargins(10, 8, 10, 8);
    config_layout->setSpacing(6);

    auto* config_title = new QLabel("WAVEFORM CONFIGURATION", config_panel);
    config_title->setObjectName("panelTitle");
    config_layout->addWidget(config_title);

    auto* waveform_row = new QHBoxLayout();
    auto* waveform_label = new QLabel("Waveform", config_panel);
    waveform_label->setObjectName("fieldLabel");
    waveform_type_ = new QComboBox(config_panel);
    waveform_type_->addItems({"Constant", "Step", "Ramp", "Pulse", "Square", "Triangular",
                              "Sinusoidal", "Exponential", "Chirp"});
    waveform_row->addWidget(waveform_label);
    waveform_row->addWidget(waveform_type_, 1);
    config_layout->addLayout(waveform_row);

    amplitude_ = makeDoubleSpin_(-voltage_limit_, voltage_limit_, 1.0, 3, 0.1);
    offset_ = makeDoubleSpin_(-voltage_limit_, voltage_limit_, 0.0, 3, 0.1);
    frequency_ = makeDoubleSpin_(0.001, 100.0, 1.0, 3, 0.1);
    chirp_end_frequency_ = makeDoubleSpin_(0.001, 200.0, 10.0, 3, 0.5);
    duty_cycle_ = makeDoubleSpin_(0.1, 99.9, 50.0, 1, 1.0);
    phase_ = makeDoubleSpin_(-360.0, 360.0, 0.0, 1, 5.0);
    pulse_width_ = makeDoubleSpin_(0.001, 3600.0, 0.5, 3, 0.05);
    delay_ = makeDoubleSpin_(0.0, 3600.0, 0.5, 3, 0.1);
    duration_ = makeDoubleSpin_(0.05, 3600.0, 5.0, 3, 0.5);
    initial_value_ = makeDoubleSpin_(-voltage_limit_, voltage_limit_, 0.0, 3, 0.1);
    final_value_ = makeDoubleSpin_(-voltage_limit_, voltage_limit_, 1.0, 3, 0.1);
    rise_time_ = makeDoubleSpin_(0.001, 3600.0, 1.0, 3, 0.1);
    tau_ = makeDoubleSpin_(0.001, 3600.0, 1.0, 3, 0.1);

    publish_rate_ = new QSpinBox(config_panel);
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

    form_left_->addRow("Amplitude [V]", amplitude_);
    form_left_->addRow("Frequency [Hz]", frequency_);
    form_left_->addRow("Duty cycle [%]", duty_cycle_);
    form_left_->addRow("Pulse width [s]", pulse_width_);
    form_left_->addRow("Duration [s]", duration_);
    form_left_->addRow("Final value [V]", final_value_);
    form_left_->addRow("Tau [s]", tau_);

    form_right_->addRow("Offset [V]", offset_);
    form_right_->addRow("Phase [deg]", phase_);
    form_right_->addRow("Chirp f2 [Hz]", chirp_end_frequency_);
    form_right_->addRow("Delay [s]", delay_);
    form_right_->addRow("Initial value [V]", initial_value_);
    form_right_->addRow("Rise time [s]", rise_time_);
    form_right_->addRow("Publish rate", publish_rate_);

    forms->addLayout(form_left_, 1);
    forms->addLayout(form_right_, 1);
    config_layout->addLayout(forms);

    auto* options = new QHBoxLayout();
    repeat_ = new QCheckBox("Repeat", config_panel);
    auto_record_ = new QCheckBox("Record with experiment", config_panel);
    output_enabled_ = new QCheckBox(QString("ARM OUTPUT  ±%1 V").arg(voltage_limit_), config_panel);
    options->addWidget(repeat_);
    options->addWidget(auto_record_);
    options->addWidget(output_enabled_);
    options->addStretch(1);
    config_layout->addLayout(options);

    waveform_help_ = new QLabel(config_panel);
    waveform_help_->setObjectName("subtitle");
    waveform_help_->setWordWrap(true);
    waveform_help_->setMaximumHeight(38);
    config_layout->addWidget(waveform_help_);

    auto* buttons = new QHBoxLayout();
    start_btn_ = new QPushButton("Start", config_panel);
    stop_btn_ = new QPushButton("Stop / 0 V", config_panel);
    stop_btn_->setObjectName("secondary");
    emergency_btn_ = new QPushButton("Emergency zero", config_panel);
    emergency_btn_->setObjectName("danger");
    buttons->addWidget(start_btn_);
    buttons->addWidget(stop_btn_);
    buttons->addWidget(emergency_btn_);
    config_layout->addLayout(buttons);

    auto* preview_panel = new QFrame(this);
    preview_panel->setObjectName("panel");
    auto* preview_layout = new QVBoxLayout(preview_panel);
    preview_layout->setContentsMargins(8, 6, 8, 8);
    preview_layout->setSpacing(4);
    auto* preview_title = new QLabel("WAVEFORM PREVIEW", preview_panel);
    preview_title->setObjectName("panelTitle");
    preview_layout->addWidget(preview_title);
    preview_plot_ = new widgets::ScopePlot("", "V", {"Preview"}, preview_panel);
    preview_plot_->setMinimumHeight(250);
    preview_plot_->setFollowLatest(false);
    preview_plot_->setYRange(-voltage_limit_ * 1.05, voltage_limit_ * 1.05);
    preview_layout->addWidget(preview_plot_, 1);

    auto* monitor_panel = new QFrame(this);
    monitor_panel->setObjectName("panel");
    auto* monitor_layout = new QVBoxLayout(monitor_panel);
    monitor_layout->setContentsMargins(8, 6, 8, 8);
    monitor_layout->setSpacing(4);

    auto* monitor_header = new QHBoxLayout();
    auto* monitor_title = new QLabel("LIVE REFERENCE / RESPONSE", monitor_panel);
    monitor_title->setObjectName("panelTitle");
    monitor_mode_ = new QComboBox(monitor_panel);
    monitor_mode_->addItems({"Voltage", "Speed", "Position"});
    monitor_mode_->setToolTip(
        "Select the quantity visualized as reference and response. "
        "The active ROS 2 controller still determines the command semantics.");
    command_value_ = makeMetric("Ref  -- V", monitor_panel);
    applied_value_ = makeMetric("Resp  -- V", monitor_panel);
    current_value_ = makeMetric("i  -- A", monitor_panel);
    monitor_header->addWidget(monitor_title);
    monitor_header->addSpacing(10);
    monitor_header->addWidget(new QLabel("View:", monitor_panel));
    monitor_header->addWidget(monitor_mode_);
    monitor_header->addStretch(1);
    monitor_header->addWidget(command_value_);
    monitor_header->addWidget(applied_value_);
    monitor_header->addWidget(current_value_);
    monitor_layout->addLayout(monitor_header);

    live_plot_ = new widgets::ScopePlot("", "V", {"Reference", "Response"}, monitor_panel);
    live_plot_->setMinimumHeight(430);
    live_plot_->setSeriesDashed(0, true);
    live_plot_->setYRange(-voltage_limit_ * 1.05, voltage_limit_ * 1.05);
    live_plot_->setWindowSeconds(15.0);
    monitor_layout->addWidget(live_plot_, 1);

    body->addWidget(config_panel, 0, 0);
    body->addWidget(preview_panel, 1, 0);
    body->addWidget(monitor_panel, 0, 1, 2, 1);
    body->setColumnStretch(0, 5);
    body->setColumnStretch(1, 8);
    body->setRowStretch(0, 6);
    body->setRowStretch(1, 5);
    root->addLayout(body, 1);

    waveform_timer_ = new QTimer(this);
    waveform_timer_->setTimerType(Qt::PreciseTimer);
    connect(waveform_timer_, &QTimer::timeout, this, &ControlPage::tickWaveform_);
    connect(start_btn_, &QPushButton::clicked, this, &ControlPage::startWaveform_);
    connect(stop_btn_, &QPushButton::clicked, this, &ControlPage::stopOutput);
    connect(emergency_btn_, &QPushButton::clicked, this, [this]() {
        output_enabled_->setChecked(false);
        stopOutput();
    });
    connect(waveform_type_, &QComboBox::currentIndexChanged, this, &ControlPage::updateControlsForWaveform_);
    connect(monitor_mode_, &QComboBox::currentIndexChanged, this, &ControlPage::updateMonitorMode_);

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

    updateControlsForWaveform_();
    updateMonitorMode_();
    refreshPreview_();
}


ControlPage::~ControlPage() {
    shutdown();
}

void ControlPage::setActive(bool active) {
    if (shutting_down_) return;
    active_ = active;
    if (preview_plot_) preview_plot_->setActive(active);
    if (live_plot_) live_plot_->setActive(active);

    // Hidden pages keep only a lightweight numeric ring buffer. QChart is
    // reconstructed in one batch when the page becomes visible again.
    if (active_ && !live_plot_synced_) rebuildLivePlot_();
}

void ControlPage::shutdown() {
    if (shutting_down_) return;
    shutting_down_ = true;

    if (waveform_timer_) waveform_timer_->stop();
    running_ = false;
    reference_by_mode_.fill(0.0);
    monitor_history_.clear();
    live_plot_synced_ = false;

    if (preview_plot_) preview_plot_->shutdown();
    if (live_plot_) live_plot_->shutdown();
}

QDoubleSpinBox* ControlPage::makeDoubleSpin_(double min, double max, double value, int decimals, double step) {
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

bool ControlPage::isRunning() const noexcept { return running_; }
bool ControlPage::autoRecordEnabled() const noexcept { return auto_record_->isChecked(); }

void ControlPage::startWaveform_() {
    if (shutting_down_) return;
    if (!output_enabled_->isChecked()) {
        waveform_help_->setText("Output is DISARMED. Enable ARM OUTPUT before starting the experiment.");
        return;
    }

    generator_.configure(waveformConfig());
    elapsed_.restart();
    running_ = true;
    const int period_ms = std::max(2, 1000 / generator_.config().publish_rate_hz);
    waveform_timer_->start(period_ms);
    emit waveformStarted(generator_.config());
    tickWaveform_();
}

void ControlPage::tickWaveform_() {
    if (shutting_down_ || !running_) return;

    const double t = static_cast<double>(elapsed_.elapsed()) / 1000.0;
    const auto& cfg = generator_.config();
    if (!cfg.repeat && t > (cfg.delay_s + cfg.duration_s)) {
        stopOutput();
        return;
    }

    const double command = std::clamp(generator_.value(t), -voltage_limit_, voltage_limit_);
    const int mode = monitor_mode_ ? std::clamp(monitor_mode_->currentIndex(), 0, 2) : 0;
    reference_by_mode_[static_cast<std::size_t>(mode)] = clampSmall(command, 1e-7);
    emit commandRequested(command);
}

void ControlPage::stopOutput() {
    waveform_timer_->stop();
    running_ = false;
    const int mode = monitor_mode_ ? std::clamp(monitor_mode_->currentIndex(), 0, 2) : 0;
    reference_by_mode_[static_cast<std::size_t>(mode)] = 0.0;
    emit commandRequested(0.0);
    emit stopRequested();
    emit waveformStopped();
}

void ControlPage::setRecordingState(bool active) {
    recording_btn_->setText(active ? "● RECORDING" : "Data Recorder");
    recording_btn_->setProperty("recording", active);
    recording_btn_->style()->unpolish(recording_btn_);
    recording_btn_->style()->polish(recording_btn_);
}

void ControlPage::updateTelemetry(const models::TelemetrySample& sample) {
    if (shutting_down_ || !sample.valid) return;

    if (live_t0_ < 0.0) live_t0_ = sample.ros_time_s;
    const double t = sample.ros_time_s - live_t0_;

    appendMonitorHistory_(sample, t);
    updateLiveMetrics_(sample);

    // QChart updates only while this page is visible. Hidden operation stores
    // compact numeric history only, so changing tabs does not restart time and
    // does not spend CPU/GPU rendering an invisible chart.
    if (active_ && live_plot_) {
        const int mode = monitor_mode_ ? std::clamp(monitor_mode_->currentIndex(), 0, 2) : 0;
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

    h.reference[0] = clampSmall(sample.command_voltage, 1e-7);
    h.reference[1] = clampSmall(reference_by_mode_[1], 1e-7);
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
    const int mode = monitor_mode_ ? std::clamp(monitor_mode_->currentIndex(), 0, 2) : 0;

    QList<QPointF> reference_points;
    QList<QPointF> response_points;
    reference_points.reserve(static_cast<qsizetype>(monitor_history_.size()));
    response_points.reserve(static_cast<qsizetype>(monitor_history_.size()));

    for (const auto& h : monitor_history_) {
        reference_points.append(QPointF(h.t, h.reference[static_cast<std::size_t>(mode)]));
        response_points.append(QPointF(h.t, h.response[static_cast<std::size_t>(mode)]));
    }

    live_plot_->setSeriesData(0, reference_points);
    live_plot_->setSeriesData(1, response_points);
    live_plot_synced_ = true;
}

void ControlPage::updateLiveMetrics_(const models::TelemetrySample& sample) {
    const int mode = monitor_mode_ ? std::clamp(monitor_mode_->currentIndex(), 0, 2) : 0;
    double reference = 0.0;
    double response = 0.0;
    QString unit;
    int decimals = 3;

    switch (mode) {
    case 1:
        reference = clampSmall(reference_by_mode_[1], 1e-7);
        response = clampSmall(sample.velocity_rad_s, 1e-7);
        unit = "rad/s";
        break;
    case 2:
        reference = wrappedRadians_(reference_by_mode_[2]);
        response = wrappedRadians_(sample.position_rad);
        unit = "rad";
        break;
    case 0:
    default:
        reference = clampSmall(sample.command_voltage, 1e-7);
        response = clampSmall(sample.applied_voltage_v, 1e-7);
        unit = "V";
        break;
    }

    command_value_->setText(QString("Ref  %1 %2").arg(fixedSmart(reference, decimals), unit));
    applied_value_->setText(QString("Resp  %1 %2").arg(fixedSmart(response, decimals), unit));
    current_value_->setText(QString("i  %1 A").arg(fixedSmart(sample.current_a, 4)));
}

double ControlPage::wrappedRadians_(double radians) const noexcept {
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 2.0 * kPi;
    double wrapped = std::fmod(radians, kTwoPi);
    if (wrapped < 0.0) wrapped += kTwoPi;
    return std::abs(wrapped) < 1e-8 || std::abs(wrapped - kTwoPi) < 1e-8 ? 0.0 : wrapped;
}

void ControlPage::updateMonitorMode_() {
    if (!live_plot_ || !monitor_mode_) return;

    live_plot_->setSeriesName(0, "Reference");
    live_plot_->setSeriesName(1, "Response");
    live_plot_->setSeriesDashed(0, true);

    switch (monitor_mode_->currentIndex()) {
    case 1:  // Speed
        live_plot_->setYLabel("rad/s");
        live_plot_->setYRange(-10.0, 10.0);
        command_value_->setText("Ref  0.000 rad/s");
        applied_value_->setText("Resp  0.000 rad/s");
        break;
    case 2:  // Position
        live_plot_->setYLabel("rad");
        live_plot_->setYRange(0.0, 2.0 * 3.14159265358979323846);
        command_value_->setText("Ref  0.000 rad");
        applied_value_->setText("Resp  0.000 rad");
        break;
    case 0:
    default:  // Voltage
        live_plot_->setYLabel("V");
        live_plot_->setYRange(-voltage_limit_ * 1.05, voltage_limit_ * 1.05);
        command_value_->setText("Ref  0.000 V");
        applied_value_->setText("Resp  0.000 V");
        break;
    }

    live_plot_synced_ = false;
    if (active_) rebuildLivePlot_();
}

void ControlPage::refreshPreview_() {
    preview_plot_->clear();
    const auto cfg = waveformConfig();
    services::WaveformGenerator preview;
    preview.configure(cfg);

    const double horizon = std::clamp(std::max(2.0, cfg.delay_s + cfg.duration_s), 2.0, 60.0);
    preview_plot_->setWindowSeconds(horizon);
    constexpr int kPoints = 320;
    for (int i = 0; i < kPoints; ++i) {
        const double t = horizon * static_cast<double>(i) / static_cast<double>(kPoints - 1);
        preview_plot_->append(0, t, std::clamp(preview.value(t), -voltage_limit_, voltage_limit_));
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

    // Start hidden, then expose only parameters that affect the selected waveform.
    const QList<QWidget*> left_fields{amplitude_, frequency_, duty_cycle_, pulse_width_, duration_, final_value_, tau_};
    const QList<QWidget*> right_fields{offset_, phase_, chirp_end_frequency_, delay_, initial_value_, rise_time_, publish_rate_};
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
    case models::WaveformType::Constant: help = "Constant: offset + amplitude."; break;
    case models::WaveformType::Step: help = "Step: initial value until delay, then final value."; break;
    case models::WaveformType::Ramp: help = "Ramp: linear transition from initial to final during rise time."; break;
    case models::WaveformType::Pulse: help = "Pulse: one-shot amplitude around offset after delay."; break;
    case models::WaveformType::Square: help = "Square: bipolar amplitude around offset with configurable duty cycle."; break;
    case models::WaveformType::Triangle: help = "Triangular: symmetric triangle around offset."; break;
    case models::WaveformType::Sine: help = "Sinusoidal: offset + amplitude·sin(2πft + phase)."; break;
    case models::WaveformType::Exponential: help = "Exponential: first-order transition from initial to final with time constant τ."; break;
    case models::WaveformType::Chirp: help = "Chirp: linear frequency sweep from f1 to f2 during the experiment."; break;
    }
    waveform_help_->setText(help);
    refreshPreview_();
}

}  // namespace qube_servo2::gui::ui::pages
