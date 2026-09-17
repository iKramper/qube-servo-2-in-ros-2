#include "qube_servo2_gui/ui/pages/recording_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTextEdit>
#include <QVBoxLayout>

namespace qube_servo2::gui::ui::pages {

namespace {
QDoubleSpinBox* makeDoubleSpin(QWidget* parent,
                               double min,
                               double max,
                               double value,
                               int decimals,
                               double step) {
    auto* spin = new QDoubleSpinBox(parent);
    spin->setRange(min, max);
    spin->setValue(value);
    spin->setDecimals(decimals);
    spin->setSingleStep(step);
    spin->setKeyboardTracking(false);
    return spin;
}

QFrame* makeSubPanel(QWidget* parent, const QString& title, QVBoxLayout*& layout_out) {
    auto* panel = new QFrame(parent);
    panel->setObjectName("subPanel");
    auto* layout = new QVBoxLayout(panel);
    layout->setContentsMargins(10, 8, 10, 8);
    layout->setSpacing(7);
    auto* label = new QLabel(title, panel);
    label->setObjectName("panelTitle");
    layout->addWidget(label);
    layout_out = layout;
    return panel;
}
}  // namespace

RecordingDialog::RecordingDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("QUBE-Servo 2 / Data Acquisition");
    setModal(false);
    setMinimumSize(780, 660);
    resize(860, 720);

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(14, 14, 14, 14);
    root->setSpacing(10);

    auto* title = new QLabel("EXPERIMENT RECORDING", this);
    title->setObjectName("title");
    root->addWidget(title);

    auto* panel = new QFrame(this);
    panel->setObjectName("panel");
    auto* panel_layout = new QVBoxLayout(panel);
    panel_layout->setContentsMargins(12, 10, 12, 10);
    panel_layout->setSpacing(9);

    auto* form = new QFormLayout();
    form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);

    auto* directory_row = new QHBoxLayout();
    directory_edit_ = new QLineEdit(panel);
    const QString documents = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    directory_edit_->setText(QDir(documents).filePath("qube_servo2_data"));
    auto* browse_btn = new QPushButton("Browse", panel);
    browse_btn->setObjectName("secondary");
    connect(browse_btn, &QPushButton::clicked, this, &RecordingDialog::browseDirectory_);
    directory_row->addWidget(directory_edit_, 1);
    directory_row->addWidget(browse_btn);
    form->addRow("Output directory", directory_row);

    prefix_edit_ = new QLineEdit("qube_run", panel);
    form->addRow("File prefix", prefix_edit_);

    notes_edit_ = new QTextEdit(panel);
    notes_edit_->setPlaceholderText("Experiment objective, controller gains, simulation / hardware conditions…");
    notes_edit_->setMaximumHeight(75);
    form->addRow("Notes", notes_edit_);

    metadata_check_ = new QCheckBox("Write companion YAML metadata file", panel);
    metadata_check_->setChecked(true);
    form->addRow("", metadata_check_);
    panel_layout->addLayout(form);

    QVBoxLayout* capture_layout = nullptr;
    auto* capture_panel = makeSubPanel(panel, "SMART CAPTURE", capture_layout);

    auto* capture_grid = new QGridLayout();
    capture_grid->setHorizontalSpacing(12);
    capture_grid->setVerticalSpacing(7);

    capture_mode_ = new QComboBox(capture_panel);
    capture_mode_->addItems({"Manual / continuous", "Time window", "N periods", "Peak count"});
    capture_grid->addWidget(new QLabel("Capture mode", capture_panel), 0, 0);
    capture_grid->addWidget(capture_mode_, 0, 1);

    sample_interval_ms_ = new QSpinBox(capture_panel);
    sample_interval_ms_->setRange(0, 10000);
    sample_interval_ms_->setValue(0);
    sample_interval_ms_->setSuffix(" ms");
    sample_interval_ms_->setToolTip("0 ms saves every telemetry sample. Increase this to decimate the file without changing peak detection.");
    capture_grid->addWidget(new QLabel("Save interval", capture_panel), 0, 2);
    capture_grid->addWidget(sample_interval_ms_, 0, 3);

    waveform_info_ = new QLabel("Waveform: --", capture_panel);
    waveform_info_->setObjectName("subtitle");
    capture_grid->addWidget(waveform_info_, 1, 0, 1, 4);
    capture_grid->setColumnStretch(1, 1);
    capture_grid->setColumnStretch(3, 1);
    capture_layout->addLayout(capture_grid);

    timing_fields_ = new QWidget(capture_panel);
    auto* timing = new QGridLayout(timing_fields_);
    timing->setContentsMargins(0, 0, 0, 0);
    timing->setHorizontalSpacing(12);
    timing->setVerticalSpacing(6);

    start_delay_s_ = makeDoubleSpin(timing_fields_, 0.0, 3600.0, 0.0, 3, 0.1);
    timing->addWidget(new QLabel("Start delay", timing_fields_), 0, 0);
    timing->addWidget(start_delay_s_, 0, 1);

    duration_field_ = new QWidget(timing_fields_);
    auto* duration_row = new QHBoxLayout(duration_field_);
    duration_row->setContentsMargins(0, 0, 0, 0);
    duration_s_ = makeDoubleSpin(duration_field_, 0.001, 86400.0, 10.0, 3, 0.5);
    duration_s_->setSuffix(" s");
    duration_row->addWidget(duration_s_);
    timing->addWidget(new QLabel("Duration", timing_fields_), 0, 2);
    timing->addWidget(duration_field_, 0, 3);

    periods_field_ = new QWidget(timing_fields_);
    auto* periods_row = new QHBoxLayout(periods_field_);
    periods_row->setContentsMargins(0, 0, 0, 0);
    period_count_ = new QSpinBox(periods_field_);
    period_count_->setRange(1, 100000);
    period_count_->setValue(10);
    period_count_->setSuffix(" periods");
    periods_row->addWidget(period_count_);
    timing->addWidget(new QLabel("Period count", timing_fields_), 1, 0);
    timing->addWidget(periods_field_, 1, 1);
    timing->setColumnStretch(1, 1);
    timing->setColumnStretch(3, 1);
    capture_layout->addWidget(timing_fields_);

    peak_fields_ = new QWidget(capture_panel);
    auto* peak_grid = new QGridLayout(peak_fields_);
    peak_grid->setContentsMargins(0, 0, 0, 0);
    peak_grid->setHorizontalSpacing(12);
    peak_grid->setVerticalSpacing(6);

    peak_signal_ = new QComboBox(peak_fields_);
    peak_signal_->addItems({"Velocity [rad/s]", "Current [A]", "Torque [N·m]", "Applied voltage [V]",
                            "Position [rad]", "Electrical power [W]", "Mechanical power [W]"});
    peak_threshold_ = makeDoubleSpin(peak_fields_, 0.0, 1.0e9, 1.0, 6, 0.1);
    peak_separation_s_ = makeDoubleSpin(peak_fields_, 0.0, 3600.0, 0.10, 3, 0.05);
    peak_target_count_ = new QSpinBox(peak_fields_);
    peak_target_count_->setRange(1, 100000);
    peak_target_count_->setValue(5);
    peak_absolute_ = new QCheckBox("Detect absolute magnitude peaks", peak_fields_);
    peak_absolute_->setChecked(true);

    peak_grid->addWidget(new QLabel("Peak signal", peak_fields_), 0, 0);
    peak_grid->addWidget(peak_signal_, 0, 1);
    peak_grid->addWidget(new QLabel("Threshold", peak_fields_), 0, 2);
    peak_grid->addWidget(peak_threshold_, 0, 3);
    peak_grid->addWidget(new QLabel("Min separation", peak_fields_), 1, 0);
    peak_grid->addWidget(peak_separation_s_, 1, 1);
    peak_grid->addWidget(new QLabel("Stop after", peak_fields_), 1, 2);
    peak_grid->addWidget(peak_target_count_, 1, 3);
    peak_grid->addWidget(peak_absolute_, 2, 0, 1, 4);
    peak_grid->setColumnStretch(1, 1);
    peak_grid->setColumnStretch(3, 1);
    capture_layout->addWidget(peak_fields_);

    auto* capture_note = new QLabel(
        "Peak detection always runs on the full telemetry stream. Save interval only reduces CSV file density. "
        "N-period mode uses the configured Sine, Square or Triangular waveform frequency.", capture_panel);
    capture_note->setObjectName("subtitle");
    capture_note->setWordWrap(true);
    capture_layout->addWidget(capture_note);

    panel_layout->addWidget(capture_panel);

    auto* status = new QFrame(panel);
    status->setObjectName("subPanel");
    auto* status_grid = new QGridLayout(status);
    status_grid->setContentsMargins(10, 8, 10, 8);
    state_label_ = new QLabel("IDLE", status);
    state_label_->setObjectName("metricValue");
    samples_label_ = new QLabel("0", status);
    samples_label_->setObjectName("metricValue");
    duration_label_ = new QLabel("0.000 s", status);
    duration_label_->setObjectName("metricValue");
    peaks_label_ = new QLabel("0", status);
    peaks_label_->setObjectName("metricValue");
    file_label_ = new QLabel("--", status);
    file_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    file_label_->setWordWrap(true);
    error_label_ = new QLabel("", status);
    error_label_->setObjectName("errorText");
    error_label_->setWordWrap(true);

    status_grid->addWidget(new QLabel("STATE", status), 0, 0);
    status_grid->addWidget(state_label_, 0, 1);
    status_grid->addWidget(new QLabel("WRITTEN", status), 0, 2);
    status_grid->addWidget(samples_label_, 0, 3);
    status_grid->addWidget(new QLabel("PEAKS", status), 0, 4);
    status_grid->addWidget(peaks_label_, 0, 5);
    status_grid->addWidget(new QLabel("ELAPSED", status), 0, 6);
    status_grid->addWidget(duration_label_, 0, 7);
    status_grid->addWidget(new QLabel("CURRENT FILE", status), 1, 0);
    status_grid->addWidget(file_label_, 1, 1, 1, 7);
    status_grid->addWidget(error_label_, 2, 0, 1, 8);
    status_grid->setColumnStretch(1, 1);
    status_grid->setColumnStretch(7, 1);
    panel_layout->addWidget(status);

    auto* buttons = new QHBoxLayout();
    start_btn_ = new QPushButton("Start recording", panel);
    stop_btn_ = new QPushButton("Stop recording", panel);
    stop_btn_->setObjectName("secondary");
    stop_btn_->setEnabled(false);
    auto* close_btn = new QPushButton("Close", panel);
    close_btn->setObjectName("secondary");
    connect(start_btn_, &QPushButton::clicked, this, &RecordingDialog::startRecordingRequested);
    connect(stop_btn_, &QPushButton::clicked, this, &RecordingDialog::stopRecordingRequested);
    connect(close_btn, &QPushButton::clicked, this, &QDialog::hide);
    buttons->addWidget(start_btn_);
    buttons->addWidget(stop_btn_);
    buttons->addStretch(1);
    buttons->addWidget(close_btn);
    panel_layout->addLayout(buttons);

    root->addWidget(panel, 1);

    connect(capture_mode_, &QComboBox::currentIndexChanged,
            this, &RecordingDialog::updateCaptureModeUi_);
    updateCaptureModeUi_();
}

QString RecordingDialog::directory() const { return directory_edit_->text().trimmed(); }
QString RecordingDialog::prefix() const { return prefix_edit_->text().trimmed(); }
QString RecordingDialog::notes() const { return notes_edit_->toPlainText(); }
bool RecordingDialog::writeMetadata() const noexcept { return metadata_check_->isChecked(); }

models::RecordingConfig RecordingDialog::recordingConfig() const {
    models::RecordingConfig c;
    c.mode = static_cast<models::RecordingMode>(capture_mode_->currentIndex());
    c.sample_interval_ms = sample_interval_ms_->value();
    c.start_delay_s = start_delay_s_->value();
    c.duration_s = duration_s_->value();
    c.period_count = period_count_->value();
    c.peak_signal = static_cast<models::PeakSignal>(peak_signal_->currentIndex());
    c.peak_threshold = peak_threshold_->value();
    c.min_peak_distance_s = peak_separation_s_->value();
    c.peak_target_count = peak_target_count_->value();
    c.absolute_peaks = peak_absolute_->isChecked();
    return c;
}

void RecordingDialog::setRecordingState(bool active) {
    if (!active) state_label_->setText("IDLE");
    start_btn_->setEnabled(!active);
    stop_btn_->setEnabled(active);
    capture_mode_->setEnabled(!active);
    sample_interval_ms_->setEnabled(!active);
}

void RecordingDialog::setStats(qint64 samples, double seconds, const QString& file_path) {
    samples_label_->setText(QString::number(samples));
    duration_label_->setText(QString::number(seconds, 'f', 3) + " s");
    file_label_->setText(file_path.isEmpty() ? "--" : file_path);
}

void RecordingDialog::setPeakCount(int peaks) {
    peaks_label_->setText(QString::number(peaks));
}

void RecordingDialog::setCaptureStatus(const QString& status) {
    state_label_->setText(status);
}

void RecordingDialog::setWaveformInfo(const models::WaveformConfig& waveform) {
    waveform_info_->setText(QString("Waveform: %1   ·   f = %2 Hz   ·   duration = %3 s")
                                .arg(waveform.name())
                                .arg(waveform.frequency_hz, 0, 'g', 6)
                                .arg(waveform.duration_s, 0, 'g', 6));
}

void RecordingDialog::showError(const QString& message) {
    error_label_->setText(message);
}

void RecordingDialog::browseDirectory_() {
    const QString selected = QFileDialog::getExistingDirectory(this, "Select data directory", directory());
    if (!selected.isEmpty()) directory_edit_->setText(selected);
}

void RecordingDialog::updateCaptureModeUi_() {
    const auto mode = static_cast<models::RecordingMode>(capture_mode_->currentIndex());

    const bool timing = mode == models::RecordingMode::TimeWindow || mode == models::RecordingMode::PeriodCount;
    timing_fields_->setVisible(timing);
    duration_field_->setVisible(mode == models::RecordingMode::TimeWindow);
    periods_field_->setVisible(mode == models::RecordingMode::PeriodCount);
    peak_fields_->setVisible(mode == models::RecordingMode::PeakCount);
}

}  // namespace qube_servo2::gui::ui::pages
