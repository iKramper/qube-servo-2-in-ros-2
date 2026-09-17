#include "qube_servo2_gui/ui/pages/recording_dialog.hpp"

#include <QCheckBox>
#include <QDir>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStandardPaths>
#include <QTextEdit>
#include <QVBoxLayout>

namespace qube_servo2::gui::ui::pages {

RecordingDialog::RecordingDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("QUBE-Servo 2 / Data Acquisition");
    setModal(false);
    setMinimumSize(720, 470);
    resize(760, 500);

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
    panel_layout->setSpacing(8);

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
    notes_edit_->setMaximumHeight(90);
    form->addRow("Notes", notes_edit_);

    metadata_check_ = new QCheckBox("Write companion YAML metadata file", panel);
    metadata_check_->setChecked(true);
    form->addRow("", metadata_check_);
    panel_layout->addLayout(form);

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
    file_label_ = new QLabel("--", status);
    file_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    file_label_->setWordWrap(true);
    error_label_ = new QLabel("", status);
    error_label_->setObjectName("errorText");
    error_label_->setWordWrap(true);

    status_grid->addWidget(new QLabel("STATE", status), 0, 0);
    status_grid->addWidget(state_label_, 0, 1);
    status_grid->addWidget(new QLabel("SAMPLES", status), 0, 2);
    status_grid->addWidget(samples_label_, 0, 3);
    status_grid->addWidget(new QLabel("DURATION", status), 0, 4);
    status_grid->addWidget(duration_label_, 0, 5);
    status_grid->addWidget(new QLabel("CURRENT FILE", status), 1, 0);
    status_grid->addWidget(file_label_, 1, 1, 1, 5);
    status_grid->addWidget(error_label_, 2, 0, 1, 6);
    status_grid->setColumnStretch(1, 1);
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
}

QString RecordingDialog::directory() const { return directory_edit_->text().trimmed(); }
QString RecordingDialog::prefix() const { return prefix_edit_->text().trimmed(); }
QString RecordingDialog::notes() const { return notes_edit_->toPlainText(); }
bool RecordingDialog::writeMetadata() const noexcept { return metadata_check_->isChecked(); }

void RecordingDialog::setRecordingState(bool active) {
    state_label_->setText(active ? "RECORDING" : "IDLE");
    start_btn_->setEnabled(!active);
    stop_btn_->setEnabled(active);
}

void RecordingDialog::setStats(qint64 samples, double seconds, const QString& file_path) {
    samples_label_->setText(QString::number(samples));
    duration_label_->setText(QString::number(seconds, 'f', 3) + " s");
    file_label_->setText(file_path.isEmpty() ? "--" : file_path);
}

void RecordingDialog::showError(const QString& message) {
    error_label_->setText(message);
}

void RecordingDialog::browseDirectory_() {
    const QString selected = QFileDialog::getExistingDirectory(this, "Select data directory", directory());
    if (!selected.isEmpty()) directory_edit_->setText(selected);
}

}  // namespace qube_servo2::gui::ui::pages
