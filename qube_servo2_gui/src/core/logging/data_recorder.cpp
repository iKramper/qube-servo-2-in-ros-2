#include "qube_servo2_gui/core/logging/data_recorder.hpp"

#include <QDateTime>
#include <QDir>

namespace qube_servo2::gui::logging {

DataRecorder::DataRecorder(QObject* parent)
    : QObject(parent) {}

DataRecorder::~DataRecorder() {
    // Do not emit Qt signals from the destructor; just release the file handle.
    if (file_.isOpen()) {
        stream_.flush();
        file_.flush();
        file_.close();
        stream_.setDevice(nullptr);
    }
}

QString DataRecorder::makeBaseName_(const QString& prefix) const {
    const QString safe_prefix = prefix.trimmed().isEmpty() ? QStringLiteral("qube_run") : prefix.trimmed();
    return safe_prefix + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
}

bool DataRecorder::start(const QString& directory,
                         const QString& prefix,
                         const QString& notes,
                         const models::WaveformConfig& waveform,
                         bool write_metadata) {
    stop();

    QDir dir(directory);
    if (!dir.exists() && !dir.mkpath(".")) {
        last_error_ = QString("Unable to create directory: %1").arg(directory);
        emit errorOccurred(last_error_);
        return false;
    }

    const QString base_name = makeBaseName_(prefix);
    current_path_ = dir.filePath(base_name + ".csv");
    file_.setFileName(current_path_);

    if (!file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        last_error_ = QString("Unable to open file: %1").arg(current_path_);
        emit errorOccurred(last_error_);
        current_path_.clear();
        return false;
    }

    stream_.setDevice(&file_);
    stream_ << "ros_time_s,command_voltage_v,position_rad,position_deg,velocity_rad_s,velocity_rpm,"
               "current_a,applied_voltage_v,back_emf_v,motor_torque_nm,electrical_power_w,mechanical_power_w\n";
    stream_.flush();

    samples_ = 0;
    elapsed_.restart();
    last_error_.clear();

    if (write_metadata) {
        writeMetadata_(dir.filePath(base_name + ".yaml"), notes, waveform);
    }

    emit recordingStateChanged(true);
    emit statsChanged(samples_, 0.0, current_path_);
    return true;
}

void DataRecorder::stop() {
    if (!file_.isOpen()) {
        return;
    }

    stream_.flush();
    file_.flush();
    file_.close();
    stream_.setDevice(nullptr);

    emit statsChanged(samples_, elapsedSeconds(), current_path_);
    emit recordingStateChanged(false);
}

void DataRecorder::append(const models::TelemetrySample& s) {
    if (!file_.isOpen() || !s.valid) {
        return;
    }

    stream_ << QString::number(s.ros_time_s, 'f', 9) << ','
            << QString::number(s.command_voltage, 'f', 6) << ','
            << QString::number(s.position_rad, 'f', 9) << ','
            << QString::number(s.position_deg(), 'f', 6) << ','
            << QString::number(s.velocity_rad_s, 'f', 9) << ','
            << QString::number(s.velocity_rpm(), 'f', 6) << ','
            << QString::number(s.current_a, 'f', 9) << ','
            << QString::number(s.applied_voltage_v, 'f', 9) << ','
            << QString::number(s.back_emf_v, 'f', 9) << ','
            << QString::number(s.motor_torque_nm, 'f', 9) << ','
            << QString::number(s.electrical_power_w(), 'f', 9) << ','
            << QString::number(s.mechanical_power_w(), 'f', 9) << '\n';

    ++samples_;
    if ((samples_ % 100) == 0) {
        stream_.flush();
        emit statsChanged(samples_, elapsedSeconds(), current_path_);
    }
}

bool DataRecorder::isRecording() const noexcept {
    return file_.isOpen();
}

qint64 DataRecorder::sampleCount() const noexcept {
    return samples_;
}

double DataRecorder::elapsedSeconds() const noexcept {
    return elapsed_.isValid() ? static_cast<double>(elapsed_.elapsed()) / 1000.0 : 0.0;
}

QString DataRecorder::currentFilePath() const {
    return current_path_;
}

QString DataRecorder::lastError() const {
    return last_error_;
}

bool DataRecorder::writeMetadata_(const QString& metadata_path,
                                  const QString& notes,
                                  const models::WaveformConfig& waveform) {
    QFile metadata(metadata_path);
    if (!metadata.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        emit errorOccurred(QString("Unable to write metadata: %1").arg(metadata_path));
        return false;
    }

    QTextStream out(&metadata);
    out << "experiment:\n";
    out << "  created_at: \"" << QDateTime::currentDateTime().toString(Qt::ISODate) << "\"\n";
    out << "  notes: \"" << QString(notes).replace('"', '\'') << "\"\n";
    out << "waveform:\n";
    out << "  type: \"" << waveform.name() << "\"\n";
    out << "  amplitude_v: " << waveform.amplitude << "\n";
    out << "  offset_v: " << waveform.offset << "\n";
    out << "  frequency_hz: " << waveform.frequency_hz << "\n";
    out << "  chirp_end_frequency_hz: " << waveform.chirp_end_frequency_hz << "\n";
    out << "  duty_cycle_percent: " << waveform.duty_cycle_percent << "\n";
    out << "  phase_deg: " << waveform.phase_deg << "\n";
    out << "  pulse_width_s: " << waveform.pulse_width_s << "\n";
    out << "  delay_s: " << waveform.delay_s << "\n";
    out << "  duration_s: " << waveform.duration_s << "\n";
    out << "  initial_value_v: " << waveform.initial_value << "\n";
    out << "  final_value_v: " << waveform.final_value << "\n";
    out << "  rise_time_s: " << waveform.rise_time_s << "\n";
    out << "  tau_s: " << waveform.tau_s << "\n";
    out << "  repeat: " << (waveform.repeat ? "true" : "false") << "\n";
    out << "  publish_rate_hz: " << waveform.publish_rate_hz << "\n";
    return true;
}

}  // namespace qube_servo2::gui::logging
