#include "qube_servo2_gui/core/logging/data_recorder.hpp"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::logging {

DataRecorder::DataRecorder(QObject* parent) : QObject(parent) {}

DataRecorder::~DataRecorder() {
    if (file_.isOpen()) file_.close();
    if (peak_file_.isOpen()) peak_file_.close();
}

QString DataRecorder::makeBaseName_(const QString& prefix) const {
    const QString safe = prefix.trimmed().isEmpty() ? QStringLiteral("qube_run") : prefix.trimmed();
    return safe + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
}

bool DataRecorder::start(const QString& directory,
                         const QString& prefix,
                         const QString& notes,
                         const models::WaveformConfig& waveform,
                         const models::RecordingConfig& config,
                         bool write_metadata) {
    stop();

    QDir dir(directory);
    if (!dir.exists() && !dir.mkpath(".")) {
        last_error_ = QString("Unable to create directory: %1").arg(directory);
        emit errorOccurred(last_error_);
        return false;
    }

    config_ = config;
    waveform_ = waveform;
    samples_ = 0;
    peaks_ = 0;
    last_written_capture_s_ = -1.0;
    last_stats_emit_s_ = -1.0e30;
    last_status_emit_s_ = -1.0e30;
    have_peak_prev2_ = false;
    have_peak_prev1_ = false;
    last_peak_capture_s_ = -1.0e30;
    last_error_.clear();

    if (config_.mode == models::RecordingMode::PeriodCount) {
        const double f = periodicFrequency_(waveform_);
        if (f <= 0.0) {
            last_error_ = "N-period recording requires Sine, Square or Triangular excitation.";
            emit errorOccurred(last_error_);
            return false;
        }
        effective_duration_s_ = static_cast<double>(std::max(1, config_.period_count)) / f;
    } else if (config_.mode == models::RecordingMode::TimeWindow) {
        effective_duration_s_ = std::max(0.001, config_.duration_s);
    } else {
        effective_duration_s_ = 0.0;
    }

    const QString base = makeBaseName_(prefix);
    current_path_ = dir.filePath(base + ".csv");
    file_.setFileName(current_path_);
    if (!file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        last_error_ = QString("Unable to open file: %1").arg(current_path_);
        emit errorOccurred(last_error_);
        return false;
    }
    stream_.setDevice(&file_);
    stream_ << "ros_time_s,capture_time_s,command_voltage_v,position_rad,position_deg,velocity_rad_s,velocity_rpm,"
               "current_a,applied_voltage_v,back_emf_v,motor_torque_nm,electrical_power_w,mechanical_power_w\n";
    stream_.flush();

    if (config_.mode == models::RecordingMode::PeakCount) {
        peak_path_ = dir.filePath(base + "_peaks.csv");
        peak_file_.setFileName(peak_path_);
        if (!peak_file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            last_error_ = QString("Unable to open peak file: %1").arg(peak_path_);
            emit errorOccurred(last_error_);
            closeFiles_();
            return false;
        }
        peak_stream_.setDevice(&peak_file_);
        peak_stream_ << "peak_index,ros_time_s,capture_time_s,signal,value\n";
        peak_stream_.flush();
    }

    if (write_metadata) {
        writeMetadata_(dir.filePath(base + ".yaml"), notes, waveform_, config_);
    }

    elapsed_.restart();
    capture_status_ = "RECORDING";
    emit recordingStateChanged(true);
    emit peakCountChanged(0);
    emit captureStatusChanged(capture_status_);
    emit statsChanged(0, 0.0, current_path_);
    return true;
}

void DataRecorder::stop() {
    if (!file_.isOpen()) return;
    stream_.flush();
    file_.flush();
    closeFiles_();
    capture_status_ = "IDLE";
    emit captureStatusChanged(capture_status_);
    emit statsChanged(samples_, elapsedSeconds(), current_path_);
    emit recordingStateChanged(false);
}

void DataRecorder::append(const models::TelemetrySample& s) {
    if (!file_.isOpen() || !s.valid) return;

    const double wall_s = elapsedSeconds();
    const double delay_s = std::max(0.0, config_.start_delay_s);
    if (wall_s < delay_s) {
        updateCaptureStatus_(wall_s);
        return;
    }
    const double capture_s = wall_s - delay_s;

    bool reached_peak_target = false;
    if (config_.mode == models::RecordingMode::PeakCount) {
        reached_peak_target = processPeak_(s, capture_s);
    }

    if ((config_.mode == models::RecordingMode::TimeWindow ||
         config_.mode == models::RecordingMode::PeriodCount) &&
        capture_s > effective_duration_s_) {
        stop();
        return;
    }

    const double interval_s = std::max(0, config_.sample_interval_ms) / 1000.0;
    const bool due = interval_s <= 0.0 || last_written_capture_s_ < 0.0 ||
                     (capture_s - last_written_capture_s_) >= interval_s;

    if (due) {
        stream_ << QString::number(s.ros_time_s, 'f', 9) << ','
                << QString::number(capture_s, 'f', 9) << ','
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
        last_written_capture_s_ = capture_s;
        ++samples_;
        if ((samples_ % 100) == 0) stream_.flush();
        if ((wall_s - last_stats_emit_s_) >= 0.4) {
            last_stats_emit_s_ = wall_s;
            emit statsChanged(samples_, wall_s, current_path_);
        }
    }

    updateCaptureStatus_(wall_s);
    if (reached_peak_target) stop();
}

bool DataRecorder::isRecording() const noexcept { return file_.isOpen(); }
qint64 DataRecorder::sampleCount() const noexcept { return samples_; }
int DataRecorder::peakCount() const noexcept { return peaks_; }
double DataRecorder::elapsedSeconds() const noexcept {
    return elapsed_.isValid() ? static_cast<double>(elapsed_.elapsed()) / 1000.0 : 0.0;
}
QString DataRecorder::currentFilePath() const { return current_path_; }
QString DataRecorder::lastError() const { return last_error_; }
QString DataRecorder::captureStatus() const { return capture_status_; }

bool DataRecorder::writeMetadata_(const QString& path,
                                  const QString& notes,
                                  const models::WaveformConfig& waveform,
                                  const models::RecordingConfig& config) {
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) return false;
    QTextStream o(&f);
    o << "generated_at: \"" << QDateTime::currentDateTime().toString(Qt::ISODate) << "\"\n";
    o << "notes: \"" << QString(notes).replace('"', "'") << "\"\n";
    o << "waveform:\n";
    o << "  type: \"" << waveform.name() << "\"\n";
    o << "  amplitude: " << waveform.amplitude << "\n";
    o << "  offset: " << waveform.offset << "\n";
    o << "  frequency_hz: " << waveform.frequency_hz << "\n";
    o << "  duration_s: " << waveform.duration_s << "\n";
    o << "  publish_rate_hz: " << waveform.publish_rate_hz << "\n";
    o << "recording:\n";
    o << "  mode: \"" << config.modeName() << "\"\n";
    o << "  sample_interval_ms: " << config.sample_interval_ms << "\n";
    o << "  start_delay_s: " << config.start_delay_s << "\n";
    o << "  duration_s: " << config.duration_s << "\n";
    o << "  period_count: " << config.period_count << "\n";
    o << "  peak_signal: \"" << config.peakSignalName() << "\"\n";
    o << "  peak_threshold: " << config.peak_threshold << "\n";
    o << "  peak_target_count: " << config.peak_target_count << "\n";
    return true;
}

double DataRecorder::periodicFrequency_(const models::WaveformConfig& w) const noexcept {
    if (w.type == models::WaveformType::Sine ||
        w.type == models::WaveformType::Square ||
        w.type == models::WaveformType::Triangle) {
        return std::max(0.0, w.frequency_hz);
    }
    return 0.0;
}

double DataRecorder::selectedPeakSignal_(const models::TelemetrySample& s) const noexcept {
    switch (config_.peak_signal) {
    case models::PeakSignal::Current: return s.current_a;
    case models::PeakSignal::Torque: return s.motor_torque_nm;
    case models::PeakSignal::AppliedVoltage: return s.applied_voltage_v;
    case models::PeakSignal::Position: return s.position_rad;
    case models::PeakSignal::ElectricalPower: return s.electrical_power_w();
    case models::PeakSignal::MechanicalPower: return s.mechanical_power_w();
    case models::PeakSignal::Velocity:
    default: return s.velocity_rad_s;
    }
}

bool DataRecorder::processPeak_(const models::TelemetrySample& sample, double capture_s) {
    double value = selectedPeakSignal_(sample);
    if (config_.absolute_peaks) value = std::abs(value);
    bool reached = false;

    if (have_peak_prev2_ && have_peak_prev1_) {
        const bool local_max = peak_prev1_value_ > peak_prev2_value_ && peak_prev1_value_ >= value;
        const bool threshold = peak_prev1_value_ >= std::abs(config_.peak_threshold);
        const bool separated = (peak_prev1_capture_s_ - last_peak_capture_s_) >=
                               std::max(0.0, config_.min_peak_distance_s);
        if (local_max && threshold && separated) {
            last_peak_capture_s_ = peak_prev1_capture_s_;
            ++peaks_;
            writePeak_(peak_prev1_, peak_prev1_capture_s_, selectedPeakSignal_(peak_prev1_));
            emit peakCountChanged(peaks_);
            reached = peaks_ >= std::max(1, config_.peak_target_count);
        }
    }

    peak_prev2_ = peak_prev1_;
    peak_prev2_value_ = peak_prev1_value_;
    peak_prev2_capture_s_ = peak_prev1_capture_s_;
    have_peak_prev2_ = have_peak_prev1_;
    peak_prev1_ = sample;
    peak_prev1_value_ = value;
    peak_prev1_capture_s_ = capture_s;
    have_peak_prev1_ = true;
    return reached;
}

void DataRecorder::writePeak_(const models::TelemetrySample& sample,
                              double capture_s,
                              double value) {
    if (!peak_file_.isOpen()) return;
    peak_stream_ << peaks_ << ','
                 << QString::number(sample.ros_time_s, 'f', 9) << ','
                 << QString::number(capture_s, 'f', 9) << ','
                 << config_.peakSignalName() << ','
                 << QString::number(value, 'f', 9) << '\n';
    peak_stream_.flush();
}

void DataRecorder::updateCaptureStatus_(double wall_s) {
    if ((wall_s - last_status_emit_s_) < 0.2 && wall_s > 0.0) return;
    last_status_emit_s_ = wall_s;
    const double delay = std::max(0.0, config_.start_delay_s);
    QString status;
    if (wall_s < delay) {
        status = QString("ARMED · starts in %1 s").arg(delay - wall_s, 0, 'f', 2);
    } else if (config_.mode == models::RecordingMode::TimeWindow ||
               config_.mode == models::RecordingMode::PeriodCount) {
        status = QString("CAPTURING · %1 / %2 s")
                     .arg(std::min(wall_s - delay, effective_duration_s_), 0, 'f', 2)
                     .arg(effective_duration_s_, 0, 'f', 2);
    } else if (config_.mode == models::RecordingMode::PeakCount) {
        status = QString("PEAK WATCH · %1 / %2").arg(peaks_).arg(std::max(1, config_.peak_target_count));
    } else {
        status = "RECORDING";
    }
    if (status != capture_status_) {
        capture_status_ = status;
        emit captureStatusChanged(capture_status_);
    }
}

void DataRecorder::closeFiles_() {
    if (file_.isOpen()) {
        stream_.flush();
        file_.flush();
        file_.close();
    }
    stream_.setDevice(nullptr);
    if (peak_file_.isOpen()) {
        peak_stream_.flush();
        peak_file_.flush();
        peak_file_.close();
    }
    peak_stream_.setDevice(nullptr);
}

}  // namespace qube_servo2::gui::logging
