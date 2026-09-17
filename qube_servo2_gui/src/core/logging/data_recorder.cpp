#include "qube_servo2_gui/core/logging/data_recorder.hpp"

#include <QDateTime>
#include <QDir>

#include <algorithm>
#include <cmath>

namespace qube_servo2::gui::logging {

namespace {
constexpr qsizetype kFlushBufferChars = 64 * 1024;
constexpr int kStatsEveryWrittenSamples = 200;
}

DataRecorder::DataRecorder(QObject* parent)
    : QObject(parent) {
    csv_buffer_.reserve(kFlushBufferChars + 4096);
}

DataRecorder::~DataRecorder() {
    // Destructors must not emit Qt signals. Just release OS resources.
    flushBuffer_();
    closeFiles_();
}

QString DataRecorder::makeBaseName_(const QString& prefix) const {
    const QString safe_prefix = prefix.trimmed().isEmpty() ? QStringLiteral("qube_run") : prefix.trimmed();
    return safe_prefix + "_" + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
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
    effective_duration_s_ = 0.0;
    last_written_capture_s_ = -1.0;
    last_stats_emit_s_ = -1.0e30;
    last_status_emit_s_ = -1.0e30;
    have_peak_prev2_ = false;
    have_peak_prev1_ = false;
    last_peak_capture_s_ = -1.0e30;
    csv_buffer_.clear();
    last_error_.clear();

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
    stream_ << "ros_time_s,capture_time_s,command_voltage_v,position_rad,position_deg,velocity_rad_s,velocity_rpm,"
               "current_a,applied_voltage_v,back_emf_v,motor_torque_nm,electrical_power_w,mechanical_power_w\n";
    stream_.flush();

    if (!prepareCapture_(waveform_, config_, dir, base_name)) {
        closeFiles_();
        current_path_.clear();
        return false;
    }

    elapsed_.restart();

    if (write_metadata) {
        writeMetadata_(dir.filePath(base_name + ".yaml"), notes, waveform_, config_);
    }

    updateCaptureStatus_(0.0);
    emit recordingStateChanged(true);
    emit peakCountChanged(0);
    emit statsChanged(samples_, 0.0, current_path_);
    return true;
}

bool DataRecorder::prepareCapture_(const models::WaveformConfig& waveform,
                                   const models::RecordingConfig& config,
                                   const QDir& directory,
                                   const QString& base_name) {
    switch (config.mode) {
    case models::RecordingMode::TimeWindow:
        effective_duration_s_ = std::max(0.001, config.duration_s);
        break;

    case models::RecordingMode::PeriodCount: {
        const double f_hz = periodicFrequency_(waveform);
        if (f_hz <= 0.0 || !std::isfinite(f_hz)) {
            last_error_ = "N-period recording requires a periodic waveform (Sine, Square or Triangular) with frequency > 0 Hz.";
            emit errorOccurred(last_error_);
            return false;
        }
        effective_duration_s_ = static_cast<double>(std::max(1, config.period_count)) / f_hz;
        break;
    }

    case models::RecordingMode::PeakCount:
        peak_path_ = directory.filePath(base_name + "_peaks.csv");
        peak_file_.setFileName(peak_path_);
        if (!peak_file_.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
            last_error_ = QString("Unable to open peak file: %1").arg(peak_path_);
            emit errorOccurred(last_error_);
            return false;
        }
        peak_stream_.setDevice(&peak_file_);
        peak_stream_ << "peak_index,ros_time_s,capture_time_s,signal,value\n";
        peak_stream_.flush();
        break;

    case models::RecordingMode::Manual:
    default:
        break;
    }

    return true;
}

void DataRecorder::stop() {
    if (!file_.isOpen()) return;

    flushBuffer_();
    closeFiles_();
    capture_status_ = "IDLE";

    emit captureStatusChanged(capture_status_);
    emit statsChanged(samples_, elapsedSeconds(), current_path_);
    emit recordingStateChanged(false);
}

void DataRecorder::append(const models::TelemetrySample& s) {
    if (!file_.isOpen() || !s.valid) return;

    const double wall_elapsed_s = elapsedSeconds();
    const double delay_s = std::max(0.0, config_.start_delay_s);
    if (wall_elapsed_s < delay_s) {
        updateCaptureStatus_(wall_elapsed_s);
        return;
    }

    const double capture_elapsed_s = wall_elapsed_s - delay_s;

    // Peak detection runs on every received telemetry sample, even when CSV
    // decimation is enabled, so peak timing is not degraded by file sampling.
    bool reached_peak_target = false;
    if (config_.mode == models::RecordingMode::PeakCount) {
        reached_peak_target = processPeak_(s, capture_elapsed_s);
    }

    if (config_.mode == models::RecordingMode::TimeWindow ||
        config_.mode == models::RecordingMode::PeriodCount) {
        if (capture_elapsed_s > effective_duration_s_) {
            stop();
            return;
        }
    }

    const double interval_s = std::max(0, config_.sample_interval_ms) / 1000.0;
    const bool due = interval_s <= 0.0 ||
                     last_written_capture_s_ < 0.0 ||
                     (capture_elapsed_s - last_written_capture_s_) >= interval_s;

    if (due) {
        csv_buffer_ += QString::number(s.ros_time_s, 'f', 9) + ',';
        csv_buffer_ += QString::number(capture_elapsed_s, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.command_voltage, 'f', 6) + ',';
        csv_buffer_ += QString::number(s.position_rad, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.position_deg(), 'f', 6) + ',';
        csv_buffer_ += QString::number(s.velocity_rad_s, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.velocity_rpm(), 'f', 6) + ',';
        csv_buffer_ += QString::number(s.current_a, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.applied_voltage_v, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.back_emf_v, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.motor_torque_nm, 'f', 9) + ',';
        csv_buffer_ += QString::number(s.electrical_power_w(), 'f', 9) + ',';
        csv_buffer_ += QString::number(s.mechanical_power_w(), 'f', 9) + '\n';

        last_written_capture_s_ = capture_elapsed_s;
        ++samples_;

        if (csv_buffer_.size() >= kFlushBufferChars) flushBuffer_();
        if ((samples_ % kStatsEveryWrittenSamples) == 0 ||
            (wall_elapsed_s - last_stats_emit_s_) >= 0.5) {
            last_stats_emit_s_ = wall_elapsed_s;
            emit statsChanged(samples_, wall_elapsed_s, current_path_);
        }
    }

    updateCaptureStatus_(wall_elapsed_s);

    if (reached_peak_target) {
        stop();
    }
}

void DataRecorder::flushBuffer_() {
    if (!file_.isOpen() || csv_buffer_.isEmpty()) return;
    stream_ << csv_buffer_;
    csv_buffer_.clear();
    stream_.flush();
    file_.flush();
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

void DataRecorder::updateCaptureStatus_(double wall_elapsed_s) {
    // UI progress does not need telemetry-rate updates. Throttling this path
    // prevents thousands of label / layout updates per second during raw acquisition.
    if ((wall_elapsed_s - last_status_emit_s_) < 0.20 && wall_elapsed_s > 0.0) return;
    last_status_emit_s_ = wall_elapsed_s;

    const double delay_s = std::max(0.0, config_.start_delay_s);
    QString next;

    if (wall_elapsed_s < delay_s) {
        next = QString("ARMED · starts in %1 s").arg(delay_s - wall_elapsed_s, 0, 'f', 2);
    } else {
        const double capture_s = wall_elapsed_s - delay_s;
        switch (config_.mode) {
        case models::RecordingMode::TimeWindow:
            next = QString("CAPTURING · %1 / %2 s")
                       .arg(std::min(capture_s, effective_duration_s_), 0, 'f', 2)
                       .arg(effective_duration_s_, 0, 'f', 2);
            break;
        case models::RecordingMode::PeriodCount: {
            const double f_hz = periodicFrequency_(waveform_);
            const double periods = capture_s * std::max(0.0, f_hz);
            next = QString("CAPTURING · %1 / %2 periods")
                       .arg(periods, 0, 'f', 2)
                       .arg(std::max(1, config_.period_count));
            break;
        }
        case models::RecordingMode::PeakCount:
            next = QString("PEAK WATCH · %1 / %2")
                       .arg(peaks_)
                       .arg(std::max(1, config_.peak_target_count));
            break;
        case models::RecordingMode::Manual:
        default:
            next = "RECORDING";
            break;
        }
    }

    if (next != capture_status_) {
        capture_status_ = next;
        emit captureStatusChanged(capture_status_);
    }
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

bool DataRecorder::processPeak_(const models::TelemetrySample& sample, double capture_elapsed_s) {
    double value = selectedPeakSignal_(sample);
    if (config_.absolute_peaks) value = std::abs(value);

    bool reached_target = false;
    if (have_peak_prev2_ && have_peak_prev1_) {
        const double threshold = std::abs(config_.peak_threshold);
        const bool local_max = peak_prev1_value_ > peak_prev2_value_ && peak_prev1_value_ >= value;
        const bool above_threshold = peak_prev1_value_ >= threshold;
        const bool separated = (peak_prev1_capture_s_ - last_peak_capture_s_) >=
                               std::max(0.0, config_.min_peak_distance_s);

        if (local_max && above_threshold && separated) {
            last_peak_capture_s_ = peak_prev1_capture_s_;
            ++peaks_;
            writePeak_(peak_prev1_, peak_prev1_capture_s_, selectedPeakSignal_(peak_prev1_));
            emit peakCountChanged(peaks_);
            reached_target = peaks_ >= std::max(1, config_.peak_target_count);
        }
    }

    peak_prev2_ = peak_prev1_;
    peak_prev2_value_ = peak_prev1_value_;
    peak_prev2_capture_s_ = peak_prev1_capture_s_;
    have_peak_prev2_ = have_peak_prev1_;

    peak_prev1_ = sample;
    peak_prev1_value_ = value;
    peak_prev1_capture_s_ = capture_elapsed_s;
    have_peak_prev1_ = true;

    return reached_target;
}

void DataRecorder::writePeak_(const models::TelemetrySample& sample,
                              double capture_elapsed_s,
                              double peak_value) {
    if (!peak_file_.isOpen()) return;
    peak_stream_ << peaks_ << ','
                 << QString::number(sample.ros_time_s, 'f', 9) << ','
                 << QString::number(capture_elapsed_s, 'f', 9) << ','
                 << config_.peakSignalName() << ','
                 << QString::number(peak_value, 'g', 12) << '\n';
    peak_stream_.flush();
}

double DataRecorder::periodicFrequency_(const models::WaveformConfig& waveform) const noexcept {
    switch (waveform.type) {
    case models::WaveformType::Square:
    case models::WaveformType::Triangle:
    case models::WaveformType::Sine:
        return waveform.frequency_hz;
    default:
        return 0.0;
    }
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

bool DataRecorder::writeMetadata_(const QString& metadata_path,
                                  const QString& notes,
                                  const models::WaveformConfig& waveform,
                                  const models::RecordingConfig& config) {
    QFile metadata(metadata_path);
    if (!metadata.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate)) {
        emit errorOccurred(QString("Unable to write metadata: %1").arg(metadata_path));
        return false;
    }

    QTextStream out(&metadata);
    out << "experiment:\n";
    out << "  created_at: \"" << QDateTime::currentDateTime().toString(Qt::ISODate) << "\"\n";
    out << "  notes: \"" << QString(notes).replace('"', '\'') << "\"\n";
    out << "recording:\n";
    out << "  mode: \"" << config.modeName() << "\"\n";
    out << "  sample_interval_ms: " << config.sample_interval_ms << "\n";
    out << "  start_delay_s: " << config.start_delay_s << "\n";
    out << "  duration_s: " << config.duration_s << "\n";
    out << "  period_count: " << config.period_count << "\n";
    out << "  peak_signal: \"" << config.peakSignalName() << "\"\n";
    out << "  peak_threshold: " << config.peak_threshold << "\n";
    out << "  min_peak_distance_s: " << config.min_peak_distance_s << "\n";
    out << "  peak_target_count: " << config.peak_target_count << "\n";
    out << "  absolute_peaks: " << (config.absolute_peaks ? "true" : "false") << "\n";
    if (config.mode == models::RecordingMode::PeriodCount) {
        out << "  effective_duration_s: " << effective_duration_s_ << "\n";
    }
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
