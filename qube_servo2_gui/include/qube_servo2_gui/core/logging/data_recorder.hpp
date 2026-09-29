#pragma once

#include "qube_servo2_gui/core/models/recording_config.hpp"
#include "qube_servo2_gui/core/models/telemetry_sample.hpp"
#include "qube_servo2_gui/core/models/waveform_config.hpp"

#include <QObject>
#include <QElapsedTimer>
#include <QFile>
#include <QString>
#include <QTextStream>

class QDir;

namespace qube_servo2::gui::logging {

class DataRecorder final : public QObject {
    Q_OBJECT

public:
    explicit DataRecorder(QObject* parent = nullptr);
    ~DataRecorder() override;

    bool start(const QString& directory,
               const QString& prefix,
               const QString& notes,
               const models::WaveformConfig& waveform,
               const models::RecordingConfig& config,
               bool write_metadata);

    void stop();
    void append(const models::TelemetrySample& sample);

    bool isRecording() const noexcept;
    qint64 sampleCount() const noexcept;
    int peakCount() const noexcept;
    double elapsedSeconds() const noexcept;
    QString currentFilePath() const;
    QString lastError() const;
    QString captureStatus() const;

signals:
    void recordingStateChanged(bool active);
    void statsChanged(qint64 samples, double seconds, const QString& file_path);
    void peakCountChanged(int peaks);
    void captureStatusChanged(const QString& status);
    void errorOccurred(const QString& message);

private:
    QString makeBaseName_(const QString& prefix) const;
    bool writeMetadata_(const QString& metadata_path,
                        const QString& notes,
                        const models::WaveformConfig& waveform,
                        const models::RecordingConfig& config);
    double periodicFrequency_(const models::WaveformConfig& waveform) const noexcept;
    double selectedPeakSignal_(const models::TelemetrySample& sample) const noexcept;
    bool processPeak_(const models::TelemetrySample& sample, double capture_elapsed_s);
    void writePeak_(const models::TelemetrySample& sample,
                    double capture_elapsed_s,
                    double peak_value);
    void updateCaptureStatus_(double wall_elapsed_s);
    void closeFiles_();

    QFile file_;
    QTextStream stream_;
    QFile peak_file_;
    QTextStream peak_stream_;
    QElapsedTimer elapsed_;

    qint64 samples_{0};
    int peaks_{0};
    QString current_path_;
    QString peak_path_;
    QString last_error_;
    QString capture_status_{"IDLE"};

    models::RecordingConfig config_{};
    models::WaveformConfig waveform_{};
    double effective_duration_s_{0.0};
    double last_written_capture_s_{-1.0};
    double last_stats_emit_s_{-1.0e30};
    double last_status_emit_s_{-1.0e30};

    bool have_peak_prev2_{false};
    bool have_peak_prev1_{false};
    models::TelemetrySample peak_prev2_{};
    models::TelemetrySample peak_prev1_{};
    double peak_prev2_value_{0.0};
    double peak_prev1_value_{0.0};
    double peak_prev2_capture_s_{0.0};
    double peak_prev1_capture_s_{0.0};
    double last_peak_capture_s_{-1.0e30};
};

}  // namespace qube_servo2::gui::logging
