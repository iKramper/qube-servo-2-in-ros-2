#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"
#include "qube_servo2_gui/core/models/waveform_config.hpp"

#include <QObject>
#include <QElapsedTimer>
#include <QFile>
#include <QString>
#include <QTextStream>

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
               bool write_metadata);

    void stop();
    void append(const models::TelemetrySample& sample);

    bool isRecording() const noexcept;
    qint64 sampleCount() const noexcept;
    double elapsedSeconds() const noexcept;
    QString currentFilePath() const;
    QString lastError() const;

signals:
    void recordingStateChanged(bool active);
    void statsChanged(qint64 samples, double seconds, const QString& file_path);
    void errorOccurred(const QString& message);

private:
    QString makeBaseName_(const QString& prefix) const;
    bool writeMetadata_(const QString& metadata_path,
                        const QString& notes,
                        const models::WaveformConfig& waveform);

    QFile file_;
    QTextStream stream_;
    QElapsedTimer elapsed_;
    qint64 samples_{0};
    QString current_path_;
    QString last_error_;
};

}  // namespace qube_servo2::gui::logging
