#pragma once

#include "qube_servo2_gui/core/models/recording_config.hpp"
#include "qube_servo2_gui/core/models/waveform_config.hpp"
#include <QDialog>

class QCheckBox; class QComboBox; class QDoubleSpinBox; class QLabel; class QLineEdit; class QPushButton; class QSpinBox; class QTextEdit; class QWidget;

namespace qube_servo2::gui::ui::pages {
class RecordingDialog final : public QDialog {
 Q_OBJECT
public:
 explicit RecordingDialog(QWidget* parent=nullptr);
 QString directory() const; QString prefix() const; QString notes() const; bool writeMetadata() const noexcept; models::RecordingConfig recordingConfig() const;
public slots:
 void setRecordingState(bool active); void setStats(qint64 samples,double seconds,const QString& file_path); void setPeakCount(int peaks); void setCaptureStatus(const QString& status); void setWaveformInfo(const qube_servo2::gui::models::WaveformConfig& waveform); void showError(const QString& message);
signals: void startRecordingRequested(); void stopRecordingRequested();
private slots: void browseDirectory_(); void updateCaptureModeUi_();
private:
 QLineEdit* directory_edit_{nullptr}; QLineEdit* prefix_edit_{nullptr}; QTextEdit* notes_edit_{nullptr}; QCheckBox* metadata_check_{nullptr};
 QComboBox* capture_mode_{nullptr}; QSpinBox* sample_interval_ms_{nullptr}; QDoubleSpinBox* start_delay_s_{nullptr}; QDoubleSpinBox* duration_s_{nullptr}; QSpinBox* period_count_{nullptr};
 QComboBox* peak_signal_{nullptr}; QDoubleSpinBox* peak_threshold_{nullptr}; QDoubleSpinBox* peak_separation_s_{nullptr}; QSpinBox* peak_target_count_{nullptr}; QCheckBox* peak_absolute_{nullptr};
 QWidget* timing_fields_{nullptr}; QWidget* duration_field_{nullptr}; QWidget* periods_field_{nullptr}; QWidget* peak_fields_{nullptr};
 QLabel* waveform_info_{nullptr}; QPushButton* start_btn_{nullptr}; QPushButton* stop_btn_{nullptr}; QLabel* state_label_{nullptr}; QLabel* file_label_{nullptr}; QLabel* samples_label_{nullptr}; QLabel* duration_label_{nullptr}; QLabel* peaks_label_{nullptr}; QLabel* error_label_{nullptr};
};
}
