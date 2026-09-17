#pragma once

#include <QDialog>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTextEdit;

namespace qube_servo2::gui::ui::pages {

class RecordingDialog final : public QDialog {
    Q_OBJECT

public:
    explicit RecordingDialog(QWidget* parent = nullptr);

    QString directory() const;
    QString prefix() const;
    QString notes() const;
    bool writeMetadata() const noexcept;

public slots:
    void setRecordingState(bool active);
    void setStats(qint64 samples, double seconds, const QString& file_path);
    void showError(const QString& message);

signals:
    void startRecordingRequested();
    void stopRecordingRequested();

private slots:
    void browseDirectory_();

private:
    QLineEdit* directory_edit_{nullptr};
    QLineEdit* prefix_edit_{nullptr};
    QTextEdit* notes_edit_{nullptr};
    QCheckBox* metadata_check_{nullptr};
    QPushButton* start_btn_{nullptr};
    QPushButton* stop_btn_{nullptr};
    QLabel* state_label_{nullptr};
    QLabel* file_label_{nullptr};
    QLabel* samples_label_{nullptr};
    QLabel* duration_label_{nullptr};
    QLabel* error_label_{nullptr};
};

}  // namespace qube_servo2::gui::ui::pages
