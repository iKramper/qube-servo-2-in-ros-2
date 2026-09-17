#pragma once

#include "qube_servo2_gui/core/models/waveform_config.hpp"

#include <QMainWindow>

class QLabel;
class QStackedWidget;
class QTabBar;
class QToolBar;
class QToolButton;
class QTimer;

namespace qube_servo2::gui::infra::ros {
class QubeRosBridge;
}

namespace qube_servo2::gui::logging {
class DataRecorder;
}

namespace qube_servo2::gui::ui::pages {
class DashboardPage;
class ControlPage;
class RecordingDialog;
class SystemPage;
}

namespace qube_servo2::gui::app {

class MainWindow final : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

    // Idempotent orderly shutdown used by window close and Ctrl+C.
    void shutdown();

private slots:
    void onNavChanged_(int index);
    void onThemeAction_();
    void onThemeChanged_();
    void showRecordingDialog_();
    void startRecording_();
    void stopRecording_();
    void onWaveformStarted_(const qube_servo2::gui::models::WaveformConfig& config);
    void onWaveformStopped_();
    void onConnectionChanged_(bool connected);
    void updateStatusBar_();

private:
    void buildUi_();
    void buildTopBar_();
    void buildStatusBar_();
    void wireSignals_();
    void updateTopBarStyle_();
    bool beginRecording_(const models::WaveformConfig& waveform);

    infra::ros::QubeRosBridge* bridge_{nullptr};
    logging::DataRecorder* recorder_{nullptr};

    QStackedWidget* stack_{nullptr};
    ui::pages::DashboardPage* dashboard_{nullptr};
    ui::pages::ControlPage* control_{nullptr};
    ui::pages::RecordingDialog* recording_dialog_{nullptr};
    ui::pages::SystemPage* system_{nullptr};

    QToolBar* topbar_{nullptr};
    QTabBar* top_tabs_{nullptr};
    QToolButton* theme_btn_{nullptr};

    QLabel* status_connection_{nullptr};
    QLabel* status_recorder_{nullptr};
    QLabel* status_command_{nullptr};
    QTimer* status_timer_{nullptr};

    models::WaveformConfig last_waveform_{};
    bool recorder_auto_started_{false};
    bool shutting_down_{false};
};

}  // namespace qube_servo2::gui::app
