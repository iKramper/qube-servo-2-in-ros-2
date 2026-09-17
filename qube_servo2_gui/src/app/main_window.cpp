#include "qube_servo2_gui/app/main_window.hpp"

#include "qube_servo2_gui/core/logging/data_recorder.hpp"
#include "qube_servo2_gui/infra/ros/qube_ros_bridge.hpp"
#include "qube_servo2_gui/ui/pages/control_page.hpp"
#include "qube_servo2_gui/ui/pages/dashboard_page.hpp"
#include "qube_servo2_gui/ui/pages/recording_dialog.hpp"
#include "qube_servo2_gui/ui/pages/system_page.hpp"
#include "qube_servo2_gui/ui/style/theme_manager.hpp"

#include <QAction>
#include <QApplication>
#include <QLabel>
#include <QMenu>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStyle>
#include <QTabBar>
#include <QTimer>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

namespace qube_servo2::gui::app {

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent) {
    setWindowTitle("QUBE-Servo 2 HMI");
    setMinimumSize(1280, 820);
    resize(1600, 920);

    bridge_ = new infra::ros::QubeRosBridge(this);
    recorder_ = new logging::DataRecorder(this);

    buildUi_();
    buildTopBar_();
    buildStatusBar_();
    wireSignals_();

    system_->setConfiguration(
        bridge_->jointName(),
        bridge_->commandTopic(),
        bridge_->jointStatesTopic(),
        bridge_->dynamicJointStatesTopic(),
        bridge_->voltageLimit());

    top_tabs_->setCurrentIndex(0);
    onNavChanged_(0);
}

MainWindow::~MainWindow() {
    shutdown();
}

void MainWindow::shutdown() {
    if (shutting_down_) {
        return;
    }
    shutting_down_ = true;

    // Stop UI timers first so no new callbacks are scheduled while resources
    // are being released.
    if (status_timer_) {
        status_timer_->stop();
    }

    // Stop signal generation without emitting another command. The ROS bridge
    // sends the final explicit 0 V below while its publisher is still alive.
    if (control_) {
        control_->shutdown();
    }

    if (recording_dialog_) {
        recording_dialog_->hide();
    }

    if (recorder_) {
        recorder_->stop();
    }

    // Release chart history immediately instead of waiting for QObject tree
    // destruction. This removes the largest dynamic buffers in the HMI.
    if (dashboard_) {
        dashboard_->shutdown();
    }

    // ROS must be torn down before rclcpp::shutdown() in main(). This publishes
    // the final zero command, stops spin/UI timers, removes the node from the
    // executor, and releases subscriptions/publisher/node in deterministic order.
    if (bridge_) {
        bridge_->shutdown(true);
    }
}

void MainWindow::buildUi_() {
    auto* central = new QWidget(this);
    setCentralWidget(central);
    auto* root = new QVBoxLayout(central);
    root->setContentsMargins(0, 0, 0, 0);
    root->setSpacing(0);

    stack_ = new QStackedWidget(central);
    dashboard_ = new ui::pages::DashboardPage(stack_);
    control_ = new ui::pages::ControlPage(bridge_->voltageLimit(), stack_);
    system_ = new ui::pages::SystemPage(stack_);
    recording_dialog_ = new ui::pages::RecordingDialog(this);

    stack_->addWidget(dashboard_);
    stack_->addWidget(control_);
    stack_->addWidget(system_);
    root->addWidget(stack_, 1);
}

void MainWindow::buildTopBar_() {
    topbar_ = new QToolBar(this);
    topbar_->setObjectName("topbar");
    topbar_->setMovable(false);
    topbar_->setFloatable(false);
    topbar_->setIconSize(QSize(18, 18));

    top_tabs_ = new QTabBar(this);
    top_tabs_->setObjectName("topTabs");
    top_tabs_->setExpanding(false);
    top_tabs_->setMovable(false);
    top_tabs_->setUsesScrollButtons(false);
    top_tabs_->setDrawBase(false);
    top_tabs_->addTab("Dashboard");
    top_tabs_->addTab("Signal Generator");
    top_tabs_->addTab("System");
    connect(top_tabs_, &QTabBar::currentChanged, this, &MainWindow::onNavChanged_);

    theme_btn_ = new QToolButton(this);
    theme_btn_->setObjectName("themeButton");
    theme_btn_->setAutoRaise(true);
    theme_btn_->setPopupMode(QToolButton::InstantPopup);
    theme_btn_->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    theme_btn_->setIcon(style()->standardIcon(QStyle::SP_DesktopIcon));
    theme_btn_->setText("Theme");

    auto* menu = new QMenu(theme_btn_);
    menu->setObjectName("themeMenu");
    auto add_theme = [&](const QString& name, ui::style::ThemeId id) {
        QAction* action = menu->addAction(name);
        action->setData(static_cast<int>(id));
        connect(action, &QAction::triggered, this, &MainWindow::onThemeAction_);
    };

    add_theme("MATLAB / Lab", ui::style::ThemeId::MATLAB);
    add_theme("Light", ui::style::ThemeId::Light);
    add_theme("Dark", ui::style::ThemeId::Dark);
    add_theme("Tron Evolution / Glass HUD", ui::style::ThemeId::TronEvolution);
    add_theme("Tron Ares", ui::style::ThemeId::TronAres);
    add_theme("Cyberpunk Neon", ui::style::ThemeId::CyberpunkNeon);
    theme_btn_->setMenu(menu);

    addToolBar(Qt::TopToolBarArea, topbar_);
    topbar_->addWidget(top_tabs_);
    auto* spacer = new QWidget(topbar_);
    spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    topbar_->addWidget(spacer);
    topbar_->addWidget(theme_btn_);

    connect(&ui::style::ThemeManager::instance(),
            &ui::style::ThemeManager::themeChanged,
            this,
            [this](ui::style::ThemeId) { onThemeChanged_(); });
    updateTopBarStyle_();
}

void MainWindow::buildStatusBar_() {
    statusBar()->setObjectName("statusBar");
    statusBar()->setSizeGripEnabled(false);

    status_connection_ = new QLabel("ROS: DISCONNECTED", this);
    status_recorder_ = new QLabel("REC: IDLE", this);
    status_command_ = new QLabel("CMD: 0.000 V", this);
    statusBar()->addPermanentWidget(status_connection_);
    statusBar()->addPermanentWidget(new QLabel("  |  ", this));
    statusBar()->addPermanentWidget(status_recorder_);
    statusBar()->addPermanentWidget(new QLabel("  |  ", this));
    statusBar()->addPermanentWidget(status_command_);

    status_timer_ = new QTimer(this);
    status_timer_->setInterval(200);
    status_timer_->setTimerType(Qt::CoarseTimer);
    connect(status_timer_, &QTimer::timeout, this, &MainWindow::updateStatusBar_);
    status_timer_->start();
}

void MainWindow::wireSignals_() {
    connect(bridge_, &infra::ros::QubeRosBridge::telemetryUpdated,
            dashboard_, &ui::pages::DashboardPage::updateTelemetry);
    connect(bridge_, &infra::ros::QubeRosBridge::telemetryUpdated,
            control_, &ui::pages::ControlPage::updateTelemetry);
    connect(bridge_, &infra::ros::QubeRosBridge::rawTelemetryUpdated,
            recorder_, &logging::DataRecorder::append);

    connect(bridge_, &infra::ros::QubeRosBridge::connectionStateChanged,
            this, &MainWindow::onConnectionChanged_);
    connect(bridge_, &infra::ros::QubeRosBridge::connectionStateChanged,
            system_, &ui::pages::SystemPage::setConnected);
    connect(bridge_, &infra::ros::QubeRosBridge::commandPublished,
            system_, &ui::pages::SystemPage::setCommand);

    connect(control_, &ui::pages::ControlPage::commandRequested,
            bridge_, &infra::ros::QubeRosBridge::publishVoltage);
    connect(control_, &ui::pages::ControlPage::stopRequested,
            bridge_, &infra::ros::QubeRosBridge::sendZero);
    connect(control_, &ui::pages::ControlPage::waveformStarted,
            this, &MainWindow::onWaveformStarted_);
    connect(control_, &ui::pages::ControlPage::waveformStopped,
            this, &MainWindow::onWaveformStopped_);
    connect(control_, &ui::pages::ControlPage::recordingDialogRequested,
            this, &MainWindow::showRecordingDialog_);

    connect(system_, &ui::pages::SystemPage::sendZeroRequested,
            bridge_, &infra::ros::QubeRosBridge::sendZero);
    connect(system_, &ui::pages::SystemPage::sendZeroRequested,
            control_, &ui::pages::ControlPage::stopOutput);

    connect(recording_dialog_, &ui::pages::RecordingDialog::startRecordingRequested,
            this, &MainWindow::startRecording_);
    connect(recording_dialog_, &ui::pages::RecordingDialog::stopRecordingRequested,
            this, &MainWindow::stopRecording_);

    connect(recorder_, &logging::DataRecorder::recordingStateChanged,
            recording_dialog_, &ui::pages::RecordingDialog::setRecordingState);
    connect(recorder_, &logging::DataRecorder::recordingStateChanged,
            control_, &ui::pages::ControlPage::setRecordingState);
    connect(recorder_, &logging::DataRecorder::statsChanged,
            recording_dialog_, &ui::pages::RecordingDialog::setStats);
    connect(recorder_, &logging::DataRecorder::peakCountChanged,
            recording_dialog_, &ui::pages::RecordingDialog::setPeakCount);
    connect(recorder_, &logging::DataRecorder::captureStatusChanged,
            recording_dialog_, &ui::pages::RecordingDialog::setCaptureStatus);
    connect(recorder_, &logging::DataRecorder::errorOccurred,
            recording_dialog_, &ui::pages::RecordingDialog::showError);
}

void MainWindow::onNavChanged_(int index) {
    if (!stack_ || index < 0 || index >= stack_->count()) {
        return;
    }

    stack_->setCurrentIndex(index);

    // Hidden pages keep their data buffers but stop periodic chart repainting.
    // This substantially reduces CPU/GPU usage when only one HMI page is visible.
    if (dashboard_) dashboard_->setActive(index == 0);
    if (control_) control_->setActive(index == 1);
}

void MainWindow::onThemeAction_() {
    auto* action = qobject_cast<QAction*>(sender());
    if (!action) return;
    ui::style::ThemeManager::instance().apply(
        static_cast<ui::style::ThemeId>(action->data().toInt()));
}

void MainWindow::onThemeChanged_() { updateTopBarStyle_(); }

void MainWindow::showRecordingDialog_() {
    recording_dialog_->setWaveformInfo(control_->waveformConfig());
    recording_dialog_->show();
    recording_dialog_->raise();
    recording_dialog_->activateWindow();
}

void MainWindow::updateTopBarStyle_() {
    const auto& spec = ui::style::ThemeManager::instance().currentSpec();

    topbar_->setStyleSheet(QString(
        "QToolBar#topbar { background:%1; border:none; border-bottom:1px solid %2; min-height:42px; }"
        "QToolButton#themeButton { background:transparent; color:%3; border:none; padding:8px 12px; border-radius:5px; }"
        "QToolButton#themeButton:hover { background:%2; color:%4; }")
        .arg(spec.panel.name(), spec.accent.name(), spec.text.name(), spec.bg.name()));

    top_tabs_->setStyleSheet(QString(
        "QTabBar#topTabs::tab { background:transparent; color:%1; border:none; padding:11px 20px; font-weight:600; }"
        "QTabBar#topTabs::tab:hover { color:%2; }"
        "QTabBar#topTabs::tab:selected { color:%2; border-bottom:3px solid %2; }")
        .arg(spec.text_muted.name(), spec.accent.name()));

    if (theme_btn_->menu()) {
        theme_btn_->menu()->setStyleSheet(QString(
            "QMenu { background:%1; color:%2; border:1px solid %3; padding:5px; }"
            "QMenu::item { padding:7px 20px; border-radius:4px; }"
            "QMenu::item:selected { background:%3; color:%4; }")
            .arg(spec.panel.name(), spec.text.name(), spec.accent.name(), spec.bg.name()));
    }
}

bool MainWindow::beginRecording_(const models::WaveformConfig& waveform) {
    return recorder_->start(
        recording_dialog_->directory(),
        recording_dialog_->prefix(),
        recording_dialog_->notes(),
        waveform,
        recording_dialog_->recordingConfig(),
        recording_dialog_->writeMetadata());
}

void MainWindow::startRecording_() { beginRecording_(control_->waveformConfig()); }
void MainWindow::stopRecording_() { recorder_->stop(); }

void MainWindow::onWaveformStarted_(const models::WaveformConfig& config) {
    last_waveform_ = config;
    recorder_auto_started_ = false;
    if (control_->autoRecordEnabled() && !recorder_->isRecording()) {
        recorder_auto_started_ = beginRecording_(config);
    }
}

void MainWindow::onWaveformStopped_() {
    if (recorder_auto_started_ && recorder_->isRecording()) recorder_->stop();
    recorder_auto_started_ = false;
}

void MainWindow::onConnectionChanged_(bool connected) {
    status_connection_->setText(connected ? "ROS: LIVE" : "ROS: DISCONNECTED / STALE");
}

void MainWindow::updateStatusBar_() {
    status_recorder_->setText(recorder_->isRecording()
                                  ? QString("REC: %1 samples").arg(recorder_->sampleCount())
                                  : "REC: IDLE");
    status_command_->setText(QString("CMD: %1 V").arg(bridge_->lastCommandVoltage(), 0, 'f', 3));
}

}  // namespace qube_servo2::gui::app
