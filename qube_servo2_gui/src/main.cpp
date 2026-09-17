#include "qube_servo2_gui/app/main_window.hpp"
#include "qube_servo2_gui/core/models/telemetry_sample.hpp"
#include "qube_servo2_gui/core/models/waveform_config.hpp"
#include "qube_servo2_gui/ui/style/theme_manager.hpp"

#include <QApplication>
#include <QObject>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>

#include <csignal>

namespace {

volatile std::sig_atomic_t g_shutdown_signal = 0;

void signalHandler(int signal_number) {
    g_shutdown_signal = signal_number;
}

}  // namespace

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);

    qRegisterMetaType<qube_servo2::gui::models::TelemetrySample>(
        "qube_servo2::gui::models::TelemetrySample");
    qRegisterMetaType<qube_servo2::gui::models::WaveformConfig>(
        "qube_servo2::gui::models::WaveformConfig");

    // ROS 2's default SIGINT/SIGTERM handler is disabled here so the Qt event
    // loop gets a chance to perform an orderly HMI shutdown first: stop the
    // waveform, send 0 V, close the recorder, remove the ROS node and release
    // chart resources. This is especially important when launched through
    // `ros2 launch` and stopped with Ctrl+C.
    rclcpp::InitOptions ros_init_options;
    rclcpp::init(
        argc,
        argv,
        ros_init_options,
        rclcpp::SignalHandlerOptions::None);

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    qube_servo2::gui::ui::style::ThemeManager::instance().apply(
        qube_servo2::gui::ui::style::ThemeId::MATLAB);

    qube_servo2::gui::app::MainWindow window;
    window.show();

    bool finalized = false;

    QTimer signal_poll_timer;
    signal_poll_timer.setInterval(40);
    signal_poll_timer.setTimerType(Qt::CoarseTimer);

    QObject::connect(&signal_poll_timer, &QTimer::timeout, &app, [&]() {
        if (g_shutdown_signal == 0 || finalized) {
            return;
        }

        // app.quit() emits aboutToQuit(), where the actual cleanup is executed.
        signal_poll_timer.stop();
        app.quit();
    });

    QObject::connect(&app, &QCoreApplication::aboutToQuit, &app, [&]() {
        if (finalized) {
            return;
        }
        finalized = true;

        signal_poll_timer.stop();
        window.shutdown();

        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    });

    signal_poll_timer.start();

    const int result = app.exec();

    // Defensive fallback in case the platform exits the event loop without
    // emitting aboutToQuit(). shutdown() is intentionally idempotent.
    if (!finalized) {
        finalized = true;
        window.shutdown();
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    return result;
}
