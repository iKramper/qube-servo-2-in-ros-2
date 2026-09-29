#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"

#include <QObject>
#include <QTimer>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <controller_manager_msgs/srv/unload_controller.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace qube_servo2::gui::infra::ros {

class QubeRosBridge final : public QObject {
    Q_OBJECT

public:
    explicit QubeRosBridge(QObject* parent = nullptr);
    ~QubeRosBridge() override;

    rclcpp::Node::SharedPtr node() const noexcept;

    QString jointStatesTopic() const;
    QString dynamicJointStatesTopic() const;
    QString jointName() const;
    QString controllerManagerName() const;

    double voltageLimit() const noexcept;
    double positionReferenceLimit() const noexcept;
    double velocityReferenceLimit() const noexcept;
    bool isConnected() const noexcept;
    double lastCommandVoltage() const noexcept;
    QString activeController() const;
    QString activeMode() const;
    bool controllerBusy() const noexcept;

    void shutdown(bool send_zero = true);

public slots:
    void publishReference(const QString& controller, const QString& mode, double value);
    void activateController(const QString& controller, const QString& mode);
    void deactivateControllers();
    void emergencyStop();
    void refreshControllers();

signals:
    void rawTelemetryUpdated(const qube_servo2::gui::models::TelemetrySample& sample);
    void telemetryUpdated(const qube_servo2::gui::models::TelemetrySample& sample);
    void connectionStateChanged(bool connected);
    void controllerStateChanged(const QString& controller, const QString& state, const QString& mode);
    void controllerOperationFinished(bool success, const QString& message);
    void referencePublished(const QString& controller, const QString& mode, double value);

private slots:
    void spinOnce_();
    void publishUiSample_();

private:
    struct PidProfile {
        double kp{0.0};
        double ki{0.0};
        double kd{0.0};
        bool anti_windup{true};
        bool angle_wraparound{false};
    };

    struct StateFeedbackProfile {
        double k_position{0.0};
        double k_velocity{0.0};
        bool angle_wraparound{false};
        double feedforward_gain{0.0};
    };

    struct ControllerSnapshot {
        std::string name;
        std::string state;
    };

    static bool isManagedController_(const std::string& name);
    static QString prettyController_(const QString& name);

    void onJointState_(const sensor_msgs::msg::JointState::SharedPtr msg);
    void onDynamicJointState_(const control_msgs::msg::DynamicJointState::SharedPtr msg);
    void touchConnection_();

    void listForActivation_();
    void afterListForActivation_(std::vector<ControllerSnapshot> controllers);
    void prepareTarget_(bool target_loaded, const std::string& target_state);
    void unloadTarget_();
    void ensureManagerParamFileAndLoad_();
    void loadTarget_();
    void setTargetParameters_();
    void configureTarget_();
    void activateTarget_();
    void finishOperation_(bool success, const QString& message);
    void setControllerManagerParamsFile_(std::function<void(bool)> done);
    void switchControllers_(const std::vector<std::string>& activate,
                            const std::vector<std::string>& deactivate,
                            std::function<void(bool)> done);

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr voltage_reference_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pid_reference_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr state_feedback_reference_pub_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_sub_;

    rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
    rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr load_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedPtr configure_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedPtr unload_controller_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

    QTimer spin_timer_;
    QTimer ui_timer_;
    QTimer controller_status_timer_;

    std::string controller_manager_name_;
    std::string controller_param_file_;
    std::string voltage_reference_topic_;
    std::string pid_reference_topic_;
    std::string state_feedback_reference_topic_;
    std::string joint_states_topic_;
    std::string dynamic_joint_states_topic_;
    std::string joint_name_;

    double voltage_limit_{10.0};
    double position_reference_limit_rad_{6.283185307179586};
    double velocity_reference_limit_rad_s_{30.0};
    int connection_timeout_ms_{1000};
    qint64 last_rx_wall_ms_{0};
    bool connected_{false};
    bool shutting_down_{false};
    bool controller_busy_{false};

    QString active_controller_{};
    QString active_mode_{};
    QString requested_controller_{};
    QString requested_mode_{};

    PidProfile pid_position_{};
    PidProfile pid_velocity_{};
    StateFeedbackProfile sf_position_{};
    StateFeedbackProfile sf_velocity_{};

    models::TelemetrySample latest_{};
};

}  // namespace qube_servo2::gui::infra::ros
