#pragma once

#include "qube_servo2_gui/core/models/telemetry_sample.hpp"

#include <QObject>
#include <QTimer>

#include <control_msgs/msg/dynamic_joint_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <memory>
#include <string>

namespace qube_servo2::gui::infra::ros {

class QubeRosBridge final : public QObject {
    Q_OBJECT

public:
    explicit QubeRosBridge(QObject* parent = nullptr);
    ~QubeRosBridge() override;

    rclcpp::Node::SharedPtr node() const noexcept;

    QString commandTopic() const;
    QString jointStatesTopic() const;
    QString dynamicJointStatesTopic() const;
    QString jointName() const;

    double voltageLimit() const noexcept;
    bool isConnected() const noexcept;
    double lastCommandVoltage() const noexcept;

    // Deterministic ROS teardown. Safe to call more than once.
    void shutdown(bool send_zero = true);

public slots:
    void publishVoltage(double voltage);
    void sendZero();

signals:
    void rawTelemetryUpdated(const qube_servo2::gui::models::TelemetrySample& sample);
    void telemetryUpdated(const qube_servo2::gui::models::TelemetrySample& sample);
    void connectionStateChanged(bool connected);
    void commandPublished(double voltage);

private slots:
    void spinOnce_();
    void publishUiSample_();

private:
    void onJointState_(const sensor_msgs::msg::JointState::SharedPtr msg);
    void onDynamicJointState_(const control_msgs::msg::DynamicJointState::SharedPtr msg);
    void touchConnection_();

    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<control_msgs::msg::DynamicJointState>::SharedPtr dynamic_sub_;

    QTimer spin_timer_;
    QTimer ui_timer_;

    std::string command_topic_;
    std::string joint_states_topic_;
    std::string dynamic_joint_states_topic_;
    std::string joint_name_;

    double voltage_limit_{10.0};
    int connection_timeout_ms_{1000};
    qint64 last_rx_wall_ms_{0};
    bool connected_{false};
    bool shutting_down_{false};

    models::TelemetrySample latest_{};
};

}  // namespace qube_servo2::gui::infra::ros
