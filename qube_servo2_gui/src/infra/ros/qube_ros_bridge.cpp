#include "qube_servo2_gui/infra/ros/qube_ros_bridge.hpp"

#include <QDateTime>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>

namespace qube_servo2::gui::infra::ros {

namespace {

double stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

}  // namespace

QubeRosBridge::QubeRosBridge(QObject* parent)
    : QObject(parent) {
    rclcpp::NodeOptions options;
    node_ = std::make_shared<rclcpp::Node>("qube_servo2_hmi", options);

    command_topic_ = node_->declare_parameter<std::string>(
        "command_topic", "/motor_voltage_controller/commands");
    joint_states_topic_ = node_->declare_parameter<std::string>(
        "joint_states_topic", "/joint_states");
    dynamic_joint_states_topic_ = node_->declare_parameter<std::string>(
        "dynamic_joint_states_topic", "/dynamic_joint_states");
    joint_name_ = node_->declare_parameter<std::string>(
        "joint_name", "motor_hub_link_joint");
    voltage_limit_ = node_->declare_parameter<double>("voltage_limit", 10.0);
    const auto connection_timeout_ms_param = node_->declare_parameter<std::int64_t>(
        "connection_timeout_ms", 1000);

    connection_timeout_ms_ = static_cast<int>(
        std::clamp<std::int64_t>(connection_timeout_ms_param, 1, 60000));
    const auto ui_refresh_hz_param = node_->declare_parameter<std::int64_t>(
        "ui_refresh_hz", 50);

    const int ui_refresh_hz = static_cast<int>(
        std::clamp<std::int64_t>(ui_refresh_hz_param, 1, 200));

    command_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 10);

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_,
        rclcpp::QoS(20),
        std::bind(&QubeRosBridge::onJointState_, this, std::placeholders::_1));

    dynamic_sub_ = node_->create_subscription<control_msgs::msg::DynamicJointState>(
        dynamic_joint_states_topic_,
        rclcpp::QoS(50),
        std::bind(&QubeRosBridge::onDynamicJointState_, this, std::placeholders::_1));

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    spin_timer_.setInterval(5);
    spin_timer_.setTimerType(Qt::PreciseTimer);
    connect(&spin_timer_, &QTimer::timeout, this, &QubeRosBridge::spinOnce_);
    spin_timer_.start();

    ui_timer_.setInterval(std::max(5, 1000 / ui_refresh_hz));
    ui_timer_.setTimerType(Qt::CoarseTimer);
    connect(&ui_timer_, &QTimer::timeout, this, &QubeRosBridge::publishUiSample_);
    ui_timer_.start();
}

QubeRosBridge::~QubeRosBridge() {
    shutdown(rclcpp::ok());
}

void QubeRosBridge::shutdown(bool send_zero) {
    if (shutting_down_) {
        return;
    }
    shutting_down_ = true;

    // Stop Qt callbacks before releasing ROS entities.
    spin_timer_.stop();
    ui_timer_.stop();
    spin_timer_.disconnect(this);
    ui_timer_.disconnect(this);

    // Final safe command while DDS publisher and ROS context are still alive.
    if (send_zero && rclcpp::ok() && command_pub_) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {0.0};
        command_pub_->publish(msg);
        latest_.command_voltage = 0.0;
    }

    connected_ = false;

    // Remove node from executor before destroying subscriptions / publisher.
    if (executor_ && node_) {
        try {
            executor_->cancel();
            executor_->remove_node(node_);
        } catch (const std::exception&) {
            // Teardown must remain noexcept-like even if the ROS context is
            // already transitioning to shutdown.
        }
    }

    dynamic_sub_.reset();
    joint_sub_.reset();
    command_pub_.reset();
    executor_.reset();
    node_.reset();

    latest_ = models::TelemetrySample{};
}

rclcpp::Node::SharedPtr QubeRosBridge::node() const noexcept {
    return node_;
}

QString QubeRosBridge::commandTopic() const { return QString::fromStdString(command_topic_); }
QString QubeRosBridge::jointStatesTopic() const { return QString::fromStdString(joint_states_topic_); }
QString QubeRosBridge::dynamicJointStatesTopic() const { return QString::fromStdString(dynamic_joint_states_topic_); }
QString QubeRosBridge::jointName() const { return QString::fromStdString(joint_name_); }

double QubeRosBridge::voltageLimit() const noexcept { return voltage_limit_; }
bool QubeRosBridge::isConnected() const noexcept { return connected_; }
double QubeRosBridge::lastCommandVoltage() const noexcept { return latest_.command_voltage; }

void QubeRosBridge::publishVoltage(double voltage) {
    if (shutting_down_ || !rclcpp::ok() || !command_pub_) return;
    const double limited = std::clamp(voltage, -voltage_limit_, voltage_limit_);
    latest_.command_voltage = limited;

    std_msgs::msg::Float64MultiArray msg;
    msg.data = {limited};
    command_pub_->publish(msg);
    emit commandPublished(limited);
}

void QubeRosBridge::sendZero() {
    if (!shutting_down_ && command_pub_) {
        publishVoltage(0.0);
    }
}

void QubeRosBridge::spinOnce_() {
    if (shutting_down_ || !rclcpp::ok() || !executor_) return;
    executor_->spin_some();
}

void QubeRosBridge::publishUiSample_() {
    if (shutting_down_) return;
    const qint64 now_ms = QDateTime::currentMSecsSinceEpoch();
    const bool new_connected = latest_.valid && ((now_ms - last_rx_wall_ms_) <= connection_timeout_ms_);
    if (new_connected != connected_) {
        connected_ = new_connected;
        emit connectionStateChanged(connected_);
    }

    if (latest_.valid) {
        emit telemetryUpdated(latest_);
    }
}

void QubeRosBridge::touchConnection_() {
    last_rx_wall_ms_ = QDateTime::currentMSecsSinceEpoch();
    latest_.valid = true;
}

void QubeRosBridge::onJointState_(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (shutting_down_) return;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] != joint_name_) {
            continue;
        }

        if (i < msg->position.size()) latest_.position_rad = msg->position[i];
        if (i < msg->velocity.size()) latest_.velocity_rad_s = msg->velocity[i];

        const double t = stampToSeconds(msg->header.stamp);
        latest_.ros_time_s = (t > 0.0) ? t : node_->now().seconds();
        touchConnection_();
        return;
    }
}

void QubeRosBridge::onDynamicJointState_(const control_msgs::msg::DynamicJointState::SharedPtr msg) {
    if (shutting_down_) return;
    for (std::size_t joint_index = 0; joint_index < msg->joint_names.size(); ++joint_index) {
        if (msg->joint_names[joint_index] != joint_name_ ||
            joint_index >= msg->interface_values.size()) {
            continue;
        }

        const auto& interfaces = msg->interface_values[joint_index];
        const std::size_t count = std::min(interfaces.interface_names.size(), interfaces.values.size());

        for (std::size_t i = 0; i < count; ++i) {
            const auto& name = interfaces.interface_names[i];
            const double value = interfaces.values[i];

            if (name == "position") latest_.position_rad = value;
            else if (name == "velocity") latest_.velocity_rad_s = value;
            else if (name == "current") latest_.current_a = value;
            else if (name == "applied_voltage") latest_.applied_voltage_v = value;
            else if (name == "back_emf") latest_.back_emf_v = value;
            else if (name == "motor_torque") latest_.motor_torque_nm = value;
        }

        const double t = stampToSeconds(msg->header.stamp);
        latest_.ros_time_s = (t > 0.0) ? t : node_->now().seconds();
        touchConnection_();
        emit rawTelemetryUpdated(latest_);
        return;
    }
}

}  // namespace qube_servo2::gui::infra::ros
