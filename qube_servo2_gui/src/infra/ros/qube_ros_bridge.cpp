#include "qube_servo2_gui/infra/ros/qube_ros_bridge.hpp"

#include <QDateTime>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <utility>

namespace qube_servo2::gui::infra::ros {

namespace {

double stampToSeconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

std::string serviceName(const std::string& manager, const std::string& service) {
    if (manager.empty() || manager == "/") return "/" + service;
    return manager + "/" + service;
}

}  // namespace

QubeRosBridge::QubeRosBridge(QObject* parent)
    : QObject(parent) {
    rclcpp::NodeOptions options;
    node_ = std::make_shared<rclcpp::Node>("qube_servo2_hmi", options);

    controller_manager_name_ = node_->declare_parameter<std::string>(
        "controller_manager_name", "/controller_manager");
    controller_param_file_ = node_->declare_parameter<std::string>("controller_param_file", "");

    if (controller_param_file_.empty()) {
        try {
            controller_param_file_ =
                ament_index_cpp::get_package_share_directory("qube_servo2_controllers") +
                "/config/controller_manager.yaml";
        } catch (const std::exception& e) {
            RCLCPP_WARN(node_->get_logger(),
                        "Unable to resolve qube_servo2_controllers parameter file: %s",
                        e.what());
        }
    }

    voltage_reference_topic_ = node_->declare_parameter<std::string>(
        "voltage_reference_topic", "/motor_voltage_controller/reference");
    pid_reference_topic_ = node_->declare_parameter<std::string>(
        "pid_reference_topic", "/qube_pid_controller/reference");
    state_feedback_reference_topic_ = node_->declare_parameter<std::string>(
        "state_feedback_reference_topic", "/qube_state_feedback_controller/reference");
    joint_states_topic_ = node_->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    dynamic_joint_states_topic_ = node_->declare_parameter<std::string>(
        "dynamic_joint_states_topic", "/dynamic_joint_states");
    joint_name_ = node_->declare_parameter<std::string>("joint_name", "motor_hub_link_joint");

    voltage_limit_ = node_->declare_parameter<double>("voltage_limit", 10.0);
    position_reference_limit_rad_ = node_->declare_parameter<double>(
        "position_reference_limit_rad", 6.283185307179586);
    velocity_reference_limit_rad_s_ = node_->declare_parameter<double>(
        "velocity_reference_limit_rad_s", 30.0);

    connection_timeout_ms_ = static_cast<int>(std::clamp<std::int64_t>(
        node_->declare_parameter<std::int64_t>("connection_timeout_ms", 1000), 1, 60000));
    const int ui_refresh_hz = static_cast<int>(std::clamp<std::int64_t>(
        node_->declare_parameter<std::int64_t>("ui_refresh_hz", 50), 1, 200));
    const int controller_refresh_ms = static_cast<int>(std::clamp<std::int64_t>(
        node_->declare_parameter<std::int64_t>("controller_status_refresh_ms", 750), 100, 10000));

    pid_position_.kp = node_->declare_parameter<double>("profiles.pid.position.kp", 50.2649);
    pid_position_.ki = node_->declare_parameter<double>("profiles.pid.position.ki", 591.3092);
    pid_position_.kd = node_->declare_parameter<double>("profiles.pid.position.kd", 0.3924);
    pid_position_.anti_windup = node_->declare_parameter<bool>("profiles.pid.position.anti_windup", true);
    pid_position_.angle_wraparound = node_->declare_parameter<bool>("profiles.pid.position.angle_wraparound", false);

    pid_velocity_.kp = node_->declare_parameter<double>("profiles.pid.velocity.kp", 0.0411);
    pid_velocity_.ki = node_->declare_parameter<double>("profiles.pid.velocity.ki", 0.4286);
    pid_velocity_.kd = node_->declare_parameter<double>("profiles.pid.velocity.kd", 0.0);
    pid_velocity_.anti_windup = node_->declare_parameter<bool>("profiles.pid.velocity.anti_windup", true);
    pid_velocity_.angle_wraparound = node_->declare_parameter<bool>("profiles.pid.velocity.angle_wraparound", false);

    sf_position_.k_position = node_->declare_parameter<double>(
        "profiles.state_feedback.position.k_position", 49.7959);
    sf_position_.k_velocity = node_->declare_parameter<double>(
        "profiles.state_feedback.position.k_velocity", 0.8213);
    sf_position_.angle_wraparound = node_->declare_parameter<bool>(
        "profiles.state_feedback.position.angle_wraparound", true);
    sf_position_.feedforward_gain = 0.0;

    sf_velocity_.k_position = node_->declare_parameter<double>(
        "profiles.state_feedback.velocity.k_position", 0.0);
    sf_velocity_.k_velocity = node_->declare_parameter<double>(
        "profiles.state_feedback.velocity.k_velocity", 1.3075);
    sf_velocity_.angle_wraparound = node_->declare_parameter<bool>(
        "profiles.state_feedback.velocity.angle_wraparound", false);
    sf_velocity_.feedforward_gain = node_->declare_parameter<double>(
        "profiles.state_feedback.velocity.feedforward_gain", 0.0);

    voltage_reference_pub_ = node_->create_publisher<std_msgs::msg::Float64>(voltage_reference_topic_, 10);
    pid_reference_pub_ = node_->create_publisher<std_msgs::msg::Float64>(pid_reference_topic_, 10);
    state_feedback_reference_pub_ = node_->create_publisher<std_msgs::msg::Float64>(state_feedback_reference_topic_, 10);

    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, rclcpp::QoS(20),
        std::bind(&QubeRosBridge::onJointState_, this, std::placeholders::_1));
    dynamic_sub_ = node_->create_subscription<control_msgs::msg::DynamicJointState>(
        dynamic_joint_states_topic_, rclcpp::QoS(50),
        std::bind(&QubeRosBridge::onDynamicJointState_, this, std::placeholders::_1));

    list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
        serviceName(controller_manager_name_, "list_controllers"));
    load_controller_client_ = node_->create_client<controller_manager_msgs::srv::LoadController>(
        serviceName(controller_manager_name_, "load_controller"));
    configure_controller_client_ = node_->create_client<controller_manager_msgs::srv::ConfigureController>(
        serviceName(controller_manager_name_, "configure_controller"));
    unload_controller_client_ = node_->create_client<controller_manager_msgs::srv::UnloadController>(
        serviceName(controller_manager_name_, "unload_controller"));
    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
        serviceName(controller_manager_name_, "switch_controller"));

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

    controller_status_timer_.setInterval(controller_refresh_ms);
    controller_status_timer_.setTimerType(Qt::CoarseTimer);
    connect(&controller_status_timer_, &QTimer::timeout, this, &QubeRosBridge::refreshControllers);
    controller_status_timer_.start();
}

QubeRosBridge::~QubeRosBridge() { shutdown(rclcpp::ok()); }

rclcpp::Node::SharedPtr QubeRosBridge::node() const noexcept { return node_; }
QString QubeRosBridge::jointStatesTopic() const { return QString::fromStdString(joint_states_topic_); }
QString QubeRosBridge::dynamicJointStatesTopic() const { return QString::fromStdString(dynamic_joint_states_topic_); }
QString QubeRosBridge::jointName() const { return QString::fromStdString(joint_name_); }
QString QubeRosBridge::controllerManagerName() const { return QString::fromStdString(controller_manager_name_); }
double QubeRosBridge::voltageLimit() const noexcept { return voltage_limit_; }
double QubeRosBridge::positionReferenceLimit() const noexcept { return position_reference_limit_rad_; }
double QubeRosBridge::velocityReferenceLimit() const noexcept { return velocity_reference_limit_rad_s_; }
bool QubeRosBridge::isConnected() const noexcept { return connected_; }
double QubeRosBridge::lastCommandVoltage() const noexcept { return latest_.applied_voltage_v; }
QString QubeRosBridge::activeController() const { return active_controller_; }
QString QubeRosBridge::activeMode() const { return active_mode_; }
bool QubeRosBridge::controllerBusy() const noexcept { return controller_busy_; }

bool QubeRosBridge::isManagedController_(const std::string& name) {
    return name == "motor_voltage_controller" ||
           name == "qube_pid_controller" ||
           name == "qube_state_feedback_controller";
}

QString QubeRosBridge::prettyController_(const QString& name) {
    if (name == "motor_voltage_controller") return "Open-loop Voltage";
    if (name == "qube_pid_controller") return "PID";
    if (name == "qube_state_feedback_controller") return "State Feedback";
    return name;
}

void QubeRosBridge::shutdown(bool send_zero) {
    if (shutting_down_) return;
    shutting_down_ = true;

    spin_timer_.stop();
    ui_timer_.stop();
    controller_status_timer_.stop();

    if (send_zero && rclcpp::ok()) {
        if (!active_controller_.isEmpty()) {
            publishReference(active_controller_, active_mode_, 0.0);
        }
    }

    connected_ = false;
    if (executor_ && node_) {
        try {
            executor_->cancel();
            executor_->remove_node(node_);
        } catch (const std::exception&) {
        }
    }

    dynamic_sub_.reset();
    joint_sub_.reset();
    voltage_reference_pub_.reset();
    pid_reference_pub_.reset();
    state_feedback_reference_pub_.reset();
    list_controllers_client_.reset();
    load_controller_client_.reset();
    configure_controller_client_.reset();
    unload_controller_client_.reset();
    switch_controller_client_.reset();
    executor_.reset();
    node_.reset();
    latest_ = models::TelemetrySample{};
}

void QubeRosBridge::publishReference(const QString& controller, const QString& mode, double value) {
    if (shutting_down_ || !rclcpp::ok()) return;

    double limited = value;
    if (controller == "motor_voltage_controller") {
        limited = std::clamp(value, -voltage_limit_, voltage_limit_);
    } else if (mode == "position") {
        limited = std::clamp(value, -position_reference_limit_rad_, position_reference_limit_rad_);
    } else {
        limited = std::clamp(value, -velocity_reference_limit_rad_s_, velocity_reference_limit_rad_s_);
    }

    std_msgs::msg::Float64 msg;
    msg.data = limited;

    if (controller == "qube_pid_controller") {
        pid_reference_pub_->publish(msg);
    } else if (controller == "qube_state_feedback_controller") {
        state_feedback_reference_pub_->publish(msg);
    } else {
        voltage_reference_pub_->publish(msg);
    }

    emit referencePublished(controller, mode, limited);
}

void QubeRosBridge::activateController(const QString& controller, const QString& mode) {
    if (shutting_down_ || controller_busy_) return;

    if (controller != "motor_voltage_controller" &&
        controller != "qube_pid_controller" &&
        controller != "qube_state_feedback_controller") {
        emit controllerOperationFinished(false, "Unknown controller: " + controller);
        return;
    }

    if (controller != "motor_voltage_controller" && mode != "position" && mode != "velocity") {
        emit controllerOperationFinished(false, "Control mode must be position or velocity.");
        return;
    }

    controller_busy_ = true;
    requested_controller_ = controller;
    requested_mode_ = controller == "motor_voltage_controller" ? "voltage" : mode;
    listForActivation_();
}

void QubeRosBridge::listForActivation_() {
    if (!list_controllers_client_ || !list_controllers_client_->service_is_ready()) {
        finishOperation_(false, "controller_manager/list_controllers is not available.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    list_controllers_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            std::vector<ControllerSnapshot> snapshots;
            for (const auto& controller : future.get()->controller) {
                snapshots.push_back({controller.name, controller.state});
            }
            afterListForActivation_(std::move(snapshots));
        });
}

void QubeRosBridge::afterListForActivation_(std::vector<ControllerSnapshot> controllers) {
    std::vector<std::string> active;
    bool target_loaded = false;
    std::string target_state;
    const std::string target = requested_controller_.toStdString();

    for (const auto& c : controllers) {
        if (c.name == target) {
            target_loaded = true;
            target_state = c.state;
        }
        if (isManagedController_(c.name) && c.state == "active") active.push_back(c.name);
    }

    if (!active.empty()) {
        switchControllers_({}, active, [this, target_loaded, target_state](bool ok) {
            if (!ok) {
                finishOperation_(false, "Unable to deactivate the currently active controller.");
                return;
            }
            prepareTarget_(target_loaded, target_state == "active" ? "inactive" : target_state);
        });
        return;
    }

    prepareTarget_(target_loaded, target_state);
}

void QubeRosBridge::prepareTarget_(bool target_loaded, const std::string& target_state) {
    if (!target_loaded) {
        ensureManagerParamFileAndLoad_();
        return;
    }

    if (target_state == "inactive" || target_state == "unconfigured" || target_state == "active") {
        // Recreate the controller node before applying a new profile. This is
        // deterministic and guarantees that on_init()/on_configure() see the
        // new mode and gains.
        unloadTarget_();
        return;
    }

    finishOperation_(false, QString("Unsupported lifecycle state '%1'.").arg(QString::fromStdString(target_state)));
}

void QubeRosBridge::unloadTarget_() {
    if (!unload_controller_client_ || !unload_controller_client_->service_is_ready()) {
        finishOperation_(false, "controller_manager/unload_controller is not available.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::UnloadController::Request>();
    request->name = requested_controller_.toStdString();
    unload_controller_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::UnloadController>::SharedFuture future) {
            if (!future.get()->ok) {
                finishOperation_(false, "Failed to unload target controller before reconfiguration.");
                return;
            }
            ensureManagerParamFileAndLoad_();
        });
}

void QubeRosBridge::ensureManagerParamFileAndLoad_() {
    if (controller_param_file_.empty()) {
        finishOperation_(false, "No controller parameter file is available for loading controllers.");
        return;
    }

    setControllerManagerParamsFile_([this](bool ok) {
        if (!ok) {
            finishOperation_(false, "Failed to assign controller params_file in controller_manager.");
            return;
        }
        loadTarget_();
    });
}

void QubeRosBridge::setControllerManagerParamsFile_(std::function<void(bool)> done) {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(node_, controller_manager_name_);
    if (!client->service_is_ready()) {
        done(false);
        return;
    }

    const std::string parameter_name = requested_controller_.toStdString() + ".params_file";
    std::vector<rclcpp::Parameter> params{
        rclcpp::Parameter(parameter_name, std::vector<std::string>{controller_param_file_})};

    client->set_parameters(
        params,
        [client, done](auto future) {
            bool ok = true;
            for (const auto& result : future.get()) ok = ok && result.successful;
            done(ok);
        });
}

void QubeRosBridge::loadTarget_() {
    if (!load_controller_client_ || !load_controller_client_->service_is_ready()) {
        finishOperation_(false, "controller_manager/load_controller is not available.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
    request->name = requested_controller_.toStdString();
    load_controller_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedFuture future) {
            if (!future.get()->ok) {
                finishOperation_(false, "Failed to load " + requested_controller_ + ".");
                return;
            }
            setTargetParameters_();
        });
}

void QubeRosBridge::setTargetParameters_() {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/" + requested_controller_.toStdString());
    if (!client->service_is_ready()) {
        finishOperation_(false, "Parameter service for " + requested_controller_ + " is not available.");
        return;
    }

    std::vector<rclcpp::Parameter> params;
    params.emplace_back("joint_name", joint_name_);
    params.emplace_back("interface_name", "voltage");
    params.emplace_back("reference_topic", "~/reference");

    if (requested_controller_ == "qube_pid_controller") {
        const auto& p = requested_mode_ == "velocity" ? pid_velocity_ : pid_position_;
        params.emplace_back("mode", requested_mode_.toStdString());
        params.emplace_back("gains.kp", p.kp);
        params.emplace_back("gains.ki", p.ki);
        params.emplace_back("gains.kd", p.kd);
        params.emplace_back("anti_windup", p.anti_windup);
        params.emplace_back("angle_wraparound", p.angle_wraparound);
    } else if (requested_controller_ == "qube_state_feedback_controller") {
        const auto& p = requested_mode_ == "velocity" ? sf_velocity_ : sf_position_;
        params.emplace_back("mode", requested_mode_.toStdString());
        params.emplace_back("gains.k_position", p.k_position);
        params.emplace_back("gains.k_velocity", p.k_velocity);
        params.emplace_back("angle_wraparound", p.angle_wraparound);
        params.emplace_back("feedforward.velocity_gain", p.feedforward_gain);
    }

    client->set_parameters(
        params,
        [this, client](auto future) {
            QString reason;
            bool ok = true;
            for (const auto& result : future.get()) {
                if (!result.successful) {
                    ok = false;
                    reason += QString::fromStdString(result.reason) + " ";
                }
            }
            if (!ok) {
                finishOperation_(false, "Failed to set controller profile: " + reason.trimmed());
                return;
            }
            configureTarget_();
        });
}

void QubeRosBridge::configureTarget_() {
    if (!configure_controller_client_ || !configure_controller_client_->service_is_ready()) {
        finishOperation_(false, "controller_manager/configure_controller is not available.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    request->name = requested_controller_.toStdString();
    configure_controller_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ConfigureController>::SharedFuture future) {
            if (!future.get()->ok) {
                finishOperation_(false, "Failed to configure " + requested_controller_ + ".");
                return;
            }
            activateTarget_();
        });
}

void QubeRosBridge::activateTarget_() {
    switchControllers_({requested_controller_.toStdString()}, {}, [this](bool ok) {
        if (!ok) {
            finishOperation_(false, "Failed to activate " + requested_controller_ + ".");
            return;
        }
        active_controller_ = requested_controller_;
        active_mode_ = requested_mode_;
        publishReference(active_controller_, active_mode_, 0.0);
        finishOperation_(true,
            QString("%1 active in %2 mode.")
                .arg(prettyController_(active_controller_), active_mode_.toUpper()));
        emit controllerStateChanged(active_controller_, "active", active_mode_);
    });
}

void QubeRosBridge::deactivateControllers() {
    if (shutting_down_ || controller_busy_) return;
    controller_busy_ = true;

    if (!active_controller_.isEmpty()) publishReference(active_controller_, active_mode_, 0.0);

    if (!list_controllers_client_ || !list_controllers_client_->service_is_ready()) {
        finishOperation_(false, "controller_manager/list_controllers is not available.");
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    list_controllers_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            std::vector<std::string> active;
            for (const auto& c : future.get()->controller) {
                if (isManagedController_(c.name) && c.state == "active") active.push_back(c.name);
            }
            if (active.empty()) {
                active_controller_.clear();
                active_mode_.clear();
                finishOperation_(true, "All QUBE command controllers are already inactive.");
                emit controllerStateChanged("", "inactive", "");
                return;
            }
            switchControllers_({}, active, [this](bool ok) {
                if (ok) {
                    active_controller_.clear();
                    active_mode_.clear();
                    emit controllerStateChanged("", "inactive", "");
                    finishOperation_(true, "QUBE command controller deactivated; voltage command returned to 0 V.");
                } else {
                    finishOperation_(false, "Failed to deactivate active QUBE controller.");
                }
            });
        });
}

void QubeRosBridge::emergencyStop() {
    if (!active_controller_.isEmpty()) publishReference(active_controller_, active_mode_, 0.0);
    deactivateControllers();
}

void QubeRosBridge::switchControllers_(
    const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate,
    std::function<void(bool)> done) {

    if (!switch_controller_client_ || !switch_controller_client_->service_is_ready()) {
        done(false);
        return;
    }

    auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    request->activate_controllers = activate;
    request->deactivate_controllers = deactivate;
    request->strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    switch_controller_client_->async_send_request(
        request,
        [done](rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
            done(future.get()->ok);
        });
}

void QubeRosBridge::finishOperation_(bool success, const QString& message) {
    controller_busy_ = false;
    emit controllerOperationFinished(success, message);
    refreshControllers();
}

void QubeRosBridge::refreshControllers() {
    if (shutting_down_ || controller_busy_ || !list_controllers_client_ ||
        !list_controllers_client_->service_is_ready()) return;

    auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
    list_controllers_client_->async_send_request(
        request,
        [this](rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
            QString found;
            QString state = "inactive";
            for (const auto& c : future.get()->controller) {
                if (isManagedController_(c.name) && c.state == "active") {
                    found = QString::fromStdString(c.name);
                    state = "active";
                    break;
                }
            }

            if (found != active_controller_) {
                active_controller_ = found;
                if (active_controller_ == "motor_voltage_controller") active_mode_ = "voltage";
                else if (active_controller_.isEmpty()) active_mode_.clear();
            }
            emit controllerStateChanged(active_controller_, state, active_mode_);
        });
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
    if (latest_.valid) emit telemetryUpdated(latest_);
}

void QubeRosBridge::touchConnection_() {
    last_rx_wall_ms_ = QDateTime::currentMSecsSinceEpoch();
    latest_.valid = true;
}

void QubeRosBridge::onJointState_(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (shutting_down_) return;
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        if (msg->name[i] != joint_name_) continue;
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
        if (msg->joint_names[joint_index] != joint_name_ || joint_index >= msg->interface_values.size()) continue;

        const auto& interfaces = msg->interface_values[joint_index];
        const std::size_t count = std::min(interfaces.interface_names.size(), interfaces.values.size());
        for (std::size_t i = 0; i < count; ++i) {
            const auto& name = interfaces.interface_names[i];
            const double value = interfaces.values[i];
            if (name == "position") latest_.position_rad = value;
            else if (name == "velocity") latest_.velocity_rad_s = value;
            else if (name == "current") latest_.current_a = value;
            else if (name == "applied_voltage") {
                latest_.applied_voltage_v = value;
                latest_.command_voltage = value;
            }
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
