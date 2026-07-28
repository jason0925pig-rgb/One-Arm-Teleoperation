#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "robot.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

class SafeOneArmServo : public rclcpp::Node {
public:
    SafeOneArmServo() : Node("safe_one_arm_servo") {
        arm_name_ = declare_parameter<std::string>("arm_name", "right");
        robot_ip_ = declare_parameter<std::string>("robot_ip", "");
        robot_port_ = declare_parameter<int>("robot_port", 10020);
        dry_run_ = declare_parameter<bool>("dry_run", true);
        hardware_motion_authorized_ =
            declare_parameter<bool>("hardware_motion_authorized", false);
        limits_configured_ = declare_parameter<bool>("limits_configured", false);
        power_on_on_arm_ = declare_parameter<bool>("power_on_on_arm", false);
        enable_robot_on_arm_ = declare_parameter<bool>("enable_robot_on_arm", false);
        disable_robot_on_disarm_ =
            declare_parameter<bool>("disable_robot_on_disarm", false);
        command_timeout_seconds_ =
            declare_parameter<double>("command_timeout_seconds", 0.30);
        feedback_timeout_seconds_ =
            declare_parameter<double>("feedback_timeout_seconds", 0.30);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 125.0);
        state_rate_hz_ = declare_parameter<double>("state_rate_hz", 20.0);
        lower_limits_ =
            declare_parameter<std::vector<double>>(
                "joint_lower_limits", std::vector<double>{});
        upper_limits_ =
            declare_parameter<std::vector<double>>(
                "joint_upper_limits", std::vector<double>{});
        max_velocity_ =
            declare_parameter<std::vector<double>>(
                "max_velocity_rad_s", std::vector<double>{});

        for (int index = 1; index <= 7; ++index) {
            joint_names_.push_back(
                arm_name_ + "_joint" + std::to_string(index));
        }
        safety_configuration_valid_ = validate_safety_configuration();

        const std::string prefix = "/" + arm_name_ + "_arm";
        command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            prefix + "/teleop_joint_command",
            10,
            std::bind(
                &SafeOneArmServo::command_callback,
                this,
                std::placeholders::_1));
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            prefix + "/joint_states", 10);
        enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/motion_enabled", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            prefix + "/safety_status", 10);
        motion_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_motion_enabled",
            std::bind(
                &SafeOneArmServo::set_motion_enabled,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

        robot_.set_sim_mode(dry_run_);
        connected_ = connect_robot();

        const auto control_period = std::chrono::microseconds(
            static_cast<std::chrono::microseconds::rep>(
                std::llround(1'000'000.0 / std::max(1.0, control_rate_hz_))));
        const auto state_period = std::chrono::microseconds(
            static_cast<std::chrono::microseconds::rep>(
                std::llround(1'000'000.0 / std::max(1.0, state_rate_hz_))));
        control_timer_ = create_wall_timer(
            control_period,
            std::bind(&SafeOneArmServo::control_tick, this));
        state_timer_ = create_wall_timer(
            state_period,
            std::bind(&SafeOneArmServo::state_tick, this));
        status_timer_ = create_wall_timer(
            500ms, std::bind(&SafeOneArmServo::publish_status, this));

        RCLCPP_WARN(
            get_logger(),
            "Safe one-arm node started: arm=%s dry_run=%d connected=%d "
            "control_period_us=%lld. "
            "It does not power, enable, or enter servo mode at startup.",
            arm_name_.c_str(),
            dry_run_,
            connected_,
            static_cast<long long>(control_period.count()));
        if (!safety_configuration_valid_) {
            RCLCPP_ERROR(
                get_logger(),
                "Motion is locked: actual seven-joint limits and velocities "
                "have not been configured.");
        }
    }

    ~SafeOneArmServo() override {
        disarm("node shutdown");
    }

private:
    static constexpr std::size_t kJointCount = 7;

    bool validate_safety_configuration() {
        if (!limits_configured_) {
            return false;
        }
        if (
            lower_limits_.size() != kJointCount ||
            upper_limits_.size() != kJointCount ||
            max_velocity_.size() != kJointCount) {
            return false;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
            if (
                !std::isfinite(lower_limits_[index]) ||
                !std::isfinite(upper_limits_[index]) ||
                lower_limits_[index] >= upper_limits_[index] ||
                !std::isfinite(max_velocity_[index]) ||
                max_velocity_[index] <= 0.0) {
                return false;
            }
        }
        return true;
    }

    bool connect_robot() {
        if (dry_run_) {
            return true;
        }
        if (robot_ip_.empty()) {
            RCLCPP_ERROR(get_logger(), "robot_ip is empty");
            return false;
        }
#if defined(ARCH_ARM64)
        const errno_t result = robot_.login_in(robot_ip_.c_str());
#else
        const errno_t result = robot_.login_in(robot_ip_.c_str(), robot_port_);
#endif
        if (result != ERR_SUCC) {
            RCLCPP_ERROR(
                get_logger(),
                "Robot login failed for %s:%d, error=%d",
                robot_ip_.c_str(),
                robot_port_,
                result);
            return false;
        }
        RCLCPP_INFO(
            get_logger(),
            "Robot login succeeded. No power/enable command has been sent.");
        return true;
    }

    void set_motion_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request->data) {
            disarm_locked("operator request");
            response->success = true;
            response->message = "motion disabled and servo mode exited";
            return;
        }
        if (!connected_) {
            response->success = false;
            response->message = "robot is not connected";
            return;
        }
        if (!safety_configuration_valid_) {
            response->success = false;
            response->message =
                "real limits/max velocities are not configured";
            return;
        }
        if (!dry_run_ && !hardware_motion_authorized_) {
            response->success = false;
            response->message =
                "hardware_motion_authorized is false; receive/readback testing only";
            return;
        }

        if (!dry_run_) {
            if (power_on_on_arm_ && robot_.power_on() != ERR_SUCC) {
                response->success = false;
                response->message = "robot power_on failed";
                return;
            }
            if (enable_robot_on_arm_) {
                robot_.clear_error();
                if (robot_.enable_robot() != ERR_SUCC) {
                    response->success = false;
                    response->message = "robot enable failed";
                    return;
                }
            }
        }

        const auto feedback_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_feedback_received_).count();
        if (!feedback_valid_ || feedback_age > feedback_timeout_seconds_) {
            response->success = false;
            response->message = "no recent valid robot-status joint feedback";
            return;
        }
        if (
            !dry_run_ &&
            (!robot_powered_on_ || !robot_enabled_ ||
             robot_emergency_stop_ || robot_protective_stop_)) {
            response->success = false;
            response->message =
                "robot status is not safe for servo mode "
                "(power/enable/e-stop/protective-stop)";
            return;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
            current_command_[index] = latest_actual_[index];
            target_[index] = latest_actual_[index];
        }
        if (!positions_within_limits(current_command_)) {
            response->success = false;
            response->message =
                "current robot pose is outside configured safe limits";
            return;
        }
        if (robot_.servo_move_enable(TRUE) != ERR_SUCC) {
            response->success = false;
            response->message = "failed to enter servo mode";
            return;
        }
        motion_enabled_ = true;
        has_target_ = false;
        last_control_tick_ = std::chrono::steady_clock::now();
        response->success = true;
        response->message =
            dry_run_ ? "dry-run motion gate enabled" : "servo motion gate enabled";
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }

    void command_callback(
        const sensor_msgs::msg::JointState::SharedPtr message) {
        std::array<double, kJointCount> ordered{};
        std::string error;
        if (!extract_ordered_positions(*message, ordered, error)) {
            RCLCPP_ERROR(get_logger(), "Rejected command: %s", error.c_str());
            std::lock_guard<std::mutex> lock(mutex_);
            disarm_locked("invalid command message: " + error);
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_enabled_) {
            return;
        }
        if (!positions_within_limits(ordered)) {
            disarm_locked("received target outside configured joint limits");
            return;
        }
        target_ = ordered;
        has_target_ = true;
        last_command_received_ = std::chrono::steady_clock::now();
    }

    bool extract_ordered_positions(
        const sensor_msgs::msg::JointState &message,
        std::array<double, kJointCount> &output,
        std::string &error) const {
        if (message.position.size() != kJointCount) {
            error = "message must contain exactly seven positions";
            return false;
        }
        if (message.name.empty()) {
            std::copy(message.position.begin(), message.position.end(), output.begin());
        } else {
            if (message.name.size() != kJointCount) {
                error = "joint name and position lengths differ";
                return false;
            }
            std::unordered_map<std::string, double> lookup;
            for (std::size_t index = 0; index < kJointCount; ++index) {
                lookup[message.name[index]] = message.position[index];
            }
            for (std::size_t index = 0; index < kJointCount; ++index) {
                const auto item = lookup.find(joint_names_[index]);
                if (item == lookup.end()) {
                    error = "joint names do not match configured arm";
                    return false;
                }
                output[index] = item->second;
            }
        }
        for (double value : output) {
            if (!std::isfinite(value)) {
                error = "target contains NaN or infinity";
                return false;
            }
        }
        return true;
    }

    template <typename Container>
    bool positions_within_limits(const Container &positions) const {
        if (!safety_configuration_valid_) {
            return false;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
            if (
                positions[index] < lower_limits_[index] ||
                positions[index] > upper_limits_[index]) {
                return false;
            }
        }
        return true;
    }

    void control_tick() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!motion_enabled_ || !has_target_) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        const double age = std::chrono::duration<double>(
            now - last_command_received_).count();
        if (age > command_timeout_seconds_) {
            disarm_locked("command watchdog timeout");
            return;
        }
        double dt = std::chrono::duration<double>(now - last_control_tick_).count();
        last_control_tick_ = now;
        dt = std::clamp(dt, 0.0, 0.1);

        JointValue command{};
        for (std::size_t index = 0; index < kJointCount; ++index) {
            const double maximum_step = max_velocity_[index] * dt;
            const double delta = std::clamp(
                target_[index] - current_command_[index],
                -maximum_step,
                maximum_step);
            current_command_[index] += delta;
            command.jVal[index] = current_command_[index];
        }
        if (!positions_within_limits(current_command_)) {
            disarm_locked("rate-limited command crossed a joint limit");
            return;
        }
        errno_t result = ERR_SUCC;
#if defined(ARCH_ARM64)
        result = robot_.servo_j(&command, MoveMode::ABS, 1);
#else
        result = robot_.edg_servo_j(0, &command, MoveMode::ABS, 1);
        if (result == ERR_SUCC) {
            result = robot_.edg_send();
        }
#endif
        if (result != ERR_SUCC) {
            disarm_locked("JAKA absolute servo command returned an error");
        }
    }

    void state_tick() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) {
            return;
        }

        // RobotStatus carries the controller's monitored joint positions.
        // On the supplied x86_64 EDG SDK, get_joint_position() can report a
        // successful call while returning an all-zero legacy buffer.
        RobotStatus robot_status{};
        const errno_t status_result = robot_.get_robot_status(&robot_status);
        if (status_result != ERR_SUCC) {
            feedback_valid_ = false;
            RCLCPP_ERROR_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Robot-status read failed, error=%d",
                status_result);
            disarm_locked("actual joint-state feedback was lost");
            return;
        }

        std::array<double, kJointCount> actual{};
        for (std::size_t index = 0; index < kJointCount; ++index) {
            actual[index] = robot_status.joint_position[index];
            if (!std::isfinite(actual[index])) {
                feedback_valid_ = false;
                RCLCPP_ERROR_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    1000,
                    "Robot-status joint feedback contains NaN or infinity");
                disarm_locked("actual joint-state feedback was invalid");
                return;
            }
        }

        latest_actual_ = actual;
        last_feedback_received_ = std::chrono::steady_clock::now();
        feedback_valid_ = true;
        robot_powered_on_ = robot_status.powered_on != 0;
        robot_enabled_ = robot_status.enabled != 0;
        robot_emergency_stop_ = robot_status.emergency_stop != 0;
        robot_protective_stop_ = robot_status.protective_stop != 0;
        robot_on_soft_limit_ = robot_status.on_soft_limit != 0;
        robot_socket_connected_ = robot_status.is_socket_connect != 0;
        robot_error_code_ = robot_status.errcode;
        robot_drag_status_ = robot_status.drag_status;

        if (!feedback_source_logged_) {
            JointValue legacy_feedback{};
            const errno_t legacy_result =
                robot_.get_joint_position(&legacy_feedback);
            double maximum_difference = 0.0;
            if (legacy_result == ERR_SUCC) {
                for (std::size_t index = 0; index < kJointCount; ++index) {
                    maximum_difference = std::max(
                        maximum_difference,
                        std::abs(legacy_feedback.jVal[index] - actual[index]));
                }
            }
            RCLCPP_INFO(
                get_logger(),
                "Robot feedback source=get_robot_status; "
                "powered_on=%d enabled=%d emergency_stop=%d "
                "protective_stop=%d socket_connected=%d error_code=%d; "
                "joint_position=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]; "
                "legacy_get_joint_position_result=%d max_difference=%.6f",
                robot_powered_on_,
                robot_enabled_,
                robot_emergency_stop_,
                robot_protective_stop_,
                robot_socket_connected_,
                robot_error_code_,
                actual[0],
                actual[1],
                actual[2],
                actual[3],
                actual[4],
                actual[5],
                actual[6],
                legacy_result,
                maximum_difference);
            feedback_source_logged_ = true;
        }

        if (
            motion_enabled_ &&
            (!robot_powered_on_ || !robot_enabled_ ||
             robot_emergency_stop_ || robot_protective_stop_ ||
             robot_error_code_ != 0)) {
            disarm_locked("robot status became unsafe");
            return;
        }

        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name = joint_names_;
        for (std::size_t index = 0; index < kJointCount; ++index) {
            message.position.push_back(actual[index]);
        }
        state_pub_->publish(message);
    }

    void disarm(const std::string &reason) {
        std::lock_guard<std::mutex> lock(mutex_);
        disarm_locked(reason);
    }

    void disarm_locked(const std::string &reason) {
        if (motion_enabled_ && connected_) {
            robot_.servo_move_enable(FALSE);
            if (!dry_run_ && disable_robot_on_disarm_) {
                robot_.disable_robot();
            }
            RCLCPP_ERROR(get_logger(), "Motion disabled: %s", reason.c_str());
        }
        motion_enabled_ = false;
        has_target_ = false;
    }

    void publish_status() {
        std_msgs::msg::Bool enabled;
        enabled.data = motion_enabled_;
        enabled_pub_->publish(enabled);

        std_msgs::msg::String status;
        std::ostringstream stream;
        stream
            << "arm=" << arm_name_
            << ";dry_run=" << dry_run_
            << ";hardware_motion_authorized=" << hardware_motion_authorized_
            << ";connected=" << connected_
            << ";feedback_valid=" << feedback_valid_
            << ";feedback_source=get_robot_status"
            << ";robot_powered_on=" << robot_powered_on_
            << ";robot_enabled=" << robot_enabled_
            << ";robot_emergency_stop=" << robot_emergency_stop_
            << ";robot_protective_stop=" << robot_protective_stop_
            << ";robot_on_soft_limit=" << robot_on_soft_limit_
            << ";robot_socket_connected=" << robot_socket_connected_
            << ";robot_error_code=" << robot_error_code_
            << ";robot_drag_status=" << robot_drag_status_
            << ";limits_configured=" << safety_configuration_valid_
            << ";motion_enabled=" << motion_enabled_
            << ";has_target=" << has_target_;
        status.data = stream.str();
        status_pub_->publish(status);
    }

    Robot robot_;
    std::string arm_name_;
    std::string robot_ip_;
    int robot_port_{};
    bool dry_run_{true};
    bool hardware_motion_authorized_{false};
    bool limits_configured_{false};
    bool safety_configuration_valid_{false};
    bool power_on_on_arm_{false};
    bool enable_robot_on_arm_{false};
    bool disable_robot_on_disarm_{false};
    bool connected_{false};
    bool feedback_valid_{false};
    bool feedback_source_logged_{false};
    bool robot_powered_on_{false};
    bool robot_enabled_{false};
    bool robot_emergency_stop_{false};
    bool robot_protective_stop_{false};
    bool robot_on_soft_limit_{false};
    bool robot_socket_connected_{false};
    bool motion_enabled_{false};
    bool has_target_{false};
    int robot_error_code_{0};
    int robot_drag_status_{0};
    double command_timeout_seconds_{0.30};
    double feedback_timeout_seconds_{0.30};
    double control_rate_hz_{125.0};
    double state_rate_hz_{20.0};
    std::vector<std::string> joint_names_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    std::vector<double> max_velocity_;
    std::array<double, kJointCount> target_{};
    std::array<double, kJointCount> current_command_{};
    std::array<double, kJointCount> latest_actual_{};
    std::chrono::steady_clock::time_point last_command_received_{};
    std::chrono::steady_clock::time_point last_control_tick_{};
    std::chrono::steady_clock::time_point last_feedback_received_{};
    std::mutex mutex_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_service_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeOneArmServo>());
    rclcpp::shutdown();
    return 0;
}
