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
        limits_configured_ = declare_parameter<bool>("limits_configured", false);
        power_on_on_arm_ = declare_parameter<bool>("power_on_on_arm", false);
        enable_robot_on_arm_ = declare_parameter<bool>("enable_robot_on_arm", false);
        disable_robot_on_disarm_ =
            declare_parameter<bool>("disable_robot_on_disarm", false);
        command_timeout_seconds_ =
            declare_parameter<double>("command_timeout_seconds", 0.30);
        control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
        state_rate_hz_ = declare_parameter<double>("state_rate_hz", 20.0);
        lower_limits_ =
            declare_parameter<std::vector<double>>("joint_lower_limits", {});
        upper_limits_ =
            declare_parameter<std::vector<double>>("joint_upper_limits", {});
        max_velocity_ =
            declare_parameter<std::vector<double>>("max_velocity_rad_s", {});

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

        const auto control_period = std::chrono::duration<double>(
            1.0 / std::max(1.0, control_rate_hz_));
        const auto state_period = std::chrono::duration<double>(
            1.0 / std::max(1.0, state_rate_hz_));
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(control_period),
            std::bind(&SafeOneArmServo::control_tick, this));
        state_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(state_period),
            std::bind(&SafeOneArmServo::state_tick, this));
        status_timer_ = create_wall_timer(
            500ms, std::bind(&SafeOneArmServo::publish_status, this));

        RCLCPP_WARN(
            get_logger(),
            "Safe one-arm node started: arm=%s dry_run=%d connected=%d. "
            "It does not power, enable, or enter servo mode at startup.",
            arm_name_.c_str(),
            dry_run_,
            connected_);
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

        JointValue actual{};
        if (robot_.get_joint_position(&actual) != ERR_SUCC) {
            response->success = false;
            response->message = "cannot read current joint position";
            return;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
            current_command_[index] = actual.jVal[index];
            target_[index] = actual.jVal[index];
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
        result = robot_.servo_j(&command, MoveMode::ABS);
#else
        result = robot_.edg_servo_j(0, &command, MoveMode::ABS);
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
        JointValue actual{};
        if (robot_.get_joint_position(&actual) != ERR_SUCC) {
            RCLCPP_ERROR(get_logger(), "Actual joint-state read failed");
            disarm_locked("actual joint-state feedback was lost");
            return;
        }
        sensor_msgs::msg::JointState message;
        message.header.stamp = now();
        message.name = joint_names_;
        for (std::size_t index = 0; index < kJointCount; ++index) {
            message.position.push_back(actual.jVal[index]);
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
            << ";connected=" << connected_
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
    bool limits_configured_{false};
    bool safety_configuration_valid_{false};
    bool power_on_on_arm_{false};
    bool enable_robot_on_arm_{false};
    bool disable_robot_on_disarm_{false};
    bool connected_{false};
    bool motion_enabled_{false};
    bool has_target_{false};
    double command_timeout_seconds_{0.30};
    double control_rate_hz_{50.0};
    double state_rate_hz_{20.0};
    std::vector<std::string> joint_names_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    std::vector<double> max_velocity_;
    std::array<double, kJointCount> target_{};
    std::array<double, kJointCount> current_command_{};
    std::chrono::steady_clock::time_point last_command_received_{};
    std::chrono::steady_clock::time_point last_control_tick_{};
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
