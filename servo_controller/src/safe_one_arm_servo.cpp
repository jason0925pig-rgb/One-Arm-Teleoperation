#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot.h"
#include "safety_slew_limiter.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
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
        hardware_power_authorized_ =
            declare_parameter<bool>("hardware_power_authorized", false);
        hardware_enable_authorized_ =
            declare_parameter<bool>("hardware_enable_authorized", false);
        hardware_drag_authorized_ =
            declare_parameter<bool>("hardware_drag_authorized", false);
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
        control_deadline_warning_factor_ =
            declare_parameter<double>("control_deadline_warning_factor", 1.5);
        control_deadline_abort_seconds_ =
            declare_parameter<double>("control_deadline_abort_seconds", 0.05);
        control_deadline_abort_consecutive_misses_ =
            declare_parameter<int>(
                "control_deadline_abort_consecutive_misses", 2);
        require_single_command_publisher_ =
            declare_parameter<bool>("require_single_command_publisher", true);
        lower_limits_ =
            declare_parameter<std::vector<double>>(
                "joint_lower_limits", std::vector<double>{});
        upper_limits_ =
            declare_parameter<std::vector<double>>(
                "joint_upper_limits", std::vector<double>{});
        max_velocity_ =
            declare_parameter<std::vector<double>>(
                "max_velocity_rad_s", std::vector<double>{});
        max_acceleration_ =
            declare_parameter<std::vector<double>>(
                "max_acceleration_rad_s2", std::vector<double>{});

        for (int index = 1; index <= 7; ++index) {
            joint_names_.push_back(
                arm_name_ + "_joint" + std::to_string(index));
        }
        safety_configuration_valid_ = validate_safety_configuration();

        const std::string prefix = "/" + arm_name_ + "_arm";
        command_topic_ = prefix + "/teleop_joint_command";
        command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            command_topic_,
            10,
            std::bind(
                &SafeOneArmServo::command_callback,
                this,
                std::placeholders::_1));
        stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/teleop/stop_request",
            10,
            std::bind(
                &SafeOneArmServo::stop_callback,
                this,
                std::placeholders::_1));
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            prefix + "/joint_states", 10);
        executed_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            prefix + "/executed_joint_command", 10);
        enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/motion_enabled", 10);
        powered_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/powered_on", 10);
        robot_enabled_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/robot_enabled", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            prefix + "/safety_status", 10);
        power_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_powered_on",
            std::bind(
                &SafeOneArmServo::set_powered_on,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        enable_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_robot_enabled",
            std::bind(
                &SafeOneArmServo::set_robot_enabled,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        drag_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_drag_enabled",
            std::bind(
                &SafeOneArmServo::set_drag_enabled,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        motion_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_motion_enabled",
            std::bind(
                &SafeOneArmServo::set_motion_enabled,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        reconnect_service_ = create_service<std_srvs::srv::Trigger>(
            prefix + "/reconnect",
            std::bind(
                &SafeOneArmServo::reconnect_robot,
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
        if (power_on_on_arm_) {
            RCLCPP_ERROR(
                get_logger(),
                "power_on_on_arm is deprecated and ignored. Use the explicit "
                "%s/set_powered_on service with hardware_power_authorized=true.",
                prefix.c_str());
        }
        if (enable_robot_on_arm_) {
            RCLCPP_ERROR(
                get_logger(),
                "enable_robot_on_arm is deprecated and ignored. Use the explicit "
                "%s/set_robot_enabled service with hardware_enable_authorized=true.",
                prefix.c_str());
        }
        if (!safety_configuration_valid_) {
            RCLCPP_ERROR(
                get_logger(),
                "Motion is locked: actual seven-joint limits, velocities, and "
                "accelerations have not been configured.");
        }
    }

    ~SafeOneArmServo() override {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (
                connected_ && !dry_run_ &&
                (drag_mode_requested_ || robot_drag_status_ != 0)) {
                const errno_t result = robot_.drag_mode_enable(FALSE);
                if (result != ERR_SUCC) {
                    RCLCPP_ERROR(
                        get_logger(),
                        "Failed to exit drag mode during node shutdown, error=%d",
                        result);
                }
            }
            drag_mode_requested_ = false;
        }
        disarm("node shutdown");
        if (sdk_session_open_) {
            robot_.login_out();
        }
    }

private:
    static constexpr std::size_t kJointCount = 7;

    bool validate_safety_configuration() {
        if (!limits_configured_) {
            return false;
        }
        if (
            !std::isfinite(control_deadline_warning_factor_) ||
            control_deadline_warning_factor_ <= 1.0 ||
            !std::isfinite(control_deadline_abort_seconds_) ||
            control_deadline_abort_seconds_ <= 0.0 ||
            control_deadline_abort_consecutive_misses_ <= 0) {
            return false;
        }
        if (
            lower_limits_.size() != kJointCount ||
            upper_limits_.size() != kJointCount ||
            max_velocity_.size() != kJointCount ||
            max_acceleration_.size() != kJointCount) {
            return false;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
            if (
                !std::isfinite(lower_limits_[index]) ||
                !std::isfinite(upper_limits_[index]) ||
                lower_limits_[index] >= upper_limits_[index] ||
                !std::isfinite(max_velocity_[index]) ||
                max_velocity_[index] <= 0.0 ||
                !std::isfinite(max_acceleration_[index]) ||
                max_acceleration_[index] <= 0.0) {
                return false;
            }
        }
        return true;
    }

    bool connect_robot() {
        if (dry_run_) {
            sdk_session_open_ = true;
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
        sdk_session_open_ = true;
        return true;
    }

    void reconnect_robot(
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (
            motion_enabled_ || servo_mode_entered_ ||
            robot_enabled_ || robot_powered_on_) {
            response->success = false;
            response->message =
                "reconnect is allowed only while servo mode, motion, robot "
                "enable, and robot power are all off";
            return;
        }
        disarm_locked("explicit reconnect requested");
        feedback_valid_ = false;
        feedback_source_logged_ = false;
        if (sdk_session_open_) {
            const errno_t logout_result = robot_.login_out();
            if (logout_result != ERR_SUCC) {
                connected_ = false;
                response->success = false;
                response->message =
                    "SDK login_out failed, error=" +
                    std::to_string(logout_result) +
                    "; restart the node instead of retrying automatically";
                return;
            }
            sdk_session_open_ = false;
        }
        connected_ = false;
        connected_ = connect_robot();
        response->success = connected_;
        response->message = connected_
            ? "SDK reconnected; no power, enable, servo, or motion call was made"
            : "SDK reconnect failed; node remains fail-stopped";
    }

    void set_powered_on(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) {
            response->success = false;
            response->message = "robot is not connected";
            return;
        }
        if (dry_run_) {
            response->success = true;
            response->message = request->data
                ? "dry-run power-on accepted; no hardware call was made"
                : "dry-run power-off accepted; no hardware call was made";
            return;
        }

        const auto feedback_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_feedback_received_).count();
        if (!feedback_valid_ || feedback_age > feedback_timeout_seconds_) {
            response->success = false;
            response->message =
                "no recent valid robot status; refusing to change power";
            return;
        }
        if (!robot_socket_connected_) {
            response->success = false;
            response->message =
                "robot status reports the controller socket is disconnected";
            return;
        }

        if (!request->data) {
            if (
                motion_enabled_ || servo_mode_entered_ || robot_enabled_ ||
                drag_mode_requested_ || robot_drag_status_ != 0) {
                response->success = false;
                response->message =
                    "refusing power_off while drag, servo, motion, or robot "
                    "enable is active";
                return;
            }
            if (!robot_powered_on_) {
                response->success = true;
                response->message = "robot is already powered off";
                return;
            }
            const errno_t result = robot_.power_off();
            if (result != ERR_SUCC) {
                response->success = false;
                response->message =
                    "robot power_off failed, error=" + std::to_string(result);
                return;
            }
            response->success = true;
            response->message =
                "power_off accepted; verify /right_arm/safety_status";
            RCLCPP_WARN(
                get_logger(),
                "Operator requested power_off. No enable or servo call was made.");
            return;
        }

        if (!hardware_power_authorized_) {
            response->success = false;
            response->message =
                "hardware_power_authorized is false; power-on is locked";
            return;
        }
        if (motion_enabled_ || servo_mode_entered_ || robot_enabled_) {
            response->success = false;
            response->message =
                "robot is already enabled; power-only state cannot be guaranteed";
            return;
        }
        if (robot_emergency_stop_ || robot_protective_stop_) {
            response->success = false;
            response->message =
                "e-stop or protective stop is active; refusing power_on";
            return;
        }
        if (robot_error_code_ != 0) {
            response->success = false;
            response->message =
                "robot error is present; refusing power_on without clearing it";
            return;
        }
        if (robot_powered_on_) {
            response->success = true;
            response->message =
                "robot is already powered on and remains not enabled";
            return;
        }

        const errno_t result = robot_.power_on();
        if (result != ERR_SUCC) {
            response->success = false;
            response->message =
                "robot power_on failed, error=" + std::to_string(result);
            return;
        }
        response->success = true;
        response->message =
            "power_on accepted; no clear-error, enable, servo, or motion call "
            "was made; verify /right_arm/safety_status";
        RCLCPP_WARN(
            get_logger(),
            "Operator requested power_on only. No clear-error, enable, servo, "
            "or motion call was made.");
    }

    void set_robot_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_) {
            response->success = false;
            response->message = "robot is not connected";
            return;
        }
        if (dry_run_) {
            response->success = true;
            response->message = request->data
                ? "dry-run robot-enable accepted; no hardware call was made"
                : "dry-run robot-disable accepted; no hardware call was made";
            return;
        }

        const auto feedback_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_feedback_received_).count();
        if (!feedback_valid_ || feedback_age > feedback_timeout_seconds_) {
            response->success = false;
            response->message =
                "no recent valid robot status; refusing to change robot enable";
            return;
        }
        if (!robot_socket_connected_) {
            response->success = false;
            response->message =
                "robot status reports the controller socket is disconnected";
            return;
        }

        if (!request->data) {
            disarm_locked("robot disable requested");
            if (servo_mode_entered_) {
                response->success = false;
                response->message =
                    "SDK did not confirm servo-mode exit; robot disable was not sent";
                return;
            }
            if (drag_mode_requested_ || robot_drag_status_ != 0) {
                const errno_t drag_result = robot_.drag_mode_enable(FALSE);
                if (drag_result != ERR_SUCC) {
                    response->success = false;
                    response->message =
                        "failed to exit drag mode before robot disable, error=" +
                        std::to_string(drag_result);
                    return;
                }
                drag_mode_requested_ = false;
            }
            if (!robot_enabled_) {
                response->success = true;
                response->message = "robot is already disabled";
                return;
            }
            const errno_t result = robot_.disable_robot();
            if (result != ERR_SUCC) {
                response->success = false;
                response->message =
                    "robot disable failed, error=" + std::to_string(result);
                return;
            }
            response->success = true;
            response->message =
                "robot disable accepted; verify /right_arm/robot_enabled";
            RCLCPP_WARN(
                get_logger(),
                "Operator requested robot disable after leaving servo mode.");
            return;
        }

        if (!hardware_enable_authorized_) {
            response->success = false;
            response->message =
                "hardware_enable_authorized is false; robot enable is locked";
            return;
        }
        if (!safety_configuration_valid_) {
            response->success = false;
            response->message =
                "real limits, velocities, and accelerations are not configured";
            return;
        }
        if (motion_enabled_ || servo_mode_entered_) {
            response->success = false;
            response->message =
                "motion or servo mode is already active; enable state cannot be changed";
            return;
        }
        if (!robot_powered_on_) {
            response->success = false;
            response->message = "robot is not powered on";
            return;
        }
        if (
            robot_emergency_stop_ || robot_protective_stop_ ||
            robot_on_soft_limit_ || robot_error_code_ != 0) {
            response->success = false;
            response->message =
                "robot status is unsafe; no clear-error command was sent";
            return;
        }
        if (robot_enabled_) {
            response->success = true;
            response->message = "robot is already enabled";
            return;
        }

        const errno_t result = robot_.enable_robot();
        if (result != ERR_SUCC) {
            response->success = false;
            response->message =
                "robot enable failed, error=" + std::to_string(result);
            return;
        }
        response->success = true;
        response->message =
            "robot enable accepted; no clear-error, servo, or motion call was "
            "made; verify /right_arm/robot_enabled";
        RCLCPP_WARN(
            get_logger(),
            "Operator requested robot enable only. No clear-error, servo, or "
            "motion call was made.");
    }

    void set_drag_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request->data) {
            if (dry_run_) {
                drag_mode_requested_ = false;
                response->success = true;
                response->message =
                    "dry-run drag disable accepted; no hardware call was made";
                return;
            }
            if (!connected_) {
                response->success = false;
                response->message = "robot is not connected";
                return;
            }
            if (!drag_mode_requested_ && robot_drag_status_ == 0) {
                response->success = true;
                response->message = "robot is already outside drag mode";
                return;
            }
            const errno_t result = robot_.drag_mode_enable(FALSE);
            if (result != ERR_SUCC) {
                response->success = false;
                response->message =
                    "failed to exit drag mode, error=" + std::to_string(result);
                return;
            }
            drag_mode_requested_ = false;
            response->success = true;
            response->message =
                "drag disable accepted; verify robot_drag_status=0";
            RCLCPP_WARN(get_logger(), "Operator requested drag mode off.");
            return;
        }

        if (!connected_) {
            response->success = false;
            response->message = "robot is not connected";
            return;
        }
        if (!hardware_drag_authorized_) {
            response->success = false;
            response->message =
                "hardware_drag_authorized is false; drag mode is locked";
            return;
        }
        if (dry_run_) {
            drag_mode_requested_ = true;
            response->success = true;
            response->message =
                "dry-run drag enable accepted; no hardware call was made";
            return;
        }
        const auto feedback_age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_feedback_received_).count();
        if (!feedback_valid_ || feedback_age > feedback_timeout_seconds_) {
            response->success = false;
            response->message =
                "no recent valid robot status; refusing drag mode";
            return;
        }
        if (
            !robot_socket_connected_ || !robot_powered_on_ || !robot_enabled_ ||
            robot_emergency_stop_ || robot_protective_stop_ ||
            robot_on_soft_limit_ || robot_error_code_ != 0) {
            response->success = false;
            response->message = "robot status is not safe for drag mode";
            return;
        }
        if (motion_enabled_ || servo_mode_entered_) {
            response->success = false;
            response->message =
                "motion/servo mode must be disabled before drag mode";
            return;
        }
        if (drag_mode_requested_ || robot_drag_status_ != 0) {
            response->success = true;
            response->message = "robot is already in drag mode";
            return;
        }
        const errno_t result = robot_.drag_mode_enable(TRUE);
        if (result != ERR_SUCC) {
            response->success = false;
            response->message =
                "failed to enter drag mode, error=" + std::to_string(result);
            return;
        }
        drag_mode_requested_ = true;
        response->success = true;
        response->message =
            "drag enable accepted; verify robot_drag_status=1";
        RCLCPP_WARN(
            get_logger(),
            "Operator enabled drag mode. Keep clear of the arm and support "
            "the payload while repositioning.");
    }

    void set_motion_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request->data) {
            disarm_locked("operator request");
            response->success = !servo_mode_entered_;
            response->message = servo_mode_entered_
                ? "motion gate closed but SDK failed to confirm servo-mode exit"
                : "motion disabled and servo mode exited";
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
        if (!dry_run_ && require_single_command_publisher_) {
            const std::size_t publisher_count = count_publishers(command_topic_);
            if (publisher_count != 1) {
                response->success = false;
                response->message =
                    "expected exactly one teleop command publisher, found " +
                    std::to_string(publisher_count);
                return;
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
             robot_emergency_stop_ || robot_protective_stop_ ||
             robot_on_soft_limit_ || robot_error_code_ != 0)) {
            response->success = false;
            response->message =
                "robot status is not safe for servo mode "
                "(power/enable/e-stop/protective-stop)";
            return;
        }
        if (!dry_run_ && (drag_mode_requested_ || robot_drag_status_ != 0)) {
            response->success = false;
            response->message =
                "drag mode must be disabled before servo motion";
            return;
        }
        for (std::size_t index = 0; index < kJointCount; ++index) {
            current_command_[index] = latest_actual_[index];
            target_[index] = latest_actual_[index];
            command_velocity_[index] = 0.0;
        }
        if (!positions_within_limits(current_command_)) {
            response->success = false;
            response->message =
                "current robot pose is outside configured safe limits";
            return;
        }
        if (servo_mode_entered_) {
            response->success = false;
            response->message =
                "servo mode is still marked active from an earlier session";
            return;
        }
        if (robot_.servo_move_enable(TRUE) != ERR_SUCC) {
            response->success = false;
            response->message = "failed to enter servo mode";
            return;
        }
        servo_mode_entered_ = true;
        motion_enabled_ = true;
        has_target_ = false;
        consecutive_control_deadline_aborts_ = 0;
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

    void stop_callback(const std_msgs::msg::Bool::SharedPtr message) {
        if (!message->data) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        disarm_locked("teleop STOP request");
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
        const auto now = std::chrono::steady_clock::now();
        const double expected_period = 1.0 / std::max(1.0, control_rate_hz_);
        double callback_period = expected_period;
        if (last_control_callback_.time_since_epoch().count() != 0) {
            callback_period = std::chrono::duration<double>(
                now - last_control_callback_).count();
        }
        last_control_callback_ = now;
        ++control_tick_count_;
        last_control_period_seconds_ = callback_period;
        max_control_period_seconds_ =
            std::max(max_control_period_seconds_, callback_period);
        if (callback_period > expected_period * control_deadline_warning_factor_) {
            ++control_deadline_misses_;
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Control timer deadline miss: actual=%.6fs expected=%.6fs "
                "total_misses=%llu",
                callback_period,
                expected_period,
                static_cast<unsigned long long>(control_deadline_misses_));
        }
        if (
            motion_enabled_ &&
            has_target_ &&
            callback_period > control_deadline_abort_seconds_) {
            ++consecutive_control_deadline_aborts_;
            RCLCPP_ERROR(
                get_logger(),
                "Severe control deadline miss %d/%d: actual=%.6fs "
                "abort_threshold=%.6fs",
                consecutive_control_deadline_aborts_,
                control_deadline_abort_consecutive_misses_,
                callback_period,
                control_deadline_abort_seconds_);
            if (
                consecutive_control_deadline_aborts_ >=
                control_deadline_abort_consecutive_misses_) {
                disarm_locked(
                    "consecutive severe control-loop deadline misses");
                return;
            }
        } else {
            consecutive_control_deadline_aborts_ = 0;
        }
        if (!motion_enabled_ || !has_target_) {
            return;
        }
        const double feedback_age = std::chrono::duration<double>(
            now - last_feedback_received_).count();
        if (!feedback_valid_ || feedback_age > feedback_timeout_seconds_) {
            disarm_locked("feedback watchdog timeout");
            return;
        }
        const double age = std::chrono::duration<double>(
            now - last_command_received_).count();
        if (age > command_timeout_seconds_) {
            disarm_locked("command watchdog timeout");
            return;
        }
        double dt = std::chrono::duration<double>(now - last_control_tick_).count();
        last_control_tick_ = now;
        dt = std::clamp(dt, 0.000001, std::min(0.1, expected_period * 2.0));

        JointValue command{};
        for (std::size_t index = 0; index < kJointCount; ++index) {
            const auto step = one_arm_safety::acceleration_limited_step(
                current_command_[index],
                command_velocity_[index],
                target_[index],
                max_velocity_[index],
                max_acceleration_[index],
                dt);
            current_command_[index] = step.position;
            command_velocity_[index] = step.velocity;
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
            return;
        }

        // Publish the exact post-slew-limit joint target accepted by the SDK.
        // The upstream teleop_joint_command is only the desired target and can
        // differ from this value while velocity/acceleration limiting is active.
        sensor_msgs::msg::JointState executed;
        executed.header.stamp = this->now();
        executed.name = joint_names_;
        executed.position.assign(
            current_command_.begin(), current_command_.end());
        executed.velocity.assign(
            command_velocity_.begin(), command_velocity_.end());
        executed_command_pub_->publish(executed);
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
            ++consecutive_feedback_failures_;
            const double feedback_age = std::chrono::duration<double>(
                std::chrono::steady_clock::now() -
                last_feedback_received_).count();
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Robot-status read failed, error=%d, consecutive=%llu, "
                "last_valid_age=%.3fs (timeout=%.3fs)",
                status_result,
                static_cast<unsigned long long>(
                    consecutive_feedback_failures_),
                feedback_age,
                feedback_timeout_seconds_);
            if (!feedback_valid_ || feedback_age > feedback_timeout_seconds_) {
                feedback_valid_ = false;
                disarm_locked("actual joint-state feedback timeout");
                connected_ = false;
            }
            return;
        }
        consecutive_feedback_failures_ = 0;

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

        if (!robot_socket_connected_) {
            feedback_valid_ = false;
            disarm_locked("controller socket disconnected");
            connected_ = false;
            RCLCPP_ERROR(
                get_logger(),
                "Controller socket disconnected. Node is fail-stopped; use the "
                "guarded reconnect service only after power/enable are confirmed off.");
            return;
        }

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
#if !defined(ARCH_ARM64)
            JointValue edg_feedback_0{};
            CartesianPose edg_pose_0{};
            const errno_t edg_result_0 =
                robot_.edg_get_stat(0, &edg_feedback_0, &edg_pose_0);
            RCLCPP_INFO(
                get_logger(),
                "Read-only EDG feedback comparison: "
                "index0_result=%d index0_joint=[%.6f, %.6f, %.6f, %.6f, %.6f, %.6f, %.6f]. "
                "No EDG send/servo/power/enable call was made.",
                edg_result_0,
                edg_feedback_0.jVal[0],
                edg_feedback_0.jVal[1],
                edg_feedback_0.jVal[2],
                edg_feedback_0.jVal[3],
                edg_feedback_0.jVal[4],
                edg_feedback_0.jVal[5],
                edg_feedback_0.jVal[6]);
#endif
            feedback_source_logged_ = true;
        }

        if (
            motion_enabled_ &&
             (!robot_powered_on_ || !robot_enabled_ ||
              robot_emergency_stop_ || robot_protective_stop_ ||
              robot_on_soft_limit_ || robot_error_code_ != 0)) {
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
        if (servo_mode_entered_ && connected_) {
            const errno_t servo_result = robot_.servo_move_enable(FALSE);
            if (servo_result == ERR_SUCC) {
                servo_mode_entered_ = false;
                last_servo_disable_error_ = 0;
            } else {
                last_servo_disable_error_ = servo_result;
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to exit servo mode, error=%d; physical e-stop may "
                    "be required.",
                    servo_result);
            }
            if (!dry_run_ && disable_robot_on_disarm_) {
                robot_.disable_robot();
            }
        }
        if (motion_enabled_) {
            RCLCPP_ERROR(get_logger(), "Motion disabled: %s", reason.c_str());
        }
        motion_enabled_ = false;
        has_target_ = false;
        consecutive_control_deadline_aborts_ = 0;
        command_velocity_.fill(0.0);
    }

    void publish_status() {
        std_msgs::msg::Bool enabled;
        enabled.data = motion_enabled_;
        enabled_pub_->publish(enabled);

        std_msgs::msg::Bool powered;
        powered.data = robot_powered_on_;
        powered_pub_->publish(powered);

        std_msgs::msg::Bool robot_enabled;
        robot_enabled.data = robot_enabled_;
        robot_enabled_pub_->publish(robot_enabled);

        std_msgs::msg::String status;
        std::ostringstream stream;
        stream
            << "arm=" << arm_name_
             << ";dry_run=" << dry_run_
             << ";hardware_power_authorized=" << hardware_power_authorized_
             << ";hardware_enable_authorized=" << hardware_enable_authorized_
             << ";hardware_drag_authorized=" << hardware_drag_authorized_
             << ";hardware_motion_authorized=" << hardware_motion_authorized_
            << ";connected=" << connected_
            << ";sdk_session_open=" << sdk_session_open_
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
            << ";drag_mode_requested=" << drag_mode_requested_
             << ";limits_configured=" << safety_configuration_valid_
             << ";motion_enabled=" << motion_enabled_
             << ";servo_mode_entered=" << servo_mode_entered_
             << ";last_servo_disable_error=" << last_servo_disable_error_
             << ";has_target=" << has_target_
             << ";control_tick_count=" << control_tick_count_
             << ";control_deadline_misses=" << control_deadline_misses_
             << ";consecutive_control_deadline_aborts="
             << consecutive_control_deadline_aborts_
             << ";last_control_period_s=" << last_control_period_seconds_
             << ";max_control_period_s=" << max_control_period_seconds_;
        status.data = stream.str();
        status_pub_->publish(status);
    }

    Robot robot_;
    std::string arm_name_;
    std::string robot_ip_;
    std::string command_topic_;
    int robot_port_{};
    bool dry_run_{true};
    bool hardware_power_authorized_{false};
    bool hardware_enable_authorized_{false};
    bool hardware_drag_authorized_{false};
    bool hardware_motion_authorized_{false};
    bool limits_configured_{false};
    bool safety_configuration_valid_{false};
    bool power_on_on_arm_{false};
    bool enable_robot_on_arm_{false};
    bool disable_robot_on_disarm_{false};
    bool require_single_command_publisher_{true};
    bool connected_{false};
    bool sdk_session_open_{false};
    bool feedback_valid_{false};
    bool feedback_source_logged_{false};
    bool robot_powered_on_{false};
    bool robot_enabled_{false};
    bool robot_emergency_stop_{false};
    bool robot_protective_stop_{false};
    bool robot_on_soft_limit_{false};
    bool robot_socket_connected_{false};
    bool motion_enabled_{false};
    bool servo_mode_entered_{false};
    bool drag_mode_requested_{false};
    bool has_target_{false};
    int robot_error_code_{0};
    int robot_drag_status_{0};
    int last_servo_disable_error_{0};
    double command_timeout_seconds_{0.30};
    double feedback_timeout_seconds_{0.30};
    double control_rate_hz_{125.0};
    double state_rate_hz_{20.0};
    double control_deadline_warning_factor_{1.5};
    double control_deadline_abort_seconds_{0.05};
    int control_deadline_abort_consecutive_misses_{2};
    int consecutive_control_deadline_aborts_{0};
    std::vector<std::string> joint_names_;
    std::vector<double> lower_limits_;
    std::vector<double> upper_limits_;
    std::vector<double> max_velocity_;
    std::vector<double> max_acceleration_;
    std::array<double, kJointCount> target_{};
    std::array<double, kJointCount> current_command_{};
    std::array<double, kJointCount> command_velocity_{};
    std::array<double, kJointCount> latest_actual_{};
    std::chrono::steady_clock::time_point last_command_received_{};
    std::chrono::steady_clock::time_point last_control_tick_{};
    std::chrono::steady_clock::time_point last_control_callback_{};
    std::chrono::steady_clock::time_point last_feedback_received_{};
    std::uint64_t control_tick_count_{0};
    std::uint64_t consecutive_feedback_failures_{0};
    std::uint64_t control_deadline_misses_{0};
    double last_control_period_seconds_{0.0};
    double max_control_period_seconds_{0.0};
    std::mutex mutex_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
        executed_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr powered_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_enabled_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr drag_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr motion_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reconnect_service_;
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
