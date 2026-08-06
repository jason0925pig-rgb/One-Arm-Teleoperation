#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "DH_gripper.hpp"
#include "ZX_gripper.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class SafeGripperController : public rclcpp::Node {
public:
    SafeGripperController() : Node("safe_gripper_controller") {
        arm_name_ = declare_parameter<std::string>("arm_name", "right");
        gripper_type_ = declare_parameter<std::string>("gripper_type", "none");
        gripper_model_ =
            declare_parameter<std::string>("gripper_model", "unknown");
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        slave_id_ = declare_parameter<int>("slave_id", 1);
        baudrate_ = declare_parameter<int>("baudrate", 115200);
        dry_run_ = declare_parameter<bool>("dry_run", true);
        configuration_complete_ =
            declare_parameter<bool>("configuration_complete", false);
        open_position_ = declare_parameter<int>("open_position", 0);
        closed_position_ = declare_parameter<int>("closed_position", 0);
        speed_ = declare_parameter<int>("speed", 20);
        force_ = declare_parameter<int>("force", 20);
        open_speed_ = declare_parameter<int>("open_speed", speed_);
        open_force_ = declare_parameter<int>("open_force", force_);
        closed_speed_ = declare_parameter<int>("closed_speed", speed_);
        closed_force_ = declare_parameter<int>("closed_force", force_);
        allowed_alarm_mask_ =
            declare_parameter<int>("allowed_alarm_mask", 0);
        movement_timeout_seconds_ =
            declare_parameter<double>("movement_timeout_seconds", 2.0);
        stop_on_movement_timeout_ =
            declare_parameter<bool>("stop_on_movement_timeout", true);

        const std::string prefix = "/" + arm_name_ + "_arm";
        command_sub_ = create_subscription<std_msgs::msg::Bool>(
            prefix + "/gripper_command",
            10,
            std::bind(
                &SafeGripperController::command_callback,
                this,
                std::placeholders::_1));
        stop_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/teleop/stop_request",
            10,
            std::bind(
                &SafeGripperController::stop_callback,
                this,
                std::placeholders::_1));
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            prefix + "/gripper_state", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            prefix + "/gripper_status", 10);
        feedback_valid_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/gripper_feedback_valid", 10);
        contact_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/gripper_contact", 10);
        executed_command_pub_ = create_publisher<std_msgs::msg::Bool>(
            prefix + "/executed_gripper_command", 10);
        enable_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_gripper_enabled",
            std::bind(
                &SafeGripperController::set_enabled,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        open_service_ = create_service<std_srvs::srv::SetBool>(
            prefix + "/set_gripper_open",
            std::bind(
                &SafeGripperController::set_open,
                this,
                std::placeholders::_1,
                std::placeholders::_2));
        poll_timer_ = create_wall_timer(
            50ms, std::bind(&SafeGripperController::poll_state, this));

        RCLCPP_WARN(
            get_logger(),
            "Gripper node started disabled: type=%s model=%s "
            "dry_run=%d configured=%d",
            gripper_type_.c_str(),
            gripper_model_.c_str(),
            dry_run_,
            configuration_complete_);
    }

    ~SafeGripperController() override {
        disable_hardware();
    }

private:
    bool configuration_valid(std::string &reason) const {
        if (!configuration_complete_) {
            reason = "configuration_complete is false";
            return false;
        }
        if (gripper_type_ != "dh" && gripper_type_ != "zx") {
            reason = "gripper_type must be dh or zx";
            return false;
        }
        if (open_position_ == closed_position_) {
            reason = "open and closed positions must differ";
            return false;
        }
        if (speed_ < 0 || speed_ > 100 || force_ < 0 || force_ > 100 ||
            open_speed_ < 0 || open_speed_ > 100 ||
            open_force_ < 0 || open_force_ > 100 ||
            closed_speed_ < 0 || closed_speed_ > 100 ||
            closed_force_ < 0 || closed_force_ > 100) {
            reason = "all speed and force parameters must be within 0..100";
            return false;
        }
        if (allowed_alarm_mask_ < 0 || allowed_alarm_mask_ > 0xFFFF) {
            reason = "allowed_alarm_mask must be within 0..65535";
            return false;
        }
        return true;
    }

    bool zx_alarm_allowed(uint16_t alarm, std::string &reason) const {
        const auto disallowed =
            static_cast<uint16_t>(alarm & ~static_cast<uint16_t>(allowed_alarm_mask_));
        if (disallowed == 0) {
            return true;
        }
        reason =
            "ZX gripper has disallowed alarm bits: alarm=" +
            std::to_string(alarm) +
            " allowed_mask=" + std::to_string(allowed_alarm_mask_);
        return false;
    }

    void set_enabled(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!request->data) {
            enabled_ = false;
            moving_ = false;
            disable_hardware();
            response->success = true;
            response->message = "gripper disabled";
            return;
        }
        std::string reason;
        if (!configuration_valid(reason)) {
            response->success = false;
            response->message = reason;
            return;
        }
        if (!dry_run_ && !initialize_hardware(reason)) {
            response->success = false;
            response->message = reason;
            return;
        }
        has_requested_state_ = false;
        enabled_ = true;
        response->success = true;
        response->message =
            dry_run_ ? "dry-run gripper enabled" : "gripper enabled";
    }

    bool initialize_hardware(std::string &reason) {
        try {
            if (gripper_type_ == "dh") {
                dh_ = std::make_unique<DH_Gripper>(
                    slave_id_, port_, baudrate_);
                if (dh_->open() < 0) {
                    reason = "failed to open DH gripper port";
                    dh_.reset();
                    return false;
                }
                if (!dh_->Initialization()) {
                    reason = "DH gripper initialization command failed";
                    dh_->close();
                    dh_.reset();
                    return false;
                }
                if (!dh_->SetTargetSpeed(speed_) || !dh_->SetTargetForce(force_)) {
                    reason = "failed to set DH speed/force";
                    return false;
                }
            } else {
                zx_ = std::make_unique<ZX_gripper>(
                    port_.c_str(), slave_id_, baudrate_);
                last_alarm_ = zx_->read_alarm();
                if (!zx_alarm_allowed(last_alarm_, reason)) {
                    zx_.reset();
                    return false;
                }
                zx_->enable(true);
            }
        } catch (const std::exception &error) {
            reason = error.what();
            disable_hardware();
            return false;
        }
        return true;
    }

    void disable_hardware() {
        try {
            if (dh_) {
                dh_->close();
                dh_.reset();
            }
            if (zx_) {
                zx_->enable(false);
                zx_.reset();
            }
            feedback_position_valid_ = false;
        } catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "Gripper disable failed: %s", error.what());
        }
    }

    bool execute_command_locked(bool requested_open, std::string &reason) {
        if (!enabled_) {
            reason = "gripper is disabled";
            return false;
        }
        if (has_requested_state_ && requested_open == requested_open_) {
            std_msgs::msg::Bool executed;
            executed.data = requested_open_;
            executed_command_pub_->publish(executed);
            reason = requested_open_ ? "gripper is already requested open"
                                     : "gripper is already requested closed";
            return true;
        }
        requested_open_ = requested_open;
        has_requested_state_ = true;
        target_position_ = requested_open_ ? open_position_ : closed_position_;
        command_speed_ = requested_open_ ? open_speed_ : closed_speed_;
        command_force_ = requested_open_ ? open_force_ : closed_force_;
        contact_ = false;
        feedback_position_valid_ = false;
        try {
            if (!dry_run_) {
                if (dh_) {
                    if (!dh_->SetTargetPosition(target_position_)) {
                        throw std::runtime_error("DH target position write failed");
                    }
                } else if (zx_) {
                    std::string reason;
                    last_alarm_ = zx_->read_alarm();
                    if (!zx_alarm_allowed(last_alarm_, reason)) {
                        throw std::runtime_error(reason);
                    }
                    zx_->temp_move(
                        target_position_, command_speed_, command_force_);
                } else {
                    throw std::runtime_error("gripper hardware is not initialized");
                }
            }
            current_position_ = target_position_;
            moving_ = true;
            movement_started_ = std::chrono::steady_clock::now();
            ++command_count_;
            if (requested_open_) {
                ++open_command_count_;
            } else {
                ++close_command_count_;
            }
            RCLCPP_WARN(
                get_logger(),
                "TELEMETRY_EVENT event=gripper_command sequence=%llu "
                "action=%s target=%d speed=%d force=%d opens=%llu closes=%llu",
                static_cast<unsigned long long>(command_count_),
                requested_open_ ? "open" : "close",
                target_position_,
                command_speed_,
                command_force_,
                static_cast<unsigned long long>(open_command_count_),
                static_cast<unsigned long long>(close_command_count_));
            std_msgs::msg::Bool executed;
            executed.data = requested_open_;
            executed_command_pub_->publish(executed);
            reason =
                std::string("gripper command accepted: requested_open=") +
                (requested_open_ ? "true" : "false") +
                " target_position=" + std::to_string(target_position_) +
                " speed=" + std::to_string(command_speed_) +
                " force=" + std::to_string(command_force_);
            return true;
        } catch (const std::exception &error) {
            enabled_ = false;
            moving_ = false;
            disable_hardware();
            reason = error.what();
            RCLCPP_ERROR(get_logger(), "Gripper command failed: %s", error.what());
            return false;
        }
    }

    void command_callback(const std_msgs::msg::Bool::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string reason;
        execute_command_locked(message->data, reason);
    }

    void set_open(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        response->success = execute_command_locked(request->data, response->message);
    }

    void stop_callback(const std_msgs::msg::Bool::SharedPtr message) {
        if (!message->data) {
            return;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        if (enabled_ || moving_) {
            stop_at_current_position("teleop STOP request");
        }
    }

    void stop_at_current_position(const std::string &reason) {
        try {
            if (!dry_run_) {
                if (dh_) {
                    int position = 0;
                    if (dh_->GetCurrentPosition(position)) {
                        dh_->SetTargetPosition(position);
                        current_position_ = position;
                    }
                } else if (zx_) {
                    // CTAG2F120 feedback-position registers return zero on the
                    // installed firmware even after a confirmed move.  Never
                    // turn that invalid readback into a new position command
                    // during STOP.  Disabling the actuator is the only
                    // deterministic no-further-motion action available here.
                    zx_->enable(false);
                    zx_.reset();
                }
            }
        } catch (const std::exception &error) {
            RCLCPP_ERROR(
                get_logger(), "Failed to stop gripper: %s", error.what());
        }
        moving_ = false;
        enabled_ = false;
        RCLCPP_ERROR(
            get_logger(),
            "TELEMETRY_EVENT event=gripper_stop reason=\"%s\" "
            "commands=%llu opens=%llu closes=%llu",
            reason.c_str(),
            static_cast<unsigned long long>(command_count_),
            static_cast<unsigned long long>(open_command_count_),
            static_cast<unsigned long long>(close_command_count_));
        RCLCPP_ERROR(get_logger(), "Gripper safety stop: %s", reason.c_str());
    }

    void poll_state() {
        std::lock_guard<std::mutex> lock(mutex_);
        const bool was_moving = moving_;
        std::string state = enabled_ ? "IDLE" : "DISABLED";
        try {
            if (enabled_ && moving_) {
                if (dry_run_) {
                    moving_ = false;
                    state = "DRY_RUN_REACHED";
                } else if (dh_) {
                    int position = 0;
                    int grip_state = 0;
                    dh_->GetCurrentPosition(position);
                    dh_->GetGripState(grip_state);
                    current_position_ = position;
                    feedback_position_valid_ = true;
                    if (grip_state == DH_Gripper::S_GRIP_CAUGHT) {
                        moving_ = false;
                        contact_ = true;
                        state = "CAUGHT";
                    } else if (grip_state == DH_Gripper::S_GRIP_ARRIVED) {
                        moving_ = false;
                        contact_ = false;
                        state = "POSITION_REACHED";
                    } else {
                        state = "MOVING";
                    }
                } else if (zx_) {
                    const int reported_position =
                        static_cast<int>(zx_->feedback_position());
                    const int minimum_position =
                        std::min(open_position_, closed_position_);
                    const int maximum_position =
                        std::max(open_position_, closed_position_);
                    feedback_position_valid_ =
                        reported_position >= minimum_position &&
                        reported_position <= maximum_position;
                    if (feedback_position_valid_) {
                        current_position_ = reported_position;
                    }
                    last_alarm_ = zx_->read_alarm();
                    std::string reason;
                    if (!zx_alarm_allowed(last_alarm_, reason)) {
                        moving_ = false;
                        contact_ = false;
                        enabled_ = false;
                        disable_hardware();
                        RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
                        state = "ALARM_STOP";
                    } else if (zx_->torque_reached()) {
                        moving_ = false;
                        contact_ = true;
                        state = "TORQUE_REACHED";
                    } else if (zx_->position_reached()) {
                        moving_ = false;
                        contact_ = false;
                        state = "POSITION_REACHED";
                    } else {
                        state = "MOVING";
                    }
                }
                const double age = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - movement_started_).count();
                if (moving_ && age > movement_timeout_seconds_) {
                    if (stop_on_movement_timeout_) {
                        stop_at_current_position("movement timeout");
                        state = "WATCHDOG_STOP";
                    } else {
                        state = "MOVEMENT_TIMEOUT_IGNORED";
                    }
                }
            }
        } catch (const std::exception &error) {
            enabled_ = false;
            moving_ = false;
            disable_hardware();
            state = std::string("ERROR:") + error.what();
        }

        if (was_moving && !moving_) {
            const double movement_seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - movement_started_).count();
            RCLCPP_WARN(
                get_logger(),
                "TELEMETRY_EVENT event=gripper_motion_finished sequence=%llu "
                "action=%s result=%s elapsed_s=%.6f contact=%d alarm=%u",
                static_cast<unsigned long long>(command_count_),
                requested_open_ ? "open" : "close",
                state.c_str(),
                movement_seconds,
                contact_,
                static_cast<unsigned int>(last_alarm_));
        }

        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = now();
        joint_state.name = {arm_name_ + "_gripper"};
        joint_state.position = {static_cast<double>(current_position_)};
        state_pub_->publish(joint_state);

        std_msgs::msg::Bool feedback_valid;
        feedback_valid.data = feedback_position_valid_;
        feedback_valid_pub_->publish(feedback_valid);

        std_msgs::msg::Bool contact;
        contact.data = contact_;
        contact_pub_->publish(contact);

        std_msgs::msg::String status;
        std::ostringstream stream;
        stream
            << "state=" << state
            << ";type=" << gripper_type_
            << ";model=" << gripper_model_
            << ";enabled=" << enabled_
            << ";moving=" << moving_
            << ";requested_open=" << requested_open_
            << ";position=" << current_position_
            << ";command_speed=" << command_speed_
            << ";command_force=" << command_force_
            << ";position_source="
            << (feedback_position_valid_
                    ? "measured"
                    : "commanded_endpoint_estimate")
            << ";feedback_position_valid=" << feedback_position_valid_
            << ";contact=" << contact_
            << ";stop_on_movement_timeout=" << stop_on_movement_timeout_
            << ";alarm=" << last_alarm_;
        status.data = stream.str();
        status_pub_->publish(status);
    }

    std::string arm_name_;
    std::string gripper_type_;
    std::string gripper_model_;
    std::string port_;
    int slave_id_{1};
    int baudrate_{115200};
    bool dry_run_{true};
    bool configuration_complete_{false};
    int open_position_{0};
    int closed_position_{0};
    int speed_{20};
    int force_{20};
    int open_speed_{20};
    int open_force_{20};
    int closed_speed_{20};
    int closed_force_{20};
    int allowed_alarm_mask_{0};
    uint16_t last_alarm_{0};
    double movement_timeout_seconds_{2.0};
    bool stop_on_movement_timeout_{true};
    bool enabled_{false};
    bool moving_{false};
    bool has_requested_state_{false};
    bool requested_open_{false};
    bool feedback_position_valid_{false};
    std::uint64_t command_count_{0};
    std::uint64_t open_command_count_{0};
    std::uint64_t close_command_count_{0};
    bool contact_{false};
    int target_position_{0};
    int current_position_{0};
    int command_speed_{0};
    int command_force_{0};
    std::chrono::steady_clock::time_point movement_started_{};
    std::unique_ptr<DH_Gripper> dh_;
    std::unique_ptr<ZX_gripper> zx_;
    std::mutex mutex_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr command_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr feedback_valid_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr contact_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr executed_command_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr open_service_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafeGripperController>());
    rclcpp::shutdown();
    return 0;
}
