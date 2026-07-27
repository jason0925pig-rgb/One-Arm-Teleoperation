#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "DH_gripper.hpp"
#include "ZX_gripper.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

// This node intentionally has no command subscriber and no motion service.
// It opens the selected gripper protocol and reads position/status only.
class GripperFeedbackProbe : public rclcpp::Node {
public:
    GripperFeedbackProbe() : Node("gripper_feedback_probe") {
        arm_name_ = declare_parameter<std::string>("arm_name", "right");
        gripper_type_ = declare_parameter<std::string>("gripper_type", "none");
        gripper_model_ =
            declare_parameter<std::string>("gripper_model", "unknown");
        port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
        slave_id_ = declare_parameter<int>("slave_id", 1);
        baudrate_ = declare_parameter<int>("baudrate", 115200);
        connect_hardware_ = declare_parameter<bool>("connect_hardware", false);
        poll_rate_hz_ = declare_parameter<double>("poll_rate_hz", 5.0);

        const std::string prefix = "/" + arm_name_ + "_arm";
        state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            prefix + "/gripper_probe_state", 10);
        status_pub_ = create_publisher<std_msgs::msg::String>(
            prefix + "/gripper_probe_status", 10);

        if (connect_hardware_) {
            connect();
        } else {
            status_ =
                "LOCKED: connect_hardware=false; no serial port was opened";
        }

        const double bounded_rate = std::max(0.5, poll_rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::duration<double>(1.0 / bounded_rate)),
            std::bind(&GripperFeedbackProbe::poll, this));

        RCLCPP_WARN(
            get_logger(),
            "Read-only gripper probe: arm=%s type=%s port=%s "
            "model=%s connect_hardware=%d. "
            "This executable contains no motion command.",
            arm_name_.c_str(),
            gripper_type_.c_str(),
            port_.c_str(),
            gripper_model_.c_str(),
            connect_hardware_);
    }

    ~GripperFeedbackProbe() override {
        if (dh_) {
            dh_->close();
        }
    }

private:
    void connect() {
        if (gripper_type_ != "dh" && gripper_type_ != "zx") {
            status_ = "ERROR: gripper_type must be dh or zx";
            return;
        }
        try {
            if (gripper_type_ == "dh") {
                dh_ = std::make_unique<DH_Gripper>(
                    slave_id_, port_, baudrate_);
                if (dh_->open() < 0) {
                    dh_.reset();
                    throw std::runtime_error("failed to open DH gripper port");
                }
            } else {
                zx_ = std::make_unique<ZX_gripper>(
                    port_.c_str(), slave_id_, baudrate_);
            }
            connected_ = true;
            status_ = "CONNECTED_READ_ONLY";
        } catch (const std::exception &error) {
            connected_ = false;
            status_ = std::string("ERROR: ") + error.what();
            RCLCPP_ERROR(get_logger(), "%s", status_.c_str());
        }
    }

    void poll() {
        bool valid_position = false;
        try {
            if (connected_ && dh_) {
                int position = 0;
                int init_state = -1;
                int grip_state = -1;
                if (!dh_->GetCurrentPosition(position)) {
                    throw std::runtime_error("DH position read failed");
                }
                dh_->GetInitState(init_state);
                dh_->GetGripState(grip_state);
                current_position_ = position;
                valid_position = true;

                std::ostringstream stream;
                stream
                    << "CONNECTED_READ_ONLY"
                    << ";type=dh"
                    << ";model=" << gripper_model_
                    << ";position=" << position
                    << ";init_state=" << init_state
                    << ";grip_state=" << grip_state;
                status_ = stream.str();
            } else if (connected_ && zx_) {
                const auto position = zx_->feedback_position();
                const bool ready = zx_->ready();
                const bool position_reached = zx_->position_reached();
                const bool torque_reached = zx_->torque_reached();
                const auto alarm = zx_->read_alarm();
                current_position_ = static_cast<double>(position);
                valid_position = true;

                std::ostringstream stream;
                stream
                    << "CONNECTED_READ_ONLY"
                    << ";type=zx"
                    << ";model=" << gripper_model_
                    << ";position=" << position
                    << ";ready=" << ready
                    << ";position_reached=" << position_reached
                    << ";torque_reached=" << torque_reached
                    << ";alarm=" << alarm;
                status_ = stream.str();
            }
        } catch (const std::exception &error) {
            connected_ = false;
            status_ = std::string("ERROR: ") + error.what();
            RCLCPP_ERROR(get_logger(), "%s", status_.c_str());
        }

        if (valid_position) {
            sensor_msgs::msg::JointState message;
            message.header.stamp = now();
            message.name = {arm_name_ + "_gripper_raw"};
            message.position = {current_position_};
            state_pub_->publish(message);
        }

        std_msgs::msg::String status;
        status.data = status_;
        status_pub_->publish(status);
    }

    std::string arm_name_;
    std::string gripper_type_;
    std::string gripper_model_;
    std::string port_;
    int slave_id_{1};
    int baudrate_{115200};
    bool connect_hardware_{false};
    bool connected_{false};
    double poll_rate_hz_{5.0};
    double current_position_{0.0};
    std::string status_;
    std::unique_ptr<DH_Gripper> dh_;
    std::unique_ptr<ZX_gripper> zx_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr state_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GripperFeedbackProbe>());
    rclcpp::shutdown();
    return 0;
}
