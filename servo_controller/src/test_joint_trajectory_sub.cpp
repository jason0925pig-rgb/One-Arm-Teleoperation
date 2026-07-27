#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "hand_control.h"
#include <chrono>
#include <vector>
#include <deque>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <mutex>
#include <unistd.h>
#include <thread>

using namespace std::chrono_literals;

class TestJointTrajectorySubscriber : public rclcpp::Node {
private:
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;  // 机器人就绪状态（右臂）
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_state_pub_;  // 手部状态
    
    // 定时器
    rclcpp::TimerBase::SharedPtr sync_timer_;  // 同步定时器
    rclcpp::TimerBase::SharedPtr hand_publish_timer_;  // 手部状态发布定时器
    
    // 手部控制相关
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_control_sub_;
    
    // 手部轨迹插补相关
    struct HandTrajectoryPoint {
        double fingers[6];  // 6个手指的位置
        double timestamp;   // 时间戳
    };
    
    std::deque<HandTrajectoryPoint> hand_trajectory_buffer_;
    std::mutex hand_mutex_;
    bool is_hand_executing_;
    int hand_current_step_;
    int hand_interpolation_steps_;
    double current_hand_state_[6];
    
    // 状态变量
    bool is_synchronized_;
    std_msgs::msg::Bool is_robot_ready_;
    std::string arm_name_;  // 机械臂名称
    
    // 手部控制对象
    HandControl hand_control_;  // 手部控制对象
    
    // 手部执行定时器
    rclcpp::TimerBase::SharedPtr hand_execute_timer_;

public:
    TestJointTrajectorySubscriber() : Node("test_joint_trajectory_subscriber"), 
                                     is_synchronized_(false),
                                     arm_name_("right"),  // 默认模拟右臂
                                     is_hand_executing_(false),
                                     hand_current_step_(0) {
        // 初始化机器人（只初始化夹爪）
        initialize_robot();
        
        // 创建发布者
        // ready_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        //     "/" + arm_name_ + "_arm/executor_ready", 10);
        
        hand_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/" + arm_name_ + "_arm/hand_states", 10);
        
        // 初始化状态
        is_robot_ready_.data = false;
        
        // 创建同步定时器 (100ms)
        // sync_timer_ = this->create_wall_timer(
        //     100ms,
        //     [this]() {
        //         auto msg = std_msgs::msg::Bool();
        //         msg.data = true;
                
        //         // 发布右臂就绪信号
        //         ready_pub_->publish(msg);
                
        //         is_robot_ready_.data = true;
        //         is_synchronized_ = true;  // 设置同步标志为true
        //         RCLCPP_DEBUG(this->get_logger(), "发送同步就绪信号");
        //     }
        // );
        
        // 创建手部状态发布定时器 (32ms)
        hand_publish_timer_ = this->create_wall_timer(
            32ms,
            std::bind(&TestJointTrajectorySubscriber::publish_hand_states, this)
        );
        
        // 初始化手部轨迹插补相关变量
        hand_interpolation_steps_ = 10;  // 32ms/8ms = 4个插值点
        
        // 初始化手部状态
        for (int i = 0; i < 6; i++) {
            current_hand_state_[i] = 0.0;
        }
        
        // 创建手部控制订阅者
        auto qos = rclcpp::QoS(1)
            .best_effort()  // 使用best_effort而不是reliable
            .durability_volatile()
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        hand_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/" + arm_name_ + "_arm/hand_control",
            qos,
            std::bind(&TestJointTrajectorySubscriber::hand_control_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "已创建手部控制订阅者");
        // 创建8ms定时器执行插值后的手部轨迹点
        hand_execute_timer_ = this->create_wall_timer(
            8ms,
            [this]() {
                try {
                    this->execute_hand_trajectory_point();
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "执行手部轨迹点异常: %s", e.what());
                }
            }
        );
        
        RCLCPP_INFO(this->get_logger(), "Test hand control node initialized");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /%s_arm/hand_control topic", arm_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing hand states...");
    }
    

    
    // 初始化机器人（只初始化夹爪）
    void initialize_robot() {
        RCLCPP_INFO(this->get_logger(), "正在初始化夹爪...");
        
        // 初始化夹爪
        if (arm_name_ == "right") {
            // 查找CH340设备并初始化手部控制
            // std::string ch340_device = "/dev/ttyUSB0";
	    std::string ch340_device = "/dev/ttyACM0";
            if (access(ch340_device.c_str(), F_OK) == -1) {
                RCLCPP_WARN(this->get_logger(), "未找到CH340设备: %s", ch340_device.c_str());
            } else {
                RCLCPP_INFO(this->get_logger(), "找到CH340设备: %s", ch340_device.c_str());
                RCLCPP_INFO(this->get_logger(), "初始化夹爪-zhixing");
                hand_control_.init_zx_gripper(ch340_device);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "夹爪初始化完成");
    }
    
    // 发布手部状态
    void publish_hand_states() {
        auto hand_state_msg = std_msgs::msg::Float32MultiArray();
        // 发布实际手部状态
        for (int i = 0; i < 6; i++) {
            hand_state_msg.data.push_back(current_hand_state_[i]);
        }
        
        hand_state_pub_->publish(hand_state_msg);
    }
    
    // 手部控制回调函数
    void hand_control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        
        if (msg->data.size() != 6) {
            RCLCPP_ERROR(this->get_logger(), "手部控制指令格式错误，期望6个值，实际收到%zu个", msg->data.size());
            return;
        }
        
        std::lock_guard<std::mutex> lock(hand_mutex_);
        
        // 创建新的手部轨迹点
        HandTrajectoryPoint new_point;
        for (size_t i = 0; i < 6; ++i) {
            new_point.fingers[i] = msg->data[i];
        }
        new_point.timestamp = this->now().seconds();
        
        // 打印手部控制指令
        RCLCPP_INFO(this->get_logger(), "收到手部控制指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            new_point.fingers[0], new_point.fingers[1], new_point.fingers[2], 
            new_point.fingers[3], new_point.fingers[4], new_point.fingers[5]);

        // 执行手部控制
        double *finger_cmd = new double[6];
        for (int i = 0; i < 6; i++) {
            finger_cmd[i] = new_point.fingers[i];
            // current_hand_state_[i] = new_point.fingers[i];
        }
        
        RCLCPP_INFO(this->get_logger(), "执行手部指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            finger_cmd[0], finger_cmd[1], finger_cmd[2], 
            finger_cmd[3], finger_cmd[4], finger_cmd[5]);
        
        // 调用手部控制函数
        if (arm_name_ == "left") {
            hand_control_.exec_inspire_gripper(finger_cmd, 1, hand_control_.fd_hand);
        } else {
            hand_control_.exec_zx_gripper(finger_cmd, 2, hand_control_.fd_hand);
        }
        for (int i = 0; i < 6; i++) {
            current_hand_state_[i] = new_point.fingers[i];
        }
        
        // 释放内存
        delete[] finger_cmd;
    }
    
    // 手部轨迹点执行函数
    void execute_hand_trajectory_point() {
        std::lock_guard<std::mutex> lock(hand_mutex_);
        
        // 首先检查缓冲区大小
        if (hand_trajectory_buffer_.size() < 2) {
            return;
        }
        
        try {
            // 获取两个相邻点进行插值
            const auto& start_point = hand_trajectory_buffer_[0];
            const auto& end_point = hand_trajectory_buffer_[1];
            
            // 计算当前插值点
            double t = static_cast<double>(hand_current_step_) / hand_interpolation_steps_;
            
            // 准备插值后的手指位置
            double interpolated_fingers[6];
            
            // 线性插值
            for (int i = 0; i < 6; i++) {
                interpolated_fingers[i] = start_point.fingers[i] + 
                    t * (end_point.fingers[i] - start_point.fingers[i]);
            }
            
            // 执行手部控制
            double *finger_cmd = new double[6];
            for (int i = 0; i < 6; i++) {
                finger_cmd[i] = interpolated_fingers[i];
                current_hand_state_[i] = interpolated_fingers[i];
            }
            
            RCLCPP_DEBUG(this->get_logger(), "执行插值手部指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                finger_cmd[0], finger_cmd[1], finger_cmd[2], 
                finger_cmd[3], finger_cmd[4], finger_cmd[5]);
            
            // 调用手部控制函数
            if (arm_name_ == "left") {
                hand_control_.exec_inspire_gripper(finger_cmd, 1, hand_control_.fd_hand);
            } else {
                hand_control_.exec_zx_gripper(finger_cmd, 2, hand_control_.fd_hand);
            }
            
            // 释放内存
            delete[] finger_cmd;
            
            // 更新步骤
            hand_current_step_++;
            
            // 如果完成当前段的插值，移除起始点
            if (hand_current_step_ >= hand_interpolation_steps_) {
                hand_trajectory_buffer_.pop_front();
                hand_current_step_ = 0;
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行手部轨迹点异常: %s", e.what());
        }
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestJointTrajectorySubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
