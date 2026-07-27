#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "hand_control.h"
#include <chrono>
#include <vector>
#include <deque>
#include <mutex>
#include <thread>

using namespace std::chrono_literals;

class TestHandControl : public rclcpp::Node {
private:
    // 发布者
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_state_pub_;  // 手部状态
    
    // 回调组
    rclcpp::CallbackGroup::SharedPtr normal_callback_group_;    // 普通回调组
    
    // 定时器
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
    bool hand_thread_running_;
    int hand_current_step_;
    int hand_interpolation_steps_;
    double current_hand_state_[6];
    std::thread hand_thread_;
    
    // 状态变量
    bool is_synchronized_;
    
    // 手部控制对象
    HandControl hand_control_;  // 手部控制对象

public:
    TestHandControl() : Node("test_hand_control"), 
                       is_synchronized_(false),
                       is_hand_executing_(false),
                       hand_thread_running_(true),
                       hand_current_step_(0) {
        // 初始化机器人
        initialize_robot();
        
        // 创建回调组
        normal_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        
        auto qos = rclcpp::QoS(10)
            .best_effort()
            .durability_volatile()
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

        auto subscription_options = rclcpp::SubscriptionOptions();
        subscription_options.callback_group = normal_callback_group_;

        // 创建手部控制订阅者
        hand_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/right_arm/hand_control",
            qos,
            std::bind(&TestHandControl::hand_control_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "已创建手部控制订阅者");
        
        // 创建手部状态发布定时器 (32ms)
        hand_publish_timer_ = this->create_wall_timer(
            32ms,
            std::bind(&TestHandControl::publish_hand_states, this)
        );
        
        // 初始化手部轨迹插补相关变量
        hand_interpolation_steps_ = 10;  // 32ms/8ms = 4个插值点
        is_hand_executing_ = false;
        hand_current_step_ = 0;
        
        // 创建手部控制线程
        hand_thread_ = std::thread([this]() {
            while (hand_thread_running_) {
                try {
                    if (is_synchronized_) {
                        execute_hand_trajectory_point();
                    }
                    // 每8ms执行一次
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "执行手部轨迹点异常: %s", e.what());
                }
            }
        });
        
        // 启动同步
        is_synchronized_ = true;
        RCLCPP_INFO(this->get_logger(), "手部控制节点已启动");
    }
    
    ~TestHandControl() {
        // 停止手部控制线程
        hand_thread_running_ = false;
        if (hand_thread_.joinable()) {
            hand_thread_.join();
        }
    }
    
    // 初始化机器人
    void initialize_robot() {
        // 初始化夹爪
        std::string arm_name = "right";
        if (arm_name == "right") {
            std::string ch340_device = "/dev/ttyUSB0";
            if (access(ch340_device.c_str(), F_OK) != -1) {
                RCLCPP_INFO(this->get_logger(), "找到CH340设备: %s", ch340_device.c_str());
                RCLCPP_INFO(this->get_logger(), "初始化夹爪-zhixing");
                hand_control_.init_zx_gripper(ch340_device);
            } else {
                RCLCPP_ERROR(this->get_logger(), "未找到CH340设备: %s", ch340_device.c_str());
            }
        }
        
        // 初始化手部状态
        for (int i = 0; i < 6; i++) {
            current_hand_state_[i] = 0.0;
        }
        
        // 创建手部状态发布者
        hand_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/right_arm/hand_states", 10);
    }
    
    // 手部控制回调函数
    void hand_control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (!is_synchronized_) {
            RCLCPP_WARN(this->get_logger(), "收到手部控制指令，但尚未同步");
            return;
        }
        
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
        RCLCPP_DEBUG(this->get_logger(), "收到手部控制指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            new_point.fingers[0], new_point.fingers[1], new_point.fingers[2], 
            new_point.fingers[3], new_point.fingers[4], new_point.fingers[5]);

        // 更新当前手部状态
        for (int i = 0; i < 6; i++) {
            current_hand_state_[i] = new_point.fingers[i];
        }

        // 如果缓冲区已满，移除最旧的点
        if (hand_trajectory_buffer_.size() >= 4) {
            hand_trajectory_buffer_.pop_front();
        }
        
        // 添加新点
        hand_trajectory_buffer_.push_back(new_point);
        RCLCPP_DEBUG(this->get_logger(), "手部轨迹点: %zu", hand_trajectory_buffer_.size());
        
        // 如果是第一个点，启动执行
        if (!is_hand_executing_ && hand_trajectory_buffer_.size() >= 2) {
            hand_current_step_ = 0;
            is_hand_executing_ = true;
            RCLCPP_INFO(this->get_logger(), "开始执行手部轨迹");
        }
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
            hand_control_.exec_zx_gripper(finger_cmd, 2, hand_control_.fd_hand);
            
            // 释放内存
            delete[] finger_cmd;
            
            // 更新步骤
            hand_current_step_++;
            
            // 检查是否完成当前轨迹段
            if (hand_current_step_ > hand_interpolation_steps_) {
                // 移除已处理的点
                hand_trajectory_buffer_.pop_front();
                hand_current_step_ = 0;
                
                // 如果缓冲区中还有足够的点，继续执行
                if (hand_trajectory_buffer_.size() < 2) {
                    is_hand_executing_ = false;
                    RCLCPP_INFO(this->get_logger(), "手部轨迹执行完成");
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行手部轨迹点异常: %s", e.what());
        }
    }
    
    // 发布手部状态
    void publish_hand_states() {
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data.resize(6);
        
        // 填充手部状态数据
        for (int i = 0; i < 6; i++) {
            msg.data[i] = current_hand_state_[i];
        }
        
        hand_state_pub_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TestHandControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}