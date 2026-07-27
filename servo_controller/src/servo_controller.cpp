#include "rclcpp/rclcpp.hpp"
#include <chrono>
//#include "arm/JAKAZuRobot.h"
#include <thread>
#include "geometry_msgs/msg/pose.hpp"  // 修改为正确的消息类型
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense>  // 添加Eigen库支持
#include <deque>
#include <Eigen/Geometry>  // 添加Eigen几何模块
#include <std_msgs/msg/bool.hpp>
#include <fstream>  // 添加文件操作头文件
#include <ctime>    // 添加时间操作头文件
#include "trajectory_online_interpolator.hpp"
#include <std_msgs/msg/float32_multi_array.hpp>
#include "hand_control.h"
#include <dirent.h>  // 添加dirent头文件
#include <pthread.h>  // 添加pthread头文件
#include <sched.h>  // 添加sched头文件
#include <sys/types.h>  // 添加sys/types头文件
#include <sys/sysinfo.h>  // 添加sys/sysinfo头文件
#include <atomic>
#include "robot.h"
#include "servo_controller/srv/joint_move.hpp"  // 添加服务消息头文件
// #include "dh_modbus_gripper.h"
// #include "DH_gripper.hpp"

using namespace std::chrono_literals;


// #define LEFT_ARM_IP "192.168.2.224"
// #define RIGHT_ARM_IP "192.168.2.225"


#define LEFT_ARM_IP "192.168.2.225"
#define RIGHT_ARM_IP "192.168.2.226"


// #define LEFT_ARM_IP "192.168.2.225"
// #define RIGHT_ARM_IP "192.168.2.227"

#define PORT_LEFT 10010
#define PORT_RIGHT 10020

class ServoControlNode : public rclcpp::Node {
private:
    // 成员变量声明
    std::ofstream cartesian_file_;
    std::ofstream joint_file_;
    double sequence_start_time_;
    double start_time_;
    double last_trajectory_time_;
    bool is_executing_;
    bool is_synchronized_;
    bool first_point_received_;
    bool enable_clock_sync_;
    std::chrono::milliseconds max_clock_diff_;
    int current_step_;
    int interpolation_steps_;
    std::mutex queue_mutex_;
    std::deque<TrajectoryPoint> trajectory_buffer_;
    TrajectoryInterpolator trajectory_interpolator_;
    //JAKAZuRobot robot_;
    //JAKAZuRobot robot_right_;
    Robot robot_;
    JointValue current_joint_pos_;
    CartesianPose current_cart_pos_;
    ModRtuComm hand;
    SignInfo sign_info_angles[8];
    HandControl hand_control_;
    // DH_Gripper *m_gripper;
    
    // ROS2相关成员
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_trajectory_sub_; // 添加关节轨迹订阅器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sync_sub_;
    rclcpp::TimerBase::SharedPtr execute_timer_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
    std_msgs::msg::Bool is_robot_ready_;

    // 添加新的成员变量
    const double position_threshold_ = 10;  // 位置差异阈值（mm）
    const double angle_threshold_ = 0.05;     // 角度差异阈值（弧度）
    bool first_point_handled_ = false;        // 标记是否已处理第一个点
    bool inv_success = true;
    
    // 添加控制模式相关成员
    int control_mode_ = 0;  // 0: 笛卡尔空间控制, 1: 关节空间控制
    std::deque<JointValue> joint_trajectory_buffer_;  // 关节轨迹缓冲区
    
    // 添加手部控制相关成员
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr hand_control_sub_;
    std::array<float, 5> hand_positions_;
    bool hand_command_received_;

    // 手部轨迹插补相关
    struct HandTrajectoryPoint {
        double fingers[6];  // 6个手指的位置
        double timestamp;   // 时间戳
    };
    
    std::deque<HandTrajectoryPoint> hand_trajectory_buffer_;
    std::mutex hand_mutex_;
    rclcpp::TimerBase::SharedPtr hand_execute_timer_;
    bool is_hand_executing_;
    int hand_current_step_;
    int hand_interpolation_steps_;
    double current_hand_state_[6];

    // 添加位姿发布器
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr current_pose_pub_;
    rclcpp::TimerBase::SharedPtr pose_publish_timer_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr hand_state_pub_;
    rclcpp::TimerBase::SharedPtr hand_publish_timer_;

    // 添加机械臂标识
    std::string arm_name_;  // "left" 或 "right"
    
    // 为不同的机械臂使用不同的话题名
    std::string getTopicPrefix() const {
        return "/" + arm_name_ + "_arm";
    }

    // 私有成员函数声明
    void initTrajectoryLog();
    void logTrajectoryPoint(double timestamp,
                           const CartesianPose& cart_pos,
                           const JointValue& joint_pos,
                           const std::string& cmd_type,
                           const std::string& status);
    void trajectory_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void joint_trajectory_callback(const sensor_msgs::msg::JointState::SharedPtr msg); // 添加关节轨迹回调函数
    void execute_trajectory_point();
    void watchdog_callback();
    void reset_state();
    bool inverseKinematics(const CartesianPose& target, JointValue& joint_solution);
    void publish_joint_states();
    void sync_check();
    void sync_callback(const std_msgs::msg::Bool::SharedPtr msg);
    bool checkFirstPoint(const CartesianPose& target_pose, const JointValue& target_joint);
    bool moveToFirstPoint(const CartesianPose& target_pose, const JointValue& target_joint);

    void hand_control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void process_hand_command();
    void execute_hand_trajectory_point();
    std::string findCH340Device();
    void publish_current_pose();
    void publish_hand_states();
    void init_robot(const char* ip, int port);

    // 在类定义中添加新的成员变量
    rclcpp::CallbackGroup::SharedPtr realtime_callback_group_;
    rclcpp::CallbackGroup::SharedPtr normal_callback_group_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> realtime_executor_;
    
    // 添加性能监控相关变量
    std::atomic<uint64_t> callback_count_;
    std::atomic<uint64_t> missed_deadlines_;
    std::chrono::high_resolution_clock::time_point last_callback_time_;
    double target_period_ms_;

    // 添加关节角度发布定时器
    rclcpp::TimerBase::SharedPtr joint_state_publish_timer_;
    
    // 添加服务相关成员
    rclcpp::Service<servo_controller::srv::JointMove>::SharedPtr joint_move_service_;
    
    // 添加服务处理函数
    void handle_joint_move(
        const std::shared_ptr<servo_controller::srv::JointMove::Request> request,
        std::shared_ptr<servo_controller::srv::JointMove::Response> response);

public:
    // 构造函数
    ServoControlNode(const std::string& name, const std::string& arm_name);
    // 析构函数
    ~ServoControlNode();
};

// 构造函数实现
ServoControlNode::ServoControlNode(const std::string& name, const std::string& arm_name)
    : Node(name), arm_name_(arm_name) {
    try {
        RCLCPP_INFO(this->get_logger(), "节点 %s 初始化", name.c_str());

        // 设置线程优先级
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            RCLCPP_WARN(get_logger(), "无法设置线程优先级，可能需要root权限");
        }

        // 设置CPU亲和性，绑定到特定核心
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(1, &cpuset);  // 绑定到CPU核心1（根据实际情况调整）
        if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
            RCLCPP_WARN(get_logger(), "无法设置CPU亲和性");
        }
        
        // 使用不同的话题名初始化发布者和订阅者
        auto qos = rclcpp::QoS(10)
            .best_effort()  // 使用best_effort而不是reliable
            .durability_volatile()
            .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        
        trajectory_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
            getTopicPrefix() + "/cartesian_trajectory",
            qos,
            std::bind(&ServoControlNode::trajectory_callback, this, std::placeholders::_1));
        
        // 创建关节轨迹订阅器
        joint_trajectory_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            getTopicPrefix() + "/joint_trajectory",
            qos,
            std::bind(&ServoControlNode::joint_trajectory_callback, this, std::placeholders::_1));
            
        // 创建实时回调组和普通回调组
        realtime_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        normal_callback_group_ = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        // 修改执行定时器的创建方式
        execute_timer_ = create_wall_timer(
            8ms,
            [this]() {
                if (this->is_synchronized_) {
                    try {
                        this->execute_trajectory_point();
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(get_logger(), "执行轨迹点异常: %s", e.what());
                    }
                }
            },
            realtime_callback_group_);

        // 使用普通回调组创建非关键定时器
        // watchdog_timer_ = create_wall_timer(
        //     50ms,
        //     [this]() {
        //         if (this->is_synchronized_) {
        //             this->watchdog_callback();
        //         }
        //     },
        //     normal_callback_group_);

        // 初始化时间戳
        start_time_ = this->now().seconds();
        last_trajectory_time_ = start_time_;
        is_executing_ = false;
        
        // 机器人连接初始化代码
        // ... existing code ...
        // (保留原来的机器人连接、上电、使能等代码)
        // 添加同步相关的发布者和订阅者
        ready_pub_ = create_publisher<std_msgs::msg::Bool>(
            getTopicPrefix() + "/executor_ready", qos);
        sync_sub_ = create_subscription<std_msgs::msg::Bool>(
          getTopicPrefix() + "/trajectory_ready",
            qos,
          std::bind(&ServoControlNode::sync_callback, this, std::placeholders::_1));
          
        // 初始化同步状态
        is_synchronized_ = false;
        first_point_received_ = false;
        is_robot_ready_.data = false;
        if (!is_synchronized_) {
            auto msg = std_msgs::msg::Bool();
            msg.data = true;
            ready_pub_->publish(msg);
            is_robot_ready_.data = true;
            RCLCPP_DEBUG(get_logger(), "发送同步就绪信号");
        }        
        // 修改同步检查的逻辑
        sync_timer_ = create_wall_timer(
            100ms,
            [this]() {
                if (!is_synchronized_) {
                    auto msg = std_msgs::msg::Bool();
                    msg.data = true;
                    ready_pub_->publish(msg);
                    is_robot_ready_.data = true;
                    RCLCPP_DEBUG(get_logger(), "发送同步就绪信号");
                }
                else{
                    // auto msg = std_msgs::msg::Bool();
                    // msg.data = true;
                    ready_pub_->publish(is_robot_ready_);
                }
            });

        //reset_state();

        // 创建轨迹记录文件
        initTrajectoryLog();

        // 在类定义中添加时间同步参数
        enable_clock_sync_ = true;  // 默认启用时间同步
        max_clock_diff_ = 50ms;  // 允许的最大时钟差异

        trajectory_interpolator_.setInterpolationPoints(4);  // 设置插值点数

        // 初始化手部控制相关变量
        hand_positions_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        hand_command_received_ = false;
        
        // 创建手部控制订阅者
        // hand_control_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        //     getTopicPrefix() + "/hand_control",
        //     qos,
        //     std::bind(&ServoControlNode::hand_control_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "已创建手部控制订阅者");

        // 初始化手部轨迹插补相关变量
        hand_interpolation_steps_ = 10;  // 32ms/8ms = 4个插值点
        is_hand_executing_ = false;
        hand_current_step_ = 0;
        
        // 创建8ms定时器执行插值后的手部轨迹点
        // hand_execute_timer_ = create_wall_timer(8ms, 
        //     std::bind(&ServoControlNode::execute_hand_trajectory_point, this));
        // hand_execute_timer_->cancel();  // 初始时停止定时器


        // m_gripper = new DH_Gripper(1, ch340_device, 115200);  //m_gripper(1, "/dev/ttyUSB0", 115200);
        // if (m_gripper->open() < 0) {
        //     RCLCPP_INFO(get_logger(), "无法找到CH340设备");
        // }

        // m_gripper->Initialization();
        // RCLCPP_INFO(get_logger(),"Gripper initialization sent.");

        // int initstate = 0;
        // int force = 100;
        // int speed = 100;
        // while (initstate != DH_Gripper::S_INIT_FINISHED) {
        //     m_gripper->GetInitState(initstate);
        // }

        // m_gripper->SetTargetForce(force);
        // RCLCPP_INFO(get_logger(),"Set current grip force %f",force);

        // m_gripper->SetTargetSpeed(speed);
        // RCLCPP_INFO(get_logger(),"Set current grip speed %f", speed);


        // 创建位姿发布器
        current_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(
            getTopicPrefix() + "/current_robot_pose", qos);
        
        // 创建定时发布器（比如每8ms发布一次）
        pose_publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(8),
            std::bind(&ServoControlNode::publish_current_pose, this));

        // 创建手部状态发布器
        // hand_state_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        //     getTopicPrefix() + "/hand_states", qos);
        // 创建定时发布器（比如每8ms发布一次）
        // hand_publish_timer_ = this->create_wall_timer(
        //     std::chrono::milliseconds(32),
        //     std::bind(&ServoControlNode::publish_hand_states, this));
        // 创建发布者发送关节状态
        
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            getTopicPrefix() + "/joint_states_xxhei", qos);

        // 创建关节状态发布定时器（8ms周期）
        joint_state_publish_timer_ = create_wall_timer(
            8ms,
            std::bind(&ServoControlNode::publish_joint_states, this));

        // 根据机械臂标识初始化不同的机器人实例
        if (arm_name_ == "left") {
            // 初始化左臂
            init_robot(LEFT_ARM_IP,PORT_LEFT);  // 需要定义左臂IP
        } else {
            // 初始化右臂
            init_robot(RIGHT_ARM_IP,PORT_RIGHT);  // 需要定义右臂IP
            // 查找CH340设备并初始化手部控制
            // std::string ch340_device = findCH340Device();
            // RCLCPP_INFO(get_logger(), "找到CH340设备: %s", ch340_device.c_str());
            
            // //std::string ch340_device = "/dev/ttyUSB0";
            // // hand_control_.init(ch340_device);
            //     // hand_control_.init_dh_gripper(ch340_device);
            // RCLCPP_INFO(get_logger(), "初始化夹爪-zhixing");
            // hand_control_.init_zx_gripper(ch340_device);

        }        

        RCLCPP_INFO(get_logger(), "初始化%s臂控制节点完成", arm_name_.c_str());
        reset_state();
        RCLCPP_INFO(get_logger(), "等待执行指令...");

        // 初始化性能监控变量
        callback_count_ = 0;
        missed_deadlines_ = 0;
        target_period_ms_ = 100.0;  // 
        last_callback_time_ = std::chrono::high_resolution_clock::now();

        // 添加状态检查定时器
        auto status_timer = create_wall_timer(
            1s,
            [this]() {
                RCLCPP_INFO(get_logger(), 
                    "状态: synchronized=%d, executing=%d, buffer_size=%zu", 
                    is_synchronized_, 
                    is_executing_, 
                    trajectory_buffer_.size());
            });

        // 创建服务
        joint_move_service_ = this->create_service<servo_controller::srv::JointMove>(
            getTopicPrefix() + "/joint_move",
            std::bind(&ServoControlNode::handle_joint_move, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            normal_callback_group_);
        
        RCLCPP_INFO(this->get_logger(), "Joint move service initialized for %s arm", arm_name_.c_str());

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "节点 %s 初始化失败: %s", name.c_str(), e.what());
        throw;
    }
}

// 析构函数实现
ServoControlNode::~ServoControlNode() {
    // delete m_gripper;
    if (cartesian_file_.is_open()) {
        cartesian_file_.close();
    }
    if (joint_file_.is_open()) {
        joint_file_.close();
    }
    robot_.servo_move_enable(FALSE);
    // robot_.disable_robot();
}

void ServoControlNode::init_robot(const char* ip, int port)
{
    RCLCPP_INFO(this->get_logger(), "初始化机器人 %s", ip);
    // 机器人连接，使用阻塞模式
    int retry_count = 0;
    const int max_retries = 2;
    bool connected = false;
    robot_.set_sim_mode(false);
    
    {
        RCLCPP_INFO(this->get_logger(), "尝试连接机器人 (第 %d 次)...", retry_count + 1);
#if defined(ARCH_ARM64)
        errno_t err = robot_.login_in(ip);
#else
        errno_t err = robot_.login_in(ip, port);
#endif
        if (err==ERR_SUCC) { 
            connected = true;
            RCLCPP_INFO(this->get_logger(), "机器人连接成功！");
        } else {
            retry_count++;
            RCLCPP_WARN(this->get_logger(), "连接失败，等待3秒后重试...");
            RCLCPP_INFO(this->get_logger(), "error code: %d", err);
            std::this_thread::sleep_for(std::chrono::seconds(3));
        }
    }
    
    if (!connected) {
        RCLCPP_ERROR(this->get_logger(), "机器人-%s-连接失败，已重试%d次！", ip, max_retries);
        return;
    }

    // 机器人上电和使能，每个步骤都添加检查和延时
    RCLCPP_INFO(this->get_logger(), "机器人上电中...");
    errno_t err_p = robot_.power_on();
    if (err_p != ERR_SUCC) {
        RCLCPP_ERROR(this->get_logger(), "机器人-%s-上电失败，error code: %d", ip, err_p);
        return;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot_.clear_error();
    robot_.set_user_frame_id(1);
    robot_.servo_move_use_joint_LPF(0.3);


    RCLCPP_INFO(this->get_logger(), "机器人使能中...");
    errno_t err_e = robot_.enable_robot();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    int retry_count_left = 0;
    while (err_e != ERR_SUCC && retry_count_left < max_retries) {
      RCLCPP_WARN(this->get_logger(), "left使能中，等待3秒...");
      std::this_thread::sleep_for(std::chrono::seconds(3));
      retry_count_left++;
      err_e = robot_.enable_robot();
    }
    JointValue start_pose;
    // if (arm_name_ == "left") {
    //     start_pose = {-2.7787346,0.949351,-1.81443,-1.06328,0.855608,0.234706,1.5708};
    // } else {
    //     //start_pose = {1.15174, -0.745911, 0.050271, -0.581103, -0.254471, -0.94187, 0.299293};
    //     //start_pose = {0.362858,0.949351,-1.81443,-1.06328,0.855608,0.234706,1.5708};
    //     start_pose = {0.499167,0.843236,-2.12323,-1.46053,0.972874,-0.2107,1.5708};
    // }
    // RCLCPP_INFO(this->get_logger(), "start_pose: %f, %f, %f, %f, %f, %f, %f", start_pose.jVal[0], start_pose.jVal[1], start_pose.jVal[2], start_pose.jVal[3], start_pose.jVal[4], start_pose.jVal[5], start_pose.jVal[6]);
    // RCLCPP_INFO(this->get_logger(), "移动到初始位置...");
    // errno_t err_m = robot_.joint_move(&start_pose, ABS, TRUE, 1);
    // RCLCPP_INFO(this->get_logger(), "Move error code : {%d}" ,err_m );


    //JointValue current_pos;
    robot_.get_joint_position(&current_joint_pos_);
    RCLCPP_INFO(this->get_logger(), "当前关节位置: %f, %f, %f, %f, %f, %f, %f", current_joint_pos_.jVal[0], current_joint_pos_.jVal[1], current_joint_pos_.jVal[2], current_joint_pos_.jVal[3], current_joint_pos_.jVal[4], current_joint_pos_.jVal[5], current_joint_pos_.jVal[6]);
    
    // 设置插值参数
    interpolation_steps_ = 10;  // 32ms/8ms = 4个插值点
    trajectory_buffer_.resize(4);  // 保存4个点用于样条插值
    
    // 获取当前位置并初始化轨迹缓冲区
    // CartesianPose current_cart;
    robot_.get_tcp_position(&current_cart_pos_);
    RCLCPP_INFO(this->get_logger(), "当前笛卡尔位置: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f", current_cart_pos_.tran.x, current_cart_pos_.tran.y, current_cart_pos_.tran.z, current_cart_pos_.rpy.rx, current_cart_pos_.rpy.ry, current_cart_pos_.rpy.rz);
    
    // 使用interpolator中的TrajectoryPoint结构初始化
    TrajectoryPoint init_point;
    init_point.x = current_cart_pos_.tran.x;
    init_point.y = current_cart_pos_.tran.y;
    init_point.z = current_cart_pos_.tran.z;
    init_point.r = current_cart_pos_.rpy.rx;
    init_point.p = current_cart_pos_.rpy.ry;
    init_point.yaw = current_cart_pos_.rpy.rz;
    init_point.timestamp = 0.0;

    // 在访问joint_positions之前，先调整向量大小
    init_point.joint_positions.resize(7);  // 确保向量大小为7

    // 然后再赋值关节角度
    for (int i = 0; i < 7; i++) {
        init_point.joint_positions[i] = current_joint_pos_.jVal[i];
    } 

    // 现在可以安全地打印关节角度
    RCLCPP_INFO(get_logger(),"init_point: %f, %f, %f, %f, %f, %f, %f", 
        init_point.joint_positions[0], 
        init_point.joint_positions[1], 
        init_point.joint_positions[2], 
        init_point.joint_positions[3], 
        init_point.joint_positions[4], 
        init_point.joint_positions[5], 
        init_point.joint_positions[6]);
    
    current_step_ = 0;

    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    RCLCPP_INFO(this->get_logger(), "启用伺服模式...");
    
    robot_.servo_move_enable(TRUE);
    
    RCLCPP_INFO(get_logger(), "机器人初始化完成");
}

// 其他成员函数实现
void ServoControlNode::initTrajectoryLog() {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    // 获取当前项目路径（相对路径）
    std::string project_path = ".";
    
    // 创建日志目录路径
    std::string log_directory = project_path + "/logs";
    
    // 确保日志目录存在（使用C++17文件系统）
    rcpputils::fs::create_directories(log_directory);    

    std::stringstream cart_filename, joint_filename;
    
    // 创建两个不同的文件名/left/right
    cart_filename << log_directory <<"/cartesian_trajectory_" << arm_name_ << "_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".txt";
    joint_filename << log_directory <<"/joint_trajectory_" << arm_name_ << "_" << std::put_time(std::localtime(&now_c), "%Y%m%d_%H%M%S") << ".txt";
    
    // 打开两个文件
    cartesian_file_.open(cart_filename.str());
    joint_file_.open(joint_filename.str());
    
    // 分别写入表头
    cartesian_file_ << "# 时间戳(s) x y z rx ry rz 指令类型 执行状态\n";
    joint_file_ << "# 时间戳(s) j1 j2 j3 j4 j5 j6 j7 指令类型 执行状态\n";
    
    RCLCPP_INFO(get_logger(), "笛卡尔轨迹文件已创建: %s", cart_filename.str().c_str());
    RCLCPP_INFO(get_logger(), "关节轨迹文件已创建: %s", joint_filename.str().c_str());
    
    // 初始化序列起始时间
    sequence_start_time_ = this->now().seconds();
}

void ServoControlNode::logTrajectoryPoint(double timestamp,
                                        const CartesianPose& cart_pos,
                                        const JointValue& joint_pos,
                                        const std::string& cmd_type,
                                        const std::string& status) {
    // 使用绝对时间，不计算相对时间
    
    // 记录笛卡尔位姿
    if (cartesian_file_.is_open()) {
        cartesian_file_ << std::fixed << std::setprecision(6)
            << timestamp << " "
            << cart_pos.tran.x << " "
            << cart_pos.tran.y << " "
            << cart_pos.tran.z << " "
            << cart_pos.rpy.rx << " "
            << cart_pos.rpy.ry << " "
            << cart_pos.rpy.rz << " "
            << cmd_type << " "
            << status << "\n";
    }
    
    // 记录关节角度
    if (joint_file_.is_open()) {
        joint_file_ << std::fixed << std::setprecision(6)
            << timestamp << " ";
        
        for (int i = 0; i < 7; ++i) {
            joint_file_ << joint_pos.jVal[i] << " ";
        }
        
        joint_file_ << cmd_type << " "
            << status << "\n";
    }
}



void ServoControlNode::trajectory_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    try {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        if (!msg) {
            RCLCPP_ERROR(get_logger(), "收到空的位姿消息");
            return;
        }

        // 打印接收到的位姿数据
        RCLCPP_DEBUG(get_logger(), "接收到位姿: pos[%.3f, %.3f, %.3f], ori[%.3f, %.3f, %.3f, %.3f]",
            msg->position.x, msg->position.y, msg->position.z,
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

        double current_time = this->now().seconds();
        inv_success = true;

        // 将接收到的笛卡尔位姿转换为CartesianPose格式
        CartesianPose cart_pos;
        cart_pos.tran.x = msg->position.x;
        cart_pos.tran.y = msg->position.y;
        cart_pos.tran.z = msg->position.z;
        // 四元数转欧拉角
        Eigen::Quaterniond q(msg->orientation.w, msg->orientation.x, 
                           msg->orientation.y, msg->orientation.z);
        
        // 计算欧拉角 ZYX顺序
        auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
        cart_pos.rpy.rx = euler[0];
        cart_pos.rpy.ry = euler[1];
        cart_pos.rpy.rz = euler[2];

        // 进行逆解

        JointValue joint_solution;
        if (!inverseKinematics(cart_pos, joint_solution)) {
            RCLCPP_ERROR(get_logger(), "逆解失败");
            inv_success = false;
        }

        // 处理第一个点
        if (!first_point_handled_) {
            if (checkFirstPoint(cart_pos, joint_solution)) {
                RCLCPP_INFO(get_logger(), "检测到初始位置差异较大，进行初始位置调整");
                if (!moveToFirstPoint(cart_pos, joint_solution)) {
                    RCLCPP_ERROR(get_logger(), "移动到初始位置失败");
                    return;
                }
            }
            first_point_handled_ = true;
            
            // 发送同步就绪信号
            auto ready_msg = std_msgs::msg::Bool();
            ready_msg.data = true;
            ready_pub_->publish(ready_msg);
            RCLCPP_INFO(get_logger(), "初始位置确认完成，发送就绪信号");
        }


        // 创建新的轨迹点
        TrajectoryPoint new_point;
        new_point.x = msg->position.x;
        new_point.y = msg->position.y;
        new_point.z = msg->position.z;
        new_point.r = cart_pos.rpy.rx;
        new_point.p = cart_pos.rpy.ry;
        new_point.yaw = cart_pos.rpy.rz;
        new_point.timestamp = current_time;
        
        // 确保在访问之前设置向量大小
        new_point.joint_positions.resize(7);
        for (int i = 0; i < 7; i++) {
            new_point.joint_positions[i] = joint_solution.jVal[i];
        }
        
        // 如果缓冲区已满，移除最旧的点
        if (trajectory_buffer_.size() >= 4) {
            trajectory_buffer_.pop_front();
        }
        
        // 添加新点
        trajectory_buffer_.push_back(new_point);
        
        // 如果是第一个点，启动执行
        if (!is_executing_ && trajectory_buffer_.size() >= 2) {
            RCLCPP_INFO(get_logger(), "开始执行轨迹");
            current_step_ = 0;
            is_executing_ = true;
            if (execute_timer_) {
                RCLCPP_INFO(get_logger(), "重置执行定时器");
                execute_timer_->reset();
            }
        }
        
        last_trajectory_time_ = current_time;
        
        // 记录日志
        CartesianPose cart_pos_log;
        cart_pos_log.tran.x = new_point.x;
        cart_pos_log.tran.y = new_point.y;
        cart_pos_log.tran.z = new_point.z;
        cart_pos_log.rpy.rx = new_point.r;
        cart_pos_log.rpy.ry = new_point.p;
        cart_pos_log.rpy.rz = new_point.yaw;
        
        JointValue joint_pos_log;
        for (int i = 0; i < 7; i++) {
            joint_pos_log.jVal[i] = new_point.joint_positions[i];
        }
        std::string status = (inv_success) ? "接收" : "逆解失败";
        logTrajectoryPoint(this->now().seconds(), cart_pos_log, joint_pos_log, "RECEIVED", status);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "处理轨迹点异常: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "处理轨迹点未知异常");
    }
}

void ServoControlNode::execute_trajectory_point() {
    auto current_time = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(
        current_time - last_callback_time_).count() / 1000.0;

    // 检查是否错过了目标周期
    if (elapsed > target_period_ms_ * 1.5) {  // 允许50%的误差
        missed_deadlines_++;
        RCLCPP_WARN(get_logger(), 
            "错过执行周期! 实际间隔: %.2f ms, 目标: %.2f ms, 总计错过: %lu",
            elapsed, target_period_ms_, missed_deadlines_.load());
    }

    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    // 根据控制模式选择不同的轨迹执行逻辑
    if (control_mode_ == 0) {
        // 笛卡尔空间控制模式
        if (trajectory_buffer_.size() < 2) {
            last_callback_time_ = current_time;
            return;
        }
        
        try {
            // 记录执行开始时间
            auto exec_start = std::chrono::high_resolution_clock::now();

            // ... existing execution code for Cartesian mode ...
            try {
                RCLCPP_DEBUG(get_logger(), "开始插值计算，缓冲区大小: %zu", trajectory_buffer_.size());
                
                // 打印第一个点的信息用于调试
                RCLCPP_DEBUG(get_logger(), "第一个点的关节角度:");
                for (size_t i = 0; i < trajectory_buffer_[0].joint_positions.size(); ++i) {
                    RCLCPP_DEBUG(get_logger(), "Joint %zu: %f", i, trajectory_buffer_[0].joint_positions[i]);
                }
                
                auto interpolated_points = trajectory_interpolator_.getInterpolatedSegment(
                    trajectory_buffer_[0], 
                    trajectory_buffer_[1]
                );
                
                if (interpolated_points.empty()) {
                    RCLCPP_ERROR(get_logger(), "插值结果为空");
                    return;
                }
                
                RCLCPP_DEBUG(get_logger(), "插值点数量: %zu, current_step_: %d", 
                             interpolated_points.size(), current_step_);
                
                if (current_step_ >= static_cast<int>(interpolated_points.size())) {
                    current_step_ = 0;
                    if (!trajectory_buffer_.empty()) {
                        trajectory_buffer_.pop_front();
                    }
                    return;
                }
                
                const auto& point = interpolated_points[current_step_];
                
                // 准备位姿数据
                CartesianPose cart_pos;
                cart_pos.tran.x = point.x;
                cart_pos.tran.y = point.y;
                cart_pos.tran.z = point.z;
                cart_pos.rpy.rx = point.r;
                cart_pos.rpy.ry = point.p;
                cart_pos.rpy.rz = point.yaw;
                
                JointValue joint_cmd;
                for (int i = 0; i < 7; i++) {
                    joint_cmd.jVal[i] = point.joint_positions[i];
                }

                // 执行关节运动
#if defined(ARCH_ARM64)
                errno_t err = robot_.servo_j(&joint_cmd, MoveMode::ABS);
#else
                errno_t err = robot_.edg_servo_j(0, &joint_cmd, MoveMode::ABS);
                robot_.edg_send();
#endif

                robot_.get_joint_position(&current_joint_pos_);
                robot_.get_tcp_position(&current_cart_pos_);
                
                // 记录执行结果
                std::string status = (err == ERR_SUCC) ? "成功" : "失败";
                logTrajectoryPoint(this->now().seconds(), current_cart_pos_, current_joint_pos_, "SERVO", status);
                
                // 更新步骤
                current_step_++;
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(get_logger(), "插值计算错误: %s", e.what());
            }
            
            // 检查执行时间
            auto exec_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - exec_start).count() / 1000.0;
                
            if (exec_time > target_period_ms_ * 0.8) {  // 如果执行时间超过周期的80%
                RCLCPP_WARN(get_logger(), "执行时间过长: %.2f ms", exec_time);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "执行轨迹点异常: %s", e.what());
        }
    } else {
        // 关节空间控制模式
        if (joint_trajectory_buffer_.size() < 2) {
            last_callback_time_ = current_time;
            return;
        }
        
        try {
            // 记录执行开始时间
            auto exec_start = std::chrono::high_resolution_clock::now();
            
            // 执行关节插值
            if (current_step_ >= interpolation_steps_) {
                current_step_ = 0;
                if (!joint_trajectory_buffer_.empty()) {
                    joint_trajectory_buffer_.pop_front();
                }
                return;
            }
            
            // 如果缓冲区中至少有两个点，执行线性插值
            if (joint_trajectory_buffer_.size() >= 2) {
                // 获取起始点和目标点
                const JointValue& start_joint = joint_trajectory_buffer_[0];
                const JointValue& end_joint = joint_trajectory_buffer_[1];
                
                // 计算插值因子
                double t = static_cast<double>(current_step_) / interpolation_steps_;
                
                // 线性插值计算当前关节位置
                JointValue current_joint_cmd;
                for (int i = 0; i < 7; i++) {
                    current_joint_cmd.jVal[i] = start_joint.jVal[i] + 
                        t * (end_joint.jVal[i] - start_joint.jVal[i]);
                }
                
                // 输出调试信息
                RCLCPP_DEBUG(get_logger(), "执行关节插值点: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f], t=%.2f",
                    current_joint_cmd.jVal[0], current_joint_cmd.jVal[1], current_joint_cmd.jVal[2],
                    current_joint_cmd.jVal[3], current_joint_cmd.jVal[4], current_joint_cmd.jVal[5],
                    current_joint_cmd.jVal[6], t);
                
                // 发送关节伺服命令
#if defined(ARCH_ARM64)
                errno_t err = robot_.servo_j(&current_joint_cmd, MoveMode::ABS);
#else
                errno_t err = robot_.edg_servo_j(0, &current_joint_cmd, MoveMode::ABS);
                robot_.edg_send();
#endif

                
                // 获取当前状态用于日志记录
                robot_.get_joint_position(&current_joint_pos_);
                robot_.get_tcp_position(&current_cart_pos_);
                
                // 记录执行结果
                std::string status = (err == ERR_SUCC) ? "成功" : "失败";
                logTrajectoryPoint(this->now().seconds(), current_cart_pos_, current_joint_pos_, "JOINT_SERVO", status);
                
                // 更新步骤
                current_step_++;
            }
            
            // 检查执行时间
            auto exec_time = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::high_resolution_clock::now() - exec_start).count() / 1000.0;
                
            if (exec_time > target_period_ms_ * 0.8) {  // 如果执行时间超过周期的80%
                RCLCPP_WARN(get_logger(), "关节控制执行时间过长: %.2f ms", exec_time);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "执行关节轨迹点异常: %s", e.what());
        }
    }
    
    callback_count_++;
    last_callback_time_ = current_time;
}

void ServoControlNode::watchdog_callback() {
    if (!is_synchronized_) return;

    double current_time = this->now().seconds();
    double time_diff = current_time - last_trajectory_time_;
    
    // if (time_diff > 2.5 && is_executing_) {  // 缩短超时时间到150ms
    //     RCLCPP_WARN(get_logger(), "轨迹接收超时 (%.3f s)，停止执行", time_diff);
    //     // reset_state();        
    //     // // 保持当前位置
    //     // CartesianPose current_pos;
    //     // robot_.get_tcp_position(&current_pos);
    //     // robot_.servo_p(&current_pos, ABS, 2);
    // }
}

void ServoControlNode::reset_state() {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    is_synchronized_ = false;
    first_point_received_ = false;
    is_executing_ = false;
    current_step_ = 0;
    control_mode_ = 0;  // 重置为笛卡尔控制模式
    
    // 清空轨迹缓冲区但保留最后一个点作为参考
    if (!trajectory_buffer_.empty()) {
        //CartesianPose current_pos;
        robot_.get_tcp_position(&current_cart_pos_);
        // JointValue current_joint;
        robot_.get_joint_position(&current_joint_pos_);
        
        TrajectoryPoint init_point;
        init_point.x = current_cart_pos_.tran.x;
        init_point.y = current_cart_pos_.tran.y;
        init_point.z = current_cart_pos_.tran.z;
        init_point.r = current_cart_pos_.rpy.rx;
        init_point.p = current_cart_pos_.rpy.ry;
        init_point.yaw = current_cart_pos_.rpy.rz;
        init_point.joint_positions.resize(7);
        for (int i = 0; i < 7; i++) {
            init_point.joint_positions[i] = current_joint_pos_.jVal[i];
        } 

        init_point.timestamp = this->now().seconds() - start_time_;  // 使用相对时间
        
        trajectory_buffer_.clear();
        for (int i = 0; i < 4; i++) {
            trajectory_buffer_.push_back(init_point);
        }
    }
    
    // 清空关节轨迹缓冲区
    if (!joint_trajectory_buffer_.empty()) {
        JointValue current_joint;
        robot_.get_joint_position(&current_joint);
        
        joint_trajectory_buffer_.clear();
        for (int i = 0; i < 4; i++) {
            joint_trajectory_buffer_.push_back(current_joint);
        }
    }
    
    // 重置时间戳
    last_trajectory_time_ = this->now().seconds();

    // 刷新文件缓冲区
    if (cartesian_file_.is_open()) {
        cartesian_file_.flush();
    }
    if (joint_file_.is_open()) {
        joint_file_.flush();
    }

    // 重置序列起始时间
    sequence_start_time_ = this->now().seconds();
    
    if (cartesian_file_.is_open()) {
        cartesian_file_ << "\n# 新序列开始\n";  // 添加序列分隔标记
        cartesian_file_.flush();
    }
    if (joint_file_.is_open()) {
        joint_file_ << "\n# 新序列开始\n";  // 添加序列分隔标记
        joint_file_.flush();
    }

    // 重置手部控制相关变量
    hand_positions_ = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    hand_command_received_ = false;

    // 重置手部轨迹
    {
        std::lock_guard<std::mutex> lock(hand_mutex_);
        hand_trajectory_buffer_.clear();
        is_hand_executing_ = false;
        hand_current_step_ = 0;
        
        if (hand_execute_timer_) {
            hand_execute_timer_->cancel();
            hand_publish_timer_->cancel();
        }

    }
    auto ready_msg = std_msgs::msg::Bool();
    ready_msg.data = true;
    ready_pub_->publish(ready_msg);
    RCLCPP_DEBUG(get_logger(), "当前关节角度: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
    current_joint_pos_.jVal[0], current_joint_pos_.jVal[1], current_joint_pos_.jVal[2],
    current_joint_pos_.jVal[3], current_joint_pos_.jVal[4], current_joint_pos_.jVal[5],
    current_joint_pos_.jVal[6]);
    RCLCPP_INFO(get_logger(), "同步完成，发送就绪信号");
    // robot_.servo_move_enable(TRUE);

    // RCLCPP_INFO(get_logger(), "is_hand_executing_: %d", is_hand_executing_);
    RCLCPP_INFO(get_logger(), "状态已重置，等待新的轨迹");
}

bool ServoControlNode::inverseKinematics(const CartesianPose& target, JointValue& joint_solution) {
    try {
        // 获取参考关节位置
        JointValue ref_joint;
        
        // 如果轨迹缓冲区不为空，使用最后一个点的关节角度作为参考
        if (!trajectory_buffer_.empty()) {
            const auto& last_point = trajectory_buffer_.back();
            for (int i = 0; i < 7; i++) {
                ref_joint.jVal[i] = last_point.joint_positions[i];
            }
            RCLCPP_DEBUG(get_logger(), "使用上一个轨迹点作为逆解参考");
        } else {
            // 如果缓冲区为空，使用当前机器人关节位置作为参考
            errno_t err = robot_.get_joint_position(&ref_joint);
            if (err != ERR_SUCC) {
                RCLCPP_ERROR(get_logger(), "获取当前关节位置失败，错误码: %d", err);
                return false;
            }
            RCLCPP_DEBUG(get_logger(), "使用当前机器人位置作为逆解参考");
        }
        
        // 打印参考关节角度
        RCLCPP_DEBUG(get_logger(), "参考关节角度: %f, %f, %f, %f, %f, %f, %f",
            ref_joint.jVal[0], ref_joint.jVal[1], ref_joint.jVal[2],
            ref_joint.jVal[3], ref_joint.jVal[4], ref_joint.jVal[5],
            ref_joint.jVal[6]);
            
        // 打印目标笛卡尔位姿
        RCLCPP_DEBUG(get_logger(), "目标位姿: x=%.3f, y=%.3f, z=%.3f, rx=%.3f, ry=%.3f, rz=%.3f",
            target.tran.x, target.tran.y, target.tran.z,
            target.rpy.rx, target.rpy.ry, target.rpy.rz);
        
        // 1. 首先尝试原始逆解
        errno_t err = robot_.kine_inverse(&ref_joint, &target, &joint_solution);
        
        // 检查逆解结果是否合理
        if (err == ERR_SUCC) {
            bool has_invalid_values = false;
            bool has_large_jumps = false;
            
            for (int i = 0; i < 7; i++) {
                // 检查无效值
                if (std::isnan(joint_solution.jVal[i]) || std::isinf(joint_solution.jVal[i])) {
                    RCLCPP_ERROR(get_logger(), "逆解结果包含无效值");
                    has_invalid_values = true;
                    break;
                }
                
                // 检查角度跳变
                double angle_diff = std::abs(joint_solution.jVal[i] - ref_joint.jVal[i]);
                if (angle_diff > 0.5) {  // 如果角度差异大于0.5rad（约28.6度）
                    RCLCPP_WARN(get_logger(), "关节 %d 角度跳变: %.2f度", 
                        i, angle_diff * 180.0 / M_PI);
                    has_large_jumps = true;
                }
            }
            
            // 如果有无效值，视为逆解失败
            if (has_invalid_values) {
                err = -1;  // 设置为非ERR_SUCC的值
            }
            // 如果有大角度跳变，进行平滑处理
            else if (has_large_jumps) {
                // 3. 对特定关节进行平滑处理
                RCLCPP_INFO(get_logger(), "检测到关节角度跳变，应用平滑处理");
                
                // 使用平滑处理
                const double smooth_factor = 0.8;  // 平滑因子，越大越接近上一个值
                
                for (int i = 0; i < 7; i++) {
                    // 
                    if (i == 4 || i == 6) {
                        double angle_diff = std::abs(joint_solution.jVal[i] - ref_joint.jVal[i]);
                        if (angle_diff > 0.3) {  // 如果角度差异大于0.3rad
                            double original = joint_solution.jVal[i];
                            // 使用平滑处理
                            joint_solution.jVal[i] = ref_joint.jVal[i] * smooth_factor + 
                                                    joint_solution.jVal[i] * (1 - smooth_factor);
                            RCLCPP_INFO(get_logger(), "对关节 %d 应用平滑处理，原值: %.2f, 平滑后: %.2f", 
                                i, original, joint_solution.jVal[i]);
                        }
                    }
                }
                return true;  // 平滑处理后返回成功
            }
            else {
                // 逆解成功且没有跳变
                return true;
            }
        }
        
        // 2. 如果原始逆解失败，尝试修改参考角度后的逆解
        if (err != ERR_SUCC) {
            RCLCPP_WARN(get_logger(), "原始逆解失败，尝试修改参考角度");
            RCLCPP_INFO(get_logger(), "err: %d", err);
            RCLCPP_INFO(get_logger(), "ref_joint: %f, %f, %f, %f, %f, %f, %f",
                ref_joint.jVal[0], ref_joint.jVal[1], ref_joint.jVal[2],
                ref_joint.jVal[3], ref_joint.jVal[4], ref_joint.jVal[5],
                ref_joint.jVal[6]); 
            RCLCPP_INFO(get_logger(), "target: %f, %f, %f, %f, %f, %f",
                target.tran.x, target.tran.y, target.tran.z,
                target.rpy.rx, target.rpy.ry, target.rpy.rz);   
            
            // 创建参考关节角度的副本
            JointValue modified_ref_joint = ref_joint;
            
            // 检查是否有接近零的关节角度
            bool modified = false;
            for (int i = 0; i < 7; i++) {
                if (std::abs(modified_ref_joint.jVal[i]) < 0.02) {
                    RCLCPP_INFO(get_logger(), "关节 %d 角度过小: %.2f度", i, modified_ref_joint.jVal[i] * 180.0 / M_PI);
                    // 反转接近零的关节角度符号
                    modified_ref_joint.jVal[i] = (modified_ref_joint.jVal[i] == 0) ? 0.01 : -modified_ref_joint.jVal[i];
                    modified = true;
                }
            }
            RCLCPP_INFO(get_logger(), "modified_ref_joint: %f, %f, %f, %f, %f, %f, %f",
                modified_ref_joint.jVal[0], modified_ref_joint.jVal[1], modified_ref_joint.jVal[2],
                modified_ref_joint.jVal[3], modified_ref_joint.jVal[4], modified_ref_joint.jVal[5],
                modified_ref_joint.jVal[6]);
            
            // 如果修改了参考角度，尝试再次逆解
            if (modified) {
                JointValue new_solution;
                errno_t new_err = robot_.kine_inverse(&modified_ref_joint, &target, &new_solution);
                
                if (new_err == ERR_SUCC) {
                    // 检查新解与原始参考角度的差异
                    bool has_jump = false;
                    for (int i = 0; i < 7; i++) {
                        double angle_diff = std::abs(new_solution.jVal[i] - ref_joint.jVal[i]);
                        if (angle_diff > 0.2) {  // 如果角度差异大于0.5rad
                            RCLCPP_WARN(get_logger(), "新解中关节 %d 角度跳变: %.2f度", 
                                i+1, angle_diff * 180.0 / M_PI);
                            has_jump = true;
                        }
                    }
                    
                    // 如果新解没有跳变，使用新解
                    if (!has_jump) {
                        RCLCPP_INFO(get_logger(), "使用修改参考角度后的逆解结果");
                        joint_solution = new_solution;
                        return true;
                    }
                    // 如果有跳变，对特定关节进行平滑处理
                    else {
                        RCLCPP_INFO(get_logger(), "新解有跳变，应用平滑处理");
                        
                        const double smooth_factor = 0.99;
                        for (int i = 0; i < 7; i++) {
                            if (i == 4 || i == 6) {  // A5和A7关节
                                double angle_diff = std::abs(new_solution.jVal[i] - ref_joint.jVal[i]);
                                if (angle_diff > 0.2) {
                                    double original = new_solution.jVal[i];
                                    new_solution.jVal[i] = ref_joint.jVal[i] * smooth_factor + 
                                                        new_solution.jVal[i] * (1 - smooth_factor);
                                    RCLCPP_INFO(get_logger(), "对关节 %d 应用平滑处理，原值: %.2f, 平滑后: %.2f, ref: %.2f", 
                                        i+1, original, new_solution.jVal[i], ref_joint.jVal[i]);
                                }
                            }
                        }
                        joint_solution = new_solution;
                        return true;
                    }
                }
            }
            
            // 4. 如果修改参考角度后的逆解仍然失败，尝试使用轨迹预测
            RCLCPP_WARN(get_logger(), "修改参考角度后逆解仍失败，尝试使用轨迹预测");
            
            // 记录连续使用预测的次数
            static int prediction_count = 0;
            const int MAX_PREDICTION_COUNT = 5;  // 最大连续预测次数
            
            // 使用最近的两个点进行预测
            if (trajectory_buffer_.size() >= 2 && prediction_count < MAX_PREDICTION_COUNT) {
                // 获取最近的两个点（buffer末尾的两个点）
                size_t last_idx = trajectory_buffer_.size() - 1;
                size_t second_last_idx = trajectory_buffer_.size() - 2;
                
                TrajectoryPoint next_point = trajectory_interpolator_.estimateNextPoint(
                    trajectory_buffer_[second_last_idx], 
                    trajectory_buffer_[last_idx]);
                    
                RCLCPP_INFO(get_logger(), "预测点关节角度: %f, %f, %f, %f, %f, %f, %f", 
                            next_point.joint_positions[0], next_point.joint_positions[1], 
                            next_point.joint_positions[2], next_point.joint_positions[3], 
                            next_point.joint_positions[4], next_point.joint_positions[5], 
                            next_point.joint_positions[6]);
                
                // 检查预测点与参考点的差异
                bool prediction_has_jump = false;
                for (int i = 0; i < 7; i++) {
                    if (i < next_point.joint_positions.size()) {
                        double angle_diff = std::abs(next_point.joint_positions[i] - ref_joint.jVal[i]);
                        if (angle_diff > 0.5) {  // 如果角度差异大于0.5rad
                            RCLCPP_WARN(get_logger(), "预测点关节 %d 角度跳变: %.2f度", 
                                i, angle_diff * 180.0 / M_PI);
                            prediction_has_jump = true;
                        }
                        
                        // 确保关节角度在有效范围内
                        const double MAX_ANGLE = M_PI;
                        const double MIN_ANGLE = -M_PI;
                        next_point.joint_positions[i] = std::min(std::max(
                            next_point.joint_positions[i], MIN_ANGLE), MAX_ANGLE);
                    }
                }
                
                // 如果预测点没有跳变，使用预测点
                if (!prediction_has_jump) {
                    for (int i = 0; i < 7; i++) {
                        if (i < next_point.joint_positions.size()) {
                            joint_solution.jVal[i] = next_point.joint_positions[i];
                        }
                    }
                    RCLCPP_INFO(get_logger(), "使用预测点作为解决方案");
                    prediction_count++;  // 增加连续预测计数
                    return true;
                }
                else {
                    // 预测点有跳变，尝试平滑处理
                    RCLCPP_INFO(get_logger(), "预测点有跳变，应用平滑处理");
                    
                    const double smooth_factor = 0.8;
                    for (int i = 0; i < 7; i++) {
                        if (i < next_point.joint_positions.size() && (i == 4 || i == 6)) {
                            double angle_diff = std::abs(next_point.joint_positions[i] - ref_joint.jVal[i]);
                            if (angle_diff > 0.3) {
                                double original = next_point.joint_positions[i];
                                next_point.joint_positions[i] = ref_joint.jVal[i] * smooth_factor + 
                                                            next_point.joint_positions[i] * (1 - smooth_factor);
                                RCLCPP_INFO(get_logger(), "对关节 %d 应用平滑处理，原值: %.2f, 平滑后: %.2f", 
                                    i, original, next_point.joint_positions[i]);
                            }
                        }
                    }
                    
                    for (int i = 0; i < 7; i++) {
                        if (i < next_point.joint_positions.size()) {
                            joint_solution.jVal[i] = next_point.joint_positions[i];
                        }
                    }
                    prediction_count++;  // 增加连续预测计数
                    return true;
                }
            }
            else {
                // 如果连续预测次数过多，重置计数器
                if (prediction_count >= MAX_PREDICTION_COUNT) {
                    RCLCPP_WARN(get_logger(), "连续预测次数过多，重置预测计数器");
                    prediction_count = 0;
                }
                
                // 5. 如果所有方法都失败，使用当前关节位置
                RCLCPP_WARN(get_logger(), "所有方法都失败，使用当前关节位置");
                JointValue current_joint;
                robot_.get_joint_position(&current_joint);
                joint_solution = current_joint;
                return false;
            }
        }
        
        // 打印逆解结果
        RCLCPP_DEBUG(get_logger(), "逆解结果: %f, %f, %f, %f, %f, %f, %f",
            joint_solution.jVal[0], joint_solution.jVal[1], joint_solution.jVal[2],
            joint_solution.jVal[3], joint_solution.jVal[4], joint_solution.jVal[5],
            joint_solution.jVal[6]);
            
        return true;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "逆解计算异常: %s", e.what());
        return false;
    }
}


void ServoControlNode::publish_joint_states() {
    try {
        // 获取当前关节角度
        // JointValue current_joint_pos;
        // if (robot_.get_joint_position(&current_joint_pos) != ERR_SUCC) {
        //     RCLCPP_ERROR(get_logger(), "获取关节角度失败");
        //     return;
        // }
        
        // 创建关节状态消息
        auto joint_state_msg = std::make_unique<sensor_msgs::msg::JointState>();
        joint_state_msg->header.stamp = this->now();
        joint_state_msg->header.frame_id = arm_name_ + "_arm";
        
        // 设置关节名称
        joint_state_msg->name = {
            arm_name_ + "_joint1",
            arm_name_ + "_joint2",
            arm_name_ + "_joint3",
            arm_name_ + "_joint4",
            arm_name_ + "_joint5",
            arm_name_ + "_joint6",
            arm_name_ + "_joint7"
        };
        
        // 设置关节角度
        for (int i = 0; i < 7; i++) {
            joint_state_msg->position.push_back(current_joint_pos_.jVal[i]);
        }
        
        // 发布消息
        joint_state_pub_->publish(std::move(joint_state_msg));
        
        // 记录日志
        RCLCPP_DEBUG(get_logger(), "发布关节角度: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        current_joint_pos_.jVal[0], current_joint_pos_.jVal[1], current_joint_pos_.jVal[2],
        current_joint_pos_.jVal[3], current_joint_pos_.jVal[4], current_joint_pos_.jVal[5],
        current_joint_pos_.jVal[6]);
            
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "发布关节角度异常: %s", e.what());
    }
}

void ServoControlNode::publish_hand_states() {
    //hand_control_.read_inspire_hand(1, hand_control_.fd_hand);
    //std_msgs::msg::Float32MultiArray hand_state_msg;
    //hand_state_msg.data = {0,0,0,0,0,0};
    //hand_state_pub_->publish(hand_state_msg);

    // 检查手部设备是否可用
    // if (hand_control_.fd_hand <= 0) {
    //     RCLCPP_WARN(get_logger(), "手部设备未初始化或不可用");
    //     return;
    // }
    
    // 发送查询命令并读取手部信息
    // if (hand_control_.read_inspire_hand(1, hand_control_.fd_hand)) {
        // 成功读取到数据，可以在此处使用tHand中的数据
        
        // 发布手部状态信息
        auto hand_state_msg = std_msgs::msg::Float32MultiArray();
        hand_state_msg.data.resize(6);  // 6个手指的角度
        


        for (int i=0;i<6;i++){
            hand_state_msg.data[i] = current_hand_state_[i];
        }


        // 发布消息（假设您已经创建了相应的发布者）
        hand_state_pub_->publish(hand_state_msg);
        
        RCLCPP_DEBUG(get_logger(), "已更新并发布手部状态信息");
    // } else {
    //     RCLCPP_DEBUG(get_logger(), "读取手部信息失败");
    // }


}

void ServoControlNode::sync_check() {
    // 只在完全初始化但尚未处理第一个点时发送就绪信号
    if (!is_synchronized_ && !first_point_handled_) {
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        ready_pub_->publish(msg);
    }
}

void ServoControlNode::sync_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        if (!is_synchronized_) {
            RCLCPP_INFO(get_logger(), "收到同步信号");
            reset_state();
            sequence_start_time_ = this->now().seconds();
            is_synchronized_ = true;
            
            // 立即发送就绪确认
            auto ready_msg = std_msgs::msg::Bool();
            ready_msg.data = true;
            ready_pub_->publish(ready_msg);
            
            RCLCPP_INFO(get_logger(), "同步完成，准备执行轨迹");
        }
    }
}

// 实现检查第一个点的函数
bool ServoControlNode::checkFirstPoint(const CartesianPose& target_pose, const JointValue& target_joint) {
    CartesianPose current_pose;
    JointValue current_joint;
    
    // 获取当前位置
    if (robot_.get_tcp_position(&current_pose) != ERR_SUCC ||
        robot_.get_joint_position(&current_joint) != ERR_SUCC) {
        RCLCPP_ERROR(get_logger(), "获取当前位置失败");
        return false;
    }
    
    // 检查位置差异
    double pos_diff = std::sqrt(
        std::pow(target_pose.tran.x - current_pose.tran.x, 2) +
        std::pow(target_pose.tran.y - current_pose.tran.y, 2) +
        std::pow(target_pose.tran.z - current_pose.tran.z, 2));
        
    // 检查姿态差异
    Eigen::Quaterniond q_target(Eigen::AngleAxisd(target_pose.rpy.rx, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(target_pose.rpy.ry, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(target_pose.rpy.rz, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond q_current(Eigen::AngleAxisd(current_pose.rpy.rx, Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(current_pose.rpy.ry, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(current_pose.rpy.rz, Eigen::Vector3d::UnitZ()));
    double angle_diff = std::max({
        std::abs(q_target.toRotationMatrix().eulerAngles(2, 1, 0)[0] - q_current.toRotationMatrix().eulerAngles(2, 1, 0)[0]),
        std::abs(q_target.toRotationMatrix().eulerAngles(2, 1, 0)[1] - q_current.toRotationMatrix().eulerAngles(2, 1, 0)[1]),
        std::abs(q_target.toRotationMatrix().eulerAngles(2, 1, 0)[2] - q_current.toRotationMatrix().eulerAngles(2, 1, 0)[2])
    });
    
    // 检查关节角度差异
    double joint_diff = 0;
    for (int i = 0; i < 7; i++) {
        joint_diff = std::max(joint_diff, 
            std::abs(target_joint.jVal[i] - current_joint.jVal[i]));
    }
    
    RCLCPP_INFO(get_logger(), "位置差异: %.3f m, 姿态差异: %.3f rad, 关节差异: %.3f rad",
                pos_diff, angle_diff, joint_diff);
    
    return (pos_diff > position_threshold_) || 
           (angle_diff > angle_threshold_);
}

// 实现移动到第一个点的函数
bool ServoControlNode::moveToFirstPoint(const CartesianPose& target_pose, const JointValue& target_joint) {
    RCLCPP_INFO(get_logger(), "正在移动到初始位置...");
    
    // 暂时禁用伺服模式
    robot_.servo_move_enable(FALSE);
    
    // 使用关节运动移动到目标位置
    errno_t err = robot_.joint_move(&target_joint, ABS, TRUE, 20.0);
    
    if (err != ERR_SUCC) {
        RCLCPP_ERROR(get_logger(), "移动到初始位置失败，错误码: %d", err);
        return false;
    }
    
    // 等待运动完成
    RobotStatus status;
    do {
        if (robot_.get_robot_status(&status) != ERR_SUCC) {
            RCLCPP_ERROR(get_logger(), "获取机器人状态失败");
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (status.inpos != 1);
    
    // 重新启用伺服模式
    robot_.servo_move_enable(TRUE);
    
    RCLCPP_INFO(get_logger(), "已到达初始位置");
    return true;
}

// 实现手部控制回调函数
void ServoControlNode::hand_control_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (!is_synchronized_) {
        RCLCPP_WARN(get_logger(), "收到手部控制指令，但尚未同步");
        return;
    }
    
    if (msg->data.size() != 6) {
        RCLCPP_ERROR(get_logger(), "手部控制指令格式错误，期望6个值，实际收到%zu个", msg->data.size());
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
    RCLCPP_INFO(get_logger(), "收到手部控制指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
        new_point.fingers[0], new_point.fingers[1], new_point.fingers[2], 
        new_point.fingers[3], new_point.fingers[4], new_point.fingers[5]);
    
    // 如果缓冲区已满，移除最旧的点
    if (hand_trajectory_buffer_.size() >= 4) {
        hand_trajectory_buffer_.pop_front();
    }
    
    // 添加新点
    hand_trajectory_buffer_.push_back(new_point);
    RCLCPP_DEBUG(get_logger(), "手部轨迹点: %zu", hand_trajectory_buffer_.size());
    RCLCPP_DEBUG(get_logger(), "is_hand_executing_: %d", is_hand_executing_);
    
    // 如果是第一个点，启动执行
    if (!is_hand_executing_ && hand_trajectory_buffer_.size() >= 2) {
        hand_current_step_ = 0;
        is_hand_executing_ = true;
        hand_execute_timer_->reset();
        hand_publish_timer_->reset();
        RCLCPP_INFO(get_logger(), "开始执行手部轨迹");
    }
    
    hand_command_received_ = true;
}

// 实现手部轨迹点执行函数
void ServoControlNode::execute_hand_trajectory_point() {
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
        double *finger_cmd = new double[NUM_HAND_CONTROL];
        for (int i = 0; i < 6; i++) {
            finger_cmd[i] = interpolated_fingers[i];
            current_hand_state_[i] = interpolated_fingers[i];
        }
        
        RCLCPP_DEBUG(get_logger(), "执行插值手部指令: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
            finger_cmd[0], finger_cmd[1], finger_cmd[2], 
            finger_cmd[3], finger_cmd[4], finger_cmd[5]);
        
        // 调用手部控制函数
        if(arm_name_=="left"){
            // hand_control_.exec_inspire_hand(finger_cmd, 1, hand_control_.fd_hand);
            hand_control_.exec_inspire_gripper(finger_cmd, 1, hand_control_.fd_hand);

        }
        else{
            // hand_control_.exec_inspire_hand(finger_cmd, 2, hand_control_.fd_hand);
            // hand_control_.exec_dh_gripper(finger_cmd, 2, hand_control_.fd_hand);

            hand_control_.exec_zx_gripper(finger_cmd, 2, hand_control_.fd_hand);

            // m_gripper->SetTargetPosition(1000);
        }
        //hand_control_.read_inspire_hand(1, hand_control_.fd_hand);
        
        // 释放内存
        delete[] finger_cmd;
        
        // 更新步骤
        hand_current_step_++;
        
        // 如果完成当前段的插值，移除起始点
        if (hand_current_step_ >= hand_interpolation_steps_) {
            hand_current_step_ = 0;
            if (!hand_trajectory_buffer_.empty()) {
                hand_trajectory_buffer_.pop_front();
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "手部轨迹插值错误: %s", e.what());
    }
}

// 实现查找CH340设备的函数
std::string ServoControlNode::findCH340Device() {
    // 根据机械臂标识返回对应的手部设备
    if (arm_name_ == "left") {
        if (access("/dev/ttyUSB1", F_OK) != -1) {
            RCLCPP_INFO(get_logger(), "找到左手设备: /dev/USB1");
            return "/dev/ttyUSB1";
        }
    } else {
        RCLCPP_INFO(get_logger(), "fffffffffffffffffffffffff");

        if (access("/dev/ttyUSB0", F_OK) != -1) {
            RCLCPP_INFO(get_logger(), "fffffffffffffffffffffffff");
            RCLCPP_INFO(get_logger(), "找到右手设备: /dev/ttyUSB0");
            return "/dev/ttyUSB0";
        }
    }

    // 如果找不到指定设备，尝试查找任何可用的CH340设备
    DIR *dir;
    struct dirent *ent;
    std::string result = "";
    
    if ((dir = opendir("/dev")) != NULL) {
        while ((ent = readdir(dir)) != NULL) {
            std::string deviceName = ent->d_name;
            if (deviceName.find("ttyUSB") != std::string::npos) {
                std::string fullPath = "/dev/" + deviceName;
                
                // 检查是否为CH340设备
                std::string syspath = "/sys/class/tty/" + deviceName + "/device/uevent";
                std::ifstream uevent(syspath);
                std::string line;
                while (std::getline(uevent, line)) {
                    if (line.find("CH340") != std::string::npos) {
                        result = fullPath;
                        break;
                    }
                }
                
                if (!result.empty()) break;
            }
        }
        closedir(dir);
    }
    
    if (result.empty()) {
        RCLCPP_WARN(get_logger(), "未找到CH340设备，使用默认设备: /dev/ttyUSB0");
        return "/dev/ttyUSB0";
    }
    
    RCLCPP_INFO(get_logger(), "找到CH340设备: %s", result.c_str());
    return result;
}

// 实现位姿发布函数
void ServoControlNode::publish_current_pose() {
    // 获取当前位姿
    //robot_.get_tcp_position(&current_cart_pos_);
    
    // 创建位姿消息
    auto pose_msg = geometry_msgs::msg::Pose();
    
    // 设置位置
    pose_msg.position.x = current_cart_pos_.tran.x;
    pose_msg.position.y = current_cart_pos_.tran.y;
    pose_msg.position.z = current_cart_pos_.tran.z;
    
    // 将欧拉角(ZYX)转换为四元数 
    double roll = current_cart_pos_.rpy.rx;
    double pitch = current_cart_pos_.rpy.ry;
    double yaw = current_cart_pos_.rpy.rz;
    
    // 计算四元数
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    // 设置方向（四元数）
    pose_msg.orientation.w = cr * cp * cy + sr * sp * sy;
    pose_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    pose_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    pose_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    
    // 发布位姿消息
    current_pose_pub_->publish(pose_msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
        "发布当前位姿: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
        current_cart_pos_.tran.x, current_cart_pos_.tran.y, current_cart_pos_.tran.z,
        current_cart_pos_.rpy.rx, current_cart_pos_.rpy.ry, current_cart_pos_.rpy.rz);
}

// void ServoControlNode::joint_trajectory_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
//     try {
//         std::lock_guard<std::mutex> lock(queue_mutex_);
        
//         if (!msg) {
//             RCLCPP_ERROR(get_logger(), "收到空的关节位置消息");
//             return;
//         }

//         // 检查关节数量
//         if (msg->position.size() != 7) {
//             RCLCPP_ERROR(get_logger(), "关节位置数量不正确，期望7个，收到%zu个", msg->position.size());
//             return;
//         }

//         // 打印接收到的关节数据
//         RCLCPP_INFO(get_logger(), "接收到关节位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
//             msg->position[0], msg->position[1], msg->position[2],
//             msg->position[3], msg->position[4], msg->position[5], msg->position[6]);

//         double current_time = this->now().seconds();

//         // 设置控制模式为关节控制
//         control_mode_ = 1;

//         // 创建关节位置结构
//         JointValue joint_pos;
//         for (int i = 0; i < 7; i++) {
//             joint_pos.jVal[i] = msg->position[i];
//         }

//         // 处理第一个点
//         if (!first_point_handled_) {
//             // 获取当前关节位置
//             JointValue current_joint;
//             if (robot_.get_joint_position(&current_joint) != ERR_SUCC) {
//                 RCLCPP_ERROR(get_logger(), "获取当前关节位置失败");
//                 return;
//             }

//             // 检查关节角度差异
//             bool large_difference = false;
//             for (int i = 0; i < 7; i++) {
//                 double diff = std::abs(joint_pos.jVal[i] - current_joint.jVal[i]);
//                 if (diff > angle_threshold_) {
//                     large_difference = true;
//                     RCLCPP_INFO(get_logger(), "关节 %d 角度差异较大: %.3f rad", i, diff);
//                 }
//             }

//             if (large_difference) {
//                 RCLCPP_INFO(get_logger(), "检测到初始关节位置差异较大，进行初始位置调整");
                
//                 // 暂时禁用伺服模式
//                 robot_.servo_move_enable(FALSE);
                
//                 // 使用关节运动移动到目标位置
//                 errno_t err = robot_.joint_move(&joint_pos, ABS, TRUE, 20.0);
                
//                 if (err != ERR_SUCC) {
//                     RCLCPP_ERROR(get_logger(), "移动到初始关节位置失败，错误码: %d", err);
//                     //return;
//                 }
                
//                 // 等待运动完成
//                 RobotStatus status;
//                 do {
//                     if (robot_.get_robot_status(&status) != ERR_SUCC) {
//                         RCLCPP_ERROR(get_logger(), "获取机器人状态失败");
//                         return;
//                     }
//                     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//                 } while (status.inpos != 1);
//                 robot_.get_joint_position(&current_joint_pos_);
                
//                 // 重新启用伺服模式
//                 robot_.servo_move_enable(TRUE);
                
//                 RCLCPP_INFO(get_logger(), "已到达初始关节位置");
//             }
            
//             first_point_handled_ = true;
            
//             // 发送同步就绪信号
//             auto ready_msg = std_msgs::msg::Bool();
//             ready_msg.data = true;
//             ready_pub_->publish(ready_msg);
//             RCLCPP_INFO(get_logger(), "初始关节位置确认完成，发送就绪信号");
//         }

//         // 如果缓冲区已满，移除最旧的点
//         if (joint_trajectory_buffer_.size() >= 4) {
//             joint_trajectory_buffer_.pop_front();
//         }
        
//         // 添加新点
//         joint_trajectory_buffer_.push_back(joint_pos);
        
//         // 如果是第一个点，启动执行
//         if (!is_executing_ && joint_trajectory_buffer_.size() >= 2) {
//             RCLCPP_INFO(get_logger(), "开始执行关节轨迹");
//             current_step_ = 0;
//             is_executing_ = true;
//             if (execute_timer_) {
//                 RCLCPP_INFO(get_logger(), "重置执行定时器");
//                 execute_timer_->reset();
//             }
//         }
        
//         last_trajectory_time_ = current_time;
        
//         // 获取当前笛卡尔位姿用于日志记录
//         // CartesianPose current_cart;
//         // if (robot_.get_tcp_position(&current_cart) != ERR_SUCC) {
//         //     RCLCPP_WARN(get_logger(), "无法获取当前笛卡尔位姿用于日志记录");
//         // }
        
//         // 记录日志
//         std::string status = "接收";
//         logTrajectoryPoint(this->now().seconds(), current_cart_pos_, joint_pos, "JOINT_RECEIVED", status);
        
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(get_logger(), "处理关节轨迹点异常: %s", e.what());
//     } catch (...) {
//         RCLCPP_ERROR(get_logger(), "处理关节轨迹点未知异常");
//     }
// }


void ServoControlNode::joint_trajectory_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // 记录回调开始时间
    double callback_start_time = this->now().seconds();
    double message_time = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
    double message_delay = callback_start_time - message_time;
    
    // 快速检查消息有效性
    if (!msg) {
        RCLCPP_ERROR(get_logger(), "收到空的关节位置消息");
        return;
    }

    // 检查关节数量
    if (msg->position.size() != 7) {
        RCLCPP_ERROR(get_logger(), "关节位置数量不正确，期望7个，收到%zu个", msg->position.size());
        return;
    }

    // 记录进入互斥锁前的时间
    double pre_lock_time = this->now().seconds();
    double pre_lock_duration = pre_lock_time - callback_start_time;
    
    // 定义时间测量变量
    double lock_duration = 0.0;
    double joint_pos_duration = 0.0;
    double buffer_duration = 0.0;
    double log_duration = 0.0;
    double mutex_duration = 0.0;
    
    // 锁定互斥锁，只在必要时持有
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        
        // 记录进入互斥锁后的时间
        double post_lock_time = this->now().seconds();
        lock_duration = post_lock_time - pre_lock_time;
        
        double current_time = this->now().seconds();

        // 设置控制模式为关节控制
        control_mode_ = 1;

        // 记录创建关节位置结构前的时间
        double pre_joint_pos_time = this->now().seconds();
        
        // 创建关节位置结构
        JointValue joint_pos;
        for (int i = 0; i < 7; i++) {
            joint_pos.jVal[i] = msg->position[i];
        }
        
        // 记录创建关节位置结构后的时间
        double post_joint_pos_time = this->now().seconds();
        joint_pos_duration = post_joint_pos_time - pre_joint_pos_time;
        
        // 记录处理缓冲区前的时间
        double pre_buffer_time = this->now().seconds();
        
        // 如果缓冲区已满，移除最旧的点
        if (joint_trajectory_buffer_.size() >= 4) {
            joint_trajectory_buffer_.pop_front();
        }
        
        // 添加新点
        joint_trajectory_buffer_.push_back(joint_pos);
        
        // 如果是第一个点，启动执行
        if (!is_executing_ && joint_trajectory_buffer_.size() >= 2) {
            current_step_ = 0;
            is_executing_ = true;
            if (execute_timer_) {
                execute_timer_->reset();
            }
        }
        
        // 记录处理缓冲区后的时间
        double post_buffer_time = this->now().seconds();
        buffer_duration = post_buffer_time - pre_buffer_time;
        
        last_trajectory_time_ = current_time;
        
        // 记录日志前的时间
        double pre_log_time = this->now().seconds();
        
        // 记录日志
        std::string status = "接收";
        logTrajectoryPoint(current_time, current_cart_pos_, joint_pos, "JOINT_RECEIVED", status);
        
        // 记录日志后的时间
        double post_log_time = this->now().seconds();
        log_duration = post_log_time - pre_log_time;
        
        // 记录互斥锁内的总时间
        mutex_duration = post_log_time - post_lock_time;
    }
    
    // 记录回调结束时间
    double callback_end_time = this->now().seconds();
    double total_duration = callback_end_time - callback_start_time;
    
    // 每100个消息打印一次详细的时间统计
    static int msg_count = 0;
    msg_count++;
    if (msg_count % 30 == 0) {
        RCLCPP_INFO(get_logger(), "Joint trajectory callback timing:");
        RCLCPP_INFO(get_logger(), "  Message delay: %.4f ms", message_delay * 1000);
        RCLCPP_INFO(get_logger(), "  Total callback time: %.4f ms", total_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Pre-lock duration: %.4f ms", pre_lock_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Lock duration: %.4f ms", lock_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Joint pos creation: %.4f ms", joint_pos_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Buffer processing: %.4f ms", buffer_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Logging duration: %.4f ms", log_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Mutex duration: %.4f ms", mutex_duration * 1000);
        RCLCPP_INFO(get_logger(), "  Message count: %d", msg_count);
    }
}


// 实现服务处理函数
void ServoControlNode::handle_joint_move(
    const std::shared_ptr<servo_controller::srv::JointMove::Request> request,
    std::shared_ptr<servo_controller::srv::JointMove::Response> response) {
    
    try {
        RCLCPP_WARN(this->get_logger(), "收到关节运动请求");
        
        // 检查关节数量
        if (request->joint_positions.size() != 7) {
            RCLCPP_ERROR(this->get_logger(), "关节位置数量不正确，期望7个，收到%zu个", request->joint_positions.size());
            response->success = false;
            response->message = "关节位置数量不正确";
            return;
        }
        
        // 创建关节位置结构
        JointValue joint_pos;
        for (int i = 0; i < 7; i++) {
            joint_pos.jVal[i] = request->joint_positions[i];
        }
        
        // 暂时禁用伺服模式
        robot_.servo_move_enable(FALSE);
        
        // 执行关节运动
        errno_t err = robot_.joint_move(&joint_pos, ABS, TRUE, request->speed);
        
        if (err != ERR_SUCC) {
            RCLCPP_ERROR(this->get_logger(), "关节运动失败，错误码: %d", err);
            response->success = false;
            response->message = "关节运动失败";
            return;
        }
        
        // 如果请求是阻塞的，等待运动完成
        if (request->is_block) {
            RCLCPP_WARN(this->get_logger(), "等待关节运动完成");
            RobotStatus status;
            do {
                if (robot_.get_robot_status(&status) != ERR_SUCC) {
                    RCLCPP_ERROR(this->get_logger(), "获取机器人状态失败");
                    response->success = false;
                    response->message = "获取机器人状态失败";
                    return;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } while (status.inpos != 1);
        }
        robot_.get_joint_position(&current_joint_pos_);
        
        // 重新启用伺服模式
        robot_.servo_move_enable(TRUE);
        
        response->success = true;
        response->message = "关节运动成功";
        RCLCPP_INFO(this->get_logger(), "关节运动完成");
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "处理关节运动请求异常: %s", e.what());
        response->success = false;
        response->message = std::string("处理请求异常: ") + e.what();
    }
}

// main函数
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    
    // 创建执行器
    rclcpp::executors::MultiThreadedExecutor executor;
    
    // 创建左臂节点
    auto left_arm_node = std::make_shared<ServoControlNode>(
        "left_arm_controller", "left");
    
    // 创建右臂节点
    auto right_arm_node = std::make_shared<ServoControlNode>(
        "right_arm_controller", "right");
    
    // 将节点添加到执行器
    executor.add_node(left_arm_node);
    executor.add_node(right_arm_node);
    
    // 使用多线程执行器运行节点
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
} 

    
