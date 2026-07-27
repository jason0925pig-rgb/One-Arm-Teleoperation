#ifndef TRAJECTORY_ONLINE_INTERPOLATOR_HPP_
#define TRAJECTORY_ONLINE_INTERPOLATOR_HPP_

#include <vector>
#include <Eigen/Dense>

// 添加插值模式枚举
enum class InterpolationMode {
    CARTESIAN,
    JOINT
};

struct TrajectoryPoint {
    double x, y, z;    // 位置
    double r, p, yaw;  // 姿态角（欧拉角）
    double timestamp;  // 时间戳
    std::vector<double> joint_positions;  // 关节角度
};

class TrajectoryInterpolator {
public:
    TrajectoryInterpolator();
    
    // 读取轨迹文件
    bool loadTrajectoryFromFile(const std::string& filename);
    
    // 设置插值点数量
    void setInterpolationPoints(int num_points);
    
    // 欧拉角转四元数
    Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw);
    
    // 四元数转欧拉角
    void quaternionToEuler(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
    
    // 球面线性插值（SLERP）
    Eigen::Quaterniond slerpQuaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, double t);
    
    // 获取两点之间的所有插值点
    std::vector<TrajectoryPoint> getInterpolatedSegment(const TrajectoryPoint& p1, const TrajectoryPoint& p2);
    
    // 获取整个轨迹的插值结果
    std::vector<TrajectoryPoint> getInterpolatedTrajectory();
    
    // 添加可视化函数
    void visualizeTrajectory(const std::vector<TrajectoryPoint>& trajectory);
    
    // 修改main函数中的可视化调用
    void interpolateAndVisualize();

    // 添加辅助函数
    bool isJointTrajectory(const TrajectoryPoint& point);
    bool validateTrajectoryPoints(const TrajectoryPoint& p1, const TrajectoryPoint& p2);
    TrajectoryPoint estimateNextPoint(const TrajectoryPoint& p1, const TrajectoryPoint& p2);

private:
    std::vector<TrajectoryPoint> trajectory_points_;
    int num_interpolation_points_;
    InterpolationMode interpolation_mode_ = InterpolationMode::JOINT;
};

#endif // TRAJECTORY_ONLINE_INTERPOLATOR_HPP_ 