#include "trajectory_online_interpolator.hpp"
#include <iostream>
#include <fstream>
#include <cmath>


TrajectoryInterpolator::TrajectoryInterpolator() : num_interpolation_points_(4) {}

bool TrajectoryInterpolator::loadTrajectoryFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    TrajectoryPoint point;
    int lineCount = 0;
    std::string line;
    
    while (std::getline(file, line)) {
        switch (lineCount % 3) {
            case 0:
                sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf", 
                       &point.x, &point.y, &point.z, &point.r, &point.p, &point.yaw);
                point.timestamp = lineCount * 0.008;
                trajectory_points_.push_back(point);
                break;
        }
        lineCount++;
    }
    file.close();
    return true;
}

void TrajectoryInterpolator::setInterpolationPoints(int num_points) {
    num_interpolation_points_ = num_points;
}

Eigen::Quaterniond TrajectoryInterpolator::eulerToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    return Eigen::Quaterniond(yawAngle * pitchAngle * rollAngle);
}

void TrajectoryInterpolator::quaternionToEuler(const Eigen::Quaterniond& q, 
                                             double& roll, double& pitch, double& yaw) {
    Eigen::Matrix3d rotation_matrix = q.toRotationMatrix();
    roll = atan2(rotation_matrix(2,1), rotation_matrix(2,2));
    pitch = -asin(rotation_matrix(2,0));
    yaw = atan2(rotation_matrix(1,0), rotation_matrix(0,0));
}

Eigen::Quaterniond TrajectoryInterpolator::slerpQuaternion(const Eigen::Quaterniond& q1, 
                                                          const Eigen::Quaterniond& q2, 
                                                          double t) {
    return q1.slerp(t, q2);
}

std::vector<TrajectoryPoint> TrajectoryInterpolator::getInterpolatedSegment(
    const TrajectoryPoint& p1, const TrajectoryPoint& p2) {
    
    std::vector<TrajectoryPoint> interpolated_points;
    
    // 根据输入点的类型判断插值模式
    // InterpolationMode mode = p1.joint_positions.empty() ? 
    //                         InterpolationMode::CARTESIAN : 
    //                         InterpolationMode::JOINT;
    //InterpolationMode mode = InterpolationMode::JOINT;
    if (interpolation_mode_ == InterpolationMode::CARTESIAN) {
        // 笛卡尔空间插值
        // 转换为四元数
        Eigen::Quaterniond q1 = eulerToQuaternion(p1.r, p1.p, p1.yaw);
        Eigen::Quaterniond q2 = eulerToQuaternion(p2.r, p2.p, p2.yaw);

        // 确保四元数走最短路径
        if (q1.dot(q2) < 0) {
            q2.coeffs() = -q2.coeffs();
        }

        for (int i = 0; i <= num_interpolation_points_; ++i) {
            double ratio = static_cast<double>(i) / num_interpolation_points_;
            TrajectoryPoint point;

            // 位置线性插值
            point.x = p1.x + (p2.x - p1.x) * ratio;
            point.y = p1.y + (p2.y - p1.y) * ratio;
            point.z = p1.z + (p2.z - p1.z) * ratio;

            // 姿态SLERP插值
            Eigen::Quaterniond q_interp = q1.slerp(ratio, q2);
            q_interp.normalize();
            quaternionToEuler(q_interp, point.r, point.p, point.yaw);

            // 时间戳线性插值
            point.timestamp = p1.timestamp + (p2.timestamp - p1.timestamp) * ratio;
            
            interpolated_points.push_back(point);
        }
    } else {
        // 关节空间插值
        if (p1.joint_positions.size() != p2.joint_positions.size()) {
            throw std::runtime_error("关节数量不匹配");
        }

        for (int i = 0; i <= num_interpolation_points_; ++i) {
            double ratio = static_cast<double>(i) / num_interpolation_points_;
            TrajectoryPoint point;
            
            // 关节角度线性插值
            point.joint_positions.resize(p1.joint_positions.size());
            for (size_t j = 0; j < p1.joint_positions.size(); ++j) {
                point.joint_positions[j] = p1.joint_positions[j] + 
                    (p2.joint_positions[j] - p1.joint_positions[j]) * ratio;
            }

            // 时间戳线性插值
            point.timestamp = p1.timestamp + (p2.timestamp - p1.timestamp) * ratio;
            
            interpolated_points.push_back(point);
        }
    }

    return interpolated_points;
}

std::vector<TrajectoryPoint> TrajectoryInterpolator::getInterpolatedTrajectory() {
    if (trajectory_points_.empty()) {
        throw std::runtime_error("轨迹为空");
    }

    std::vector<TrajectoryPoint> interpolated_trajectory;
    
    for (size_t i = 0; i < trajectory_points_.size() - 1; ++i) {
        auto segment = getInterpolatedSegment(trajectory_points_[i], trajectory_points_[i + 1]);
        
        // 避免重复添加连接点（除了第一段）
        if (i > 0) {
            segment.erase(segment.begin());
        }
        
        interpolated_trajectory.insert(interpolated_trajectory.end(), 
                                    segment.begin(), 
                                    segment.end());
    }

    return interpolated_trajectory;
}

void TrajectoryInterpolator::visualizeTrajectory(const std::vector<TrajectoryPoint>& trajectory) {

}

void TrajectoryInterpolator::interpolateAndVisualize() {
    try {
        auto interpolated_trajectory = getInterpolatedTrajectory();
        visualizeTrajectory(interpolated_trajectory);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
    }
}

// 添加辅助函数用于检查插值模式
bool TrajectoryInterpolator::isJointTrajectory(const TrajectoryPoint& point) {
    return !point.joint_positions.empty();
}

// 添加辅助函数用于验证轨迹点的一致性
bool TrajectoryInterpolator::validateTrajectoryPoints(
    const TrajectoryPoint& p1, const TrajectoryPoint& p2) {
    
    bool is_p1_joint = isJointTrajectory(p1);
    bool is_p2_joint = isJointTrajectory(p2);
    
    // 确保两个点都是同一类型的轨迹
    if (is_p1_joint != is_p2_joint) {
        std::cerr << "轨迹点类型不一致" << std::endl;
        return false;
    }
    
    // 如果是关节轨迹，检查关节数量是否匹配
    if (is_p1_joint && p1.joint_positions.size() != p2.joint_positions.size()) {
        std::cerr << "关节数量不匹配：" << p1.joint_positions.size() << " vs " << p2.joint_positions.size() << std::endl;
        return false;
    }
    
    return true;
}

// 预估下一个点关节角度
TrajectoryPoint TrajectoryInterpolator::estimateNextPoint(
    const TrajectoryPoint& p1, const TrajectoryPoint& p2) {
    
    // 基于前两个点的变化趋势预测第三个点
    TrajectoryPoint estimated_point;
    
    // 时间戳预测（假设等时间间隔）
    double time_interval = p2.timestamp - p1.timestamp;
    estimated_point.timestamp = p2.timestamp + time_interval;
    
    // 如果是关节轨迹
    if (interpolation_mode_ == InterpolationMode::JOINT) {
        // 确保关节数组大小一致
        size_t joint_count = std::max(p1.joint_positions.size(), p2.joint_positions.size());
        estimated_point.joint_positions.resize(joint_count);
        
        // 线性外推预测每个关节角度
        for (size_t i = 0; i < joint_count; ++i) {
            if (i < p1.joint_positions.size() && i < p2.joint_positions.size()) {
                // 计算变化率
                double rate_of_change = (p2.joint_positions[i] - p1.joint_positions[i]) / time_interval;
                // 线性外推
                estimated_point.joint_positions[i] = p2.joint_positions[i] + rate_of_change * time_interval;
                
                // 添加角度限制
                // const double MAX_ANGLE = M_PI;
                // const double MIN_ANGLE = -M_PI;
                // estimated_point.joint_positions[i] = std::min(std::max(
                //     estimated_point.joint_positions[i], MIN_ANGLE), MAX_ANGLE);
            } else if (i < p2.joint_positions.size()) {
                // 如果p1没有这个关节数据，直接使用p2的
                estimated_point.joint_positions[i] = p2.joint_positions[i];
            }
        }
    }
    
    // 如果是笛卡尔轨迹，也进行类似预测
    else{
        if (p1.x != 0 || p1.y != 0 || p1.z != 0) {
            // 位置预测
            double x_rate = (p2.x - p1.x) / time_interval;
            double y_rate = (p2.y - p1.y) / time_interval;
            double z_rate = (p2.z - p1.z) / time_interval;
            
            estimated_point.x = p2.x + x_rate * time_interval;
            estimated_point.y = p2.y + y_rate * time_interval;
            estimated_point.z = p2.z + z_rate * time_interval;
            
            // 姿态预测
            double r_rate = (p2.r - p1.r) / time_interval;
            double p_rate = (p2.p - p1.p) / time_interval;
            double yaw_rate = (p2.yaw - p1.yaw) / time_interval;
            
            estimated_point.r = p2.r + r_rate * time_interval;
            estimated_point.p = p2.p + p_rate * time_interval;
            estimated_point.yaw = p2.yaw + yaw_rate * time_interval;
        }
    }
    
    return estimated_point;
}
