#ifndef REFLECTOR_SLAM_NODE_H
#define REFLECTOR_SLAM_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <laser_reflector_detect/reflectordata.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

// ICP相关结构体
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double _x, double _y) : x(_x), y(_y) {}
};

struct LaserPoint {
    double range;
    double angle;
    double intensity;
    int index;
    Point2D toPoint() const {
        return Point2D(range * cos(angle), range * sin(angle));
    }
};

class ReflectorSlamNode
{
public:
    ReflectorSlamNode();
    ~ReflectorSlamNode();
    bool init();
    void run();

private:
    // 激光数据处理
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
    std::vector<LaserPoint> extractHighIntensityPoints(const sensor_msgs::LaserScan& scan);
    
    // 反光柱检测
    struct Reflector {
        Point2D center;
        double avgIntensity;
        int pointCount;
    };
    std::vector<Reflector> detectReflectors(const std::vector<LaserPoint>& points, const sensor_msgs::LaserScan& scan);
    
    // ICP激光里程计
    bool icpMatch(const std::vector<Point2D>& prevScan, const std::vector<Point2D>& currScan, 
                  Eigen::Matrix3d& transform, double& score);
    std::vector<Point2D> transformPoints(const std::vector<Point2D>& points, const Eigen::Matrix3d& T);
    
    // 建图
    void updateGlobalMap(const std::vector<Reflector>& reflectors, const Eigen::Matrix3d& robotPose);
    void saveMapToCSV(const std::string& filename);
    
    // 发布TF
    void publishTF(const Eigen::Matrix3d& pose);
    
    // 工具函数
    double computeDistance(const Point2D& p1, const Point2D& p2);
    double normalizeAngle(double angle);

private:
    ros::NodeHandle nh_;
    
    // 订阅
    ros::Subscriber laser_sub_;
    
    // 发布
    ros::Publisher path_pub_;
    ros::Publisher reflector_map_pub_;
    ros::Publisher current_pose_pub_;
    ros::Publisher reflector_points_pub_;
    
    // TF
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener* tf_listener_;
    
    // 参数
    double min_reflector_intensity_;
    double reflector_radius_;
    int min_reflector_sample_count_;
    int icp_max_iterations_;
    double icp_convergence_threshold_;
    double map_frame_distance_threshold_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string map_save_path_;
    
    // 状态
    Eigen::Matrix3d current_pose_;  // 机器人当前位姿 (map坐标系)
    Eigen::Matrix3d prev_odom_pose_;
    bool is_first_scan_;
    std::vector<Point2D> prev_scan_points_;
    nav_msgs::Path robot_path_;
    
    // 全局地图中的反光柱
    struct GlobalReflector {
        double x, y;
        int id;
    };
    std::vector<GlobalReflector> global_reflectors_;
    int next_reflector_id_;
};

#endif // REFLECTOR_SLAM_NODE_H