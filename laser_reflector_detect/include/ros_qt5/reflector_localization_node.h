#ifndef REFLECTOR_LOCALIZATION_NODE_H
#define REFLECTOR_LOCALIZATION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
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

class ReflectorLocalizationNode
{
public:
    ReflectorLocalizationNode();
    ~ReflectorLocalizationNode();
    bool init();
    void run();

private:
    // 激光回调
    void laserCallback(const sensor_msgs::LaserScanConstPtr& scan);
    std::vector<LaserPoint> extractHighIntensityPoints(const sensor_msgs::LaserScan& scan);
    
    // 反光柱检测
    struct Reflector {
        Point2D center;
        double avgIntensity;
        int pointCount;
    };
    std::vector<Reflector> detectReflectors(const std::vector<LaserPoint>& points, const sensor_msgs::LaserScan& scan);
    
    // 定位
    struct RobotPose {
        double x, y, theta;
    };
    void matchAndLocalize(const std::vector<Reflector>& localReflectors);
    bool findBestMatch(const std::vector<Reflector>& localReflectors, RobotPose& pose);
    void optimizePose(const std::vector<std::pair<Point2D, Point2D>>& matches, RobotPose& pose);
    
    // 地图加载
    bool loadMapFromCSV(const std::string& filename);
    
    // 发布
    void publishTF(const RobotPose& pose);
    void publishMarkers();
    
    // 工具
    double computeDistance(const Point2D& p1, const Point2D& p2);
    double normalizeAngle(double angle);

private:
    ros::NodeHandle nh_;
    
    // 订阅
    ros::Subscriber laser_sub_;
    
    // 发布
    ros::Publisher pose_pub_;
    ros::Publisher matched_reflectors_pub_;
    ros::Publisher confidence_pub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // 参数
    double min_reflector_intensity_;
    double reflector_radius_;
    int min_reflector_sample_count_;
    double match_distance_threshold_;
    std::string map_frame_;
    std::string lidar_frame_;
    std::string base_frame_;
    std::string map_file_path_;
    int min_match_count_;
    
    // 地图数据
    std::vector<Point2D> map_reflectors_;
    
    // 当前状态
    RobotPose current_pose_;
    bool pose_initialized_;
};

#endif // REFLECTOR_LOCALIZATION_NODE_H