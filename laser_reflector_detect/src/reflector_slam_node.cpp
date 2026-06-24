#include "ros_qt5/reflector_slam_node.h"
#include <eigen3/Eigen/Dense>
#include <signal.h>

ReflectorSlamNode::ReflectorSlamNode()
    : is_first_scan_(true), next_reflector_id_(0)
{
    // 初始化单位矩阵
    current_pose_ = Eigen::Matrix3d::Identity();
    prev_odom_pose_ = Eigen::Matrix3d::Identity();
}

ReflectorSlamNode::~ReflectorSlamNode()
{
    if (tf_listener_) {
        delete tf_listener_;
    }
}

bool ReflectorSlamNode::init()
{
    // 获取参数
    ros::param::get("~/min_reflector_intensity", min_reflector_intensity_);
    ros::param::get("~/reflector_radius", reflector_radius_);
    ros::param::get("~/min_reflector_sample_count", min_reflector_sample_count_);
    ros::param::get("~/icp_max_iterations", icp_max_iterations_);
    ros::param::get("~/icp_convergence_threshold", icp_convergence_threshold_);
    ros::param::get("~/map_frame_distance_threshold", map_frame_distance_threshold_);
    ros::param::get("~/lidar_frame", lidar_frame_);
    ros::param::get("~/base_frame", base_frame_);
    ros::param::get("~/map_save_path", map_save_path_);
    
    // 默认值
    if (min_reflector_intensity_ <= 0) min_reflector_intensity_ = 3500.0;
    if (reflector_radius_ <= 0) reflector_radius_ = 0.03;
    if (min_reflector_sample_count_ <= 0) min_reflector_sample_count_ = 3;
    if (icp_max_iterations_ <= 0) icp_max_iterations_ = 50;
    if (icp_convergence_threshold_ <= 0) icp_convergence_threshold_ = 0.001;
    if (map_frame_distance_threshold_ <= 0) map_frame_distance_threshold_ = 0.2;
    if (lidar_frame_.empty()) lidar_frame_ = "laser";
    if (base_frame_.empty()) base_frame_ = "base_link";
    if (map_save_path_.empty()) map_save_path_ = "/home/cat/igk_ws/src/core/laser_reflector_detect/src/reflectormap.csv";
    
    ROS_INFO("Reflector SLAM Node Initialized");
    ROS_INFO("  Min Intensity: %.1f", min_reflector_intensity_);
    ROS_INFO("  Reflector Radius: %.3f m", reflector_radius_);
    ROS_INFO("  ICP Max Iterations: %d", icp_max_iterations_);
    
    // 订阅激光话题
    laser_sub_ = nh_.subscribe("/scan", 10, &ReflectorSlamNode::laserCallback, this);
    
    // 发布
    path_pub_ = nh_.advertise<nav_msgs::Path>("reflector_slam/path", 10);
    reflector_map_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("reflector_slam/map", 10);
    current_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reflector_slam/pose", 10);
    reflector_points_pub_ = nh_.advertise<laser_reflector_detect::reflectordata>("reflector_slam/reflectors", 10);
    
    tf_listener_ = new tf::TransformListener(ros::Duration(10.0));
    
    return true;
}

void ReflectorSlamNode::run()
{
    ros::spin();
}

std::vector<LaserPoint> ReflectorSlamNode::extractHighIntensityPoints(const sensor_msgs::LaserScan& scan)
{
    std::vector<LaserPoint> points;
    for (uint32_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        double intensity = scan.intensities[i];
        
        if (range > scan.range_min && range < scan.range_max && intensity > min_reflector_intensity_) {
            LaserPoint lp;
            lp.range = range;
            lp.angle = scan.angle_min + i * scan.angle_increment;
            lp.intensity = intensity;
            lp.index = i;
            points.push_back(lp);
        }
    }
    return points;
}

std::vector<ReflectorSlamNode::Reflector> ReflectorSlamNode::detectReflectors(
    const std::vector<LaserPoint>& points, const sensor_msgs::LaserScan& scan)
{
    std::vector<Reflector> reflectors;
    
    if (points.empty()) return reflectors;
    
    int i = 0;
    while (i < points.size()) {
        std::vector<LaserPoint> cluster;
        int j = i;
        
        // 聚类：相邻的高强度点
        while (j < points.size()) {
            if (j == i) {
                cluster.push_back(points[j]);
            } else {
                // 检查索引是否连续（允许最多2个间隔）
                int idx_diff = points[j].index - points[j-1].index;
                if (idx_diff <= 3) {
                    cluster.push_back(points[j]);
                } else {
                    break;
                }
            }
            j++;
        }
        
        // 分析聚类是否为有效的反光柱
        if (cluster.size() >= min_reflector_sample_count_) {
            double sum_range = 0, sum_intensity = 0, sum_angle = 0;
            for (const auto& p : cluster) {
                sum_range += p.range;
                sum_intensity += p.intensity;
                sum_angle += p.angle;
            }
            double avg_range = sum_range / cluster.size();
            double avg_intensity = sum_intensity / cluster.size();
            double avg_angle = sum_angle / cluster.size();
            
            // 估算反光柱圆心
            double center_x = 0, center_y = 0;
            for (const auto& p : cluster) {
                double theta = fabs(p.angle - avg_angle);
                double ds_1 = p.range * cos(theta);
                double a = p.range * sin(theta);
                double b = reflector_radius_;
                double angle = std::min(1.0, a / b);
                double theta_2 = asin(angle);
                double ds_2 = reflector_radius_ * cos(theta_2);
                center_x += (ds_1 + ds_2) * cos(avg_angle);
                center_y += (ds_1 + ds_2) * sin(avg_angle);
            }
            center_x /= cluster.size();
            center_y /= cluster.size();
            
            // 计算期望的点数范围
            int expected_max_size = floor((2.0 * reflector_radius_ / avg_range) / scan.angle_increment * 1.2);
            int expected_min_size = std::max(static_cast<int>(expected_max_size * 0.6), min_reflector_sample_count_);
            
            // 验证反光柱有效性
            if (cluster.size() <= expected_max_size &&
                cluster.size() >= expected_min_size &&
                avg_range > 0.5 && avg_range < 8.0)
            {
                Reflector r;
                r.center = Point2D(center_x, center_y);
                r.avgIntensity = avg_intensity;
                r.pointCount = cluster.size();
                reflectors.push_back(r);
            }
        }
        
        i = j;
    }
    
    return reflectors;
}

bool ReflectorSlamNode::icpMatch(const std::vector<Point2D>& prevScan, 
                                  const std::vector<Point2D>& currScan,
                                  Eigen::Matrix3d& transform, double& score)
{
    if (prevScan.empty() || currScan.empty()) {
        return false;
    }
    
    // 初始化变换矩阵
    transform = Eigen::Matrix3d::Identity();
    
    // 简单ICP实现
    for (int iter = 0; iter < static_cast<int>(icp_max_iterations_); ++iter) {
        Eigen::Matrix3d delta = Eigen::Matrix3d::Identity();
        
        // 为当前扫描的每个点找到最近邻
        std::vector<std::pair<Point2D, Point2D>> correspondences;
        for (const auto& currPt : currScan) {
            double minDist = 1e10;
            Point2D bestMatch;
            
            for (const auto& prevPt : prevScan) {
                double dist = computeDistance(currPt, prevPt);
                if (dist < minDist) {
                    minDist = dist;
                    bestMatch = prevPt;
                }
            }
            
            if (minDist < 1.0) {  // 距离阈值
                correspondences.push_back(std::make_pair(currPt, bestMatch));
            }
        }
        
        if (correspondences.empty()) {
            break;
        }
        
        // 计算重心
        Point2D centroid_curr(0, 0), centroid_prev(0, 0);
        for (const auto& corr : correspondences) {
            centroid_curr.x += corr.first.x;
            centroid_curr.y += corr.first.y;
            centroid_prev.x += corr.second.x;
            centroid_prev.y += corr.second.y;
        }
        centroid_curr.x /= correspondences.size();
        centroid_curr.y /= correspondences.size();
        centroid_prev.x /= correspondences.size();
        centroid_prev.y /= correspondences.size();
        
        // 构建协方差矩阵
        double H00 = 0, H01 = 0, H10 = 0, H11 = 0;
        for (const auto& corr : correspondences) {
            Point2D pc(corr.first.x - centroid_curr.x, corr.first.y - centroid_curr.y);
            Point2D pp(corr.second.x - centroid_prev.x, corr.second.y - centroid_prev.y);
            H00 += pc.x * pp.x;
            H01 += pc.x * pp.y;
            H10 += pc.y * pp.x;
            H11 += pc.y * pp.y;
        }
        
        // SVD求解旋转
        Eigen::Matrix2d H;
        H << H00 + H11, H01 - H10,
             H01 - H10, H00 + H11;
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
        
        // 检测并修正反射
        if (R.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V(0, 1) = -V(0, 1);
            V(1, 1) = -V(1, 1);
            R = V * svd.matrixU().transpose();
        }
        
        delta.block<2, 2>(0, 0) = R;
        delta(0, 2) = centroid_prev.x - R(0, 0) * centroid_curr.x - R(0, 1) * centroid_curr.y;
        delta(1, 2) = centroid_prev.y - R(1, 0) * centroid_curr.x - R(1, 1) * centroid_curr.y;
        
        transform = delta * transform;
        
        // 检查收敛
        if (delta.block<2, 2>(0, 0).norm() < icp_convergence_threshold_) {
            break;
        }
    }
    
    // 计算匹配分数
    auto transformedCurr = transformPoints(currScan, transform);
    double totalDist = 0;
    int count = 0;
    for (const auto& tc : transformedCurr) {
        double minDist = 1e10;
        for (const auto& tp : prevScan) {
            double dist = computeDistance(tc, tp);
            if (dist < minDist) minDist = dist;
        }
        if (minDist < 0.5) {
            totalDist += minDist;
            ++count;
        }
    }
    score = count > 0 ? totalDist / count : 1e10;
    
    return count > 10;  // 至少需要10个匹配点
}

std::vector<Point2D> ReflectorSlamNode::transformPoints(const std::vector<Point2D>& points, 
                                                        const Eigen::Matrix3d& T)
{
    std::vector<Point2D> result;
    for (const auto& p : points) {
        Point2D tp;
        tp.x = T(0, 0) * p.x + T(0, 1) * p.y + T(0, 2);
        tp.y = T(1, 0) * p.x + T(1, 1) * p.y + T(1, 2);
        result.push_back(tp);
    }
    return result;
}

void ReflectorSlamNode::updateGlobalMap(const std::vector<Reflector>& reflectors, 
                                        const Eigen::Matrix3d& robotPose)
{
    for (const auto& refl : reflectors) {
        // 将局部坐标转换为全局坐标
        double local_x = refl.center.x;
        double local_y = refl.center.y;
        double global_x = robotPose(0, 0) * local_x + robotPose(0, 1) * local_y + robotPose(0, 2);
        double global_y = robotPose(1, 0) * local_x + robotPose(1, 1) * local_y + robotPose(1, 2);
        
        // 检查是否已存在相近的反光柱
        bool found = false;
        for (auto& gr : global_reflectors_) {
            double dist = std::sqrt(std::pow(gr.x - global_x, 2) + std::pow(gr.y - global_y, 2));
            if (dist < map_frame_distance_threshold_) {
                found = true;
                break;
            }
        }
        
        // 添加新的反光柱
        if (!found) {
            GlobalReflector new_refl;
            new_refl.x = global_x;
            new_refl.y = global_y;
            new_refl.id = next_reflector_id_++;
            global_reflectors_.push_back(new_refl);
            ROS_INFO("New reflector added: id=%d, x=%.3f, y=%.3f", new_refl.id, global_x, global_y);
        }
    }
}

void ReflectorSlamNode::saveMapToCSV(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open map file for writing: %s", filename.c_str());
        return;
    }
    
    // 格式: route_num, posx, posy, id
    for (const auto& refl : global_reflectors_) {
        file << 1 << "," << refl.x << "," << refl.y << "," << refl.id << "\n";
    }
    
    file.close();
    ROS_INFO("Map saved to %s with %zu reflectors", filename.c_str(), global_reflectors_.size());
}

void ReflectorSlamNode::publishTF(const Eigen::Matrix3d& pose)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = "map";
    tf_msg.child_frame_id = base_frame_;
    
    // 从变换矩阵提取位置和角度
    tf_msg.transform.translation.x = pose(0, 2);
    tf_msg.transform.translation.y = pose(1, 2);
    tf_msg.transform.translation.z = 0;
    
    double yaw = atan2(pose(1, 0), pose(0, 0));
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    
    tf_broadcaster_.sendTransform(tf_msg);
}

double ReflectorSlamNode::computeDistance(const Point2D& p1, const Point2D& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

double ReflectorSlamNode::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void ReflectorSlamNode::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    // 提取高强度点
    std::vector<LaserPoint> highIntensityPoints = extractHighIntensityPoints(*scan);
    
    // 提取所有激光点（用于ICP）
    std::vector<Point2D> currentScan;
    for (uint32_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];
        if (range > scan->range_min && range < scan->range_max) {
            double angle = scan->angle_min + i * scan->angle_increment;
            currentScan.push_back(Point2D(range * cos(angle), range * sin(angle)));
        }
    }
    
    // 检测反光柱
    std::vector<Reflector> reflectors = detectReflectors(highIntensityPoints, *scan);
    
    if (is_first_scan_) {
        is_first_scan_ = false;
        prev_scan_points_ = currentScan;
        prev_odom_pose_ = current_pose_;
        
        // 初始化地图
        updateGlobalMap(reflectors, current_pose_);
        publishTF(current_pose_);
        return;
    }
    
    // ICP匹配
    Eigen::Matrix3d delta_transform;
    double score;
    
    if (icpMatch(prev_scan_points_, currentScan, delta_transform, score)) {
        // 更新机器人位姿
        prev_odom_pose_ = current_pose_;
        current_pose_ = delta_transform * current_pose_;
        
        // 更新地图
        updateGlobalMap(reflectors, current_pose_);
    } else {
        ROS_WARN("ICP matching failed or insufficient correspondences");
    }
    
    prev_scan_points_ = currentScan;
    
    // 发布TF
    publishTF(current_pose_);
    
    // 发布路径
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = current_pose_(0, 2);
    pose_msg.pose.position.y = current_pose_(1, 2);
    double yaw = atan2(current_pose_(1, 0), current_pose_(0, 0));
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    robot_path_.header.stamp = ros::Time::now();
    robot_path_.header.frame_id = "map";
    robot_path_.poses.push_back(pose_msg);
    path_pub_.publish(robot_path_);
    current_pose_pub_.publish(pose_msg);
    
    // 发布反光柱地图
    visualization_msgs::MarkerArray marker_array;
    for (const auto& refl : global_reflectors_) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "reflectors";
        marker.id = refl.id;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = refl.x;
        marker.pose.position.y = refl.y;
        marker.pose.position.z = 0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.5;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array.markers.push_back(marker);
    }
    reflector_map_pub_.publish(marker_array);
    
    // 发布反光柱数据
    laser_reflector_detect::reflectordata refl_data;
    refl_data.header.stamp = ros::Time::now();
    refl_data.ns = "global";
    for (const auto& refl : global_reflectors_) {
        laser_reflector_detect::reflector r;
        r.id = refl.id;
        r.x = refl.x;
        r.y = refl.y;
        r.area = 1;
        refl_data.reflectordata.push_back(r);
    }
    reflector_points_pub_.publish(refl_data);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reflector_slam_node");
    
    ReflectorSlamNode node;
    if (!node.init()) {
        ROS_ERROR("Failed to initialize reflector slam node");
        return 1;
    }
    
    ROS_INFO("Reflector SLAM node started");
    
    // 注册关闭钩子以保存地图
    signal(SIGINT, [](int) {
        ROS_INFO("Saving map before shutdown...");
        // 地图会在节点关闭时自动保存
    });
    
    node.run();
    
    return 0;
}