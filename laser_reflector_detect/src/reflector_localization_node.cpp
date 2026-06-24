#include "ros_qt5/reflector_localization_node.h"
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float32.h>

ReflectorLocalizationNode::ReflectorLocalizationNode()
    : tf_listener_(tf_buffer_), pose_initialized_(false)
{
    current_pose_.x = 0;
    current_pose_.y = 0;
    current_pose_.theta = 0;
}

ReflectorLocalizationNode::~ReflectorLocalizationNode()
{
}

bool ReflectorLocalizationNode::init()
{
    // 获取参数
    ros::param::get("~/min_reflector_intensity", min_reflector_intensity_);
    ros::param::get("~/reflector_radius", reflector_radius_);
    ros::param::get("~/min_reflector_sample_count", min_reflector_sample_count_);
    ros::param::get("~/match_distance_threshold", match_distance_threshold_);
    ros::param::get("~/map_frame", map_frame_);
    ros::param::get("~/lidar_frame", lidar_frame_);
    ros::param::get("~/base_frame", base_frame_);
    ros::param::get("~/map_file_path", map_file_path_);
    ros::param::get("~/min_match_count", min_match_count_);
    
    // 默认值
    if (min_reflector_intensity_ <= 0) min_reflector_intensity_ = 3500.0;
    if (reflector_radius_ <= 0) reflector_radius_ = 0.03;
    if (min_reflector_sample_count_ <= 0) min_reflector_sample_count_ = 3;
    if (match_distance_threshold_ <= 0) match_distance_threshold_ = 0.2;
    if (map_frame_.empty()) map_frame_ = "map";
    if (lidar_frame_.empty()) lidar_frame_ = "laser";
    if (base_frame_.empty()) base_frame_ = "base_link";
    if (min_match_count_ <= 0) min_match_count_ = 3;
    
    std::string default_map_path = "/home/cat/igk_ws/src/core/laser_reflector_detect/src/reflectormap.csv";
    if (map_file_path_.empty()) {
        map_file_path_ = default_map_path;
    }
    
    ROS_INFO("Reflector Localization Node Initialized");
    ROS_INFO("  Map file: %s", map_file_path_.c_str());
    ROS_INFO("  Min Match Count: %d", min_match_count_);
    ROS_INFO("  Match Distance Threshold: %.3f m", match_distance_threshold_);
    
    // 加载地图
    if (!loadMapFromCSV(map_file_path_)) {
        ROS_ERROR("Failed to load map from %s", map_file_path_.c_str());
        return false;
    }
    
    ROS_INFO("  Loaded %zu reflectors from map", map_reflectors_.size());
    
    // 订阅
    laser_sub_ = nh_.subscribe("/scan", 10, &ReflectorLocalizationNode::laserCallback, this);
    
    // 发布
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("reflector_localization/pose", 10);
    matched_reflectors_pub_ = nh_.advertise<laser_reflector_detect::reflectordata>("reflector_localization/matched", 10);
    confidence_pub_ = nh_.advertise<std_msgs::Float32>("reflector_localization/confidence", 10);
    
    return true;
}

void ReflectorLocalizationNode::run()
{
    ros::spin();
}

bool ReflectorLocalizationNode::loadMapFromCSV(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open map file: %s", filename.c_str());
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> values;
        
        while (std::getline(ss, cell, ',')) {
            try {
                values.push_back(std::stod(cell));
            } catch (...) {
                continue;
            }
        }
        
        if (values.size() >= 3) {
            // 格式: route_num, posx, posy, id(可选)
            Point2D pt(values[1], values[2]);
            map_reflectors_.push_back(pt);
        }
    }
    
    file.close();
    return !map_reflectors_.empty();
}

std::vector<LaserPoint> ReflectorLocalizationNode::extractHighIntensityPoints(const sensor_msgs::LaserScan& scan)
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

std::vector<ReflectorLocalizationNode::Reflector> ReflectorLocalizationNode::detectReflectors(
    const std::vector<LaserPoint>& points, const sensor_msgs::LaserScan& scan)
{
    std::vector<Reflector> reflectors;
    
    if (points.empty()) return reflectors;
    
    int i = 0;
    while (i < points.size()) {
        std::vector<LaserPoint> cluster;
        int j = i;
        
        while (j < points.size()) {
            if (j == i) {
                cluster.push_back(points[j]);
            } else {
                int idx_diff = points[j].index - points[j-1].index;
                if (idx_diff <= 3) {
                    cluster.push_back(points[j]);
                } else {
                    break;
                }
            }
            j++;
        }
        
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
            
            int expected_max_size = floor((2.0 * reflector_radius_ / avg_range) / scan.angle_increment * 1.2);
            int expected_min_size = std::max(static_cast<int>(expected_max_size * 0.6), min_reflector_sample_count_);
            
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

void ReflectorLocalizationNode::matchAndLocalize(const std::vector<Reflector>& localReflectors)
{
    if (map_reflectors_.empty() || localReflectors.empty()) {
        return;
    }
    
    RobotPose estimatedPose;
    if (!findBestMatch(localReflectors, estimatedPose)) {
        ROS_WARN("Could not find valid pose match");
        return;
    }
    
    // 更新位姿（带平滑）
    if (pose_initialized_) {
        double alpha = 0.7;  // 平滑系数
        current_pose_.x = alpha * estimatedPose.x + (1 - alpha) * current_pose_.x;
        current_pose_.y = alpha * estimatedPose.y + (1 - alpha) * current_pose_.y;
        current_pose_.theta = normalizeAngle(alpha * estimatedPose.theta + (1 - alpha) * current_pose_.theta);
    } else {
        current_pose_ = estimatedPose;
        pose_initialized_ = true;
    }
    
    publishTF(current_pose_);
    
    // 发布位姿
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = current_pose_.x;
    pose_msg.pose.position.y = current_pose_.y;
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(current_pose_.theta);
    pose_pub_.publish(pose_msg);
    
    // 发布匹配的反光柱
    laser_reflector_detect::reflectordata matched_data;
    matched_data.header.stamp = ros::Time::now();
    matched_data.ns = "matched";
    for (const auto& refl : localReflectors) {
        laser_reflector_detect::reflector r;
        r.x = refl.center.x;
        r.y = refl.center.y;
        r.area = 0;
        r.id = 0;
        matched_data.reflectordata.push_back(r);
    }
    matched_reflectors_pub_.publish(matched_data);
    
    // 置信度
    std_msgs::Float32 confidence;
    confidence.data = std::min(1.0f, static_cast<float>(localReflectors.size()) / 5.0f);
    confidence_pub_.publish(confidence);
}

bool ReflectorLocalizationNode::findBestMatch(const std::vector<Reflector>& localReflectors, RobotPose& pose)
{
    // 局部反光柱两两之间的距离和角度
    struct LocalInfo {
        Point2D pt;
        int id1, id2;
        double dist;
        double angle1, angle2;
    };
    
    std::vector<LocalInfo> localInfos;
    for (int i = 0; i < static_cast<int>(localReflectors.size()) - 1; ++i) {
        for (int j = i + 1; j < static_cast<int>(localReflectors.size()); ++j) {
            LocalInfo info;
            info.pt = localReflectors[i].center;
            info.id1 = i;
            info.id2 = j;
            info.dist = computeDistance(localReflectors[i].center, localReflectors[j].center);
            info.angle1 = atan2(localReflectors[i].center.y, localReflectors[i].center.x);
            info.angle2 = atan2(localReflectors[j].center.y, localReflectors[j].center.x);
            localInfos.push_back(info);
        }
    }
    
    // 地图反光柱两两之间的距离和角度
    struct MapInfo {
        Point2D pt1, pt2;
        double dist;
    };
    
    std::vector<MapInfo> mapInfos;
    for (int i = 0; i < static_cast<int>(map_reflectors_.size()) - 1; ++i) {
        for (int j = i + 1; j < static_cast<int>(map_reflectors_.size()); ++j) {
            MapInfo info;
            info.pt1 = map_reflectors_[i];
            info.pt2 = map_reflectors_[j];
            info.dist = computeDistance(map_reflectors_[i], map_reflectors_[j]);
            mapInfos.push_back(info);
        }
    }
    
    // 尝试不同的局部-地图配对
    std::vector<std::pair<Point2D, Point2D>> bestMatches;
    double bestScore = 1e10;
    
    for (const auto& localInfo : localInfos) {
        for (const auto& mapInfo : mapInfos) {
            // 距离匹配
            double distRatio = localInfo.dist / (mapInfo.dist + 0.001);
            if (distRatio < 0.8 || distRatio > 1.2) continue;
            
            // 角度匹配
            double localAngle = atan2(localInfo.pt.y, localInfo.pt.x);
            double mapAngle1 = atan2(mapInfo.pt1.y, mapInfo.pt1.x);
            double angleDiff = fabs(normalizeAngle(localAngle - mapAngle1));
            if (angleDiff > 0.5) continue;
            
            // 得分
            double score = fabs(distRatio - 1.0) + angleDiff;
            if (score < bestScore) {
                bestScore = score;
                
                // 记录匹配
                bestMatches.clear();
                bestMatches.push_back(std::make_pair(localInfo.pt, mapInfo.pt1));
                bestMatches.push_back(std::make_pair(localReflectors[localInfo.id2].center, mapInfo.pt2));
            }
        }
    }
    
    if (bestMatches.size() >= 2) {
        // 使用三角形定位计算位姿
        // 已知：局部坐标点A、B和对应的地图坐标点A'、B'
        // 求：变换矩阵 (R, t) 使得 R*A + t = A', R*B + t = B'
        
        // 计算旋转和平移
        Point2D local_centroid(0, 0), map_centroid(0, 0);
        for (const auto& m : bestMatches) {
            local_centroid.x += m.first.x;
            local_centroid.y += m.first.y;
            map_centroid.x += m.second.x;
            map_centroid.y += m.second.y;
        }
        local_centroid.x /= bestMatches.size();
        local_centroid.y /= bestMatches.size();
        map_centroid.x /= bestMatches.size();
        map_centroid.y /= bestMatches.size();
        
        // 计算旋转
        double H00 = 0, H01 = 0, H10 = 0, H11 = 0;
        for (const auto& m : bestMatches) {
            double dx = m.first.x - local_centroid.x;
            double dy = m.first.y - local_centroid.y;
            double dx_prime = m.second.x - map_centroid.x;
            double dy_prime = m.second.y - map_centroid.y;
            H00 += dx * dx_prime + dy * dy_prime;
            H01 += dx * dy_prime - dy * dx_prime;
            H10 += dx * dy_prime - dy * dx_prime;
            H11 += dx * dx_prime + dy * dy_prime;
        }
        
        double theta = atan2(H01 + H10, H00 - H11);
        double cos_t = cos(theta);
        double sin_t = sin(theta);
        
        pose.x = map_centroid.x - cos_t * local_centroid.x + sin_t * local_centroid.y;
        pose.y = map_centroid.y - sin_t * local_centroid.x - cos_t * local_centroid.y;
        pose.theta = theta;
        
        // 优化位姿
        optimizePose(bestMatches, pose);
        
        return true;
    }
    
    return false;
}

void ReflectorLocalizationNode::optimizePose(const std::vector<std::pair<Point2D, Point2D>>& matches, RobotPose& pose)
{
    // 使用Ceres-like优化（简化的梯度下降）
    double x = pose.x, y = pose.y, theta = pose.theta;
    
    for (int iter = 0; iter < 20; ++iter) {
        double dx = 0, dy = 0, dtheta = 0;
        double cost = 0;
        
        for (const auto& m : matches) {
            double local_x = m.first.x;
            double local_y = m.first.y;
            double map_x = m.second.x;
            double map_y = m.second.y;
            
            // 预测的地图坐标
            double pred_x = x + cos(theta) * local_x - sin(theta) * local_y;
            double pred_y = y + sin(theta) * local_x + cos(theta) * local_y;
            
            // 误差
            double ex = map_x - pred_x;
            double ey = map_y - pred_y;
            cost += ex * ex + ey * ey;
            
            // 梯度
            dx += ex;
            dy += ey;
            dtheta += -(local_x * sin(theta) + local_y * cos(theta)) * ex +
                        (local_x * cos(theta) - local_y * sin(theta)) * ey;
        }
        
        double step = 0.5;
        x += step * dx / matches.size();
        y += step * dy / matches.size();
        theta += step * dtheta / matches.size();
        theta = normalizeAngle(theta);
    }
    
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
}

void ReflectorLocalizationNode::publishTF(const RobotPose& pose)
{
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = ros::Time::now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = base_frame_;
    
    tf_msg.transform.translation.x = pose.x;
    tf_msg.transform.translation.y = pose.y;
    tf_msg.transform.translation.z = 0;
    
    tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, pose.theta);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    
    tf_broadcaster_.sendTransform(tf_msg);
}

void ReflectorLocalizationNode::publishMarkers()
{
    visualization_msgs::MarkerArray marker_array;
    
    // 地图反光柱（绿色）
    for (size_t i = 0; i < map_reflectors_.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = map_frame_;
        marker.ns = "map_reflectors";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = map_reflectors_[i].x;
        marker.pose.position.y = map_reflectors_[i].y;
        marker.pose.position.z = 0;
        marker.scale.x = 0.15;
        marker.scale.y = 0.15;
        marker.scale.z = 0.3;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 0.8;
        marker_array.markers.push_back(marker);
    }
    
    // 可以发布其他可视化...
}

double ReflectorLocalizationNode::computeDistance(const Point2D& p1, const Point2D& p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

double ReflectorLocalizationNode::normalizeAngle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void ReflectorLocalizationNode::laserCallback(const sensor_msgs::LaserScanConstPtr& scan)
{
    // 提取高强度点
    std::vector<LaserPoint> highIntensityPoints = extractHighIntensityPoints(*scan);
    
    // 检测反光柱
    std::vector<Reflector> reflectors = detectReflectors(highIntensityPoints, *scan);
    
    if (reflectors.size() >= static_cast<size_t>(min_match_count_)) {
        matchAndLocalize(reflectors);
    }
    
    publishMarkers();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reflector_localization_node");
    
    ReflectorLocalizationNode node;
    if (!node.init()) {
        ROS_ERROR("Failed to initialize reflector localization node");
        return 1;
    }
    
    ROS_INFO("Reflector Localization node started");
    node.run();
    
    return 0;
}