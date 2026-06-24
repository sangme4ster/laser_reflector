/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/ros_qt5/qnode.h"
#include "visualization_msgs/MarkerArray.h"
#include "math.h"
#include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>

#include <visualization_msgs/Marker.h>

#include <laser_reflector_detect/reflectordata.h>


//  #define VISUAL_RVIZ

/*****************************************************************************
** Namespaces
*****************************************************************************/
int set_go_flag = 0;
int set_num = 0;
int go_num = 0;
bool save_point_ok_flag = true;
bool save_csv_ok_flag = true;


// 服务传递的参数变量
//  允许添加反光柱
bool SrvAddEnable = false;
// 清空反光柱
bool SrvClear = false;
// 删除指定id的反光柱[大于-1说明需要删除]
int SrvId = -1;

visualization_msgs::Marker marker_;

laser_reflector_detect::reflectordata reflectormsgSaved;    // 已存储发反光柱坐标集合
laser_reflector_detect::reflectordata reflectormsgRealTime; // 实时反光柱坐标集合
laser_reflector_detect::reflector reflectorbase;

using namespace std;
QNode::QNode(ros::NodeHandle node_handle, tf2_ros::Buffer *buffer)
    : tf_buffer_node_(buffer)
{
    n = node_handle;
    tf_ = new tf::TransformListener(ros::Duration(10.0));
}

QNode::~QNode()
{
    if (ros::isStarted())
    {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    // wait();
    if (tf_ != NULL)
        delete tf_;
}

int QNode::round_int(double val)
{
    return (val > 0.0) ? floor(val + 0.5) : ceil(val - 0.5);
}
////从csv文件中读取reflector map pose
int linecnt = 0;

void QNode::loadmap()
{
    std::ifstream indata;
    std::string filename = "/home/cat/igk_ws/src/core/laser_reflector_detect/src/savereflector.csv";
    indata.open(filename);

    if (!indata.is_open())
    {
        ROS_ERROR("无法打开文件: %s", filename.c_str());
        return;
    }

    std::string line;
    int linecnt = 0;
    csv_reflector_info_vector.clear();

    while (std::getline(indata, line))
    {
        linecnt++;
        std::stringstream lineStream(line);
        std::string cell;
        reflector_info tmp_info;
        double getpose[7] = {0.0}; // 初始化为零
        int i = 0;

        while (std::getline(lineStream, cell, ','))
        {
            if (i >= 7)
            {
                ROS_WARN("在第 %d 行超出预期的列数，忽略多余数据。", linecnt);
                break; // 防止溢出
            }
            try
            {
                getpose[i] = std::stod(cell); // 字符串转为 double
            }
            catch (const std::invalid_argument &e)
            {
                ROS_ERROR("第 %d 行，第 %d 列数据无效: %s", linecnt, i, e.what());
                break; // 停止解析该行
            }
            catch (const std::out_of_range &e)
            {
                ROS_ERROR("第 %d 行，第 %d 列值超出范围: %s", linecnt, i, e.what());
                break; // 停止解析该行
            }
            i++;
        }

        // 仅在读取到足够数据时才推入向量
        if (i >= 4)
        { // 根据需要的最少变量调整
            tmp_info.route_num = getpose[0];
            tmp_info.posx = getpose[1];
            tmp_info.posy = getpose[2];
            tmp_info.id = getpose[3];
            csv_reflector_info_vector.push_back(tmp_info);
        }
        else
        {
            ROS_WARN("第 %d 行数据不足，期望至少有 4 个值。", linecnt);
        }
    }
    std::cout << "route_num, posx, posy, id" << std::endl;
    for (auto info : csv_reflector_info_vector)
    {
        std::cout << info.route_num << ", "
                  << info.posx << ", "
                  << info.posy << ", "
                  << info.id << std::endl;
    }
    ROS_INFO("处理的总行数 = %d", linecnt);
    indata.close();
}

bool QNode::init()
{
    if (!ros::master::check())
    {
        ROS_INFO("ros master fail");
        return false;
    }
    else
    {
        ROS_INFO("ros master success");
    }

    ros::param::get("/laser_reflector_detect_node/reflector_combined_length", reflector_combined_length);
    // 反光柱的半径
    ros::param::get("/laser_reflector_detect_node/reflector_radius", reflector_radius);
    // 检测反光柱的最小点数
    ros::param::get("/laser_reflector_detect_node/min_reflector_sample_count", min_reflector_sample_count);
    // 检测反光柱的最小强度
    ros::param::get("/laser_reflector_detect_node/min_reflector_intensity", min_reflector_intensity);
    // 导航时定位的反光柱的大小
    ros::param::get("/laser_reflector_detect_node/kLandmarkMarkerScale", kLandmarkMarkerScale);
    // 使用反光柱的模式
    ros::param::get("/laser_reflector_detect_node/reflector_combination_mode", reflector_combination_mode);
    // 激光坐标系的名称
    ros::param::get("/laser_reflector_detect_node/lidar_frame", lidar_frame_);
    // 地图坐标系的名称
    ros::param::get("/laser_reflector_detect_node/map_frame", map_frame_);
    // 导航时可允许的反光柱检测误差
    ros::param::get("/laser_reflector_detect_node/match_distance_accepted", match_distance_acceped);

    // 导航时可允许的反光柱检测误差
    ros::param::get("/laser_reflector_detect_node/work_mode", work_mode);

    cout << "lidar_frame_: " << lidar_frame_ << endl;
    cout << "map_frame_: " << map_frame_ << endl;
    cout << "match_distance_acceped: " << match_distance_acceped << endl;

    cout << "work_mode: " << work_mode << endl;

    // 订阅激光话题
    scan_sub_ = n.subscribe("/scan", 1, &QNode::laserProcess, this);
    // 订阅TF
    tf_sub_ = n.subscribe("/tf", 1, &QNode::tfProcess, this);

    // 订阅全局反光板话题
    global_reflector_pos_sub_ = n.subscribe("/landmark_poses_list", 1, &QNode::getGlobalReflector, this);

    // 订阅机器人当前位姿话题
    //  kobe
    robot_pose_sub_ = n.subscribe("/robot_pose", 1, &QNode::getRobotPose, this);
    tracked_pose_sub_ = n.subscribe("/tracked_pose", 1, &QNode::getTrackedPose, this);
    // 检测到的反光板
    reflector_points_ = n.advertise<geometry_msgs::PointStamped>("reflector_points", 1);

    // 发送给cartographer的landmark
    reflector_landmark_ = n.advertise<cartographer_ros_msgs::LandmarkList>("Laserlandmark", 1);

    // 定位时检测到的反光板
    current_landmark_list_pub_ = n.advertise<::visualization_msgs::MarkerArray>("current_landmark_list", 1);

    // 机器人到雷达的外参
    // imu_link_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0.34, 0.02, 0.4));
    imu_link_to_lidar = tf::Transform(tf::createQuaternionFromRPY(0, 0, 3.14), tf::Vector3(-0.34, 0.02, 0.4));

    // 雷达未矫正的移动路线
    lidar_uncorrected_pub_ = n.advertise<nav_msgs::Path>("lidar/uncorrcted_path", 1);

    // 雷达矫正后的移动路线
    lidar_corrected_pub_ = n.advertise<nav_msgs::Path>("lidar/corrcted_path", 1);

    // 雷达矫正与未矫正之间的连线
    error_edge_pub_ = n.advertise<visualization_msgs::MarkerArray>("/lidar/error_edge", 1);

    // 反光柱矫正后的机器人位姿
    robot_pose_pub_ = n.advertise<cartographer_ros_msgs::RobotPose>("/reflector_robot_pose", 1);

    pub_marker_ = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    pub_reflectormsgSaved = n.advertise<laser_reflector_detect::reflectordata>("reflectormsgSaved", 1);
    pub_reflectormsgRealTime = n.advertise<laser_reflector_detect::reflectordata>("reflectormsgRealTime", 1);
    // 从csv文件构建反光住特征地图
    loadmap();
    return true;
}

void QNode::log(const LogLevel &level, const string &msg)
{
}

// kobe add
// 保存landmark全局坐标
void QNode::savecsvfuc()
{
    ofstream outFile1;
    char const *ch = "/home/cat/igk_ws/src/core/laser_reflector_detect/src/savereflector.csv";
    outFile1.open(ch);
    int i = 0;
    for (auto item : csv_reflector_info_vector)
    {
        outFile1 << item.route_num << ",";
        outFile1 << item.posx << ",";
        outFile1 << item.posy << ",";
        outFile1 << i;
        outFile1 << "\n";
        i++;
    }
    outFile1.close();
    ROS_INFO("save csv ok\n");
}

// 用CERES 优化 pose
void QNode::OptimizeCurrentPose(vector<pair<pair<double, double>, pair<double, double>>> &match_result, RobotPose &robot_pose)
{

    ceres::Problem problem;
    double pose[3] = {robot_pose.x, robot_pose.y, robot_pose.theta};

    for (auto fpt_pair : match_result)
    {
        problem.AddResidualBlock(new ceres::AutoDiffCostFunction<FeaturePairCost, 2, 3>(
                                     new FeaturePairCost(fpt_pair.second, fpt_pair.first)),
                                 new ceres::CauchyLoss(0.2), pose);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    robot_pose.x = pose[0];
    robot_pose.y = pose[1];
    robot_pose.theta = pose[2];
}

inline double QNode::ComputeRadius(pair<double, double> &pt)
{
    return sqrt(pow(pt.first, 2) + pow(pt.second, 2));
}
inline double QNode::ComputeDistance(pair<double, double> &pt1, pair<double, double> &pt2)
{
    return sqrt(pow(pt1.first - pt2.first, 2) + pow(pt1.second - pt2.second, 2));
}

bool QNode::CheckResultValid(vector<double> &radius_list,
                             vector<pair<pair<double, double>, pair<double, double>>> &match_result,
                             pair<double, double> &result, int id1, int id2, double d12)
{
    int result_sz = match_result.size();
    double r1 = radius_list[id1], r2 = radius_list[id2];
    pair<double, double> pt1 = match_result[id1].first, pt2 = match_result[id2].first;

    double a = (r1 * r1 - r2 * r2 + d12 * d12) / (2 * d12); //
    double x0 = pt1.first + a / d12 * (pt2.first - pt1.first);
    double y0 = pt1.second + a / d12 * (pt2.second - pt1.second);

    double h = sqrt(r1 * r1 - a * a);
    pair<double, double> res_1, res_2;
    res_1.first = x0 + h / d12 * (pt2.second - pt1.second);
    res_1.second = y0 - h / d12 * (pt2.first - pt1.first);
    res_2.first = x0 - h / d12 * (pt2.second - pt1.second);
    res_2.second = y0 + h / d12 * (pt2.first - pt1.first);

    for (int id = 0; id < result_sz; id++)
    {
        if (id != id1 && id != id2)
        {
            pair<double, double> pt_check = match_result[id].first;
            double r_check = radius_list[id];

            double check_distance_1 = fabs(ComputeDistance(res_1, pt_check) - r_check);
            double check_distance_2 = fabs(ComputeDistance(res_2, pt_check) - r_check);

            if (check_distance_1 <= check_distance_2)
            {
                if (check_distance_1 < match_distance_acceped)
                {
                    result = res_1;
                    return true;
                }
            }
            else
            {
                if (check_distance_2 < match_distance_acceped)
                {
                    result = res_2;
                    return true;
                }
            }
        }
    }
    return false;
}
/**
 * @brief
 *
 * @param match_result global points - local points
 * @param robot_pose
 * @return true
 * @return false
 */
bool QNode::InitialCurrentPose(vector<pair<pair<double, double>, pair<double, double>>> &match_result,
                               RobotPose &robot_pose)
{
    int result_sz = match_result.size();
    vector<double> radius_list;
    for (int i = 0; i < result_sz; i++)
    {
        radius_list.push_back(ComputeRadius(match_result[i].second));
    }

    for (int i = 0; i < result_sz - 1; i++)
    {
        for (int j = i + 1; j < result_sz; j++)
        {
            double dij = ComputeDistance(match_result[i].first, match_result[j].first);
            double score = (radius_list[i] + radius_list[j]) / dij;
            if (score > 1.2)
            {
                pair<double, double> res;
                // 0414
                if (CheckResultValid(radius_list, match_result, res, i, j, dij))
                {
                    pair<double, double> global_tar_pt = match_result[i].first;
                    pair<double, double> local_pt = match_result[i].second;
                    double angle_global = atan2(global_tar_pt.second - res.second, global_tar_pt.first - res.first);
                    double angle_local = atan2(local_pt.second, local_pt.first);

                    double initial_value_angle = angle_global - angle_local;
                    // 数值有时在±π区间之外
                    //                     double tmp_ratio = (abs(initial_value_angle))/(2*M_PI);
                    //                     unsigned int  tmp_ratio_i = floor(tmp_ratio);
                    //                     if(tmp_ratio > 1)
                    //                     {
                    //                         if(initial_value_angle > 0)
                    //                         {
                    //                             initial_value_angle = initial_value_angle - tmp_ratio_i*2*M_PI;
                    //                         }
                    //                         else if(initial_value_angle < 0)
                    //                         {
                    //                             initial_value_angle = initial_value_angle + tmp_ratio_i*2*M_PI;
                    //                         }
                    //                     }
                    if (initial_value_angle > 2 * M_PI)
                    {
                        initial_value_angle -= 2 * M_PI;
                    }
                    else if (initial_value_angle < -2 * M_PI)
                    {
                        initial_value_angle += 2 * M_PI;
                    }
                    robot_pose.x = res.first;
                    robot_pose.y = res.second;
                    robot_pose.theta = initial_value_angle;
                    return true;
                }
            }
        }
    }

    return false;
}

// 发布标记
void QNode::Publish()
{
    // while (pub_marker_.getNumSubscribers() < 1)
    // {
    //     sleep(1);
    // }
    marker_.header.stamp = ros::Time();
    pub_marker_.publish(marker_);
}

// 设置反光柱标记
void QNode::set_marker_fixed_property(float mx, float my, int i, string str)
{
    /*决定从哪个视图可以看到标记r*/
    marker_.header.frame_id = "map";
    marker_.ns = str;
    marker_.id = i;
    // 设置标记类型：圆柱体
    marker_.type = visualization_msgs::Marker::CYLINDER;

    // 设置标记坐标
    marker_.pose.position.x = mx;
    marker_.pose.position.y = my;
    marker_.pose.position.z = 0;
    if (str == "RealReflector")
    {
        // 设置标记尺寸
        marker_.scale.x = 0.2; // m
        marker_.scale.y = 0.2;
        marker_.scale.z = 0.2;

        /// 设置标记颜色
        marker_.color.a = 0.5; // Don't forget to set the alpha!
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 0.0;
        // 设置标记动作
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.lifetime = ros::Duration(0.5); //(sec,nsec),0 forever
    }
    else
    {
        // 设置标记尺寸
        marker_.scale.x = 0.09; // m
        marker_.scale.y = 0.09;
        marker_.scale.z = 0.50;

        /// 设置标记颜色
        marker_.color.a = 1.0; // Don't forget to set the alpha!
        marker_.color.r = 1.0;
        marker_.color.g = 0.0;
        marker_.color.b = 0.0;

        // 设置标记动作
        marker_.action = visualization_msgs::Marker::ADD;
        marker_.lifetime = ros::Duration(); //(sec,nsec),0 forever
    }
}
void QNode::tfProcess(const tf2_msgs::TFMessage &tfMessage) {
    if (tfMessage.transforms.empty()) {
        ROS_WARN("No transforms received!");
        return;
    }

    const auto &transform = tfMessage.transforms[0];

    // 检查 child_frame_id 和 header.frame_id
    if (transform.child_frame_id == "/laser" && transform.header.frame_id == "/base_link" && tf_laser_baselink_first) {
        tf_laser_baselink_first = false;
        // 提取 translation 对象中的 x, y, z
        const auto &translation = transform.transform.translation;
        lidarAttributes.offsetX = translation.x;  // 激光雷达相对于机器人的 X 位置
        lidarAttributes.offsetY = translation.y;  // 激光雷达相对于机器人的 Y 位置
        lidarAttributes.offsetZ = translation.z;  // 激光雷达相对于机器人的 Z 位置

        // 提取 rotation 对象中的 x, y, z, w
        const auto &quaternion = transform.transform.rotation;

        // 直接计算欧拉角
        double sinr_cosp = 2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
        double cosr_cosp = 1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
        lidarAttributes.rotationRoll = std::atan2(sinr_cosp, cosr_cosp);

        double sinp = 2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
        if (std::abs(sinp) >= 1)
            lidarAttributes.rotationPitch = std::copysign(M_PI / 2, sinp); // 超出范围时使用 90 度
        else
            lidarAttributes.rotationPitch = std::asin(sinp);

        double siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
        double cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
        lidarAttributes.rotationYaw = std::atan2(siny_cosp, cosy_cosp);

        imu_link_to_lidar = tf::Transform(tf::createQuaternionFromRPY(lidarAttributes.rotationRoll,  lidarAttributes.rotationPitch, lidarAttributes.rotationYaw), tf::Vector3(lidarAttributes.offsetX, lidarAttributes.offsetY, lidarAttributes.offsetZ));
        
        // 打印结果
        ROS_INFO("Lidar Offset: [%f, %f, %f]", lidarAttributes.offsetX, lidarAttributes.offsetY, lidarAttributes.offsetZ);
        ROS_INFO("Lidar Rotation: [roll: %f, pitch: %f, yaw: %f]", lidarAttributes.rotationRoll, lidarAttributes.rotationPitch, lidarAttributes.rotationYaw);
    }
}

void QNode::laserProcess(const sensor_msgs::LaserScanConstPtr &scan)
{
    // 检测是否清除反光住
    if (SrvClear == true)
    {
        SrvClear = false;
        // 清空向量
        csv_reflector_info_vector.clear();
        // 保存文件
        QNode::savecsvfuc();
    }


    int reflectornumber = 0;
    tf::Pose laser_pose;
    vector<laser_processor::Sample *> scan_filter;

    for (uint32_t i = 0; i < scan->ranges.size(); i++)
    {
        laser_processor::Sample *s = laser_processor::Sample::Extract(i, min_reflector_intensity, *scan);
        if (s != NULL)
        {
            scan_filter.push_back(s);
        }
    }
    ROS_INFO("---------------------------------------------------------");
    ROS_INFO("scan_filter.size = %d", scan_filter.size());
    // if (scan_filter.size() == 0)
    //     return;

    // 获取反光住的圆心
    double center_x, center_y, center_yaw, center_count, center_distance;
    vector<Reflector_pos> reflectors_;
    bool is_mark_start = true;

    // 存储反光柱临时变量
    vector<pair<double, double>> reflector_data; // first is angle, second is distance;
    // 存储反光柱强度数据
    vector<int> intensity_data;

    // 清空未存储的反光柱坐标
    reflectormsgRealTime.reflectordata.clear();
    reflectormsgSaved.header.stamp = ros::Time::now();
    reflectormsgSaved.ns = "RealTime";

    cartographer_ros_msgs::RobotPose current_pose;
    current_pose.covariance_score = 0;
    // 检测有多少个反光柱
    for (int i = 0; i < scan_filter.size(); i++)
    {
        is_mark_start = true;

        for (int j = i; j < scan_filter.size(); j++)
        {
            laser_processor::Sample *p = scan_filter[j];

            double item_r = p->range;
            double item_i = p->intensity;
            // origin angle range is -180~180,transfer to 0~360
            double item_a = scan->angle_min + p->index * scan->angle_increment + M_PI;

            if (is_mark_start == true)
            {

                center_count = 1;
                // 开始记录单个反光柱的角度和距离
                reflector_data.clear();
                reflector_data.push_back(pair<double, double>(item_a, item_r));
                // cout << "yf.agv index " << p->index << " item_angle " << item_a << endl;
                // 开始记录单个反光柱的强度
                intensity_data.clear();
                intensity_data.push_back(item_i);

                is_mark_start = false;
            }
            else
            {
                // 检测强度点是否连续来进行分割
                if ((p->index - last_index) < 3 && (j != scan_filter.size() - 1))
                {
                    reflector_data.push_back(pair<double, double>(item_a, item_r));

                    intensity_data.push_back(item_i);

                    // ROS_INFO("p->index = %d,last_index = %d,j = %d,center_count = %.2f", p->index, last_index, j, center_count);

                    center_count++;
                }
                else
                {
                    // 记录当前的反光柱点平均距离
                    double sum_range_of_target = 0;
                    for (auto it : reflector_data)
                    {
                        sum_range_of_target += it.second;
                    }
                    double avg_range = sum_range_of_target / reflector_data.size();

                    // 记录当前反光柱子的平均强度
                    double sum_intensity_of_target = 0;
                    for (auto it : intensity_data)
                    {
                        sum_intensity_of_target += it;
                    }
                    double avg_intensity = sum_intensity_of_target / intensity_data.size();

                    // 计算反光柱的有效点数量 放宽至理想值的0.6~1.2。
                    int max_size = floor((2.0 * reflector_radius / avg_range) / scan->angle_increment * 1.2);
                    int min_size = max_size * 0.6 > min_reflector_sample_count ? max_size * 0.6 : min_reflector_sample_count;

                    if (center_count <= max_size &&
                        center_count >= min_size &&
                        avg_intensity > min_reflector_intensity &&
                        avg_range > 0.5 &&
                        avg_range < 8)
                    {
                        // ROS_INFO("avg_range = %.2f", avg_range);
                        // ROS_INFO("avg_intensity = %.2f", avg_intensity);
                        // ROS_INFO("max_size = %d,min_size = %d", max_size, min_size);
                        // 计算角度
                        for (int i = 0; i < reflector_data.size(); i++)
                        {
                            center_yaw += (reflector_data[i].first);
                        }
                        center_yaw /= center_count; // 平均值
                        // 1 -----如果是反光住，使用考虑半径的计算方式
                        for (int i = 0; i < reflector_data.size(); i++)
                        {
                            // part_1
                            double theta = fabs(reflector_data[i].first - center_yaw);
                            double ds_1 = reflector_data[i].second * cos(theta);

                            // part_2
                            double a = reflector_data[i].second * sin(theta);
                            double b = reflector_radius;
                            double angle = (a / b) > 1 ? 1.0 : a / b; // Bug fixed!!!
                            double theta_2 = asin(angle);
                            double ds_2 = reflector_radius * cos(theta_2);
                            center_distance += (ds_1 + ds_2);
                        }
                        center_distance /= center_count; // average
                        center_yaw -= M_PI;              // transfer to -180~180

                        Reflector_pos item;
                        item.center_x = center_distance * cos(center_yaw);
                        item.center_y = center_distance * sin(center_yaw);
                        item.center_yaw = center_yaw;
                        reflectors_.push_back(item);

                        // 2 ------反光贴  换一种计算方法--0921  只需要计算均值
                        //  double tmp_sumx=0;
                        //  double tmp_sumy=0;
                        //  double tmp_avex,tmp_avey;
                        //  for (int i = 0; i < reflector_data.size(); i++)
                        //  {
                        //      reflector_data[i].first =   reflector_data[i].first - M_PI;
                        //      double tmp_x;
                        //      double tmp_y;
                        //      tmp_x = reflector_data[i].second*cos(reflector_data[i].first);
                        //      tmp_y = reflector_data[i].second*sin(reflector_data[i].first);
                        //      tmp_sumx += tmp_x;
                        //      tmp_sumy += tmp_y;
                        //  }
                        //  if(reflector_data.size() > 0)
                        //  {
                        //      tmp_avex = tmp_sumx/reflector_data.size();
                        //      tmp_avey = tmp_sumy/reflector_data.size();
                        //      Reflector_pos item;
                        //      item.center_x = tmp_avex;
                        //      item.center_y = tmp_avey;
                        //      item.center_yaw = center_yaw;
                        //      reflectors_.push_back(item);
                        //  }

                        //----end
                    }

                    center_x = 0.0;
                    center_y = 0.0;
                    center_count = 0;
                    center_yaw = 0;
                    center_distance = 0;
                    i = j - 1;
                    break;
                }
            }
            last_index = p->index;
        }
    }

    cartographer_ros_msgs::LandmarkList reflector_LandMarkList;
    vector<tf::Vector3> global_reflector_vector;
    global_reflector_vector.clear();
    if (work_mode == 2) // 2模式中，cartographer纯定位，添加反光住
    {
        // if (reflectors_.size() >= 3)
        // {
        //     // cout << "反光住 可以参与定位 " << endl;
        // }
        // else
        // {
        //     return;
        // }
        double distance_sum = 0.0;
        double distance_ave;
        for (int i = 0; i < reflectors_.size(); i++)
        {
            // 获取 map 到 lidar的tf
            tf::Pose current_laser_pose = imu_link_to_map * imu_link_to_lidar;
            tf::Vector3 local_reflector_point;
            local_reflector_point.setValue(reflectors_[i].center_x, reflectors_[i].center_y, 0);
            // kobe tf
            tf::Vector3 global_reflector_point = current_laser_pose * local_reflector_point;
            global_reflector_vector.push_back(global_reflector_point);

            double tmp_distance = sqrt(pow(reflectors_[i].center_x, 2) + pow(reflectors_[i].center_y, 2));
            // cout << "反光住距离 " << tmp_distance << endl;
            distance_sum = distance_sum + tmp_distance;
        }

        distance_ave = distance_sum / reflectors_.size();
        // cout << "和 " << distance_sum << "平均 " << distance_ave << endl;

        if (save_point_ok_flag == false)
        {
            if (set_go_flag == 1) // 设置路径，保存反光住坐标点
            {
                reflector_info tmp_info;
                for (auto item : global_reflector_vector)
                {
                    tmp_info.route_num = set_num;
                    tmp_info.posx = item[0];
                    tmp_info.posy = item[1];
                    reflector_info_vector.push_back(tmp_info);
                }
            }
            ROS_INFO("reflector_info_vector.size = %d", reflector_info_vector.size());
            cout << "saved point ok " << endl;
            save_point_ok_flag = true;
        }

        if (set_go_flag == 3)
        {
            if (save_csv_ok_flag == false)
            {
                savecsvfuc();
                save_csv_ok_flag = true;
            }
        }
        // ROS_INFO("---------------------------------------------------------");
    }
    //-----------
    // 根据反光住局部地图，进行精细定位
    else if (work_mode == 3 && can_use_reflector_localization == true && reflectors_.size() >= 3) // cartographer结合反光住局部地图 导航模式
    {
        match_v.clear();
        for (int i = 0; i < reflectors_.size(); i++)
        {
            // 获取 map 到 lidar的tf
            tf::Pose current_laser_pose = imu_link_to_map * imu_link_to_lidar;
            tf::Vector3 local_reflector_point;
            local_reflector_point.setValue(reflectors_[i].center_x, reflectors_[i].center_y, 0);
            // kobe tf
            tf::Vector3 global_reflector_point = current_laser_pose * local_reflector_point;

            rbpos_slam.x = current_laser_pose.getOrigin().x();
            rbpos_slam.y = current_laser_pose.getOrigin().y();
            double roll, pitch, yaw;
            tf::Quaternion q = current_laser_pose.getRotation();
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            rbpos_slam.theta = yaw;

            // global_reflector_vector 是根据pose和local 计算出的全局坐标

            // csv_reflector_info_vector 是csv中保存的反光住
            bool findOk = false;
            // 二者遍历匹配
            for (auto csviterm : csv_reflector_info_vector)
            {
                double x_ij = fabs(global_reflector_point[0] - csviterm.posx);
                double y_ij = fabs(global_reflector_point[1] - csviterm.posy);
                double distance = sqrt(pow(x_ij, 2) + pow(y_ij, 2));
                // cout << "distance" << distance << endl;

                if (distance < match_distance_acceped)
                {
                    pair<double, double> tmp_local = make_pair(local_reflector_point[0], local_reflector_point[1]);
                    pair<double, double> tmp_global = make_pair(csviterm.posx, csviterm.posy);
                    pair<pair<double, double>, pair<double, double>> tmp_match;
                    tmp_match = make_pair(tmp_global, tmp_local);
                    match_v.push_back(tmp_match);

                    findOk = true;
                    break;
                }
            }
            // 没有找到就存储，服务参数允许添加，这里才能加，否则不管
            if (findOk == false && can_save_reflector == true && SrvAddEnable == true)
            {
                // 更新
                reflector_info tmp_info;
                // 根据需要的最少变量调整
                tmp_info.route_num = 1;
                tmp_info.posx = global_reflector_point[0];
                tmp_info.posy = global_reflector_point[1];
                tmp_info.id = csv_reflector_info_vector.size() + 1;
                csv_reflector_info_vector.push_back(tmp_info);
                // 保存文件
                QNode::savecsvfuc();
            }
        }

        // POSE的两种计算方式
        // 1 ----rbpos，即从cartographer纯定位获取
        //  OptimizeCurrentPose(match_v,rbpos_slam);
        // RobotPose rbpos_tmp = rbpos_slam;
        // 2  根据柱子地图反算
        if (!InitialCurrentPose(match_v, rbpos_reflector))
        {
            cout << "get pose failed " << endl;
        }
        else
        {
            // cout << "reflector robot pose: " << "x " << rbpos_reflector.x << " y  " << rbpos_reflector.y << " yaw " << rbpos_reflector.theta << endl;

            OptimizeCurrentPose(match_v, rbpos_reflector);

            // cout << "CERES-reflector robot pose: " << "x " << rbpos_reflector.x << " y  " << rbpos_reflector.y << " yaw " << rbpos_reflector.theta << endl;
            RobotPose rbpos_tmp = rbpos_reflector;

            geometry_msgs::Quaternion tmp_qua;

            // 输入欧拉角，转化成四元数在终端输出
            tmp_qua = tf::createQuaternionMsgFromRollPitchYaw(0, 0, rbpos_tmp.theta);
            tf::Pose ceres_laser_pose;

            ceres_laser_pose.setOrigin(tf::Vector3(rbpos_tmp.x,
                                                   rbpos_tmp.y,
                                                   0));

            ceres_laser_pose.setRotation(tf::Quaternion(tmp_qua.x,
                                                        tmp_qua.y,
                                                        tmp_qua.z,
                                                        tmp_qua.w));
            // 再根据rbpos 反算出imu link pose 发布话题
            tf::Pose ceres_imu_pose = ceres_laser_pose * imu_link_to_lidar.inverse();
            double tmp_x = 1000 * ceres_imu_pose.getOrigin().x();
            double tmp_y = 1000 * ceres_imu_pose.getOrigin().y();
            double roll, pitch, yaw;
            tf::Quaternion q = ceres_imu_pose.getRotation();
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
            double tmp_theta = yaw;
            // cout << "ceres pose: " << "x " << tmp_x << " y  " << tmp_y << " yaw " << tmp_theta << endl;

            // 发布ceres优化后的pose

            current_pose.robot_pose.position.x = ceres_imu_pose.getOrigin().x();
            current_pose.robot_pose.position.y = ceres_imu_pose.getOrigin().y();
            current_pose.robot_pose.position.z = 0;
            current_pose.robot_pose.orientation.w = q.getW();
            current_pose.robot_pose.orientation.x = q.getX();
            current_pose.robot_pose.orientation.y = q.getY();
            current_pose.robot_pose.orientation.z = q.getZ();
            current_pose.covariance_score = 0.9;
            current_pose.current_trajectory = trajectory_id;
            current_pose.last_update_duration = 0.1;
            current_pose.last_update_pose = current_pose.robot_pose;
        }
    }
    robot_pose_pub_.publish(current_pose);
    // ROS_INFO("reflectors_.size = %d", reflectors_.size());
    reflectornumber = 0;
    reflectormsgRealTime.ns = "RealTime";
    // Publish RealReflector
    for (int i = 0; i < reflectors_.size(); i++)
    {
        // 获取 map 到 lidar的tf
        tf::Pose current_laser_pose = imu_link_to_map * imu_link_to_lidar;
        tf::Vector3 local_reflector_point;
        local_reflector_point.setValue(reflectors_[i].center_x, reflectors_[i].center_y, 0);
        // kobe tf
        tf::Vector3 global_reflector_point = current_laser_pose * local_reflector_point;
        reflectornumber++;
        // ROS_INFO("reflectornumber = %d", reflectornumber);
        set_marker_fixed_property(global_reflector_point[0], global_reflector_point[1], reflectornumber, "RealReflector");

        // 添加到未存储列表
        reflectorbase.area = 0;
        reflectorbase.id = i;
        reflectorbase.x = global_reflector_point[0];
        reflectorbase.y = global_reflector_point[1];
        reflectormsgRealTime.reflectordata.push_back(reflectorbase);
        // 发布marker
        Publish();
    }
    // 发布未存储反光柱
    pub_reflectormsgRealTime.publish(reflectormsgRealTime);
    // 发布已存储反光柱坐标
    reflectormsgSaved.reflectordata.clear();
    reflectormsgSaved.header.stamp = ros::Time::now();
    reflectormsgSaved.ns = "Saved";

    for (auto csviterm : csv_reflector_info_vector)
    {
        reflectorbase.area = csviterm.route_num;
        reflectorbase.id = csviterm.id;
        reflectorbase.x = csviterm.posx;
        reflectorbase.y = csviterm.posy;
        reflectormsgSaved.reflectordata.push_back(reflectorbase);
        // set_marker_fixed_property(csviterm.posx, csviterm.posy, reflectornumber,"gobleReflector");
        // Publish();
    }

    pub_reflectormsgSaved.publish(reflectormsgSaved);
}
// 获取 cartographer发出的 landmark pose  list
void QNode::getGlobalReflector(const visualization_msgs::MarkerArrayConstPtr &data)
{
    for (int i = 0; i < data->markers.size(); i++)
    {
        landmark_relfector it;
        int id = data->markers[i].id;
        it.x = data->markers[i].pose.position.x;
        it.y = data->markers[i].pose.position.y;
        it.z = data->markers[i].pose.position.z;
        global_reflectors[id] = it;
    }
}

void QNode::getRobotPose(const cartographer_ros_msgs::RobotPose::ConstPtr &data)
{
    if (data->covariance_score > 0.4)
    {
        can_use_reflector_localization = true;
    }
    else
    {
        can_use_reflector_localization = false;
    }

    // trajectory_id = data->current_trajectory;
}
void QNode::getTrackedPose(const geometry_msgs::PoseStamped &data)
{

    tf::Quaternion q_tem;
    q_tem.setX(data.pose.orientation.x);
    q_tem.setY(data.pose.orientation.y);
    q_tem.setZ(data.pose.orientation.z);
    q_tem.setW(data.pose.orientation.w);

    robot_pose_yaw = tf::getYaw(q_tem);

    tf::Vector3 p_tem(data.pose.position.x,
                      data.pose.position.y,
                      data.pose.position.z);
    // kobe imu_link
    imu_link_to_map = tf::Transform(q_tem, p_tem);
    // ROS_INFO("noex = %f", data.pose.position.x);
    // ROS_INFO("nowy = %f", data.pose.position.y);
    // 比较位置变化
    double x = fabs(data.pose.position.x - lastPostion.pose.position.x);
    double y = fabs(data.pose.position.y - lastPostion.pose.position.y);
    double distance = sqrt(pow(x, 2) + pow(y, 2));

    double yal = fabs(robot_pose_yaw - lastPostion.pose.orientation.z) * 180 / 3.14;
    // 位置超过3mm，角度大于0.1度，不允许存储
    if (distance > 0.003 || yal > 0.1)
    {
        can_save_reflector = false;
        ROS_INFO("false");
    }
    else
    {
        can_save_reflector = true;
        ROS_INFO("true");
    }
    ROS_INFO("angle = %f", yal);
    // ROS_INFO("lastx = %f", lastPostion.pose.position.x);
    // ROS_INFO("lasty = %f", lastPostion.pose.position.y);
    // 缓存位置
    lastPostion.pose.position.x = data.pose.position.x;
    lastPostion.pose.position.y = data.pose.position.y;
    lastPostion.pose.orientation.z = robot_pose_yaw;

    // if (data->covariance_score > 0.0)
    // {
    //     can_use_reflector_localization = true;
    // }
    // else
    // {
    //     can_use_reflector_localization = false;
    // }

    // trajectory_id = data->current_trajectory;
}

// 作用：判断点是否在多边形内
// p指目标点， ptPolygon指多边形的点集合， nCount指多边形的边数
bool QNode::PtInPolygon(Point_pos p, vector<Point_pos> &ptPolygon, int nCount)
{
    // 交点个数
    int nCross = 0;
    for (int i = 0; i < nCount; i++)
    {
        Point_pos p1 = ptPolygon[i];
        Point_pos p2 = ptPolygon[(i + 1) % nCount]; // 点P1与P2形成连线

        if (p1.y == p2.y)
            continue;
        if (p.y < min(p1.y, p2.y))
            continue;
        if (p.y >= max(p1.y, p2.y))
            continue;
        // 求交点的x坐标（由直线两点式方程转化而来）
        double x = (double)(p.y - p1.y) * (double)(p2.x - p1.x) / (double)(p2.y - p1.y) + p1.x;
        // 只统计p1p2与p向右射线的交点
        if (x > p.x)
        {
            nCross++;
        }
    }

    // 交点为偶数，点在多边形之外
    // 交点为奇数，点在多边形之内
    if ((nCross % 2) == 1)
    {
        // g_proj_log.ShowInfo("点在区域内");
        return true;
    }
    else
    {
        // g_proj_log.ShowInfo("点在区域外");
        return false;
    }
}
