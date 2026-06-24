/**
 * @file /include/qt_test/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_test_QNODE_HPP_
#define qt_test_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/package.h>
#endif
#include <string>
// #include <QThread>
#include <QStringListModel>

#include <sensor_msgs/LaserScan.h>
#include <cartographer_ros_msgs/LandmarkEntry.h>
#include <cartographer_ros_msgs/LandmarkList.h>
#include <geometry_msgs/PoseStamped.h>

#include <../include/ros_qt5/laser_processor.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <ceres/ceres.h>

#include <chrono>

#include <cartographer_ros_msgs/RobotPose.h>
#include <nav_msgs/Path.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Quaternion.h>

// #include <cartographer_ros_msgs/WorkingState.h>

#include <vector>
using namespace std;

struct Point_pos
{
    double x;
    double y;

    Point_pos(double x_, double y_)
    {
        this->x = x_;
        this->y = y_;
    }
};

struct Reflector_pos
{

    double center_x;
    double center_y;
    double center_yaw;

    bool operator<(const Reflector_pos &other) const
    {
        double this_distance = hypot(center_x, center_y);
        double other_distance = hypot(other.center_x, other.center_y);

        return this_distance < other_distance;
    }
};

// 用于局部精定位的反光住信息
struct reflector_info
{
    int route_num;
    int id;
    double posx;
    double posy;
};
// 机器人的位姿
struct RobotPose
{
    double x = 0;
    double y = 0;
    double theta = 0;
};

struct landmark_relfector
{
    double x;
    double y;
    double z;
};

// CERES的 损失函数
typedef struct FeaturePairCost
{
    FeaturePairCost(pair<double, double> pt1, pair<double, double> pt2) : _pt1(pt1), _pt2(pt2) {}

    template <typename T>
    bool operator()(const T *const pose, T *residual) const
    {
        T x_tr = T(_pt1.first) * cos(pose[2]) - T(_pt1.second) * sin(pose[2]) + pose[0];
        T y_tr = T(_pt1.first) * sin(pose[2]) + T(_pt1.second) * cos(pose[2]) + pose[1];

        residual[0] = x_tr - T(_pt2.first);
        residual[1] = y_tr - T(_pt2.second);

        return true;
    }

    const pair<double, double> _pt1, _pt2;
} FeaturePairCost;
class ReflectorCostFunctor
{

public:
    ReflectorCostFunctor(double range, landmark_relfector object)
        : measure_ramge(range),
          reflector(object) {}

    template <typename T>
    bool operator()(const T *x, T *e) const
    {
        T d_x = x[0] - T(reflector.x);
        T d_y = x[1] - T(reflector.y);
        // e[0] = T(measure_ramge) - T(ceres::sqrt(ceres::pow(T(d_x),2)+ceres::pow(T(d_y),2)));
        e[0] = T(measure_ramge) - sqrt(T(d_x) * T(d_x) + T(d_y) * T(d_y));
        return true;
    }

private:
    double measure_ramge;
    landmark_relfector reflector;
};

// 定义结构体来存储激光雷达属性
struct LidarAttributes
{
    double offsetX;       // 激光雷达相对于机器人中心的位置 X（以米为单位）
    double offsetY;       // 激光雷达相对于机器人中心的位置 Y（以米为单位）
    double offsetZ;       // 激光雷达相对于机器人中心的位置 Z（以米为单位）
    double rotationRoll;  // 激光传感器绕 X 轴的旋转角度（横滚角）
    double rotationPitch; // 激光传感器绕 Y 轴的旋转角度（俯仰角）
    double rotationYaw;   // 激光传感器绕 Z 轴的旋转角度（偏航角）
};

class QNode
{
    // Q_OBJECT
public:
    QNode(ros::NodeHandle node_handle, tf2_ros::Buffer *buffer);
    virtual ~QNode();
    bool init();

    void tfProcess(const tf2_msgs::TFMessage &tfMessage);

    /*********************
    ** Logging
    **********************/
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel *loggingModel() { return &logging_model; }
    void log(const LogLevel &level, const std::string &msg);

    bool get_IK(float pos[6], float joints[6]);

    int round_int(double val);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

private:
    int init_argc;
    char **init_argv;
    QStringListModel logging_model;

    ros::Subscriber scan_sub_;
    ros::Subscriber tf_sub_;
    ros::Subscriber global_reflector_pos_sub_;
    ros::Subscriber robot_pose_sub_;
    ros::Subscriber tracked_pose_sub_;
    void Publish();
    void set_marker_fixed_property(float mx, float my, int i, string str);
    void laserProcess(const sensor_msgs::LaserScanConstPtr &scan);
    void getGlobalReflector(const visualization_msgs::MarkerArrayConstPtr &data);
    void getRobotPose(const cartographer_ros_msgs::RobotPose::ConstPtr &data);
    bool PtInPolygon(Point_pos p, std::vector<Point_pos> &ptPolygon, int nCount);

    // 声明全局变量
    LidarAttributes lidarAttributes;
    bool tf_laser_baselink_first = true;

    // kobe add
    void loadmap();
    void savecsvfuc();
    void getTrackedPose(const geometry_msgs::PoseStamped &data);
    void OptimizeCurrentPose(vector<pair<pair<double, double>, pair<double, double>>> &, RobotPose &);
    inline double ComputeRadius(pair<double, double> &pt);
    inline double ComputeDistance(pair<double, double> &pt1, pair<double, double> &pt2);
    bool InitialCurrentPose(vector<pair<pair<double, double>, pair<double, double>>> &match_result,
                            RobotPose &robot_pose);

    bool CheckResultValid(vector<double> &radius_list,
                          vector<pair<pair<double, double>, pair<double, double>>> &match_result,
                          pair<double, double> &result, int id1, int id2, double d12);

    sensor_msgs::LaserScan scan_;

    ros::Publisher reflector_points_;
    ros::Publisher reflector_landmark_;
    ros::Publisher current_landmark_list_pub_;
    ros::Publisher reflector_localization_pos_pub_;
    ros::Publisher lidar_uncorrected_pub_;
    ros::Publisher lidar_corrected_pub_;
    ros::Publisher error_edge_pub_;
    ros::Publisher robot_pose_pub_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_reflectormsgSaved;
    ros::Publisher pub_reflectormsgRealTime;
    double reflector_radius = 0.0 / 2.0;
    int min_reflector_sample_count = 8;
    int min_reflector_intensity = 1000;

    double kLandmarkMarkerScale = 0.05;

    double reflector_combined_length = 0.5;
    int reflector_combination_mode = 0;

    double angle_error;
    double distance_error;
    int last_index = 0;

    std::map<int, landmark_relfector> global_reflectors;
    std::string lidar_frame_;
    std::string map_frame_;

    tf::TransformListener *tf_;

    double match_distance_acceped;
    int work_mode;

    std::map<Reflector_pos, int /*id*/> global_match_data;
    std::map<double /*yaw*/, Reflector_pos> global_match_sorted_data;
    // kobe
    //  tf::Transform base_link_to_map;
    tf::Transform imu_link_to_map;

    // tf::Transform base_link_to_lidar;
    // kobe
    tf::Transform imu_link_to_lidar;
    bool can_use_reflector_localization = false;
    bool can_save_reflector = false;

    geometry_msgs::PoseStamped lastPostion;
    // test localization error
    double max_change_y = 0;
    double max_change_x = 0;
    double max_change_angle = 0;
    double max_change = 0;

    double max_x = -10;
    double min_x = 10;
    double max_y = -10;
    double min_y = 10;

    ros::NodeHandle n;

    nav_msgs::Path lidar_uncorrected_path_;
    nav_msgs::Path lidar_corrected_path_;
    visualization_msgs::MarkerArray error_edge_markers;
    visualization_msgs::Marker markerNode;
    visualization_msgs::Marker markerEdge;

    double robot_pose_yaw;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string trajectory_id;

    // test
    tf2_ros::Buffer *tf_buffer_node_;

    std::vector<reflector_info> reflector_info_vector;

    std::vector<reflector_info> csv_reflector_info_vector;
    RobotPose rbpos_slam;
    RobotPose rbpos_reflector;

    vector<pair<pair<double, double>, pair<double, double>>> match_v;
};

#endif /* qt_test_QNODE_HPP_ */
