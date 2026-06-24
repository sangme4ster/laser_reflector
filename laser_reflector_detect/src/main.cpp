#include <../include/ros_qt5/qnode.h>
#include <ros/ros.h>
#include "laser_reflector_detect/igk_reflector_srv.h"

// 允许添加反光柱
extern bool SrvAddEnable;
//清空反光柱
extern bool SrvClear;
//删除指定id的反光柱
extern int SrvId;

// bool 返回值由于标志是否处理成功
bool doReq(laser_reflector_detect::igk_reflector_srv::Request& req,
          laser_reflector_detect::igk_reflector_srv::Response& resp){
    resp.response = req.command;
    switch(req.command)
   {
    case 1://开始添加反光柱
        SrvAddEnable = true;
        ROS_INFO("开始添加反光柱");
        break;
    case 2://停止添加反光柱
        SrvAddEnable = false;
        ROS_INFO("停止添加反光柱");
        break;
    case 3://清空反光柱
        SrvClear = true;
        ROS_INFO("清空反光柱");
        break;
    case 4://删除指定id的反光柱
        SrvId = req.id;
        ROS_INFO("删除%d号反光柱:",req.id);
        break;   
    }
    return true;
}

int main(int argc, char *argv[])
{
    ROS_INFO("ros master start init");
    ros::init(argc, argv, "LaserReflectorDetect");
    ROS_INFO("ros master finish init");

    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    //循环运行
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);
   
    QNode qnode(ros::NodeHandle(""),&tf_buffer);

    qnode.init();


     // 3.创建 ROS 句柄
    ros::NodeHandle nh;
    // 4.创建 服务 对象
    ros::ServiceServer server = nh.advertiseService("igk_reflector_srv",doReq);
    
    ros::spin();
    return 0;
}


