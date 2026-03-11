#ifndef RTK_RELOCATOR_HPP
#define RTK_RELOCATOR_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// Forward declaration — avoids circular include
class PlaceRecognition;

/**
 * @class RtkRelocator
 * @brief 独立的 RTK 重定位模块
 *
 * 从 PlaceRecognition 中提取出来，负责：
 *   1. 订阅 RTK GPS + 欧拉角话题
 *   2. 经纬度 → UTM 转换
 *   3. 从全局地图获取 Z 高度
 *   4. 计算初始变换并调用 PlaceRecognition::search_and_pub_loop()
 *
 * 使用方法:
 *   在 PlaceRecognition 构造函数中:
 *     rtk_relocator_ = std::make_unique<RtkRelocator>(nh, this);
 */
class RtkRelocator
{
public:
    /**
     * @brief 构造函数
     * @param nh  ROS 节点句柄（用于读取参数、创建订阅器）
     * @param pr  PlaceRecognition 指针（用于访问共享数据和调用 search_and_pub_loop）
     */
    RtkRelocator(ros::NodeHandle &nh, PlaceRecognition *pr);

    // ── RTK 回调 ─────────────────────────────────────────────────────────
    /** @brief 带欧拉角的 RTK 回调（双话题同步） */
    void RTKPoseCallback(const geometry_msgs::Vector3Stamped::ConstPtr &euler_msg,
                         const sensor_msgs::NavSatFix::ConstPtr &fix_msg);

    /** @brief 仅 NavSatFix 的 RTK 回调（无欧拉角时退化为 Identity 旋转） */
    void RTKPoseCallback(const sensor_msgs::NavSatFix::ConstPtr &fix_msg);

    // ── 工具函数 ─────────────────────────────────────────────────────────
    static Eigen::Matrix3d getRotationMatrixFromEuler(
        const Eigen::Vector3d &euler_angles,
        const std::string &order);

    static Eigen::Matrix3d getRotationMatrixFromYaw(double yaw_angle_radians);

private:
    // ── 指向宿主的指针 ──────────────────────────────────────────────────
    PlaceRecognition *pr_;

    // ── ROS 订阅 / 同步器 ──────────────────────────────────────────────
    message_filters::Subscriber<sensor_msgs::NavSatFix>      rtk_fix_sub_;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> rtk_odom_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        geometry_msgs::Vector3Stamped, sensor_msgs::NavSatFix> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;

    // ── RTK 参数 ────────────────────────────────────────────────────────
    Eigen::Vector3d RTK_imu_trans_;
    double latitude_  = 0.0;
    double longitude_ = 0.0;
    double altitude_  = 0.0;
    double utm_x_0_   = 0.0;
    double utm_y_0_   = 0.0;
    int    zone_      = 0;
    bool   northp_    = true;

    // ── 节流控制 ────────────────────────────────────────────────────────
    int    rtk_skip_counter_    = 0;
    int    rtk_skip_threshold_  = 10;
    double rtk_min_interval_    = 0.5;
    ros::Time last_rtk_process_time_;

    // ── 内部辅助 ────────────────────────────────────────────────────────
    static double normalizeAngle(double angle);
};

#endif // RTK_RELOCATOR_HPP
