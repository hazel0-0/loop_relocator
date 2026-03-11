#include "rtk_relocator.hpp"
#include "../place_recognition.hpp"   // 需要完整定义来访问 PlaceRecognition 成员

#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  构造函数
// ═══════════════════════════════════════════════════════════════════════════════

RtkRelocator::RtkRelocator(ros::NodeHandle &nh, PlaceRecognition *pr)
    : pr_(pr)
{
    // ── 读取参数 ──────────────────────────────────────────────────────────
    double x, y, z;
    nh.param<double>("RTK_imu_trans/x", x, 0.0);
    nh.param<double>("RTK_imu_trans/y", y, 0.0);
    nh.param<double>("RTK_imu_trans/z", z, 0.0);
    RTK_imu_trans_ << x, y, z;
    std::cout << "RTK_imu_trans: " << RTK_imu_trans_.transpose() << std::endl;

    nh.param<double>("latitude",  latitude_,  0.0);
    nh.param<double>("longitude", longitude_, 0.0);
    nh.param<double>("altitude",  altitude_,  0.0);
    std::cout << "RTK origin — lat: " << latitude_
              << "  lon: " << longitude_
              << "  alt: " << altitude_ << std::endl;

    // ── UTM 原点 ──────────────────────────────────────────────────────────
    GeographicLib::UTMUPS::Forward(latitude_, longitude_, zone_, northp_, utm_x_0_, utm_y_0_);
    std::cout << "UTM origin — x0: " << utm_x_0_
              << "  y0: " << utm_y_0_
              << "  zone: " << zone_ << std::endl;

    // ── 节流参数 ──────────────────────────────────────────────────────────
    nh.param<int>("rtk_skip_threshold", rtk_skip_threshold_, 10);
    nh.param<double>("rtk_min_interval", rtk_min_interval_, 0.5);
    rtk_skip_counter_ = 0;
    last_rtk_process_time_ = ros::Time::now();
    ROS_INFO("RTK throttling: skip_threshold=%d, min_interval=%.2fs",
             rtk_skip_threshold_, rtk_min_interval_);

    // ── 订阅与同步 ──────────────────────────────────────────────────────
    rtk_odom_sub_.subscribe(nh, "/gps/euler", 10);
    rtk_fix_sub_.subscribe(nh, "/gps/fix", 10);
    sync_.reset(new Sync(SyncPolicy(10), rtk_odom_sub_, rtk_fix_sub_));
    sync_->registerCallback(
        boost::bind(&RtkRelocator::RTKPoseCallback, this, _1, _2));
}

// ═══════════════════════════════════════════════════════════════════════════════
//  normalizeAngle
// ═══════════════════════════════════════════════════════════════════════════════

double RtkRelocator::normalizeAngle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  getRotationMatrixFromEuler
// ═══════════════════════════════════════════════════════════════════════════════

Eigen::Matrix3d RtkRelocator::getRotationMatrixFromEuler(
    const Eigen::Vector3d &euler_angles,
    const std::string &order)
{
    double roll  = normalizeAngle(euler_angles.y());
    double pitch = normalizeAngle(euler_angles.x());
    double yaw   = normalizeAngle(90.0 * M_PI / 180.0 - euler_angles.z());

    Eigen::Matrix3d R;

    if (order == "ZYX")
    {
        double cy = cos(yaw),  sy = sin(yaw);
        double cp = cos(pitch), sp = sin(pitch);
        double cr = cos(roll),  sr = sin(roll);

        R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
             sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
             -sp,   cp*sr,            cp*cr;
    }
    else if (order == "XYZ")
    {
        double cr = cos(roll),  sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);
        double cy = cos(yaw),   sy = sin(yaw);

        R << cp*cy, -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy,
             cp*sy,  cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy,
             -sp,    sr*cp,             cr*cp;
    }
    else if (order == "ZXY")
    {
        double cy = cos(yaw),  sy = sin(yaw);
        double cr = cos(roll), sr = sin(roll);
        double cp = cos(pitch), sp = sin(pitch);

        R << cy*cr - sy*sr*sp, -sy*cp, cy*sr + sy*cr*sp,
             sy*cr + cy*sr*sp,  cy*cp, sy*sr - cy*cr*sp,
             -cp*sr,            sp,    cp*cr;
    }
    else
    {
        ROS_ERROR("Unsupported rotation order: %s. Using identity.", order.c_str());
        R = Eigen::Matrix3d::Identity();
    }
    return R;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  getRotationMatrixFromYaw
// ═══════════════════════════════════════════════════════════════════════════════

Eigen::Matrix3d RtkRelocator::getRotationMatrixFromYaw(double yaw_angle_radians)
{
    double c = std::cos(yaw_angle_radians);
    double s = std::sin(yaw_angle_radians);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RTKPoseCallback  (带欧拉角的双话题版本)
// ═══════════════════════════════════════════════════════════════════════════════

void RtkRelocator::RTKPoseCallback(
    const geometry_msgs::Vector3Stamped::ConstPtr &euler_msg,
    const sensor_msgs::NavSatFix::ConstPtr &fix_msg)
{
    // ── 节流 ──────────────────────────────────────────────────────────────
    rtk_skip_counter_++;
    if (rtk_skip_counter_ < rtk_skip_threshold_)
    {
        ROS_DEBUG("RTK: Skipping [%d/%d]", rtk_skip_counter_, rtk_skip_threshold_);
        return;
    }
    rtk_skip_counter_ = 0;

    ros::Time now = ros::Time::now();
    if ((now - last_rtk_process_time_).toSec() < rtk_min_interval_)
    {
        ROS_DEBUG("RTK: time throttle");
        return;
    }
    last_rtk_process_time_ = now;

    // ── 协方差检查 ────────────────────────────────────────────────────────
    ROS_INFO("Received RTK initial pose estimate:");
    double start_time = ros::Time::now().toSec();

    const auto &cov = fix_msg->position_covariance;
    if (cov[0] > 0.0005 || cov[4] > 0.00054756)
    {
        ROS_ERROR("RTK covariance too high, skip");
        return;
    }

    // ── 欧拉角 → 旋转矩阵 ────────────────────────────────────────────────
    Eigen::Vector3d euler_angle(euler_msg->vector.x,
                                euler_msg->vector.y,
                                euler_msg->vector.z);
    Eigen::Matrix3d rotation_matrix = getRotationMatrixFromEuler(euler_angle, "ZYX");

    // ── 经纬度 → UTM 位置 ────────────────────────────────────────────────
    double utm_x, utm_y;
    GeographicLib::UTMUPS::Forward(fix_msg->latitude, fix_msg->longitude,
                                   zone_, northp_, utm_x, utm_y);

    Eigen::Vector3d position(utm_x - utm_x_0_, utm_y - utm_y_0_, 0.0);
    std::cout << "RTK position: " << position.transpose() << std::endl;

    position = position + RTK_imu_trans_ - rotation_matrix.inverse() * RTK_imu_trans_;

    // ── 从地图获取 Z ──────────────────────────────────────────────────────
    if (!pr_->estimateZFromMap(position, 1.0))
        return;

    // ── 计算变换并调用匹配 ────────────────────────────────────────────────
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
    init_pose.first  = position;
    init_pose.second = rotation_matrix;

    auto init_tf = pr_->computeInitTransform(init_pose);

    // 发布 RTK 位姿可视化
    Eigen::Quaterniond quat(init_pose.second);
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp    = ros::Time::now();
    pose_msg.header.frame_id = pr_->visualization_frame_id;
    pose_msg.pose.pose.position.x    = init_pose.first.x();
    pose_msg.pose.pose.position.y    = init_pose.first.y();
    pose_msg.pose.pose.position.z    = init_pose.first.z();
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();
    pr_->pubMatchedPose.publish(pose_msg);

    std::cout << "RTK init_transform t: " << init_tf.first.transpose() << std::endl;
    std::cout << "RTK init_transform R:\n" << init_tf.second << std::endl;
    std::cout << "current_cloud size2: " << pr_->current_cloud.size() << std::endl;

    double lidar_point_time = ros::Time::now().toSec();
    std::cout << "lidar_point_time: " << lidar_point_time - start_time << std::endl;

    pr_->search_and_pub_loop(init_tf, pr_->current_cloud, pr_->current_pose, pr_->rtk_loop_options);

    double end_time = ros::Time::now().toSec();
    std::cout << "matching time: " << end_time - start_time << std::endl;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  RTKPoseCallback  (仅 NavSatFix 的退化版本)
// ═══════════════════════════════════════════════════════════════════════════════

void RtkRelocator::RTKPoseCallback(
    const sensor_msgs::NavSatFix::ConstPtr &fix_msg)
{
    ROS_INFO("Received RTK initial pose estimate (no euler):");
    double start_time = ros::Time::now().toSec();

    const auto &cov = fix_msg->position_covariance;
    if (cov[0] > 0.0005 || cov[4] > 0.00054756)
    {
        ROS_ERROR("RTK covariance too high, skip");
        return;
    }

    // ── 经纬度 → UTM ────────────────────────────────────────────────────
    double utm_x, utm_y;
    GeographicLib::UTMUPS::Forward(fix_msg->latitude, fix_msg->longitude,
                                   zone_, northp_, utm_x, utm_y);

    Eigen::Vector3d position(utm_x - utm_x_0_, utm_y - utm_y_0_, 0.0);
    std::cout << "RTK position: " << position.transpose() << std::endl;

    // ── 从地图获取 Z ──────────────────────────────────────────────────────
    if (!pr_->estimateZFromMap(position, 2.0))
        return;

    std::cout << "RTK position (with Z): " << position.transpose() << std::endl;

    // ── 计算变换（无旋转信息，退化为 Identity） ────────────────────────────
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
    init_pose.first  = position;
    init_pose.second = Eigen::Matrix3d::Identity();

    auto init_tf = pr_->computeInitTransform(init_pose);

    Eigen::Quaterniond quat(init_pose.second);
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp    = ros::Time::now();
    pose_msg.header.frame_id = pr_->visualization_frame_id;
    pose_msg.pose.pose.position.x    = init_pose.first.x();
    pose_msg.pose.pose.position.y    = init_pose.first.y();
    pose_msg.pose.pose.position.z    = init_pose.first.z();
    pose_msg.pose.pose.orientation.x = quat.x();
    pose_msg.pose.pose.orientation.y = quat.y();
    pose_msg.pose.pose.orientation.z = quat.z();
    pose_msg.pose.pose.orientation.w = quat.w();
    pr_->pubMatchedPose.publish(pose_msg);

    std::cout << "RTK init_transform t: " << init_tf.first.transpose() << std::endl;
    std::cout << "RTK init_transform R:\n" << init_tf.second << std::endl;
    std::cout << "current_cloud size2: " << pr_->current_cloud.size() << std::endl;

    double lidar_point_time = ros::Time::now().toSec();
    std::cout << "lidar_point_time: " << lidar_point_time - start_time << std::endl;

    pr_->search_and_pub_loop(init_tf, pr_->current_cloud, pr_->current_pose, pr_->rtk_loop_options);

    double end_time = ros::Time::now().toSec();
    std::cout << "matching time: " << end_time - start_time << std::endl;
}
