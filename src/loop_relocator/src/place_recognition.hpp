#ifndef PLACE_RECOGNITION_HPP
#define PLACE_RECOGNITION_HPP
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <deque>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
// #include "include/btc.h"
#include "include/utils.h"
#include "submap_match.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/GeodesicLine.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <carstatemsgs/CarState.h>
// namespace btc_descriptor {

class PlaceRecognition {
public:
    // Constructor
    PlaceRecognition(ros::NodeHandle &nh);
    //for test only
    void car_odom_callback(const carstatemsgs::CarState::ConstPtr &msg);

    //void loadstateCallback(const ros::TimerEvent &event);
    void loadstateCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg);

    // void search_and_pub_loop(pcl::PointCloud<pcl::PointXYZI> transform_cloud, int submap_id);
    void search_and_pub_loop(std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform, 
        pcl::PointCloud<pcl::PointXYZI> transform_cloud, std::pair<Eigen::Vector3d, Eigen::Matrix3d> current_pose_);
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    Eigen::Matrix3d getRotationMatrixFromYaw(double yaw_angle_radians);
    Eigen::Matrix3d getRotationMatrixFromEuler(
        const Eigen::Vector3d& euler_angles, 
        const std::string& order);
    void RTKPoseCallback(const geometry_msgs::Vector3Stamped::ConstPtr& euler_msg,
        const sensor_msgs::NavSatFix::ConstPtr& fix_msg);
    void RTKPoseCallback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg);
    void load_global_pcd();
    // void load_config();
    void addTransform(
    std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& window,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& new_tf,
    size_t window_size);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> computeFilteredAverage(
        const std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& window,
        double threshold);    
    // std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
    pcl::PointCloud<pcl::PointXYZI> whole_cloud;
    pcl::PointCloud<pcl::PointXYZI> current_cloud;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> current_pose;

    // std::vector<float> read_lidar_data(const std::string lidar_data_path);
    std::string setting_path = "";
    std::string pcds_dir = "";
    std::string pose_file = "";
    std::string result_file = "";
    std::string curr_pcds_dir = "";
    std::string curr_pose_file = "";
    std::string btc_save_path = "";
    std::string seq_key = "";
    std::string full_pcd_dir = "";
    bool finish = false;
    bool save_btc = false; // Initialize saveBtc to false
    bool UTM_init = false;
    // double cloud_overlap_thr = 0.5;
    // bool calc_gt_enable = false;
    // bool read_bin = true;
    // BtcDescManager *btc_manager;
    IKD_MATCH *ikd_match;

    // ConfigSetting config_setting;
    // ros::Publisher pubOdomAftMapped;
    ros::Publisher pubCureentCloud;
    // ros::Publisher pubCurrentBinary;
    // ros::Publisher pubPath;
    ros::Publisher pubCurrentCloud;
    ros::Publisher pubCurrentPose;
    ros::Publisher pubTransformedCloud;
    ros::Publisher pubRawTransformedCloud;

    ros::Publisher pubMatchedPose;
    ros::Publisher pubMatchedCloud;
    // ros::Publisher pubMatchedBinary;
    // ros::Publisher pubLoopStatus;
    // ros::Publisher pubBTC;
    ros::Timer state_Timer;
    ros::Publisher pubLoopTransform;
    ros::Publisher pubWholeCloud;
    ros::Subscriber initialpose_sub_;

    // ros::Subscriber rtk_covariance_sub_;

    message_filters::Subscriber<sensor_msgs::NavSatFix> rtk_fix_sub_;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> rtk_odom_sub_;
    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3Stamped, sensor_msgs::NavSatFix> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
      //for test only
     ros::Subscriber car_odom_sub;


    GeographicLib::LocalCartesian geo_converter;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;

    double geocentric_x;
    double geocentric_y;
    double geocentric_z;
    double utm_x_0 = 0.0;
    double utm_y_0 = 0.0;
    int zone;
    bool northp = true;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    double count = 0.0;
    double all_count = 0.0;
private:

    // Example private member
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>
    PclOdomSyncPolicy;
    typedef message_filters::Synchronizer<PclOdomSyncPolicy>
    PclOdomSynchronizer;

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    
    std::shared_ptr<PclOdomSynchronizer> pcl_odom_sync_Ptr_;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> loop_transforms;
    std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> window;
    bool mannul_init_flag = false;
    bool RTK_init_flag = false;
    // std_msgs::ColorRGBA color_tp;
    // std_msgs::ColorRGBA color_fp;
    // std_msgs::ColorRGBA color_path;
    // double scale_tp = 4.0;
    // double scale_fp = 5.0;
    // double scale_path = 3.0;
    Eigen::Vector3d lidar_imu_trans;
    Eigen::Vector3d RTK_imu_trans;
    std::mutex ikd_match_mtx;

};

// } // namespace btc_descriptor

#endif // PLACE_RECOGNITION_HPP

