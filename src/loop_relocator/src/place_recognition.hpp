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
#include <memory>

// 前向声明
class RtkRelocator;
class QrRelocator;

class PlaceRecognition {
public:
    // Constructor
    PlaceRecognition(ros::NodeHandle &nh);
    //for test only
    void car_odom_callback(const carstatemsgs::CarState::ConstPtr &msg);

    //void loadstateCallback(const ros::TimerEvent &event);
    void loadstateCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg);

    // void search_and_pub_loop(pcl::PointCloud<pcl::PointXYZI> transform_cloud, int submap_id);

    /**
     * @brief search_and_pub_loop 的开关索引
     *
     * 用法示例:
     *   search_and_pub_loop(tf, cloud, pose);                       // 全部默认开启
     *   search_and_pub_loop(tf, cloud, pose, {true, false, true});  // 关闭滑动窗口
     *   search_and_pub_loop(tf, cloud, pose, {false, false, false});// 全部关闭（仅发布变换）
     */
    enum LoopOption {
        OPT_ICP_REFINE    = 0,  ///< 是否做 ICP 精匹配
        OPT_SLIDING_WINDOW = 1, ///< 是否用滑动窗口一致性检查
        OPT_VISUALIZE      = 2, ///< 是否发布可视化点云
        LOOP_OPTION_COUNT  = 3
    };

    void search_and_pub_loop(std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform, 
        pcl::PointCloud<pcl::PointXYZI> transform_cloud, std::pair<Eigen::Vector3d, Eigen::Matrix3d> current_pose_,
        std::vector<bool> options = {true, true, true});

    /**
     * @brief ICP 点云精匹配：用初始 loop_transform 变换点云并发布预览，
     *        然后调用 ikd_match->cloud_matching 进行 ICP 精配准
     * @param[in,out] loop_transform  输入初始变换，输出 ICP 精配准后的变换
     * @param transform_cloud         当前帧点云（世界系）
     * @param current_pose_           当前里程计位姿
     * @return true 如果 ICP 收敛
     */
    bool refineWithICP(std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
        const pcl::PointCloud<pcl::PointXYZI> &transform_cloud,
        const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &current_pose_);

    /**
     * @brief 统一的点云可视化发布函数
     *
     * 功能：
     *   1. 发布原始点云到 /cloud_cureent
     *   2. 用 loop_transform 变换点云
     *   3. 按匹配结果着色（成功=绿色，失败=红色）发布到 /cloud_transformed
     *
     * @param transform_cloud  当前帧点云（世界系）
     * @param loop_transform   变换 (t, R)
     * @param is_true_loop     匹配是否成功（决定颜色：绿/红）
     */
    void publishVisualizationClouds(
        const pcl::PointCloud<pcl::PointXYZI> &transform_cloud,
        const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
        bool is_true_loop);

    /**
     * @brief 用 loop_transform 变换点云并发布到指定 publisher
     * @param cloud       原始点云
     * @param transform   变换 (t, R)：p' = R * p + t
     * @param publisher   要发布的 ROS publisher
     * @param intensity   变换后点的 intensity 值（默认 100）
     */
    void transformAndPublishCloud(
        const pcl::PointCloud<pcl::PointXYZI> &cloud,
        const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform,
        ros::Publisher &publisher,
        float intensity = 100.0f);

    /**
     * @brief 将 loop_transform 转换为位姿（求逆）并发布 PoseStamped
     *
     * 变换关系: pose = inv(loop_transform)
     *   pose.R = loop_transform.R^T
     *   pose.t = -pose.R * loop_transform.t
     *
     * @param loop_transform  变换 (t, R)
     * @param publisher       要发布的 ROS publisher
     */
    void publishLoopTransformAsPose(
        const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
        ros::Publisher &publisher);

    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void load_global_pcd();

    /**
     * @brief 根据 (x,y) 位置从全局点云中估算 Z 高度
     * @param position  输入/输出：position.z() 会被更新为区域内平均 Z
     * @param side_length  搜索区域的边长（米）
     * @return true 如果区域内有点, false 如果区域为空
     */
    bool estimateZFromMap(Eigen::Vector3d &position, double side_length) const;

    /**
     * @brief 根据目标位姿和当前位姿，计算初始变换 T，使得 T * current_pose = init_pose
     * @param init_pose  目标位姿 (平移, 旋转)
     * @return 变换 (t, R)，其中 R = init_R * current_R^T, t = init_t - R * current_t
     */
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> computeInitTransform(
        const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &init_pose) const;

    // void load_config();
    void addTransform(
    std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& window,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& new_tf,
    size_t window_size);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> computeFilteredAverage(
        const std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& window,
        double threshold);

    /**
     * @brief 滑动窗口一致性检查：添加变换到窗口、计算滤波平均、
     *        检查最近 similarity_count 次结果的平移/旋转是否收敛
     * @param loop_transform  当前 ICP 匹配得到的 loop_transform
     * @param window_size     滑动窗口大小（默认 5）
     * @param filter_threshold 滤波阈值（默认 0.1 米）
     * @param similarity_count 连续一致次数（默认 3）
     * @param trans_threshold  平移变化阈值（默认 0.06 米）
     * @param angle_threshold  旋转变化阈值（默认 2.0°）
     * @return true 如果滑动窗口内连续 similarity_count 次结果一致
     */
    bool checkSlidingWindowConsistency(
        const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
        size_t window_size = 5,
        double filter_threshold = 0.1,
        int similarity_count = 3,
        double trans_threshold = 0.06,
        double angle_threshold = 2.0);
    // std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> pose_list;
    pcl::PointCloud<pcl::PointXYZI> whole_cloud;
    pcl::PointCloud<pcl::PointXYZI> current_cloud;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> current_pose;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> last_loop_transform;

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
    ros::Publisher pubMapLocalization;  // 发布地图定位位置
    ros::Publisher pubWholeCloud;
    ros::Subscriber initialpose_sub_;

    // ros::Subscriber rtk_covariance_sub_;

      //for test only
     ros::Subscriber car_odom_sub;


    GeographicLib::LocalCartesian geo_converter;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    double count = 0.0;
    double all_count = 0.0;
    std::string visualization_frame_id;  // 可视化消息的 frame_id

    // RTK 重定位模块
    std::unique_ptr<RtkRelocator> rtk_relocator_;
    // QR 重定位模块
    std::unique_ptr<QrRelocator> qr_relocator_;

    // ── 三种定位源的开关与 options ──────────────────────────────────────
    bool manual_loop_enable = true;
    std::vector<bool> manual_loop_options;  // {icp, sliding_window, visualize}

    bool rtk_loop_enable = true;
    std::vector<bool> rtk_loop_options;

    bool qr_loop_enable = true;
    std::vector<bool> qr_loop_options;
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
    // double scale_tp = 4.0;
    // double scale_fp = 5.0;
    // double scale_path = 3.0;
    Eigen::Vector3d lidar_imu_trans;
    std::mutex ikd_match_mtx;
    
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_tranform;  // 初始变换（如果不存在则添加）

};

// } // namespace btc_descriptor

#endif // PLACE_RECOGNITION_HPP

