#ifndef QR_RELOCATOR_HPP
#define QR_RELOCATOR_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <Aruco_detection/ArucoMarkers.h>

#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <thread>
#include <atomic>

// Forward declaration
class PlaceRecognition;

/**
 * @class QrRelocator
 * @brief 基于 ArUco / QR 二维码的重定位模块
 *
 * 从 qr_target_visualizer.cpp 提取出来，负责：
 *   1. 读取 config.yaml 中的 QR 码角点坐标与 ArUco ID
 *   2. 计算充电桩坐标系变换（世界原点 = 充电桩质心）
 *   3. 为每个 ArUco ID 建立 T_world_marker
 *   4. 订阅 /aruco_detection_node/markers，计算机器人世界位姿
 *   5. 调用 PlaceRecognition::computeInitTransform + search_and_pub_loop
 *
 * 同时保留 RViz 可视化（静态靶标、相机/机器人标记、TF 等），
 * 也可以独立运行可视化节点 (qr_target_visualizer)。
 *
 * 使用方法:
 *   在 PlaceRecognition 构造函数中:
 *     qr_relocator_ = std::make_unique<QrRelocator>(nh, this);
 */
class QrRelocator
{
public:
    // ── 内部数据结构 ────────────────────────────────────────────────────
    struct Corner { std::string id; double east, north, up; };

    struct Target
    {
        std::string name;
        std::string description;
        std::vector<int> aruco_ids;
        std::vector<Corner> corners;
    };

    struct RigidTransform
    {
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
    };

    struct MarkerWorldPose
    {
        Eigen::Matrix4d T;      // T_world_marker (4×4)
        bool is_front;
    };

    // ── 构造函数 ────────────────────────────────────────────────────────
    /**
     * @param nh   ROS NodeHandle
     * @param pr   PlaceRecognition 指针 (用于 computeInitTransform / search_and_pub_loop)
     */
    QrRelocator(ros::NodeHandle &nh, PlaceRecognition *pr);
    ~QrRelocator();

private:
    // ── ArUco 回调 ──────────────────────────────────────────────────────
    void arucoCallback(const Aruco_detection::ArucoMarkers::ConstPtr &msg);

    /**
     * @brief 测试回调：反推二维码世界位置
     *
     * 适用场景：没有预先测量的二维码绝对坐标。
     * 利用 PlaceRecognition 提供的机器人当前世界位姿（odom + last_loop_transform），
     * 结合 ArUco 检测得到的相对位姿 T_camera_marker，反推出二维码在世界坐标系中的
     * 位置和朝向，并通过 visualization_msgs::MarkerArray 在 RViz 中可视化。
     *
     * 变换链:
     *   T_world_robot  = last_loop_transform * current_pose   (从 PlaceRecognition 获取)
     *   T_world_camera = T_world_robot * T_robot_camera
     *   T_world_marker = T_world_camera * T_camera_marker     (反推结果)
     */
    void arucoTestCallback(const Aruco_detection::ArucoMarkers::ConstPtr &msg);

    // ── 配置加载 ────────────────────────────────────────────────────────
    std::vector<Target> loadConfig(const std::string &path);
    RigidTransform computeDockTransform(const std::vector<Target> &targets);
    void transformTargets(std::vector<Target> &targets, const RigidTransform &tf);
    std::map<int, MarkerWorldPose> buildMarkerWorldPoses(const std::vector<Target> &targets);

    // ── 工具 ────────────────────────────────────────────────────────────
    static Eigen::Quaterniond rotToQuat(const Eigen::Matrix3d &R);

    // ── 宿主指针 ────────────────────────────────────────────────────────
    PlaceRecognition *pr_;

    // ── ROS ─────────────────────────────────────────────────────────────
    ros::Subscriber aruco_sub_;
    ros::Publisher  pub_matched_pose_;
    ros::Publisher  pub_estimated_markers_;   ///< 反推的二维码位姿可视化 (MarkerArray)
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // ── 测试模式 ────────────────────────────────────────────────────────
    bool test_mode_ = false;  ///< 为 true 时使用 arucoTestCallback 替代 arucoCallback

    /// 测试模式下：最近一次回调中所有检测到的 marker 的反推世界位姿
    struct EstimatedMarker {
        int aruco_id;
        Eigen::Matrix4d T_world_marker;  ///< 反推的世界位姿
    };
    std::vector<EstimatedMarker> test_estimated_markers_;  ///< 受 mtx_ 保护

    /// 键盘监听线程（测试模式）
    std::thread keyboard_thread_;
    std::atomic<bool> running_{true};
    void keyboardListenerLoop();

    /// 将当前缓存的反推位姿保存到 YAML 文件
    /// @param marker_name  用户输入的名称（如 "qr_code_3"）
    /// @param marker_size  二维码板物理尺寸（米），用于从中心+姿态生成四个角点
    void saveEstimatedMarkersToYaml(const std::string &marker_name,
                                    double marker_size = 0.20);

    /// 配置文件路径（用于保存）
    std::string config_file_path_;

    // ── 数据 ────────────────────────────────────────────────────────────
    std::map<int, MarkerWorldPose> marker_world_poses_;
    std::string frame_id_;
    std::mutex  mtx_;

    // Camera-to-robot extrinsic: T_robot_camera
    Eigen::Matrix4d T_robot_camera_;

    // Height-difference filter
    double expected_height_diff_;
    double height_diff_tol_;
};

#endif // QR_RELOCATOR_HPP
