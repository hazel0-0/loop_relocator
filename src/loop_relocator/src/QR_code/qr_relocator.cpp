#include "qr_relocator.hpp"
#include "../place_recognition.hpp"   // 完整定义

#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fstream>
#include <iomanip>

// ═══════════════════════════════════════════════════════════════════════════════
//  YAML config loader
// ═══════════════════════════════════════════════════════════════════════════════

std::vector<QrRelocator::Target> QrRelocator::loadConfig(const std::string &path)
{
    YAML::Node root = YAML::LoadFile(path);
    std::vector<Target> targets;
    for (const auto &t : root["targets"])
    {
        Target tgt;
        tgt.name        = t["name"].as<std::string>();
        tgt.description = t["description"].as<std::string>();
        if (t["aruco_ids"])
            for (const auto &id_node : t["aruco_ids"])
                tgt.aruco_ids.push_back(id_node.as<int>());
        for (const auto &c : t["corners"])
        {
            Corner corner;
            corner.id   = c["id"].as<std::string>();
            corner.east = c["east"].as<double>();
            corner.north = c["north"].as<double>();
            corner.up    = c["up"].as<double>();
            tgt.corners.push_back(corner);
        }
        targets.push_back(tgt);
    }
    return targets;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Dock-centred transform (world origin = charging dock centroid)
// ═══════════════════════════════════════════════════════════════════════════════

QrRelocator::RigidTransform QrRelocator::computeDockTransform(
    const std::vector<Target> &targets)
{
    const Target *dock = nullptr;
    for (const auto &tgt : targets)
        if (tgt.name == "charging_dock") { dock = &tgt; break; }
    if (!dock || dock->corners.size() < 4)
        throw std::runtime_error("charging_dock not found or < 4 corners");

    auto toV = [](const Corner &c) {
        return Eigen::Vector3d(c.east, c.north, c.up);
    };
    Eigen::Vector3d p1 = toV(dock->corners[0]);  // 3-1
    Eigen::Vector3d p2 = toV(dock->corners[1]);  // 3-2
    Eigen::Vector3d p3 = toV(dock->corners[2]);  // 3-3

    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for (const auto &c : dock->corners) origin += toV(c);
    origin /= 4.0;

    Eigen::Vector3d x_axis = (p1 - p3).normalized();
    Eigen::Vector3d y_axis = (p1 - p2).normalized();
    y_axis = (y_axis - y_axis.dot(x_axis) * x_axis).normalized();
    Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

    RigidTransform tf;
    tf.R.col(0) = x_axis;
    tf.R.col(1) = y_axis;
    tf.R.col(2) = z_axis;
    tf.t = origin;
    return tf;
}

void QrRelocator::transformTargets(std::vector<Target> &targets,
                                   const RigidTransform &tf)
{
    for (auto &tgt : targets)
        for (auto &c : tgt.corners)
        {
            Eigen::Vector3d local = tf.R.transpose() *
                (Eigen::Vector3d(c.east, c.north, c.up) - tf.t);
            c.east  = local.x();
            c.north = local.y();
            c.up    = local.z();
        }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Build world-frame pose for each ArUco marker
// ═══════════════════════════════════════════════════════════════════════════════

std::map<int, QrRelocator::MarkerWorldPose>
QrRelocator::buildMarkerWorldPoses(const std::vector<Target> &targets)
{
    std::map<int, MarkerWorldPose> poses;

    for (const auto &tgt : targets)
    {
        if (tgt.aruco_ids.empty() || tgt.corners.size() < 4) continue;

        auto toV = [](const Corner &c) {
            return Eigen::Vector3d(c.east, c.north, c.up);
        };
        Eigen::Vector3d c0 = toV(tgt.corners[0]);
        Eigen::Vector3d c1 = toV(tgt.corners[1]);
        Eigen::Vector3d c2 = toV(tgt.corners[2]);
        Eigen::Vector3d c3 = toV(tgt.corners[3]);
        Eigen::Vector3d centre = (c0 + c1 + c2 + c3) / 4.0;

        Eigen::Vector3d right_raw = ((c1 - c0) + (c3 - c2)).normalized();
        Eigen::Vector3d up_raw    = ((c0 - c2) + (c1 - c3)).normalized();
        Eigen::Vector3d normal    = right_raw.cross(up_raw).normalized();
        if (normal.y() < 0) normal = -normal;

        // Front face: ArUco Z=normal(≈Y+), Y=up(≈Z+), X=Y×Z
        if (tgt.aruco_ids.size() >= 1)
        {
            Eigen::Vector3d aruco_z = normal;
            Eigen::Vector3d world_up(0, 0, 1);
            Eigen::Vector3d aruco_y = (world_up - world_up.dot(aruco_z) * aruco_z).normalized();
            Eigen::Vector3d aruco_x = aruco_y.cross(aruco_z).normalized();

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,1>(0,0) = aruco_x;
            T.block<3,1>(0,1) = aruco_y;
            T.block<3,1>(0,2) = aruco_z;
            T.block<3,1>(0,3) = centre;

            poses[tgt.aruco_ids[0]] = {T, true};
            ROS_INFO("QR: Front marker ID=%d  centre=(%.3f,%.3f,%.3f)",
                     tgt.aruco_ids[0], centre.x(), centre.y(), centre.z());
        }

        // Back face: ArUco Z=-normal(≈Y-), Y=up(≈Z+), X=Y×Z
        if (tgt.aruco_ids.size() >= 2)
        {
            Eigen::Vector3d aruco_z = -normal;
            Eigen::Vector3d world_up(0, 0, 1);
            Eigen::Vector3d aruco_y = (world_up - world_up.dot(aruco_z) * aruco_z).normalized();
            Eigen::Vector3d aruco_x = aruco_y.cross(aruco_z).normalized();

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,1>(0,0) = aruco_x;
            T.block<3,1>(0,1) = aruco_y;
            T.block<3,1>(0,2) = aruco_z;
            T.block<3,1>(0,3) = centre;

            poses[tgt.aruco_ids[1]] = {T, false};
            ROS_INFO("QR: Back  marker ID=%d  centre=(%.3f,%.3f,%.3f)",
                     tgt.aruco_ids[1], centre.x(), centre.y(), centre.z());
        }
    }
    return poses;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Quaternion helper
// ═══════════════════════════════════════════════════════════════════════════════

Eigen::Quaterniond QrRelocator::rotToQuat(const Eigen::Matrix3d &R)
{
    Eigen::Quaterniond q(R);
    q.normalize();
    return q;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Constructor
// ═══════════════════════════════════════════════════════════════════════════════

QrRelocator::QrRelocator(ros::NodeHandle &nh, PlaceRecognition *pr)
    : pr_(pr),
      T_robot_camera_(Eigen::Matrix4d::Identity()),
      expected_height_diff_(-0.5),
      height_diff_tol_(0.10)
{
    // ── Load config ──────────────────────────────────────────────────────
    std::string pkg_path = ros::package::getPath("loop_relocator");
    std::string cfg_default = pkg_path + "/config/qr_config.yaml";
    std::string config_file;
    nh.param<std::string>("qr_config_file", config_file, cfg_default);
    nh.param<std::string>("qr_frame_id", frame_id_, "world");
    config_file_path_ = config_file;

    std::vector<Target> targets;
    try
    {
        targets = loadConfig(config_file);
        ROS_INFO("QR: Loaded %zu target(s) from %s",
                 targets.size(), config_file.c_str());
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("QR: Failed to load config: %s", e.what());
        return;
    }

    // ── Filter params ────────────────────────────────────────────────────
    try
    {
        YAML::Node root = YAML::LoadFile(config_file);
        if (root["camera_marker_height_diff"])
            expected_height_diff_ = root["camera_marker_height_diff"].as<double>();
        if (root["height_diff_tolerance"])
            height_diff_tol_ = root["height_diff_tolerance"].as<double>();
        ROS_INFO("QR: Height filter: expected_diff=%.3f, tol=%.3f",
                 expected_height_diff_, height_diff_tol_);
    }
    catch (const std::exception &e)
    {
        ROS_WARN("QR: Could not load filter params: %s", e.what());
    }

    // ── Camera-to-robot extrinsics ───────────────────────────────────────
    try
    {
        YAML::Node root = YAML::LoadFile(config_file);
        if (root["camera_to_robot"])
        {
            auto node   = root["camera_to_robot"];
            auto t_node = node["translation"];
            auto r_node = node["rotation"];

            Eigen::Vector3d t_rc(t_node[0].as<double>(),
                                 t_node[1].as<double>(),
                                 t_node[2].as<double>());
            Eigen::Matrix3d R_rc;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R_rc(i, j) = r_node[i][j].as<double>();

            T_robot_camera_ = Eigen::Matrix4d::Identity();
            T_robot_camera_.block<3,3>(0,0) = R_rc;
            T_robot_camera_.block<3,1>(0,3) = t_rc;
            ROS_INFO("QR: Camera-to-robot extrinsic loaded.");
        }
    }
    catch (const std::exception &e)
    {
        ROS_WARN("QR: Could not load camera_to_robot: %s", e.what());
    }

    // ── Dock-centred transform ───────────────────────────────────────────
    try
    {
        RigidTransform dock_tf = computeDockTransform(targets);
        transformTargets(targets, dock_tf);
        ROS_INFO("QR: Dock-centred transform applied.");
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("QR: Dock transform failed: %s", e.what());
        return;
    }

    // ── Build ArUco world poses ──────────────────────────────────────────
    marker_world_poses_ = buildMarkerWorldPoses(targets);
    ROS_INFO("QR: %zu ArUco ID -> world pose mappings.",
             marker_world_poses_.size());

    // ── Publisher for matched pose visualization ─────────────────────────
    pub_matched_pose_ = nh.advertise<nav_msgs::Odometry>("/qr_matched_pose", 10);

    // ── 反推二维码位姿的可视化 Publisher ──────────────────────────────────
    pub_estimated_markers_ = nh.advertise<visualization_msgs::MarkerArray>(
        "/qr_estimated_markers", 10);

    // ── 测试模式开关 ─────────────────────────────────────────────────────
    nh.param<bool>("qr_loop/test_mode", test_mode_, false);

    // ── Subscribe to ArUco ───────────────────────────────────────────────
    if (test_mode_)
    {
        aruco_sub_ = nh.subscribe("/aruco_detection_node/markers", 10,
                                  &QrRelocator::arucoTestCallback, this);
        ROS_INFO("QR: *** TEST MODE *** Subscribed to /aruco_detection_node/markers "
                 "(back-computing marker world poses)");

        // 启动键盘监听线程
        keyboard_thread_ = std::thread(&QrRelocator::keyboardListenerLoop, this);
    }
    else
    {
        aruco_sub_ = nh.subscribe("/aruco_detection_node/markers", 10,
                                  &QrRelocator::arucoCallback, this);
        ROS_INFO("QR: Subscribed to /aruco_detection_node/markers");
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  ArUco callback — compute robot world pose → init_transform → search_and_pub
// ═══════════════════════════════════════════════════════════════════════════════

void QrRelocator::arucoCallback(const Aruco_detection::ArucoMarkers::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx_);

    for (const auto &marker : msg->markers)
    {
        int aruco_id = marker.id;
        auto it = marker_world_poses_.find(aruco_id);
        if (it == marker_world_poses_.end())
        {
            ROS_WARN_THROTTLE(5.0,
                "QR: ArUco ID=%d not in config, ignoring.", aruco_id);
            continue;
        }

        const Eigen::Matrix4d &T_world_marker = it->second.T;

        // ── T_camera_marker from detection ───────────────────────────────
        const auto &p = marker.pose.position;
        const auto &o = marker.pose.orientation;
        Eigen::Quaterniond q_cm(o.w, o.x, o.y, o.z);
        Eigen::Matrix3d R_cm = q_cm.toRotationMatrix();
        Eigen::Vector3d t_cm(p.x, p.y, p.z);

        // ── T_world_camera = T_world_marker * inv(T_camera_marker) ──────
        Eigen::Matrix4d T_marker_cam = Eigen::Matrix4d::Identity();
        T_marker_cam.block<3,3>(0,0) = R_cm.transpose();
        T_marker_cam.block<3,1>(0,3) = -R_cm.transpose() * t_cm;

        Eigen::Matrix4d T_world_cam = T_world_marker * T_marker_cam;
        Eigen::Vector3d cam_pos = T_world_cam.block<3,1>(0,3);

        // ── Height-difference filter ─────────────────────────────────────
        {
            double marker_z = T_world_marker.block<3,1>(0,3).z();
            double actual_diff = cam_pos.z() - marker_z;
            double diff_error = std::abs(actual_diff - expected_height_diff_);
            if (diff_error > height_diff_tol_)
            {
                ROS_WARN_THROTTLE(1.0,
                    "QR: ArUco ID=%d REJECTED height diff=%.3fm (expect %.3f±%.3f)",
                    aruco_id, actual_diff, expected_height_diff_, height_diff_tol_);
                continue;
            }
        }

        // ── T_world_robot = T_world_camera * inv(T_robot_camera) ────────
        Eigen::Matrix3d R_rc = T_robot_camera_.block<3,3>(0,0);
        Eigen::Vector3d t_rc = T_robot_camera_.block<3,1>(0,3);
        Eigen::Matrix4d T_camera_robot = Eigen::Matrix4d::Identity();
        T_camera_robot.block<3,3>(0,0) = R_rc.transpose();
        T_camera_robot.block<3,1>(0,3) = -R_rc.transpose() * t_rc;

        Eigen::Matrix4d T_world_robot = T_world_cam * T_camera_robot;
        Eigen::Vector3d robot_pos = T_world_robot.block<3,1>(0,3);
        Eigen::Matrix3d robot_R   = T_world_robot.block<3,3>(0,0);

        ROS_INFO_THROTTLE(1.0,
            "QR: Robot via ArUco ID=%d (%s): pos=(%.3f, %.3f, %.3f)",
            aruco_id, it->second.is_front ? "front" : "back",
            robot_pos.x(), robot_pos.y(), robot_pos.z());

        // ── 构造 init_pose 并调用 PlaceRecognition 的匹配流程 ────────────
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
        init_pose.first  = robot_pos;
        init_pose.second = robot_R;

        auto init_tf = pr_->computeInitTransform(init_pose);

        // 发布 QR 位姿可视化
        Eigen::Quaterniond quat(init_pose.second);
        nav_msgs::Odometry pose_msg;
        pose_msg.header.stamp    = msg->header.stamp;
        pose_msg.header.frame_id = pr_->visualization_frame_id;
        pose_msg.pose.pose.position.x    = init_pose.first.x();
        pose_msg.pose.pose.position.y    = init_pose.first.y();
        pose_msg.pose.pose.position.z    = init_pose.first.z();
        pose_msg.pose.pose.orientation.x = quat.x();
        pose_msg.pose.pose.orientation.y = quat.y();
        pose_msg.pose.pose.orientation.z = quat.z();
        pose_msg.pose.pose.orientation.w = quat.w();
        pub_matched_pose_.publish(pose_msg);
        pr_->pubMatchedPose.publish(pose_msg);

        std::cout << "QR init_transform t: " << init_tf.first.transpose() << std::endl;
        std::cout << "QR init_transform R:\n" << init_tf.second << std::endl;
        std::cout << "current_cloud size: " << pr_->current_cloud.size() << std::endl;

        double start_time = ros::Time::now().toSec();
        pr_->search_and_pub_loop(init_tf, pr_->current_cloud, pr_->current_pose, pr_->qr_loop_options);
        double end_time = ros::Time::now().toSec();
        std::cout << "QR matching time: " << end_time - start_time << std::endl;

        // Publish TF: world → robot_qr_estimated
        {
            Eigen::Quaterniond robot_q = rotToQuat(robot_R);
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp    = msg->header.stamp;
            tf_msg.header.frame_id = pr_->visualization_frame_id;
            tf_msg.child_frame_id  = "robot_qr_estimated";
            tf_msg.transform.translation.x = robot_pos.x();
            tf_msg.transform.translation.y = robot_pos.y();
            tf_msg.transform.translation.z = robot_pos.z();
            tf_msg.transform.rotation.x = robot_q.x();
            tf_msg.transform.rotation.y = robot_q.y();
            tf_msg.transform.rotation.z = robot_q.z();
            tf_msg.transform.rotation.w = robot_q.w();
            tf_broadcaster_.sendTransform(tf_msg);
        }

        break;  // use first valid detection
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  测试回调 — 反推二维码世界位置并可视化
//
//  变换链:
//    T_world_robot  = last_loop_transform ∘ current_pose
//    T_world_camera = T_world_robot * T_robot_camera
//    T_world_marker = T_world_camera * T_camera_marker
// ═══════════════════════════════════════════════════════════════════════════════

void QrRelocator::arucoTestCallback(const Aruco_detection::ArucoMarkers::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(mtx_);

    // ── 1. 从 PlaceRecognition 获取机器人世界位姿 ─────────────────────────
    //    map_pose = last_loop_transform * current_pose
    const auto &cur  = pr_->current_pose;          // (t, R) from odometry
    const auto &loop = pr_->last_loop_transform;    // (t, R) accumulated correction

    Eigen::Matrix4d T_world_robot = Eigen::Matrix4d::Identity();
    T_world_robot.block<3,3>(0,0) = loop.second * cur.second;
    T_world_robot.block<3,1>(0,3) = loop.second * cur.first + loop.first;

    // ── 2. T_world_camera = T_world_robot * T_robot_camera ───────────────
    Eigen::Matrix4d T_world_camera = T_world_robot * T_robot_camera_;

    // ── 3. 对每个检测到的 ArUco，反推 T_world_marker ─────────────────────
    //    仅在本帧确实检测到标记时才替换缓存，否则保留上一帧结果
    std::vector<EstimatedMarker> current_frame_markers;

    for (const auto &marker : msg->markers)
    {
        int aruco_id = marker.id;

        // T_camera_marker from ArUco detection
        const auto &p = marker.pose.position;
        const auto &o = marker.pose.orientation;
        Eigen::Quaterniond q_cm(o.w, o.x, o.y, o.z);
        Eigen::Matrix4d T_camera_marker = Eigen::Matrix4d::Identity();
        T_camera_marker.block<3,3>(0,0) = q_cm.toRotationMatrix();
        T_camera_marker.block<3,1>(0,3) = Eigen::Vector3d(p.x, p.y, p.z);

        // 反推: T_world_marker = T_world_camera * T_camera_marker
        Eigen::Matrix4d T_world_marker = T_world_camera * T_camera_marker;
        Eigen::Vector3d marker_pos = T_world_marker.block<3,1>(0,3);
        Eigen::Matrix3d marker_R   = T_world_marker.block<3,3>(0,0);
        Eigen::Quaterniond marker_q(marker_R);
        marker_q.normalize();

        ROS_INFO_THROTTLE(0.5,
            "QR_TEST: ArUco ID=%d  estimated world pos=(%.3f, %.3f, %.3f)",
            aruco_id, marker_pos.x(), marker_pos.y(), marker_pos.z());

        // 缓存反推结果
        current_frame_markers.push_back({aruco_id, T_world_marker});

        // ── 发布 TF: world → aruco_estimated_<id> ────────────────────────
        {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp    = msg->header.stamp;
            tf_msg.header.frame_id = pr_->visualization_frame_id;
            tf_msg.child_frame_id  = "aruco_estimated_" + std::to_string(aruco_id);
            tf_msg.transform.translation.x = marker_pos.x();
            tf_msg.transform.translation.y = marker_pos.y();
            tf_msg.transform.translation.z = marker_pos.z();
            tf_msg.transform.rotation.x = marker_q.x();
            tf_msg.transform.rotation.y = marker_q.y();
            tf_msg.transform.rotation.z = marker_q.z();
            tf_msg.transform.rotation.w = marker_q.w();
            tf_broadcaster_.sendTransform(tf_msg);
        }
    }

    // 仅在本帧有检测结果时才更新缓存，空帧保留上次结果
    if (!current_frame_markers.empty())
        test_estimated_markers_ = std::move(current_frame_markers);

    // 无论本帧是否有新检测，都基于最新缓存重新发布可视化，防止 marker 过期消失
    if (!test_estimated_markers_.empty())
    {
        visualization_msgs::MarkerArray pub_array;
        int pub_id = 0;
        ros::Time now = ros::Time::now();

        for (const auto &em : test_estimated_markers_)
        {
            Eigen::Vector3d pos = em.T_world_marker.block<3,1>(0,3);
            Eigen::Matrix3d R   = em.T_world_marker.block<3,3>(0,0);
            Eigen::Quaterniond q(R);
            q.normalize();

            // 立方体
            {
                visualization_msgs::Marker cube;
                cube.header.stamp    = now;
                cube.header.frame_id = pr_->visualization_frame_id;
                cube.ns       = "qr_test_markers";
                cube.id       = pub_id++;
                cube.type     = visualization_msgs::Marker::CUBE;
                cube.action   = visualization_msgs::Marker::ADD;
                cube.pose.position.x    = pos.x();
                cube.pose.position.y    = pos.y();
                cube.pose.position.z    = pos.z();
                cube.pose.orientation.x = q.x();
                cube.pose.orientation.y = q.y();
                cube.pose.orientation.z = q.z();
                cube.pose.orientation.w = q.w();
                cube.scale.x = 0.20;   // X 轴 = marker 水平方向（宽度）
                cube.scale.y = 0.20;   // Y 轴 = marker 竖直方向（高度）
                cube.scale.z = 0.02;   // Z 轴 = marker 法线方向（厚度）
                cube.color.r = 0.0f;
                cube.color.g = 1.0f;
                cube.color.b = 0.0f;
                cube.color.a = 0.7f;
                cube.lifetime = ros::Duration(2.0);
                pub_array.markers.push_back(cube);
            }

            // 文字标签
            {
                visualization_msgs::Marker text;
                text.header.stamp    = now;
                text.header.frame_id = pr_->visualization_frame_id;
                text.ns       = "qr_test_labels";
                text.id       = pub_id++;
                text.type     = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text.action   = visualization_msgs::Marker::ADD;
                text.pose.position.x = pos.x();
                text.pose.position.y = pos.y();
                text.pose.position.z = pos.z() + 0.15;
                text.scale.z = 0.08;
                text.color.r = 1.0f;
                text.color.g = 1.0f;
                text.color.b = 1.0f;
                text.color.a = 1.0f;
                text.lifetime = ros::Duration(2.0);
                char buf[128];
                std::snprintf(buf, sizeof(buf), "ID=%d (%.3f, %.3f, %.3f)",
                    em.aruco_id, pos.x(), pos.y(), pos.z());
                text.text = buf;
                pub_array.markers.push_back(text);
            }

            // 法线箭头
            {
                visualization_msgs::Marker arrow;
                arrow.header.stamp    = now;
                arrow.header.frame_id = pr_->visualization_frame_id;
                arrow.ns       = "qr_test_normals";
                arrow.id       = pub_id++;
                arrow.type     = visualization_msgs::Marker::ARROW;
                arrow.action   = visualization_msgs::Marker::ADD;
                Eigen::Vector3d z_axis = R.col(2);
                geometry_msgs::Point start, end;
                start.x = pos.x();  start.y = pos.y();  start.z = pos.z();
                end.x = pos.x() + z_axis.x() * 0.3;
                end.y = pos.y() + z_axis.y() * 0.3;
                end.z = pos.z() + z_axis.z() * 0.3;
                arrow.points.push_back(start);
                arrow.points.push_back(end);
                arrow.scale.x = 0.02;
                arrow.scale.y = 0.04;
                arrow.scale.z = 0.0;
                arrow.color.r = 0.0f;
                arrow.color.g = 0.5f;
                arrow.color.b = 1.0f;
                arrow.color.a = 0.9f;
                arrow.lifetime = ros::Duration(2.0);
                pub_array.markers.push_back(arrow);
            }
        }

        pub_estimated_markers_.publish(pub_array);
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  析构函数
// ═══════════════════════════════════════════════════════════════════════════════

QrRelocator::~QrRelocator()
{
    running_ = false;
    if (keyboard_thread_.joinable())
        keyboard_thread_.join();
}

// ═══════════════════════════════════════════════════════════════════════════════
//  键盘监听线程 — 测试模式下等待空格键触发保存
// ═══════════════════════════════════════════════════════════════════════════════

void QrRelocator::keyboardListenerLoop()
{
    ROS_INFO("QR_TEST: Keyboard listener started.");
    ROS_INFO("QR_TEST: Press [SPACE] to save current estimated marker poses.");
    ROS_INFO("QR_TEST: Press [q] to quit keyboard listener.");

    // 设置终端为非规范模式（逐字符读取，无需回车）
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);  // 关闭行缓冲和回显
    newt.c_cc[VMIN]  = 0;               // 非阻塞
    newt.c_cc[VTIME] = 1;               // 100ms 超时
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (running_ && ros::ok())
    {
        char ch = 0;
        int n = read(STDIN_FILENO, &ch, 1);
        if (n <= 0) continue;

        if (ch == ' ')
        {
            // 检查是否有缓存数据
            std::vector<EstimatedMarker> snapshot;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                snapshot = test_estimated_markers_;
            }

            if (snapshot.empty())
            {
                std::cout << "\n[QR_TEST] No estimated markers available. "
                          << "Make sure ArUco markers are visible.\n" << std::flush;
                continue;
            }

            // 显示当前缓存的 marker 信息
            std::cout << "\n╔══════════════════════════════════════════════╗\n";
            std::cout <<   "║     QR_TEST: Save Estimated Marker Poses     ║\n";
            std::cout <<   "╚══════════════════════════════════════════════╝\n";
            std::cout << "  Detected " << snapshot.size() << " marker(s):\n";
            for (const auto &em : snapshot)
            {
                Eigen::Vector3d pos = em.T_world_marker.block<3,1>(0,3);
                std::cout << "    ArUco ID=" << em.aruco_id
                          << "  pos=(" << std::fixed << std::setprecision(4)
                          << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
            }

            // 切回规范模式以读取用户输入
            tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

            std::cout << "\n  Enter marker name (e.g. qr_code_3): " << std::flush;
            std::string marker_name;
            std::getline(std::cin, marker_name);

            if (marker_name.empty())
            {
                std::cout << "  [Cancelled] Empty name, skipping save.\n" << std::flush;
            }
            else
            {
                std::cout << "  Enter marker board size in meters [0.20]: " << std::flush;
                std::string size_str;
                std::getline(std::cin, size_str);
                double marker_size = 0.20;
                if (!size_str.empty())
                {
                    try { marker_size = std::stod(size_str); }
                    catch (...) { marker_size = 0.20; }
                }

                // 暂存 snapshot 用于保存
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    // 使用最新数据（可能已更新）
                    snapshot = test_estimated_markers_;
                }
                saveEstimatedMarkersToYaml(marker_name, marker_size);
            }

            // 恢复非规范模式
            tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        }
        else if (ch == 'q' || ch == 'Q')
        {
            std::cout << "\n[QR_TEST] Keyboard listener stopped.\n" << std::flush;
            break;
        }
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// ═══════════════════════════════════════════════════════════════════════════════
//  保存反推的二维码位姿到 YAML 文件
//
//  从 marker 中心位置 + 姿态 + board_size 生成 4 个角点：
//    corner0 = centre + R * ( +half, +half, 0)   (右上)
//    corner1 = centre + R * ( -half, +half, 0)   (左上)
//    corner2 = centre + R * ( +half, -half, 0)   (右下)
//    corner3 = centre + R * ( -half, -half, 0)   (左下)
//  其中 R 的 X 轴 = marker 水平方向，Y 轴 = marker 竖直方向
// ═══════════════════════════════════════════════════════════════════════════════

void QrRelocator::saveEstimatedMarkersToYaml(const std::string &marker_name,
                                              double marker_size)
{
    std::vector<EstimatedMarker> snapshot;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        snapshot = test_estimated_markers_;
    }

    if (snapshot.empty())
    {
        std::cout << "  [Error] No estimated markers to save.\n" << std::flush;
        return;
    }

    // 收集所有 aruco_ids
    std::vector<int> aruco_ids;
    for (const auto &em : snapshot)
        aruco_ids.push_back(em.aruco_id);

    // 用第一个 marker 的位姿来生成角点（如果有多个，取第一个）
    const Eigen::Matrix4d &T = snapshot[0].T_world_marker;
    Eigen::Vector3d centre = T.block<3,1>(0,3);
    Eigen::Vector3d x_axis = T.block<3,1>(0,0);  // marker 水平方向
    Eigen::Vector3d y_axis = T.block<3,1>(0,1);  // marker 竖直方向

    double half = marker_size / 2.0;

    // 4 个角点 (ENU 坐标)
    struct SaveCorner { std::string id; Eigen::Vector3d pos; };
    int idx = 1;
    std::string prefix = std::to_string(aruco_ids[0]) + "-";
    std::vector<SaveCorner> corners = {
        {prefix + std::to_string(idx++), centre + x_axis * half + y_axis * half},
        {prefix + std::to_string(idx++), centre - x_axis * half + y_axis * half},
        {prefix + std::to_string(idx++), centre + x_axis * half - y_axis * half},
        {prefix + std::to_string(idx++), centre - x_axis * half - y_axis * half},
    };

    // ── 追加写入 YAML 文件 ───────────────────────────────────────────────
    // 读取现有文件
    YAML::Node root;
    try { root = YAML::LoadFile(config_file_path_); }
    catch (...) { /* 文件不存在或解析失败，创建新的 */ }

    if (!root["targets"]) root["targets"] = YAML::Node(YAML::NodeType::Sequence);

    // 构建新 target 节点
    YAML::Node new_target;
    new_target["name"] = marker_name;
    new_target["description"] = "Auto-saved from QR test mode";

    YAML::Node ids_node(YAML::NodeType::Sequence);
    for (int id : aruco_ids) ids_node.push_back(id);
    new_target["aruco_ids"] = ids_node;
    // 让 aruco_ids 使用 flow 风格 [1, 3]
    new_target["aruco_ids"].SetStyle(YAML::EmitterStyle::Flow);

    YAML::Node corners_node(YAML::NodeType::Sequence);
    for (const auto &c : corners)
    {
        YAML::Node cn;
        cn["id"]    = c.id;
        cn["east"]  = std::round(c.pos.x() * 10000.0) / 10000.0;
        cn["north"] = std::round(c.pos.y() * 10000.0) / 10000.0;
        cn["up"]    = std::round(c.pos.z() * 10000.0) / 10000.0;
        corners_node.push_back(cn);
    }
    new_target["corners"] = corners_node;

    root["targets"].push_back(new_target);

    // 写回文件
    std::ofstream fout(config_file_path_);
    if (!fout.is_open())
    {
        std::cout << "  [Error] Cannot open " << config_file_path_ << " for writing.\n"
                  << std::flush;
        return;
    }

    YAML::Emitter emitter;
    emitter << root;
    fout << emitter.c_str() << std::endl;
    fout.close();

    std::cout << "\n  ✅ Saved target \"" << marker_name << "\" with "
              << aruco_ids.size() << " ArUco ID(s) and "
              << corners.size() << " corners to:\n"
              << "     " << config_file_path_ << "\n\n";
    std::cout << "  Corners:\n";
    for (const auto &c : corners)
    {
        std::cout << "    " << c.id
                  << "  east=" << std::fixed << std::setprecision(4) << c.pos.x()
                  << "  north=" << c.pos.y()
                  << "  up=" << c.pos.z() << "\n";
    }
    std::cout << std::flush;
}
