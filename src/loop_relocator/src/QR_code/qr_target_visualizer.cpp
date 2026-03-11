/**
 * qr_target_visualizer.cpp
 *
 * 1) Reads QR-code / charging-dock corner coordinates from config.yaml (ENU).
 * 2) Transforms all points into a local "world" frame whose origin is the
 *    centroid of the charging dock, X along 3-3→3-1, Y along 3-2→3-1.
 * 3) Subscribes to /aruco_detection_node/markers (Aruco_detection/ArucoMarkers).
 *    For each detected ArUco ID, looks up which QR target it belongs to,
 *    computes the camera's world-frame pose via:
 *        T_world_camera = T_world_marker * inv(T_camera_marker)
 *    and publishes the camera as an RViz marker + TF.
 *
 * ArUco marker coordinate convention (OpenCV):
 *   Z axis points OUT of the marker face.
 *   Front markers (ID 1, 0): ArUco Z+ aligns with world Y+
 *   Back  markers (ID 3, 2): ArUco Z+ aligns with world Y−
 *
 * Published topics:
 *   /qr_targets/markers        (visualization_msgs/MarkerArray) — static targets
 *   /qr_targets/camera_markers (visualization_msgs/MarkerArray) — camera pose
 *   /qr_targets/camera_pose    (geometry_msgs/PoseStamped)
 *   /tf                        world → camera_estimated
 *
 * Parameters:
 *   ~config_file, ~visualization_frame_id ("world"), ~publish_rate (1.0 Hz)
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <Aruco_detection/ArucoMarkers.h>

#include <string>
#include <vector>
#include <array>
#include <map>
#include <mutex>
#include <cmath>
#include <stdexcept>

// ═══════════════════════════════════════════════════════════════════════════════
//  Data structures
// ═══════════════════════════════════════════════════════════════════════════════

struct Corner { std::string id; double east, north, up; };

struct Target
{
    std::string name;
    std::string description;
    std::vector<int> aruco_ids;   // [front_id, back_id]
    std::vector<Corner> corners;
};

struct RigidTransform
{
    Eigen::Matrix3d R;   // columns = local X, Y, Z expressed in ENU
    Eigen::Vector3d t;   // origin in ENU
};

struct MarkerWorldPose
{
    Eigen::Matrix4d T;   // T_world_marker (4×4 homogeneous)
    bool is_front;
};

// ═══════════════════════════════════════════════════════════════════════════════
//  YAML loader
// ═══════════════════════════════════════════════════════════════════════════════

static std::vector<Target> loadConfig(const std::string& path)
{
    YAML::Node root = YAML::LoadFile(path);
    std::vector<Target> targets;

    for (const auto& t : root["targets"])
    {
        Target tgt;
        tgt.name        = t["name"].as<std::string>();
        tgt.description = t["description"].as<std::string>();

        if (t["aruco_ids"])
            for (const auto& id_node : t["aruco_ids"])
                tgt.aruco_ids.push_back(id_node.as<int>());

        for (const auto& c : t["corners"])
        {
            Corner corner;
            corner.id    = c["id"].as<std::string>();
            corner.east  = c["east"].as<double>();
            corner.north = c["north"].as<double>();
            corner.up    = c["up"].as<double>();
            tgt.corners.push_back(corner);
        }
        targets.push_back(tgt);
    }
    return targets;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Charging-dock-centred frame
// ═══════════════════════════════════════════════════════════════════════════════

static RigidTransform computeDockTransform(const std::vector<Target>& targets)
{
    const Target* dock = nullptr;
    for (const auto& tgt : targets)
        if (tgt.name == "charging_dock") { dock = &tgt; break; }
    if (!dock || dock->corners.size() < 4)
        throw std::runtime_error("charging_dock not found or < 4 corners");

    auto toV = [](const Corner& c){ return Eigen::Vector3d(c.east, c.north, c.up); };
    Eigen::Vector3d p1 = toV(dock->corners[0]);  // 3-1
    Eigen::Vector3d p2 = toV(dock->corners[1]);  // 3-2
    Eigen::Vector3d p3 = toV(dock->corners[2]);  // 3-3

    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    for (const auto& c : dock->corners) origin += toV(c);
    origin /= 4.0;

    // X-axis: along 3-3 → 3-1 direction
    Eigen::Vector3d x_axis = (p1 - p3).normalized();
    // Y-axis: along 3-2 → 3-1 direction, orthogonalised
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

static void transformTargets(std::vector<Target>& targets, const RigidTransform& tf)
{
    for (auto& tgt : targets)
        for (auto& c : tgt.corners)
        {
            Eigen::Vector3d local = tf.R.transpose() *
                (Eigen::Vector3d(c.east, c.north, c.up) - tf.t);
            c.east = local.x();
            c.north = local.y();
            c.up = local.z();
        }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Compute world-frame pose of each ArUco marker from QR target corners
// ═══════════════════════════════════════════════════════════════════════════════
//
//  The markers are wall-mounted and vertical.  Instead of relying on the
//  surveyed corner ordering matching ArUco corner conventions, we derive
//  the ArUco frame from known physical constraints:
//
//  For the front face (camera sees it from the Y+ side):
//    ArUco Z+ = face outward normal ≈ world Y+
//    ArUco Y+ = "up" on the marker  ≈ world Z+
//    ArUco X+ = Y × Z  (right-hand rule)
//
//  For the back face (camera sees it from the Y− side):
//    ArUco Z+ = face outward normal ≈ world Y−
//    ArUco Y+ = "up" on the marker  ≈ world Z+
//    ArUco X+ = Y × Z  (right-hand rule)
//
//  The face normal is computed from the surveyed corners and then aligned
//  to the closest Y direction.  The "up" vector is orthogonalised against
//  the normal so the frame is perfectly orthonormal.
//
//  Front markers (ID 1 for qr1, ID 0 for qr2):
//    ArUco Z+ → world Y+
//
//  Back markers (ID 3 for qr1, ID 2 for qr2):
//    ArUco Z+ → world Y−

static std::map<int, MarkerWorldPose> buildMarkerWorldPoses(
    const std::vector<Target>& targets)
{
    std::map<int, MarkerWorldPose> poses;

    for (const auto& tgt : targets)
    {
        if (tgt.aruco_ids.empty() || tgt.corners.size() < 4) continue;

        auto toV = [](const Corner& c){
            return Eigen::Vector3d(c.east, c.north, c.up);
        };

        Eigen::Vector3d c0 = toV(tgt.corners[0]);
        Eigen::Vector3d c1 = toV(tgt.corners[1]);
        Eigen::Vector3d c2 = toV(tgt.corners[2]);
        Eigen::Vector3d c3 = toV(tgt.corners[3]);
        Eigen::Vector3d centre = (c0 + c1 + c2 + c3) / 4.0;

        // Compute face normal from surveyed corners
        Eigen::Vector3d right_raw = ((c1 - c0) + (c3 - c2)).normalized();
        Eigen::Vector3d up_raw    = ((c0 - c2) + (c1 - c3)).normalized();
        Eigen::Vector3d normal    = right_raw.cross(up_raw).normalized();

        // Ensure normal points towards Y+ (front-face outward)
        if (normal.y() < 0)
            normal = -normal;

        // ── Front face ────────────────────────────────────────────────────
        //  ArUco Z = normal (≈ Y+),  ArUco Y = up (≈ Z+),  ArUco X = Y×Z
        if (tgt.aruco_ids.size() >= 1)
        {
            Eigen::Vector3d aruco_z = normal;
            // "up" on the marker ≈ world Z+, orthogonalise against Z-axis
            Eigen::Vector3d world_up(0, 0, 1);
            Eigen::Vector3d aruco_y = (world_up - world_up.dot(aruco_z) * aruco_z).normalized();
            Eigen::Vector3d aruco_x = aruco_y.cross(aruco_z).normalized();

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,1>(0,0) = aruco_x;
            T.block<3,1>(0,1) = aruco_y;
            T.block<3,1>(0,2) = aruco_z;   // ArUco Z → Y+
            T.block<3,1>(0,3) = centre;

            MarkerWorldPose mwp;
            mwp.T = T;
            mwp.is_front = true;
            poses[tgt.aruco_ids[0]] = mwp;
            ROS_INFO("  Front marker ID=%d  centre=(%.3f, %.3f, %.3f)",
                     tgt.aruco_ids[0], centre.x(), centre.y(), centre.z());
            ROS_INFO("    ArUco X=(%.3f,%.3f,%.3f) Y=(%.3f,%.3f,%.3f) Z=(%.3f,%.3f,%.3f)",
                     aruco_x.x(), aruco_x.y(), aruco_x.z(),
                     aruco_y.x(), aruco_y.y(), aruco_y.z(),
                     aruco_z.x(), aruco_z.y(), aruco_z.z());
        }

        // ── Back face ─────────────────────────────────────────────────────
        //  ArUco Z = -normal (≈ Y−),  ArUco Y = up (≈ Z+),  ArUco X = Y×Z
        if (tgt.aruco_ids.size() >= 2)
        {
            Eigen::Vector3d aruco_z = -normal;
            Eigen::Vector3d world_up(0, 0, 1);
            Eigen::Vector3d aruco_y = (world_up - world_up.dot(aruco_z) * aruco_z).normalized();
            Eigen::Vector3d aruco_x = aruco_y.cross(aruco_z).normalized();

            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,1>(0,0) = aruco_x;
            T.block<3,1>(0,1) = aruco_y;
            T.block<3,1>(0,2) = aruco_z;   // ArUco Z → Y−
            T.block<3,1>(0,3) = centre;

            MarkerWorldPose mwp;
            mwp.T = T;
            mwp.is_front = false;
            poses[tgt.aruco_ids[1]] = mwp;
            ROS_INFO("  Back  marker ID=%d  centre=(%.3f, %.3f, %.3f)",
                     tgt.aruco_ids[1], centre.x(), centre.y(), centre.z());
            ROS_INFO("    ArUco X=(%.3f,%.3f,%.3f) Y=(%.3f,%.3f,%.3f) Z=(%.3f,%.3f,%.3f)",
                     aruco_x.x(), aruco_x.y(), aruco_x.z(),
                     aruco_y.x(), aruco_y.y(), aruco_y.z(),
                     aruco_z.x(), aruco_z.y(), aruco_z.z());
        }
    }
    return poses;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Quaternion helper
// ═══════════════════════════════════════════════════════════════════════════════

static Eigen::Quaterniond rotToQuat(const Eigen::Matrix3d& R)
{
    Eigen::Quaterniond q(R);
    q.normalize();
    return q;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Colours
// ═══════════════════════════════════════════════════════════════════════════════

static const std::array<std::array<float,3>, 3> kColours = {{
    {0.2f, 0.8f, 0.2f},   // green  – qr_code_1
    {0.2f, 0.5f, 1.0f},   // blue   – qr_code_2
    {1.0f, 0.6f, 0.1f}    // orange – charging_dock
}};

// ═══════════════════════════════════════════════════════════════════════════════
//  Build static target markers
// ═══════════════════════════════════════════════════════════════════════════════

static visualization_msgs::MarkerArray buildTargetMarkers(
    const std::vector<Target>& targets,
    const std::string& frame_id,
    const ros::Time& stamp)
{
    visualization_msgs::MarkerArray array;
    int id = 0;

    auto toPoint = [](const Corner& c) {
        geometry_msgs::Point p;
        p.x = c.east; p.y = c.north; p.z = c.up;
        return p;
    };

    // ── Coordinate axes at origin ──────────────────────────────────────────
    {
        double axis_len = 1.0;
        const char* labels[] = {"X", "Y", "Z"};
        double dirs[][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
        float  cols[][3] = {{1,0,0}, {0,1,0}, {0,0,1}};
        for (int i = 0; i < 3; ++i)
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id = frame_id; mk.header.stamp = stamp;
            mk.ns = "axes"; mk.id = id++;
            mk.type = visualization_msgs::Marker::ARROW;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = 0.03; mk.scale.y = 0.06; mk.scale.z = 0.08;
            mk.color.r = cols[i][0]; mk.color.g = cols[i][1];
            mk.color.b = cols[i][2]; mk.color.a = 1.0f;
            mk.pose.orientation.w = 1.0;
            geometry_msgs::Point s, e;
            s.x = s.y = s.z = 0;
            e.x = dirs[i][0]*axis_len;
            e.y = dirs[i][1]*axis_len;
            e.z = dirs[i][2]*axis_len;
            mk.points.push_back(s); mk.points.push_back(e);
            array.markers.push_back(mk);

            visualization_msgs::Marker lbl;
            lbl.header.frame_id = frame_id; lbl.header.stamp = stamp;
            lbl.ns = "axis_labels"; lbl.id = id++;
            lbl.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            lbl.action = visualization_msgs::Marker::ADD;
            lbl.pose.position.x = dirs[i][0]*(axis_len+0.12);
            lbl.pose.position.y = dirs[i][1]*(axis_len+0.12);
            lbl.pose.position.z = dirs[i][2]*(axis_len+0.12);
            lbl.pose.orientation.w = 1.0;
            lbl.scale.z = 0.15;
            lbl.color.r = cols[i][0]; lbl.color.g = cols[i][1];
            lbl.color.b = cols[i][2]; lbl.color.a = 1.0f;
            lbl.text = labels[i];
            array.markers.push_back(lbl);
        }
    }

    // ── Per-target markers ─────────────────────────────────────────────────
    for (size_t ti = 0; ti < targets.size(); ++ti)
    {
        const Target& tgt = targets[ti];
        const auto& col   = kColours[ti % kColours.size()];
        const size_t n    = tgt.corners.size();

        // 1. LINE_STRIP polygon
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id = frame_id; mk.header.stamp = stamp;
            mk.ns = tgt.name; mk.id = id++;
            mk.type = visualization_msgs::Marker::LINE_STRIP;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = 0.02;
            mk.color.r = col[0]; mk.color.g = col[1];
            mk.color.b = col[2]; mk.color.a = 1.0f;
            mk.pose.orientation.w = 1.0;
            std::vector<int> order;
            if (n == 4) order = {0, 1, 3, 2, 0};
            else { for (size_t i = 0; i <= n; ++i) order.push_back(i % n); }
            for (int idx : order)
                mk.points.push_back(toPoint(tgt.corners[idx]));
            array.markers.push_back(mk);
        }

        // 2. SPHERE_LIST: corners
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id = frame_id; mk.header.stamp = stamp;
            mk.ns = tgt.name + "_pts"; mk.id = id++;
            mk.type = visualization_msgs::Marker::SPHERE_LIST;
            mk.action = visualization_msgs::Marker::ADD;
            mk.scale.x = mk.scale.y = mk.scale.z = 0.06;
            mk.color.r = col[0]; mk.color.g = col[1];
            mk.color.b = col[2]; mk.color.a = 1.0f;
            mk.pose.orientation.w = 1.0;
            for (const auto& c : tgt.corners) mk.points.push_back(toPoint(c));
            array.markers.push_back(mk);
        }

        // 3. TEXT labels at corners
        for (const auto& c : tgt.corners)
        {
            visualization_msgs::Marker mk;
            mk.header.frame_id = frame_id; mk.header.stamp = stamp;
            mk.ns = tgt.name + "_labels"; mk.id = id++;
            mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position.x = c.east;
            mk.pose.position.y = c.north;
            mk.pose.position.z = c.up + 0.10;
            mk.pose.orientation.w = 1.0;
            mk.scale.z = 0.10;
            mk.color.r = mk.color.g = mk.color.b = mk.color.a = 1.0f;
            char buf[128];
            snprintf(buf, sizeof(buf), "%s\n(%.3f, %.3f, %.3f)",
                     c.id.c_str(), c.east, c.north, c.up);
            mk.text = buf;
            array.markers.push_back(mk);
        }

        // 4. Title at centroid
        {
            double cx = 0, cy = 0, cz = 0;
            for (const auto& c : tgt.corners)
            { cx += c.east; cy += c.north; cz += c.up; }
            cx /= n; cy /= n; cz /= n;
            visualization_msgs::Marker mk;
            mk.header.frame_id = frame_id; mk.header.stamp = stamp;
            mk.ns = tgt.name + "_title"; mk.id = id++;
            mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            mk.action = visualization_msgs::Marker::ADD;
            mk.pose.position.x = cx;
            mk.pose.position.y = cy;
            mk.pose.position.z = cz + 0.25;
            mk.pose.orientation.w = 1.0;
            mk.scale.z = 0.18;
            mk.color.r = col[0]; mk.color.g = col[1];
            mk.color.b = col[2]; mk.color.a = 1.0f;
            mk.text = tgt.name;
            array.markers.push_back(mk);
        }
    }

    return array;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Build camera visualization markers
// ═══════════════════════════════════════════════════════════════════════════════

static visualization_msgs::MarkerArray buildCameraMarkers(
    const Eigen::Matrix4d& T_world_cam,
    const std::string& frame_id,
    const ros::Time& stamp,
    int detected_id)
{
    visualization_msgs::MarkerArray array;
    Eigen::Vector3d pos = T_world_cam.block<3,1>(0,3);
    Eigen::Matrix3d R   = T_world_cam.block<3,3>(0,0);
    Eigen::Quaterniond q = rotToQuat(R);

    // Camera axes (shorter than world axes)
    double cam_axis_len = 0.5;
    float axis_cols[][3] = {{1,0.3f,0.3f}, {0.3f,1,0.3f}, {0.3f,0.3f,1}};
    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d dir = R.col(i) * cam_axis_len;
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "camera_axes"; mk.id = i;
        mk.type = visualization_msgs::Marker::ARROW;
        mk.action = visualization_msgs::Marker::ADD;
        mk.scale.x = 0.02; mk.scale.y = 0.04; mk.scale.z = 0.06;
        mk.color.r = axis_cols[i][0]; mk.color.g = axis_cols[i][1];
        mk.color.b = axis_cols[i][2]; mk.color.a = 1.0f;
        mk.pose.orientation.w = 1.0;
        geometry_msgs::Point s, e;
        s.x = pos.x(); s.y = pos.y(); s.z = pos.z();
        e.x = pos.x() + dir.x();
        e.y = pos.y() + dir.y();
        e.z = pos.z() + dir.z();
        mk.points.push_back(s); mk.points.push_back(e);
        array.markers.push_back(mk);
    }

    // Camera body cube
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "camera_body"; mk.id = 0;
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.scale.x = 0.08; mk.scale.y = 0.15; mk.scale.z = 0.06;
        mk.color.r = 0.9f; mk.color.g = 0.9f;
        mk.color.b = 0.9f; mk.color.a = 0.8f;
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z();
        mk.pose.orientation.x = q.x();
        mk.pose.orientation.y = q.y();
        mk.pose.orientation.z = q.z();
        mk.pose.orientation.w = q.w();
        array.markers.push_back(mk);
    }

    // Camera label
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "camera_label"; mk.id = 0;
        mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mk.action = visualization_msgs::Marker::ADD;
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z() + 0.25;
        mk.pose.orientation.w = 1.0;
        mk.scale.z = 0.12;
        mk.color.r = 1.0f; mk.color.g = 1.0f;
        mk.color.b = 0.0f; mk.color.a = 1.0f;
        char buf[256];
        snprintf(buf, sizeof(buf), "Camera (ArUco ID %d)\n(%.3f, %.3f, %.3f)",
                 detected_id, pos.x(), pos.y(), pos.z());
        mk.text = buf;
        array.markers.push_back(mk);
    }

    return array;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Build detected ArUco marker frame visualization
// ═══════════════════════════════════════════════════════════════════════════════

static visualization_msgs::MarkerArray buildDetectedMarkerAxes(
    const Eigen::Matrix4d& T_world_marker,
    const std::string& frame_id,
    const ros::Time& stamp,
    int aruco_id,
    bool is_front)
{
    visualization_msgs::MarkerArray array;
    Eigen::Vector3d pos = T_world_marker.block<3,1>(0,3);
    Eigen::Matrix3d R   = T_world_marker.block<3,3>(0,0);

    double axis_len = 0.3;
    const char* axis_names[] = {"ArX", "ArY", "ArZ"};
    float cols[][3] = {{1.0f, 0.2f, 0.2f}, {0.2f, 1.0f, 0.2f}, {0.2f, 0.2f, 1.0f}};
    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d dir = R.col(i) * axis_len;
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "aruco_marker_axes"; mk.id = aruco_id * 10 + i;
        mk.type = visualization_msgs::Marker::ARROW;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);  // auto-expire
        mk.scale.x = 0.015; mk.scale.y = 0.03; mk.scale.z = 0.04;
        mk.color.r = cols[i][0]; mk.color.g = cols[i][1];
        mk.color.b = cols[i][2]; mk.color.a = 0.9f;
        mk.pose.orientation.w = 1.0;
        geometry_msgs::Point s, e;
        s.x = pos.x(); s.y = pos.y(); s.z = pos.z();
        e.x = pos.x() + dir.x();
        e.y = pos.y() + dir.y();
        e.z = pos.z() + dir.z();
        mk.points.push_back(s); mk.points.push_back(e);
        array.markers.push_back(mk);
    }

    // Label at marker position
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "aruco_marker_label"; mk.id = aruco_id;
        mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z() + 0.20;
        mk.pose.orientation.w = 1.0;
        mk.scale.z = 0.10;
        mk.color.r = 1.0f; mk.color.g = 0.5f;
        mk.color.b = 0.0f; mk.color.a = 1.0f;
        char buf[128];
        snprintf(buf, sizeof(buf), "ArUco %d (%s)", aruco_id,
                 is_front ? "front" : "back");
        mk.text = buf;
        array.markers.push_back(mk);
    }

    return array;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Build "observed marker in world" visualization
//  T_world_marker_obs = T_world_camera * T_camera_marker
//  Compare this with the pre-computed T_world_marker to verify correctness.
// ═══════════════════════════════════════════════════════════════════════════════

static visualization_msgs::MarkerArray buildObservedMarkerVis(
    const Eigen::Matrix4d& T_world_marker_obs,
    const Eigen::Matrix4d& T_world_marker_pre,
    const std::string& frame_id,
    const ros::Time& stamp,
    int aruco_id)
{
    visualization_msgs::MarkerArray array;
    Eigen::Vector3d obs_pos = T_world_marker_obs.block<3,1>(0,3);
    Eigen::Vector3d pre_pos = T_world_marker_pre.block<3,1>(0,3);
    Eigen::Matrix3d obs_R   = T_world_marker_obs.block<3,3>(0,0);

    // Observed marker position — magenta sphere
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "observed_marker_pos"; mk.id = aruco_id;
        mk.type = visualization_msgs::Marker::SPHERE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.scale.x = mk.scale.y = mk.scale.z = 0.10;
        mk.color.r = 1.0f; mk.color.g = 0.0f;
        mk.color.b = 1.0f; mk.color.a = 0.9f;
        mk.pose.position.x = obs_pos.x();
        mk.pose.position.y = obs_pos.y();
        mk.pose.position.z = obs_pos.z();
        mk.pose.orientation.w = 1.0;
        array.markers.push_back(mk);
    }

    // Observed marker axes (shorter, semi-transparent)
    double axis_len = 0.25;
    float cols[][3] = {{1.0f,0.0f,0.0f}, {0.0f,1.0f,0.0f}, {0.0f,0.0f,1.0f}};
    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d dir = obs_R.col(i) * axis_len;
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "observed_marker_axes"; mk.id = aruco_id * 10 + i;
        mk.type = visualization_msgs::Marker::ARROW;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.scale.x = 0.012; mk.scale.y = 0.025; mk.scale.z = 0.035;
        mk.color.r = cols[i][0]; mk.color.g = cols[i][1];
        mk.color.b = cols[i][2]; mk.color.a = 0.7f;
        mk.pose.orientation.w = 1.0;
        geometry_msgs::Point s, e;
        s.x = obs_pos.x(); s.y = obs_pos.y(); s.z = obs_pos.z();
        e.x = obs_pos.x()+dir.x(); e.y = obs_pos.y()+dir.y(); e.z = obs_pos.z()+dir.z();
        mk.points.push_back(s); mk.points.push_back(e);
        array.markers.push_back(mk);
    }

    // Line connecting observed and pre-computed positions
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "observed_marker_error"; mk.id = aruco_id;
        mk.type = visualization_msgs::Marker::LINE_STRIP;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.scale.x = 0.01;
        mk.color.r = 1.0f; mk.color.g = 1.0f;
        mk.color.b = 0.0f; mk.color.a = 0.8f;
        mk.pose.orientation.w = 1.0;
        geometry_msgs::Point pa, pb;
        pa.x = obs_pos.x(); pa.y = obs_pos.y(); pa.z = obs_pos.z();
        pb.x = pre_pos.x(); pb.y = pre_pos.y(); pb.z = pre_pos.z();
        mk.points.push_back(pa); mk.points.push_back(pb);
        array.markers.push_back(mk);
    }

    // Label with position and error
    {
        double err = (obs_pos - pre_pos).norm();
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "observed_marker_label"; mk.id = aruco_id;
        mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.pose.position.x = obs_pos.x();
        mk.pose.position.y = obs_pos.y();
        mk.pose.position.z = obs_pos.z() + 0.18;
        mk.pose.orientation.w = 1.0;
        mk.scale.z = 0.08;
        mk.color.r = 1.0f; mk.color.g = 0.0f;
        mk.color.b = 1.0f; mk.color.a = 1.0f;
        char buf[256];
        snprintf(buf, sizeof(buf),
            "Observed ID%d\n(%.3f,%.3f,%.3f)\nerr=%.3fm",
            aruco_id, obs_pos.x(), obs_pos.y(), obs_pos.z(), err);
        mk.text = buf;
        array.markers.push_back(mk);
    }

    return array;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Build robot-dog markers  (body box + axes + forward arrow + label)
// ═══════════════════════════════════════════════════════════════════════════════

static visualization_msgs::MarkerArray buildRobotMarkers(
    const Eigen::Matrix4d& T_world_robot,
    const std::string& frame_id,
    const ros::Time& stamp,
    int detected_id)
{
    visualization_msgs::MarkerArray array;
    Eigen::Vector3d pos = T_world_robot.block<3,1>(0,3);
    Eigen::Matrix3d R   = T_world_robot.block<3,3>(0,0);
    Eigen::Quaterniond q = rotToQuat(R);

    // Robot body axes (X=forward, Y=left, Z=up)
    double axis_len = 0.6;
    float axis_cols[][3] = {{1.0f, 0.2f, 0.2f},    // X red
                            {0.2f, 1.0f, 0.2f},    // Y green
                            {0.2f, 0.2f, 1.0f}};   // Z blue
    for (int i = 0; i < 3; ++i)
    {
        Eigen::Vector3d dir = R.col(i) * axis_len;
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "robot_axes"; mk.id = i;
        mk.type = visualization_msgs::Marker::ARROW;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.scale.x = 0.025; mk.scale.y = 0.05; mk.scale.z = 0.07;
        mk.color.r = axis_cols[i][0]; mk.color.g = axis_cols[i][1];
        mk.color.b = axis_cols[i][2]; mk.color.a = 1.0f;
        mk.pose.orientation.w = 1.0;
        geometry_msgs::Point s, e;
        s.x = pos.x(); s.y = pos.y(); s.z = pos.z();
        e.x = pos.x() + dir.x();
        e.y = pos.y() + dir.y();
        e.z = pos.z() + dir.z();
        mk.points.push_back(s); mk.points.push_back(e);
        array.markers.push_back(mk);
    }

    // Robot body box (0.7m long, 0.4m wide, 0.3m tall — approximate dog body)
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "robot_body"; mk.id = 0;
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.scale.x = 0.7;  // length along robot X (forward)
        mk.scale.y = 0.4;  // width along robot Y (left)
        mk.scale.z = 0.3;  // height along robot Z (up)
        mk.color.r = 0.0f; mk.color.g = 0.8f;
        mk.color.b = 0.8f; mk.color.a = 0.5f;
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z();
        mk.pose.orientation.x = q.x();
        mk.pose.orientation.y = q.y();
        mk.pose.orientation.z = q.z();
        mk.pose.orientation.w = q.w();
        array.markers.push_back(mk);
    }

    // Forward arrow (thick, along robot X)
    {
        Eigen::Vector3d fwd = R.col(0) * 0.8;
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "robot_forward"; mk.id = 0;
        mk.type = visualization_msgs::Marker::ARROW;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.scale.x = 0.04; mk.scale.y = 0.08; mk.scale.z = 0.10;
        mk.color.r = 0.0f; mk.color.g = 1.0f;
        mk.color.b = 0.0f; mk.color.a = 0.9f;
        mk.pose.orientation.w = 1.0;
        geometry_msgs::Point s, e;
        s.x = pos.x(); s.y = pos.y(); s.z = pos.z();
        e.x = pos.x() + fwd.x();
        e.y = pos.y() + fwd.y();
        e.z = pos.z() + fwd.z();
        mk.points.push_back(s); mk.points.push_back(e);
        array.markers.push_back(mk);
    }

    // Robot label
    {
        visualization_msgs::Marker mk;
        mk.header.frame_id = frame_id; mk.header.stamp = stamp;
        mk.ns = "robot_label"; mk.id = 0;
        mk.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        mk.action = visualization_msgs::Marker::ADD;
        mk.lifetime = ros::Duration(0);
        mk.pose.position.x = pos.x();
        mk.pose.position.y = pos.y();
        mk.pose.position.z = pos.z() + 0.35;
        mk.pose.orientation.w = 1.0;
        mk.scale.z = 0.14;
        mk.color.r = 0.0f; mk.color.g = 1.0f;
        mk.color.b = 1.0f; mk.color.a = 1.0f;
        char buf[256];
        snprintf(buf, sizeof(buf), "Robot Dog (via ID %d)\n(%.3f, %.3f, %.3f)",
                 detected_id, pos.x(), pos.y(), pos.z());
        mk.text = buf;
        array.markers.push_back(mk);
    }

    return array;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  Global state for ArUco callback
// ═══════════════════════════════════════════════════════════════════════════════

static std::map<int, MarkerWorldPose>  g_marker_world_poses;
static std::string                     g_frame_id;
static ros::Publisher*                 g_camera_marker_pub_ptr  = nullptr;
static ros::Publisher*                 g_camera_pose_pub_ptr    = nullptr;
static ros::Publisher*                 g_aruco_marker_pub_ptr   = nullptr;
static ros::Publisher*                 g_obs_marker_pub_ptr     = nullptr;
static ros::Publisher*                 g_robot_marker_pub_ptr   = nullptr;
static ros::Publisher*                 g_robot_pose_pub_ptr     = nullptr;
static tf2_ros::TransformBroadcaster*  g_tf_broadcaster_ptr     = nullptr;
static std::mutex                      g_mutex;

// Height-difference filter
static double g_expected_height_diff = 0.80;  // camera Z − marker Z
static double g_height_diff_tol      = 0.10;  // tolerance

// Camera-to-robot extrinsic: T_robot_camera (camera pose in robot body frame)
//   p_robot = R_rc * p_camera + t_rc
static Eigen::Matrix4d g_T_robot_camera = Eigen::Matrix4d::Identity();

// ═══════════════════════════════════════════════════════════════════════════════
//  ArUco callback
// ═══════════════════════════════════════════════════════════════════════════════
//
//  ArUco detection gives T_camera_marker: pose of the marker in camera frame.
//     p_camera = R_cm * p_marker + t_cm
//
//  We want T_world_camera:
//     T_world_camera = T_world_marker * inv(T_camera_marker)
//
//  inv(T_camera_marker):  R' = R_cm^T,  t' = -R_cm^T * t_cm

static void arucoCallback(const Aruco_detection::ArucoMarkers::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(g_mutex);

    for (const auto& marker : msg->markers)
    {
        int aruco_id = marker.id;
        auto it = g_marker_world_poses.find(aruco_id);
        if (it == g_marker_world_poses.end())
        {
            ROS_WARN_THROTTLE(5.0,
                "Detected ArUco ID=%d not in config, ignoring.", aruco_id);
            continue;
        }

        const Eigen::Matrix4d& T_world_marker = it->second.T;

        // Build T_camera_marker from the message
        const auto& p = marker.pose.position;
        const auto& o = marker.pose.orientation;
        Eigen::Quaterniond q_cm(o.w, o.x, o.y, o.z);
        Eigen::Matrix3d R_cm = q_cm.toRotationMatrix();
        Eigen::Vector3d t_cm(p.x, p.y, p.z);

        ROS_INFO_THROTTLE(2.0,
            "ArUco ID=%d  T_cam_marker: t=(%.3f,%.3f,%.3f) q=(x=%.4f,y=%.4f,z=%.4f,w=%.4f)",
            aruco_id, t_cm.x(), t_cm.y(), t_cm.z(),
            o.x, o.y, o.z, o.w);

        // Verify T_world_marker
        {
            Eigen::Vector3d wm_pos = T_world_marker.block<3,1>(0,3);
            Eigen::Matrix3d wm_R   = T_world_marker.block<3,3>(0,0);
            ROS_INFO_THROTTLE(2.0,
                "  T_world_marker: pos=(%.3f,%.3f,%.3f)",
                wm_pos.x(), wm_pos.y(), wm_pos.z());
            ROS_INFO_THROTTLE(2.0,
                "    X-col=(%.3f,%.3f,%.3f) Y-col=(%.3f,%.3f,%.3f) Z-col=(%.3f,%.3f,%.3f)",
                wm_R(0,0), wm_R(1,0), wm_R(2,0),
                wm_R(0,1), wm_R(1,1), wm_R(2,1),
                wm_R(0,2), wm_R(1,2), wm_R(2,2));
        }

        // inv(T_camera_marker) = T_marker_camera
        Eigen::Matrix4d T_marker_cam = Eigen::Matrix4d::Identity();
        T_marker_cam.block<3,3>(0,0) = R_cm.transpose();
        T_marker_cam.block<3,1>(0,3) = -R_cm.transpose() * t_cm;

        // Camera position in marker frame (for debug)
        {
            Eigen::Vector3d cam_in_marker = -R_cm.transpose() * t_cm;
            ROS_INFO_THROTTLE(2.0,
                "  Camera in marker frame: (%.3f,%.3f,%.3f)",
                cam_in_marker.x(), cam_in_marker.y(), cam_in_marker.z());
        }

        // T_world_camera = T_world_marker * T_marker_camera
        Eigen::Matrix4d T_world_cam = T_world_marker * T_marker_cam;
        Eigen::Vector3d cam_pos = T_world_cam.block<3,1>(0,3);
        Eigen::Quaterniond cam_q = rotToQuat(T_world_cam.block<3,3>(0,0));

        // ── Height-difference filter ──────────────────────────────────────
        {
            double marker_z = T_world_marker.block<3,1>(0,3).z();
            double actual_diff = cam_pos.z() - marker_z;
            double diff_error = std::abs(actual_diff - g_expected_height_diff);
            if (diff_error > g_height_diff_tol)
            {
                ROS_WARN_THROTTLE(1.0,
                    "ArUco ID=%d REJECTED: height diff=%.3fm (expected %.3f±%.3f, got %.3f)",
                    aruco_id, actual_diff, g_expected_height_diff,
                    g_height_diff_tol, actual_diff);
                continue;
            }
        }

        ROS_INFO_THROTTLE(1.0,
            "Camera via ArUco ID=%d (%s): pos=(%.3f, %.3f, %.3f)",
            aruco_id, it->second.is_front ? "front" : "back",
            cam_pos.x(), cam_pos.y(), cam_pos.z());

        // Publish camera markers
        if (g_camera_marker_pub_ptr)
        {
            auto cam_markers = buildCameraMarkers(
                T_world_cam, g_frame_id, msg->header.stamp, aruco_id);
            g_camera_marker_pub_ptr->publish(cam_markers);
        }

        // Publish detected ArUco marker axes
        if (g_aruco_marker_pub_ptr)
        {
            auto aruco_axes = buildDetectedMarkerAxes(
                T_world_marker, g_frame_id, msg->header.stamp,
                aruco_id, it->second.is_front);
            g_aruco_marker_pub_ptr->publish(aruco_axes);
        }

        // Compute observed marker world pose:
        //   T_world_marker_obs = T_world_camera * T_camera_marker
        // This should match T_world_marker if everything is correct.
        {
            Eigen::Matrix4d T_cam_marker = Eigen::Matrix4d::Identity();
            T_cam_marker.block<3,3>(0,0) = R_cm;
            T_cam_marker.block<3,1>(0,3) = t_cm;

            Eigen::Matrix4d T_world_marker_obs = T_world_cam * T_cam_marker;
            Eigen::Vector3d obs_pos = T_world_marker_obs.block<3,1>(0,3);
            Eigen::Vector3d pre_pos = T_world_marker.block<3,1>(0,3);
            double err = (obs_pos - pre_pos).norm();

            ROS_INFO_THROTTLE(2.0,
                "  Observed marker world pos: (%.3f,%.3f,%.3f)  "
                "pre-computed: (%.3f,%.3f,%.3f)  err=%.3fm",
                obs_pos.x(), obs_pos.y(), obs_pos.z(),
                pre_pos.x(), pre_pos.y(), pre_pos.z(), err);

            if (g_obs_marker_pub_ptr)
            {
                auto obs_vis = buildObservedMarkerVis(
                    T_world_marker_obs, T_world_marker,
                    g_frame_id, msg->header.stamp, aruco_id);
                g_obs_marker_pub_ptr->publish(obs_vis);
            }
        }

        // Publish PoseStamped
        if (g_camera_pose_pub_ptr)
        {
            geometry_msgs::PoseStamped ps;
            ps.header.stamp    = msg->header.stamp;
            ps.header.frame_id = g_frame_id;
            ps.pose.position.x = cam_pos.x();
            ps.pose.position.y = cam_pos.y();
            ps.pose.position.z = cam_pos.z();
            ps.pose.orientation.x = cam_q.x();
            ps.pose.orientation.y = cam_q.y();
            ps.pose.orientation.z = cam_q.z();
            ps.pose.orientation.w = cam_q.w();
            g_camera_pose_pub_ptr->publish(ps);
        }

        // Publish TF: world → camera_estimated
        if (g_tf_broadcaster_ptr)
        {
            geometry_msgs::TransformStamped tf_msg;
            tf_msg.header.stamp    = msg->header.stamp;
            tf_msg.header.frame_id = g_frame_id;
            tf_msg.child_frame_id  = "camera_estimated";
            tf_msg.transform.translation.x = cam_pos.x();
            tf_msg.transform.translation.y = cam_pos.y();
            tf_msg.transform.translation.z = cam_pos.z();
            tf_msg.transform.rotation.x = cam_q.x();
            tf_msg.transform.rotation.y = cam_q.y();
            tf_msg.transform.rotation.z = cam_q.z();
            tf_msg.transform.rotation.w = cam_q.w();
            g_tf_broadcaster_ptr->sendTransform(tf_msg);
        }

        // ── Compute robot dog world pose ──────────────────────────────────
        //  T_robot_camera describes camera pose in robot frame.
        //  T_world_camera = T_world_robot * T_robot_camera
        //  => T_world_robot = T_world_camera * inv(T_robot_camera)
        {
            Eigen::Matrix3d R_rc = g_T_robot_camera.block<3,3>(0,0);
            Eigen::Vector3d t_rc = g_T_robot_camera.block<3,1>(0,3);
            Eigen::Matrix4d T_camera_robot = Eigen::Matrix4d::Identity();
            T_camera_robot.block<3,3>(0,0) = R_rc.transpose();
            T_camera_robot.block<3,1>(0,3) = -R_rc.transpose() * t_rc;

            Eigen::Matrix4d T_world_robot = T_world_cam * T_camera_robot;
            Eigen::Vector3d robot_pos = T_world_robot.block<3,1>(0,3);
            Eigen::Quaterniond robot_q = rotToQuat(T_world_robot.block<3,3>(0,0));

            ROS_INFO_THROTTLE(1.0,
                "Robot Dog via ArUco ID=%d: pos=(%.3f, %.3f, %.3f)",
                aruco_id, robot_pos.x(), robot_pos.y(), robot_pos.z());

            // Publish robot markers
            if (g_robot_marker_pub_ptr)
            {
                auto robot_markers = buildRobotMarkers(
                    T_world_robot, g_frame_id, msg->header.stamp, aruco_id);
                g_robot_marker_pub_ptr->publish(robot_markers);
            }

            // Publish robot PoseStamped
            if (g_robot_pose_pub_ptr)
            {
                geometry_msgs::PoseStamped ps;
                ps.header.stamp    = msg->header.stamp;
                ps.header.frame_id = g_frame_id;
                ps.pose.position.x = robot_pos.x();
                ps.pose.position.y = robot_pos.y();
                ps.pose.position.z = robot_pos.z();
                ps.pose.orientation.x = robot_q.x();
                ps.pose.orientation.y = robot_q.y();
                ps.pose.orientation.z = robot_q.z();
                ps.pose.orientation.w = robot_q.w();
                g_robot_pose_pub_ptr->publish(ps);
            }

            // Publish TF: world → robot_estimated
            if (g_tf_broadcaster_ptr)
            {
                geometry_msgs::TransformStamped tf_msg;
                tf_msg.header.stamp    = msg->header.stamp;
                tf_msg.header.frame_id = g_frame_id;
                tf_msg.child_frame_id  = "robot_estimated";
                tf_msg.transform.translation.x = robot_pos.x();
                tf_msg.transform.translation.y = robot_pos.y();
                tf_msg.transform.translation.z = robot_pos.z();
                tf_msg.transform.rotation.x = robot_q.x();
                tf_msg.transform.rotation.y = robot_q.y();
                tf_msg.transform.rotation.z = robot_q.z();
                tf_msg.transform.rotation.w = robot_q.w();
                g_tf_broadcaster_ptr->sendTransform(tf_msg);
            }
        }

        break;  // use first valid detection
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
//  main
// ═══════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv)
{
    ros::init(argc, argv, "qr_target_visualizer");
    ros::NodeHandle nh("~");

    // ── Parameters ────────────────────────────────────────────────────────
    std::string pkg_path    = ros::package::getPath("loop_relocator");
    std::string cfg_default = pkg_path + "/config/qr_config.yaml";

    std::string config_file;
    nh.param<std::string>("config_file", config_file, cfg_default);
    nh.param<std::string>("visualization_frame_id", g_frame_id, "world");

    double publish_rate;
    nh.param<double>("publish_rate", publish_rate, 1.0);

    // ── Load config ───────────────────────────────────────────────────────
    std::vector<Target> targets;
    try
    {
        targets = loadConfig(config_file);
        ROS_INFO("Loaded %zu target(s) from %s", targets.size(), config_file.c_str());
        for (const auto& t : targets)
        {
            std::string ids_str;
            for (int aid : t.aruco_ids) ids_str += std::to_string(aid) + " ";
            ROS_INFO("  -> %s (%zu corners) aruco_ids=[%s]",
                     t.name.c_str(), t.corners.size(), ids_str.c_str());
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to load config: %s", e.what());
        return 1;
    }

    // ── Load filter parameters from config ────────────────────────────────
    try
    {
        YAML::Node root = YAML::LoadFile(config_file);
        if (root["camera_marker_height_diff"])
            g_expected_height_diff = root["camera_marker_height_diff"].as<double>();
        if (root["height_diff_tolerance"])
            g_height_diff_tol = root["height_diff_tolerance"].as<double>();
        ROS_INFO("Height filter: expected_diff=%.3f m, tolerance=%.3f m",
                 g_expected_height_diff, g_height_diff_tol);
    }
    catch (const std::exception& e)
    {
        ROS_WARN("Could not load filter params, using defaults: %s", e.what());
    }

    // ── Load camera-to-robot extrinsics from config ───────────────────────
    try
    {
        YAML::Node root = YAML::LoadFile(config_file);
        if (root["camera_to_robot"])
        {
            auto node = root["camera_to_robot"];
            auto t_node = node["translation"];
            auto r_node = node["rotation"];

            Eigen::Vector3d t_rc(
                t_node[0].as<double>(),
                t_node[1].as<double>(),
                t_node[2].as<double>());

            Eigen::Matrix3d R_rc;
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    R_rc(i, j) = r_node[i][j].as<double>();

            g_T_robot_camera = Eigen::Matrix4d::Identity();
            g_T_robot_camera.block<3,3>(0,0) = R_rc;
            g_T_robot_camera.block<3,1>(0,3) = t_rc;

            ROS_INFO("Camera-to-robot extrinsic loaded:");
            ROS_INFO("  Translation: [%.3f, %.3f, %.3f]",
                     t_rc.x(), t_rc.y(), t_rc.z());
            ROS_INFO("  Rotation:");
            ROS_INFO("    [%.3f, %.3f, %.3f]", R_rc(0,0), R_rc(0,1), R_rc(0,2));
            ROS_INFO("    [%.3f, %.3f, %.3f]", R_rc(1,0), R_rc(1,1), R_rc(1,2));
            ROS_INFO("    [%.3f, %.3f, %.3f]", R_rc(2,0), R_rc(2,1), R_rc(2,2));
        }
        else
        {
            ROS_WARN("No camera_to_robot extrinsic in config, robot = camera.");
        }
    }
    catch (const std::exception& e)
    {
        ROS_WARN("Could not load camera_to_robot extrinsic: %s", e.what());
    }

    // ── Dock-centred transform ────────────────────────────────────────────
    RigidTransform dock_tf;
    try
    {
        dock_tf = computeDockTransform(targets);
        ROS_INFO("Dock-centred frame computed.");
        ROS_INFO("  Origin (ENU): [%.4f, %.4f, %.4f]",
                 dock_tf.t.x(), dock_tf.t.y(), dock_tf.t.z());
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Dock transform failed: %s", e.what());
        return 1;
    }

    transformTargets(targets, dock_tf);

    ROS_INFO("Transformed corners (world frame):");
    for (const auto& t : targets)
        for (const auto& c : t.corners)
            ROS_INFO("  %s: (%.4f, %.4f, %.4f)", c.id.c_str(),
                     c.east, c.north, c.up);

    // ── Build ArUco marker world poses ────────────────────────────────────
    ROS_INFO("Building ArUco marker world poses...");
    g_marker_world_poses = buildMarkerWorldPoses(targets);
    ROS_INFO("Configured %zu ArUco ID -> world pose mappings.",
             g_marker_world_poses.size());

    // ── Publishers ────────────────────────────────────────────────────────
    ros::Publisher target_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/qr_targets/markers", 1, true);
    ros::Publisher camera_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/qr_targets/camera_markers", 1, true);
    ros::Publisher aruco_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/qr_targets/aruco_marker_axes", 1, true);
    ros::Publisher obs_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/qr_targets/observed_marker", 1, true);
    ros::Publisher robot_marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "/qr_targets/robot_markers", 1, true);
    ros::Publisher camera_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/qr_targets/camera_pose", 10);
    ros::Publisher robot_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/qr_targets/robot_pose", 10);

    g_camera_marker_pub_ptr  = &camera_marker_pub;
    g_camera_pose_pub_ptr    = &camera_pose_pub;
    g_aruco_marker_pub_ptr   = &aruco_marker_pub;
    g_obs_marker_pub_ptr     = &obs_marker_pub;
    g_robot_marker_pub_ptr   = &robot_marker_pub;
    g_robot_pose_pub_ptr     = &robot_pose_pub;

    tf2_ros::TransformBroadcaster tf_broadcaster;
    g_tf_broadcaster_ptr = &tf_broadcaster;

    // ── Subscriber ────────────────────────────────────────────────────────
    ros::NodeHandle nh_global;
    ros::Subscriber aruco_sub = nh_global.subscribe(
        "/aruco_detection_node/markers", 10, arucoCallback);
    ROS_INFO("Subscribed to /aruco_detection_node/markers");

    // ── Main loop ─────────────────────────────────────────────────────────
    ros::Rate rate(publish_rate);
    ROS_INFO("Publishing at %.1f Hz, frame='%s'", publish_rate, g_frame_id.c_str());

    while (ros::ok())
    {
        auto msg = buildTargetMarkers(targets, g_frame_id, ros::Time::now());
        target_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
