#include "place_recognition.hpp"
#include "RTK/rtk_relocator.hpp"
#include "QR_code/qr_relocator.hpp"
#include <pcl/point_types.h>  // 确保包含必要的头文件
#include <pcl/point_cloud.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/common.h>
PlaceRecognition::PlaceRecognition(ros::NodeHandle &nh) : ikd_match_mtx()
{
  // Constructor implementation
  
  // nh.param<double>("cloud_overlap_thr", cloud_overlap_thr, 0.5);
  // nh.param<std::string>("setting_path", setting_path, "");
  // nh.param<std::string>("pcds_dir", pcds_dir, "");
  // nh.param<std::string>("pose_file", pose_file, "");
  // nh.param<std::string>("curr_pcds_dir", curr_pcds_dir, "");
  // nh.param<std::string>("curr_pose_file", curr_pose_file, "");
  // nh.param<std::string>("btc_save_dir", btc_save_path, "");
  // nh.param<std::string>("seq_key", seq_key, "");
  // nh.param<bool>("read_bin", read_bin, true);
  nh.param<std::string>("full_pcd_dir", full_pcd_dir, "");
  
  // 读取可视化 frame_id 参数
  nh.param<std::string>("visualization_frame_id", visualization_frame_id, "base");

  double x, y, z;
  nh.param<double>("lidar_imu_trans/x", x, 0.0);
  nh.param<double>("lidar_imu_trans/y", y, 0.0);
  nh.param<double>("lidar_imu_trans/z", z, 0.0);
  lidar_imu_trans << x, y, z;
  
  std::cout << "lidar_imu_trans: " << lidar_imu_trans.transpose() << std::endl;



  pubCureentCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_cureent", 100);
  pubCurrentCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_current", 100);
  // pubCurrentBinary =
  //       nh.advertise<sensor_msgs::PointCloud2>("/cloud_key_points", 100);
  // pubPath =
  //       nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
  pubCurrentPose =
        nh.advertise<nav_msgs::Odometry>("/current_pose", 10);
  pubMatchedPose =
        nh.advertise<nav_msgs::Odometry>("/matched_pose", 10);
  pubWholeCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_whole", 1, true);
  pubTransformedCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_transformed", 100);
  pubRawTransformedCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_raw_transformed", 100);
  pubMatchedCloud =
        nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched", 100);
  // pubMatchedBinary =
  //       nh.advertise<sensor_msgs::PointCloud2>("/cloud_matched_key_points", 100);
  // pubLoopStatus =
  //       nh.advertise<visualization_msgs::MarkerArray>("/loop_status", 100);
  // pubBTC =
  //     nh.advertise<visualization_msgs::MarkerArray>("descriptor_line", 10);
  
  //state_Timer = nh.createTimer(ros::Duration(1.0), &PlaceRecognition::loadstateCallback, this);
  initialpose_sub_ = nh.subscribe("/initialpose", 10, &PlaceRecognition::initialPoseCallback, this);
  //for test only
  car_odom_sub = nh.subscribe("/simulation/PosePub", 100, &PlaceRecognition::car_odom_callback,this);

  cloud_sub_.subscribe(nh, "/cloud_registered_body", 10);
  odom_sub_.subscribe(nh, "/Odometry", 10);


  pcl_odom_sync_Ptr_ = std::make_shared<PclOdomSynchronizer>(PclOdomSyncPolicy(10), cloud_sub_, odom_sub_);
  pcl_odom_sync_Ptr_->registerCallback(boost::bind(&PlaceRecognition::loadstateCallback, this, _1, _2)); 
  
  pubLoopTransform = 
        nh.advertise<geometry_msgs::PoseStamped>("/loop_transform", 10);
  pubMapLocalization = 
        nh.advertise<geometry_msgs::PoseStamped>("/map_localization", 10);
    
    
    // load_config_setting(setting_path, config_setting);
    // btc_manager = new BtcDescManager(config_setting);
    ikd_match = new IKD_MATCH();

    // double lat = 0.0;
    // double lon = 0.0;
    // double alt = 0.0;
    // geo_converter.Reverse(geocentric_x, geocentric_y, geocentric_z, lat, lon, alt);
    // //geo_converter.Reset(latitude, longitude, altitude);
    // geo_converter.Reset(lat, lon, alt);
    // altitude = alt;

    // ── 读取三种定位源的开关与 options ──────────────────────────────────
    auto loadLoopOptions = [&](const std::string &prefix,
                               bool &enable_out,
                               std::vector<bool> &opts_out,
                               bool default_icp, bool default_sw, bool default_viz)
    {
        nh.param<bool>(prefix + "/enable",         enable_out,  true);
        bool icp, sw, viz;
        nh.param<bool>(prefix + "/icp_refine",     icp,  default_icp);
        nh.param<bool>(prefix + "/sliding_window", sw,   default_sw);
        nh.param<bool>(prefix + "/visualize",      viz,  default_viz);
        opts_out = {icp, sw, viz};
        ROS_INFO("%s: enable=%d, icp=%d, sliding_window=%d, visualize=%d",
                 prefix.c_str(), enable_out, icp, sw, viz);
    };

    loadLoopOptions("manual_loop", manual_loop_enable, manual_loop_options,
                    true, false, true);
    loadLoopOptions("rtk_loop",    rtk_loop_enable,    rtk_loop_options,
                    true, true, true);
    loadLoopOptions("qr_loop",     qr_loop_enable,     qr_loop_options,
                    true, true, true);

    // 初始化 RTK 重定位模块（仅在启用时创建）
    if (rtk_loop_enable)
        rtk_relocator_ = std::make_unique<RtkRelocator>(nh, this);

    // 初始化 QR 重定位模块（仅在启用时创建）
    if (qr_loop_enable)
        qr_relocator_ = std::make_unique<QrRelocator>(nh, this);

    // 初始化同步器
    last_loop_transform.first = Eigen::Vector3d::Zero();
    last_loop_transform.second = Eigen::Matrix3d::Identity();
}
//for test only
void PlaceRecognition::car_odom_callback(const carstatemsgs::CarState::ConstPtr &msg)
{
  // current_pose.first = Eigen::Vector3d(msg->x, msg->y, 0.0);
  // current_pose.second = Eigen::Quaterniond(0, 0, sin(msg->yaw/2), cos(msg->yaw/2)).toRotationMatrix();
}

void PlaceRecognition::addTransform(
  std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& window,
  const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& new_tf,
  size_t window_size)
{
  // 维护固定大小的滑动窗口
  if (window.size() >= window_size) {
      window.pop_front();
  }
  window.push_back(new_tf);
}

// 函数2: 计算滤波后的平均变换
std::pair<Eigen::Vector3d, Eigen::Matrix3d> PlaceRecognition::computeFilteredAverage(
  const std::deque<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>& window,
  double threshold)
{
  // 空窗口处理
  if (window.empty()) {
      return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity()};
  }
  
  // 计算初始平移平均值
  Eigen::Vector3d trans_sum = Eigen::Vector3d::Zero();
  for (const auto& tf : window) {
      trans_sum += tf.first;
  }
  Eigen::Vector3d trans_mean = trans_sum / window.size();
  
  // 计算初始旋转平均值
  Eigen::Quaterniond q_mean(Eigen::Matrix3d::Identity());
  if (!window.empty()) {
      Eigen::Quaterniond q_ref(window[0].second);
      std::vector<Eigen::Quaterniond> quats;
      
      for (const auto& tf : window) {
          Eigen::Quaterniond q(tf.second);
          if (q_ref.dot(q) < 0.0) q.coeffs() *= -1;
          quats.push_back(q);
      }
      
      q_mean = quats[0];
      for (size_t i = 1; i < quats.size(); ++i) {
          q_mean = q_mean.slerp(1.0/(i+1), quats[i]);
      }
  }
  
  // 滤波：移除与平移平均值差距过大的变换
  std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> filtered;
  for (const auto& tf : window) {
      if ((tf.first - trans_mean).norm() <= threshold) {
          filtered.push_back(tf);
      }
  }
  
  // 如果没有变换剩余，返回初始平均值
  if (filtered.empty()) {
      return {trans_mean, q_mean.toRotationMatrix()};
  }
  std::cout << "window: {" ;
  // 计算滤波后的平移平均值
  trans_sum.setZero();
  for (const auto& tf : filtered) {
      trans_sum += tf.first;
      std::cout << tf.first.transpose()<< "}, { ";
  }
  std::cout <<"}" << std::endl;
  Eigen::Vector3d filtered_trans_mean = trans_sum / filtered.size();
  
  // 计算滤波后的旋转平均值
  if (!filtered.empty()) {
      Eigen::Quaterniond q_ref(filtered[0].second);
      std::vector<Eigen::Quaterniond> quats;
      
      for (const auto& tf : filtered) {
          Eigen::Quaterniond q(tf.second);
          if (q_ref.dot(q) < 0.0) q.coeffs() *= -1;
          quats.push_back(q);
      }
      
      q_mean = quats[0];
      for (size_t i = 1; i < quats.size(); ++i) {
          q_mean = q_mean.slerp(1.0/(i+1), quats[i]);
      }
  }
  
  return {filtered_trans_mean, q_mean.toRotationMatrix()};
}

bool PlaceRecognition::checkSlidingWindowConsistency(
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    size_t window_size,
    double filter_threshold,
    int similarity_count,
    double trans_threshold,
    double angle_threshold)
{
    addTransform(window, loop_transform, window_size);

    auto filtered_tf = computeFilteredAverage(window, filter_threshold);
    loop_transforms.push_back(filtered_tf);

    std::cout << "sliding window loop transform: " << filtered_tf.first.transpose() << std::endl;

    if (static_cast<int>(loop_transforms.size()) < similarity_count)
        return false;

    for (int i = 1; i < similarity_count; i++) {
        const auto& prev = loop_transforms[loop_transforms.size() - i - 1];
        const auto& curr = loop_transforms[loop_transforms.size() - i];

        double delta_trans = (curr.first - prev.first).norm();
        if (delta_trans > trans_threshold)
            return false;

        Eigen::AngleAxisd angle_axis(curr.second * prev.second.transpose());
        double delta_angle = angle_axis.angle() * 180.0 / M_PI;
        if (delta_angle > angle_threshold)
            return false;
    }
    return true;
}

bool PlaceRecognition::refineWithICP(
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    const pcl::PointCloud<pcl::PointXYZI> &transform_cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &current_pose_)
{
    // 用初始 loop_transform 变换点云，发布预览
    transformAndPublishCloud(transform_cloud, loop_transform, pubRawTransformedCloud);

    // ICP 精配准
    bool if_converged;
    {
        std::lock_guard<std::mutex> lock_guard(ikd_match_mtx);
        auto cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(transform_cloud);
        if_converged = ikd_match->cloud_matching(cloud_ptr, current_pose_, loop_transform);
        std::cout << "ICP loop transform: " << loop_transform.first.transpose() << std::endl;
    }
    return if_converged;
}

void PlaceRecognition::transformAndPublishCloud(
    const pcl::PointCloud<pcl::PointXYZI> &cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &transform,
    ros::Publisher &publisher,
    float intensity)
{
    pcl::PointCloud<pcl::PointXYZI> transformed;
    transformed.resize(cloud.size());
    for (size_t i = 0; i < cloud.size(); i++) {
        Eigen::Vector3d pv = point2vec(cloud.points[i]);
        pv = transform.second * pv + transform.first;
        transformed.points[i] = vec2point(pv);
        transformed.points[i].intensity = intensity;
    }
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(transformed, pub_cloud);
    pub_cloud.header.frame_id = visualization_frame_id;
    publisher.publish(pub_cloud);
}

void PlaceRecognition::publishLoopTransformAsPose(
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    ros::Publisher &publisher)
{
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose_transform;
    pose_transform.second = loop_transform.second.transpose();
    pose_transform.first  = -pose_transform.second * loop_transform.first;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp    = ros::Time::now();
    pose_msg.header.frame_id = visualization_frame_id;
    pose_msg.pose.position.x = pose_transform.first.x();
    pose_msg.pose.position.y = pose_transform.first.y();
    pose_msg.pose.position.z = pose_transform.first.z();
    Eigen::Quaterniond quat(pose_transform.second);
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    publisher.publish(pose_msg);
}

void PlaceRecognition::publishVisualizationClouds(
    const pcl::PointCloud<pcl::PointXYZI> &transform_cloud,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform,
    bool is_true_loop)
{
    sensor_msgs::PointCloud2 pub_cloud;

    // 1. 发布原始点云
    pcl::toROSMsg(transform_cloud, pub_cloud);
    pub_cloud.header.frame_id = visualization_frame_id;
    pubCureentCloud.publish(pub_cloud);

    // 2. 用 loop_transform 变换点云
    pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
    transformed_cloud.resize(transform_cloud.size());
    for (size_t i = 0; i < transform_cloud.size(); i++) {
        Eigen::Vector3d pv = point2vec(transform_cloud.points[i]);
        pv = loop_transform.second * pv + loop_transform.first;
        transformed_cloud.points[i] = vec2point(pv);
        transformed_cloud.points[i].intensity = transform_cloud.points[i].intensity;
    }

    // 3. 着色并发布（成功=绿色，失败=红色）
    uint8_t r = is_true_loop ? 0   : 255;
    uint8_t g = is_true_loop ? 255 : 0;
    uint8_t b = 0;

    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud.width  = transformed_cloud.width;
    color_cloud.height = transformed_cloud.height;
    color_cloud.is_dense = transformed_cloud.is_dense;
    color_cloud.points.resize(transformed_cloud.points.size());
    for (size_t i = 0; i < transformed_cloud.points.size(); ++i) {
        color_cloud.points[i].x = transformed_cloud.points[i].x;
        color_cloud.points[i].y = transformed_cloud.points[i].y;
        color_cloud.points[i].z = transformed_cloud.points[i].z;
        color_cloud.points[i].r = r;
        color_cloud.points[i].g = g;
        color_cloud.points[i].b = b;
    }
    pcl::toROSMsg(color_cloud, pub_cloud);
    pub_cloud.header.frame_id = visualization_frame_id;
    pubTransformedCloud.publish(pub_cloud);
}

void PlaceRecognition::search_and_pub_loop(std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform, 
  pcl::PointCloud<pcl::PointXYZI> transform_cloud, std::pair<Eigen::Vector3d, Eigen::Matrix3d> current_pose_,
  std::vector<bool> options){
 // std::cout << "current_cloud size3: " << transform_cloud.size() << std::endl;

  // 解析开关（不足的位默认 true）
  options.resize(LOOP_OPTION_COUNT, true);
  bool enable_icp_refine    = options[OPT_ICP_REFINE];
  bool enable_sliding_window = options[OPT_SLIDING_WINDOW];
  bool enable_visualize      = options[OPT_VISUALIZE];

  std::mutex mtx;
  std::unique_lock<std::mutex> lock(mtx);
  
  // ── 1. ICP 精匹配（可选） ─────────────────────────────────────────────
  bool if_converged;
  if (enable_icp_refine)
  {
    if_converged = refineWithICP(loop_transform, transform_cloud, current_pose_);
  }
  else
  {
    // 跳过 ICP，直接视为收敛
    if_converged = true;
  }

  // ── 2. 判定是否为有效匹配 ─────────────────────────────────────────────
  bool is_true_loop = false;
  if(if_converged)
  {
    if (enable_sliding_window)
    {
      is_true_loop = checkSlidingWindowConsistency(loop_transform);
    }
    else
    {
      is_true_loop = true;
    }
  }
  
  std::cout  << "is_true_loop: " << is_true_loop << std::endl;
  all_count++;
  
  // ── 3. 发布匹配结果 ───────────────────────────────────────────────────
  if(is_true_loop)
  {
    count++;
    publishLoopTransformAsPose(loop_transform, pubLoopTransform);
    
    // 更新上次匹配成功时的变换
    last_loop_transform = loop_transform;
  }
  
  std::cout << "all_count: " << all_count << ", count: " << count << "rate: " << (double)count/all_count << std::endl;

  // ── 4. 可视化（可选） ─────────────────────────────────────────────────
  if (enable_visualize)
  {
    publishVisualizationClouds(transform_cloud, loop_transform, is_true_loop);
  }
}


void PlaceRecognition::loadstateCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg, const nav_msgs::Odometry::ConstPtr& odom_msg)
{

    Eigen::Vector3d translation(
      odom_msg->pose.pose.position.x,
      odom_msg->pose.pose.position.y,
      odom_msg->pose.pose.position.z);
  
    Eigen::Quaterniond quat(
      odom_msg->pose.pose.orientation.w,
      odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y,
      odom_msg->pose.pose.orientation.z);
  
    current_pose.second = quat.toRotationMatrix();
    current_pose.first = translation + lidar_imu_trans - current_pose.second * lidar_imu_trans;    
                                                                                                                                                                                
    //rotation * lidar_imu_trans + translation;
    pcl::fromROSMsg(*cloud_msg, current_cloud);
    //std::cout << "cloud size: " << transform_cloud.size() << std::endl;
    //std::cout << "odom_msg: " << translation.transpose() << std::endl;
    
    for (size_t j = 0; j < current_cloud.size(); j++) {
      Eigen::Vector3d pv = point2vec(current_cloud.points[j]);
      pv = current_pose.second * pv  + translation + lidar_imu_trans;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         
      current_cloud.points[j] = vec2point(pv);
      current_cloud.points[j].intensity = 100;
    }
    
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(current_cloud, pub_cloud);
    pub_cloud.header.frame_id = visualization_frame_id;
    pubCurrentCloud.publish(pub_cloud);
   // std::cout << "current_cloud size1: " << current_cloud.size() << std::endl;


    // std::pair<Eigen::Vector3d, Eigen::Matrix3d> map_localization_pose;
    // map_localization_pose.second = last_loop_transform.second * current_pose.second;
    // map_localization_pose.first =  last_loop_transform.second * current_pose.first + last_loop_transform.first;
  
    // // 发布地图定位位置
    // geometry_msgs::PoseStamped map_loc_msg;
    // map_loc_msg.header.stamp = ros::Time::now();
    // map_loc_msg.header.frame_id = visualization_frame_id;
    // map_loc_msg.pose.position.x = map_localization_pose.first.x();
    // map_loc_msg.pose.position.y = map_localization_pose.first.y();
    // map_loc_msg.pose.position.z = map_localization_pose.first.z();
    // Eigen::Quaterniond map_quat(map_localization_pose.second);
    // map_loc_msg.pose.orientation.x = map_quat.x();
    // map_loc_msg.pose.orientation.y = map_quat.y();
    // map_loc_msg.pose.orientation.z = map_quat.z();
    // map_loc_msg.pose.orientation.w = map_quat.w();
    // pubMapLocalization.publish(map_loc_msg);

    return;
}

bool PlaceRecognition::estimateZFromMap(Eigen::Vector3d &position, double side_length) const
{
    double half = side_length / 2.0;
    double min_x = position.x() - half, max_x = position.x() + half;
    double min_y = position.y() - half, max_y = position.y() + half;

    pcl::PointCloud<pcl::PointXYZI>::Ptr area_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto &pt : whole_cloud)
    {
        if (pt.x >= min_x && pt.x <= max_x &&
            pt.y >= min_y && pt.y <= max_y)
            area_cloud->push_back(pt);
    }
    if (area_cloud->empty())
    {
        std::cout << "estimateZFromMap: No points found in the specified area." << std::endl;
        return false;
    }

    double sum_z = 0.0;
    for (const auto &pt : *area_cloud) sum_z += pt.z;
    position.z() = sum_z / area_cloud->size();
    return true;
}

std::pair<Eigen::Vector3d, Eigen::Matrix3d> PlaceRecognition::computeInitTransform(
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &init_pose) const
{
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_tf;
    init_tf.second = init_pose.second * current_pose.second.transpose();
    init_tf.first  = init_pose.first  - init_tf.second * current_pose.first;
    return init_tf;
}

void PlaceRecognition::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (!manual_loop_enable) {
        ROS_WARN("Manual loop is disabled, ignoring /initialpose");
        return;
    }

    ROS_INFO("Received initial pose estimate:");
    static float past_z = 0.0;
    auto position = msg->pose.pose.position;

    // 使用公共函数从地图估算 Z 值
    Eigen::Vector3d pos_eigen(position.x, position.y, 0.0);
    if (estimateZFromMap(pos_eigen, 1.0)) {
        position.z = pos_eigen.z();
        past_z = position.z;
        ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                  position.x, position.y, position.z);
    } else {
        position.z = past_z;
        ROS_INFO("  Position (fallback z): x=%.3f, y=%.3f, z=%.3f", 
                  position.x, position.y, position.z);
    }

   
    
    // 3. 提取朝向信息（四元数）
    const auto& orientation = msg->pose.pose.orientation;
    ROS_INFO("  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
              orientation.x, orientation.y, orientation.z, orientation.w);
   std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
   init_pose.first = Eigen::Vector3d(position.x, position.y, position.z);
   init_pose.second = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).toRotationMatrix();
   auto init_tranform = computeInitTransform(init_pose);
   search_and_pub_loop(init_tranform, current_cloud, current_pose, manual_loop_options);
   
}

void PlaceRecognition::load_global_pcd()
{
  pcl::PCDReader reader;
  
  //whole_cloud.reserve(1000000);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>());

  //pcl::io::loadPCDFile<pcl::PointXYZ>(full_pcd_dir, *cloud);
  if (reader.read(full_pcd_dir, *cloud) == -1) {
    ROS_ERROR_STREAM("Couldn't read file " << full_pcd_dir);
    return;
  }
  
  std::cout << "cloud size: " << cloud->size() << std::endl;
  whole_cloud.resize(cloud->size());
  double x_all;
  for(int i = 0; i < cloud->size(); i++)
  {
    auto &pt = whole_cloud.points[i];
    pt.x = cloud->points[i].x;
    pt.y = cloud->points[i].y;
    pt.z = cloud->points[i].z;
    pt.intensity = 100;
    x_all += pt.x; 
  }
  std::cout<<"x_average"<< x_all/whole_cloud.size()<<std::endl;
  std::cout << "whole_cloud size: " << whole_cloud.size() << std::endl;

//检查全局坐标系
  // pcl::PointXYZI min_pt, max_pt;
  // pcl::getMinMax3D(whole_cloud, min_pt, max_pt);

  // std::cout << "点云坐标范围:" << std::endl;
  // std::cout << "X: " << min_pt.x << " 到 " << max_pt.x << std::endl;
  // std::cout << "Y: " << min_pt.y << " 到 " << max_pt.y << std::endl;
  // std::cout << "Z: " << min_pt.z << " 到 " << max_pt.z << std::endl;

  // // 判断标准：
  // // - 如果坐标值很小（如-100到100米）：局部坐标系
  // // - 如果坐标值很大（如几十万米）：UTM全局坐标系
  // // - 如果经纬度值（如116.xxx, 39.xxx）：经纬度坐标系

  // if (max_pt.x < 1000 && min_pt.x > -1000) {
  //     std::cout << "这很可能是局部坐标系下的点云" << std::endl;
  // } else if (max_pt.x > 100000) {
  //     std::cout << "这很可能是UTM全局坐标系下的点云" << std::endl;
  // }
  
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(whole_cloud, pub_cloud);
  pub_cloud.header.frame_id = visualization_frame_id;
  
  pub_cloud.header.seq = 0;
 // pub_cloud.header.stamp = ros::Time(0);

  pub_cloud.is_dense = false;
  pubWholeCloud.publish(pub_cloud);
  ikd_match->cloud_matching_init(full_pcd_dir);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "place_recognition_node");
  ros::NodeHandle nh;
  PlaceRecognition place_recog(nh);
  place_recog.load_global_pcd();
  
  // 使用 ros::spin() 代替无延迟的循环，避免CPU 100%占用
  ros::spin();
  
  return 0;
}
