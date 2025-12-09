#include "place_recognition.hpp"
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

  double x, y, z;
  nh.param<double>("lidar_imu_trans/x", x, 0.0);
  nh.param<double>("lidar_imu_trans/y", y, 0.0);
  nh.param<double>("lidar_imu_trans/z", z, 0.0);
  lidar_imu_trans << x, y, z;
  
  std::cout << "lidar_imu_trans: " << lidar_imu_trans.transpose() << std::endl;
  
  nh.param<double>("RTK_imu_trans/x", x, 0.0);
  nh.param<double>("RTK_imu_trans/y", y, 0.0);
  nh.param<double>("RTK_imu_trans/z", z, 0.0);
  RTK_imu_trans << x, y, z;
  std::cout << "RTK_imu_trans: " << RTK_imu_trans.transpose() << std::endl;
 
  nh.param<double>("latitude", latitude, 0.0);
  nh.param<double>("longitude", longitude, 0.0);
  nh.param<double>("altitude", altitude, 0.0);
  std::cout << "latitude: " << latitude << std::endl;
  std::cout << "longitude: " << longitude << std::endl;
  std::cout << "altitude: " << altitude << std::endl;
  nh.param<double>("geocentric_x", geocentric_x, 0.0);
  nh.param<double>("geocentric_y", geocentric_y, 0.0);
  nh.param<double>("geocentric_z", geocentric_z, 0.0);

  // color_tp.a = 1.0;
  // color_tp.r = 0.0 / 255.0;
  // color_tp.g = 255.0 / 255.0;
  // color_tp.b = 0.0 / 255.0;

  // color_fp.a = 1.0;
  // color_fp.r = 1.0;
  // color_fp.g = 0.0;
  // color_fp.b = 0.0;

  // color_path.a = 0.8;
  // color_path.r = 255.0 / 255.0;
  // color_path.g = 255.0 / 255.0;
  // color_path.b = 255.0 / 255.0;




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
    
    
    // load_config_setting(setting_path, config_setting);
    // btc_manager = new BtcDescManager(config_setting);
    ikd_match = new IKD_MATCH();

    double lat = 0.0;
    double lon = 0.0;
    double alt = 0.0;
    // geo_converter.Reverse(geocentric_x, geocentric_y, geocentric_z, lat, lon, alt);
    
    // //geo_converter.Reset(latitude, longitude, altitude);
    // geo_converter.Reset(lat, lon, alt);
    // altitude = alt;
    // 创建消息同步器
    // rtk_odom_sub_.subscribe(nh, "/gps/euler", 10);
    // rtk_fix_sub_.subscribe(nh, "/gps/fix", 10);
    // sync_.reset(new Sync(SyncPolicy(10), rtk_odom_sub_, rtk_fix_sub_));
    // sync_->registerCallback(boost::bind(&PlaceRecognition::RTKPoseCallback, this, _1, _2));
    rtk_covariance_sub_= nh.subscribe("/rtk_agent/navsatfix", 10, &PlaceRecognition::RTKPoseCallback, this);
    // 初始化同步器

}
void PlaceRecognition::car_odom_callback(const carstatemsgs::CarState::ConstPtr &msg)
{
  current_pose.first = Eigen::Vector3d(msg->x, msg->y, 0.0);
  current_pose.second = Eigen::Quaterniond(0, 0, sin(msg->yaw/2), cos(msg->yaw/2)).toRotationMatrix();
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

void PlaceRecognition::search_and_pub_loop(std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform, 
  pcl::PointCloud<pcl::PointXYZI> transform_cloud, std::pair<Eigen::Vector3d, Eigen::Matrix3d> current_pose_){
  std::cout << "current_cloud size3: " << transform_cloud.size() << std::endl;
  ros::Rate slow_loop(1000);
  std::mutex mtx;
  std::unique_lock<std::mutex> lock(mtx);
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform_raw;
  loop_transform_raw.first = loop_transform.first;
  loop_transform_raw.second = loop_transform.second;
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::PointCloud<pcl::PointXYZI> transformed_cloud_raw;
  transformed_cloud_raw.resize(transform_cloud.size());
  
  for (size_t i = 0; i < transform_cloud.size(); i++) {
    Eigen::Vector3d pv = point2vec(transform_cloud.points[i]);
    pv = loop_transform.second * pv + loop_transform.first;
    transformed_cloud_raw.points[i] = vec2point(pv);
    transformed_cloud_raw.points[i].intensity = 100;
  }
  pcl::toROSMsg(transformed_cloud_raw, pub_cloud);
  pub_cloud.header.frame_id = "base";
  pubRawTransformedCloud.publish(pub_cloud);
  bool if_converged;   

  {
    std::lock_guard<std::mutex> lock_guard(ikd_match_mtx);
    // ikd_match->cloud_matching( btc_manager->key_cloud_vec_[search_result.first],
    //   transform_cloud.makeShared(),
    //   pose_list[submap_id],
    //   loop_transform);
    if_converged = ikd_match->cloud_matching(transform_cloud.makeShared(), current_pose_, loop_transform);
    
    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!loop transform: " << loop_transform.first.transpose() << std::endl;
  }
  bool is_true_loop = false;
  //for test only
  if (mannul_init_flag)
  {
    is_true_loop = true;
    mannul_init_flag = false;
    //
  }
  if(if_converged)
  {
    if (mannul_init_flag)
    {
      is_true_loop = true;
      mannul_init_flag = false;
      //
    }else if(RTK_init_flag)
    {
      RTK_init_flag = false;
      //std::cout << "loop transform norm: " << loop_transform.first.norm() << std::endl;
      
      addTransform(window, loop_transform, 5);  // 窗口大小=5
      
      // 计算滤波后的平均值
      auto loop_transform = computeFilteredAverage(window, 0.1);  // 阈值=0.1
      loop_transforms.push_back(loop_transform);
      int similarity_count = 3;
      const double trans_threshold = 0.06;  // 平移变化阈值（米）
      const double angle_threshold = 2.0;  // 旋转变化阈值（度）
      std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!loop transform: " << loop_transform.first.transpose()
      << std::endl;
      if (loop_transforms.size() >= similarity_count) {
          
          for (int i = 1; i < similarity_count; i++) {
              const auto& prev = loop_transforms[loop_transforms.size() - i - 1];
              const auto& curr = loop_transforms[loop_transforms.size() - i];
              
              // 检查平移变化
              double delta_trans = (curr.first - prev.first).norm();
              if (delta_trans > trans_threshold) {
                  break;
              }
              
              // 检查旋转变化（将旋转矩阵转换为角度差）
              Eigen::AngleAxisd angle_axis(curr.second * prev.second.transpose());
              double delta_angle = angle_axis.angle() * 180.0 / M_PI;
              if (delta_angle > angle_threshold) {
                  break;
              }

            // if ((cloud_overlap >= cloud_overlap_thr))
            // {
              is_true_loop = true;
            //}
          }
          
          
      }

    }
  } else{
    if (mannul_init_flag || RTK_init_flag)
    {
      is_true_loop = false;
      mannul_init_flag = false;
      RTK_init_flag = false;
    } 
  }
  
  std::cout  << "is_true_loop: " << is_true_loop << std::endl;
  all_count++;
  if(is_true_loop)
  {
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> RTK1_to_RTK2; 
    RTK1_to_RTK2.second = loop_transform.second * loop_transform_raw.second.transpose();
    RTK1_to_RTK2.first = loop_transform.first - RTK1_to_RTK2.second * loop_transform_raw.first;
    std::cout << "222222222222222222222222222222R_RTK1_to_RTK2: " << RTK1_to_RTK2.second << std::endl;
    std::cout << "222222222222222222222222222222T_RTK1_to_RTK2: " << RTK1_to_RTK2.first.transpose() << std::endl;
    count++;
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> pose_transform;
    
    pose_transform.second = loop_transform.second.transpose();
    pose_transform.first = - pose_transform.second * loop_transform.first ;
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "base";
    pose_msg.pose.position.x = pose_transform.first.x();
    pose_msg.pose.position.y = pose_transform.first.y();
    pose_msg.pose.position.z = pose_transform.first.z();
    Eigen::Quaterniond quat(pose_transform.second);
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();
    pubLoopTransform.publish(pose_msg);

    std::cout << "all_count: " << all_count << ", count: " << count << "rate: " << (double)count/all_count << std::endl;
  }  
  
    // visuliazaion
  
  pcl::toROSMsg(transform_cloud, pub_cloud);
  pub_cloud.header.frame_id = "base";
  pubCureentCloud.publish(pub_cloud);
  
  pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
  transformed_cloud.resize(transform_cloud.size());
  for (size_t i = 0; i < transform_cloud.size(); i++) {
    Eigen::Vector3d pv = point2vec(transform_cloud.points[i]);
    pv = loop_transform.second * pv + loop_transform.first;
    transformed_cloud.points[i] = vec2point(pv);
    transformed_cloud.points[i].intensity = transform_cloud.points[i].intensity;
  }
  if(is_true_loop)
  {
    // 创建带颜色的点云
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud.width = transformed_cloud.width;
    color_cloud.height = transformed_cloud.height;
    color_cloud.points.resize(transformed_cloud.points.size());
    
    // 设置颜色（这里设为绿色：R=0, G=255, B=0）
    for (size_t i = 0; i < transformed_cloud.points.size(); ++i) {
        color_cloud.points[i].x = transformed_cloud.points[i].x;
        color_cloud.points[i].y = transformed_cloud.points[i].y;
        color_cloud.points[i].z = transformed_cloud.points[i].z;
        color_cloud.points[i].r = 0;    // 红色分量
        color_cloud.points[i].g = 255;  // 绿色分量
        color_cloud.points[i].b = 0;    // 蓝色分量
    }
    
    pcl::toROSMsg(color_cloud, pub_cloud);
    pub_cloud.header.frame_id = "base";
    pubTransformedCloud.publish(pub_cloud);
    slow_loop.sleep();
  }else{
    // 同上，可设置不同颜色（这里设为红色）
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud.width = transformed_cloud.width;
    color_cloud.height = transformed_cloud.height;
    color_cloud.is_dense = transformed_cloud.is_dense;
    color_cloud.points.resize(transformed_cloud.points.size());
    for (size_t i = 0; i < transformed_cloud.points.size(); ++i) {
        color_cloud.points[i].x = transformed_cloud.points[i].x;
        color_cloud.points[i].y = transformed_cloud.points[i].y;
        color_cloud.points[i].z = transformed_cloud.points[i].z;
        color_cloud.points[i].r = 255;  // 红色
        color_cloud.points[i].g = 0;
        color_cloud.points[i].b = 0;
    }
    pcl::toROSMsg(color_cloud, pub_cloud);
    pub_cloud.header.frame_id = "base";
    pubTransformedCloud.publish(pub_cloud);
    slow_loop.sleep();
  }
   //auto t_query_end = std::chrono::high_resolution_clock::now();
   //querying_time.push_back(time_inc(t_query_end, t_query_begin));
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
    pub_cloud.header.frame_id = "base";
    pubCurrentCloud.publish(pub_cloud);
    std::cout << "current_cloud size1: " << current_cloud.size() << std::endl;
    return;
}

void PlaceRecognition::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    ROS_INFO("Received initial pose estimate:");
    static float past_z = 0.0;
    auto position = msg->pose.pose.position;

    float side_length = 1.0f;
    float half_side = side_length / 2.0f;
    float min_x = position.x - half_side;
    float max_x = position.x + half_side;
    float min_y = position.y - half_side;
    float max_y = position.y + half_side;

    // 筛选区域内的点
    pcl::PointCloud<pcl::PointXYZI>::Ptr area_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
    for (const auto& point : whole_cloud) {
        if (point.x >= min_x && point.x <= max_x &&
            point.y >= min_y && point.y <= max_y) {
            area_cloud->push_back(point);
        }
    } 
    // 检查区域内是否有足够多的点
    if (area_cloud->empty()) {
      std::cout << "No points found in the specified area." << std::endl;
      //for test only
      //return;
      position.z = past_z;
      ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                position.x, position.y, position.z);
    }else{
 // 计算平均Z值
        float sum_z = 0.0f;
        for (const auto& point : *area_cloud) {
          sum_z += point.z;
        }
        float average_z = sum_z / area_cloud->size();

        position.z = average_z;
        past_z = position.z;
        ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f", 
                  position.x, position.y, position.z);
    }

   
    
    // 3. 提取朝向信息（四元数）
    const auto& orientation = msg->pose.pose.orientation;
    ROS_INFO("  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
              orientation.x, orientation.y, orientation.z, orientation.w);
   std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
   init_pose.first = Eigen::Vector3d(position.x, position.y, position.z);
   init_pose.second = Eigen::Quaterniond(orientation.w, orientation.x, orientation.y, orientation.z).toRotationMatrix();
   mannul_init_flag = true;
   std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_tranform;
   init_tranform.first = init_pose.first - init_pose.second * current_pose.second.transpose() * current_pose.first;
   init_tranform.second = init_pose.second * current_pose.second.transpose();
    search_and_pub_loop(init_tranform, current_cloud, current_pose);
   
}
// header: 
//   seq: 2961
//   stamp: 
//     secs: 1757708843
//     nsecs: 663201093
//   frame_id: "navsat_link"
// status: 
//   status: 0
//   service: 0
// latitude: 30.2640231824
// longitude: 120.29553101983
// altitude: 8.2441
// position_covariance: [9.025e-05, 0.0, 0.0, 0.0, 8.099999999999999e-05, 0.0, 0.0, 0.0, 0.00054756]
// position_covariance_type: 2

// header: 
//   seq: 29933
//   stamp: 
//     secs: 1757708856
//     nsecs:  62826156
//   frame_id: "euler_link"
// vector: 
//   x: -6.108652381980153e-05
//   y: 0.04188092073085593
//   z: 0.9040945151670787
double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
/**
 * @brief 将欧拉角转换为旋转矩阵
 * 
 * @param euler_angles 欧拉角向量 (roll, pitch, yaw) 单位弧度
 * @param order 旋转顺序，默认为ZYX（yaw-pitch-roll）
 * @return Eigen::Matrix3d 对应的旋转矩阵
 */
Eigen::Matrix3d PlaceRecognition::getRotationMatrixFromEuler(
    const Eigen::Vector3d& euler_angles, 
    const std::string& order)
{
    double roll = normalizeAngle(euler_angles.y());
    double pitch = normalizeAngle(euler_angles.x());
    double yaw = normalizeAngle(90*M_PI/180 - euler_angles.z());
    
    Eigen::Matrix3d R;
    
    if (order == "ZYX") {
        // 常用顺序：yaw (Z) -> pitch (Y) -> roll (X)
        double cy = cos(yaw);
        double sy = sin(yaw);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cr = cos(roll);
        double sr = sin(roll);
        
        R << cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr,
             sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr,
             -sp,   cp*sr,           cp*cr;
             
    } else if (order == "XYZ") {
        // 顺序：roll (X) -> pitch (Y) -> yaw (Z)
        double cr = cos(roll);
        double sr = sin(roll);
        double cp = cos(pitch);
        double sp = sin(pitch);
        double cy = cos(yaw);
        double sy = sin(yaw);
        
        R << cp*cy, -cr*sy + sr*sp*cy, sr*sy + cr*sp*cy,
             cp*sy, cr*cy + sr*sp*sy, -sr*cy + cr*sp*sy,
             -sp,   sr*cp,            cr*cp;
             
    } else if (order == "ZXY") {
        // 顺序：yaw (Z) -> roll (X) -> pitch (Y)
        double cy = cos(yaw);
        double sy = sin(yaw);
        double cr = cos(roll);
        double sr = sin(roll);
        double cp = cos(pitch);
        double sp = sin(pitch);
        
        R << cy*cr - sy*sr*sp, -sy*cp, cy*sr + sy*cr*sp,
             sy*cr + cy*sr*sp, cy*cp,  sy*sr - cy*cr*sp,
             -cp*sr,           sp,     cp*cr;
             
    } else {
        ROS_ERROR("Unsupported rotation order: %s. Using identity matrix.", order.c_str());
        R = Eigen::Matrix3d::Identity();
    }
    
    return R;
}
Eigen::Matrix3d PlaceRecognition::getRotationMatrixFromYaw(double yaw_angle_radians) {
  Eigen::Matrix3d rotation_matrix;
  
  double cos_yaw = std::cos(yaw_angle_radians);
  double sin_yaw = std::sin(yaw_angle_radians);
  
  // 绕Z轴的旋转矩阵
  rotation_matrix << cos_yaw, -sin_yaw, 0,
                     sin_yaw,  cos_yaw, 0,
                     0,        0,       1;
  
  return rotation_matrix;
}
// void PlaceRecognition::RTKPoseCallback(
//     const geometry_msgs::Vector3Stamped::ConstPtr& euler_msg,
//     const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
// {

//     ROS_INFO("Received RTK initial pose estimate:");
//     double start_time = ros::Time::now().toSec();

    
//     const auto& covariance = fix_msg->position_covariance;
//     if(covariance[0] > 0.0005 || covariance[4] > 0.00054756)
//     {
//       ROS_ERROR("RTK covariance is too high, skip this pose");
//       return;

//     }
//     Eigen::Vector3d euler_angle(euler_msg->vector.x, euler_msg->vector.y, euler_msg->vector.z);
//     //
//     //Eigen::Matrix3d rotation_matrix = getRotationMatrixFromYaw(-euler_angle.z+90*M_PI/180);
//     Eigen::Matrix3d rotation_matrix = getRotationMatrixFromEuler(euler_angle, "ZYX");
//     latitude = fix_msg->latitude;
//     longitude = fix_msg->longitude;
//     altitude = 0.0;

//     //经纬度转ENU
//     double local_E, local_N, local_U;
//     geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
//     std::cout << "local_E: " << local_E << ", local_N: " << local_N << ", local_U: " << local_U << std::endl;
//     Eigen::Vector3d position(local_E, local_N, local_U);
//     //position = position + RTK_imu_trans - rotation_matrix.inverse() * RTK_imu_trans;

//     //get z from the global input map
//     double side_length = 1.0;
//     double half_side = side_length / 2.0;
    
//     // 修复坐标访问方式
//     double min_x = position.x() - half_side;
//     double max_x = position.x() + half_side;
//     double min_y = position.y() - half_side;
//     double max_y = position.y() + half_side;

//     // 筛选区域内的点
//     pcl::PointCloud<pcl::PointXYZI>::Ptr area_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    
//     for (const auto& point : whole_cloud) {
//         if (point.x >= min_x && point.x <= max_x &&
//             point.y >= min_y && point.y <= max_y) {
//             area_cloud->push_back(point);
//         }
//     } 
//     // 检查区域内是否有足够多的点
//     if (area_cloud->empty()) {
//       std::cout << "No points found in the specified area." << std::endl;
//       return;
//     }

//     // 计算平均Z值
//     double sum_z = 0.0;
//     for (const auto& point : *area_cloud) {
//       sum_z += point.z;
//     }
//     double average_z = sum_z / area_cloud->size();

//     position.z() = average_z;
//     // ROS_INFO("  Position: x=%.3, y=%.3f, z=%.3f", 
//     //          position.x(), position.y(), position.z());
    

   
//    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
//    init_pose.first = Eigen::Vector3d(position.x(), position.y(), position.z());
//    init_pose.second = rotation_matrix;
//    RTK_init_flag = true;
//    std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_tranform;
//    init_tranform.second = init_pose.second * current_pose.second.transpose();
//    init_tranform.first = init_pose.first - init_pose.second * current_pose.second.transpose() * current_pose.first;
//    Eigen::Quaterniond quat(init_pose.second);
//    nav_msgs::Odometry pose_msg;
//    pose_msg.pose.pose.position.x = init_pose.first.x();
//    pose_msg.pose.pose.position.y = init_pose.first.y();
//    pose_msg.pose.pose.position.z = init_pose.first.z();
//    pose_msg.pose.pose.orientation.x = quat.x();
//    pose_msg.pose.pose.orientation.y = quat.y();
//    pose_msg.pose.pose.orientation.z = quat.z();
//    pose_msg.pose.pose.orientation.w = quat.w();
//    pose_msg.header.frame_id = "base";
//    pose_msg.header.stamp = ros::Time::now();
//    pubMatchedPose.publish(pose_msg);

//    std::cout<<"RTK init_tranform: "<<init_tranform.first.transpose()<<std::endl;
//    std::cout<<"RTK init_tranform: "<<init_tranform.second<<std::endl;
   
//    std::cout << "current_cloud size2: " << current_cloud.size() << std::endl;

//    double lidar_point_time = ros::Time::now().toSec();
//     std::cout << "lidar_point_time: " << lidar_point_time - start_time << std::endl;
//    search_and_pub_loop(init_tranform, current_cloud, current_pose);

//    double end_time = ros::Time::now().toSec();
//    std::cout << "matching time: " << end_time - start_time << std::endl;
   
// }

void PlaceRecognition::RTKPoseCallback(
  const sensor_msgs::NavSatFix::ConstPtr& fix_msg)
{

  ROS_INFO("Received RTK initial pose estimate:");
  double start_time = ros::Time::now().toSec();

  
  const auto& covariance = fix_msg->position_covariance;
  if(covariance[0] > 0.0005 || covariance[4] > 0.00054756)
  {
    ROS_ERROR("RTK covariance is too high, skip this pose");
    return;

  }
 
 
  //altitude = 0.0;
  //altitude = fix_msg->altitude;
  //经纬度转ENU



  int zone; 
  bool northp = true;
  double utm_x, utm_y;
  if(!UTM_init)
  {
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_x_0, utm_y_0);
    UTM_init = true;
    std::cout<< "latitude:"<< latitude << " longitude:" << longitude << " utm_x_0:" <<utm_x_0 <<" utm_y_0:" <<utm_y_0 <<std::endl;
    return;
  }else{
    latitude = fix_msg->latitude;
    longitude = fix_msg->longitude;
    GeographicLib::UTMUPS::Forward(latitude, longitude, zone, northp, utm_x, utm_y);
    std::cout << "自动计算的带号: " << zone << std::endl;
   

  }
  Eigen::Vector3d position(utm_x, utm_y, 0.0);
  position.x() = position.x() - utm_x_0;
  position.y() = position.y() - utm_y_0;
  std::cout<<"position:" << position.transpose()<< std::endl;
 
  // double local_E, local_N, local_U;
  // geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
  
  //std::cout << "local_E: " << local_E << ", local_N: " << local_N << ", local_U: " << local_U << std::endl;
  //Eigen::Vector3d position(local_E, local_N, local_U);
  //position = position + RTK_imu_trans - rotation_matrix.inverse() * RTK_imu_trans;

  //get z from the global input map
  double side_length = 1.0;
  double half_side = side_length / 2.0;
  
  // 修复坐标访问方式
  double min_x = position.x() - half_side;
  double max_x = position.x() + half_side;
  double min_y = position.y() - half_side;
  double max_y = position.y() + half_side;

  // 筛选区域内的点
  pcl::PointCloud<pcl::PointXYZI>::Ptr area_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  
  for (const auto& point : whole_cloud) {
      if (point.x >= min_x && point.x <= max_x &&
          point.y >= min_y && point.y <= max_y) {
          area_cloud->push_back(point);
      }
  } 
  // 检查区域内是否有足够多的点
  if (area_cloud->empty()) {
    std::cout << "No points found in the specified area." << std::endl;
    // return;
  }

  // // 计算平均Z值
  // double sum_z = 0.0;
  // for (const auto& point : *area_cloud) {
  //   sum_z += point.z;
  // }
  // double average_z = sum_z / area_cloud->size();

  // position.z() = average_z;
  std::cout<<"position:" << position.transpose()<< std::endl;
  

 
 std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_pose;
 init_pose.first = Eigen::Vector3d(position.x(), position.y(), position.z());
 init_pose.second = Eigen::Matrix3d::Identity();
 RTK_init_flag = true;
 std::pair<Eigen::Vector3d, Eigen::Matrix3d> init_tranform;
 init_tranform.second = init_pose.second * current_pose.second.transpose();
 init_tranform.first = init_pose.first - init_pose.second * current_pose.second.transpose() * current_pose.first;
 Eigen::Quaterniond quat(init_pose.second);
 nav_msgs::Odometry pose_msg;
 pose_msg.pose.pose.position.x = init_pose.first.x();
 pose_msg.pose.pose.position.y = init_pose.first.y();
 pose_msg.pose.pose.position.z = init_pose.first.z();
 pose_msg.pose.pose.orientation.x = quat.x();
 pose_msg.pose.pose.orientation.y = quat.y();
 pose_msg.pose.pose.orientation.z = quat.z();
 pose_msg.pose.pose.orientation.w = quat.w();
 pose_msg.header.frame_id = "base";
 pose_msg.header.stamp = ros::Time::now();
 pubMatchedPose.publish(pose_msg);

 std::cout<<"RTK init_tranform: "<<init_tranform.first.transpose()<<std::endl;
 std::cout<<"RTK init_tranform: "<<init_tranform.second<<std::endl;
 
 std::cout << "current_cloud size2: " << current_cloud.size() << std::endl;

 double lidar_point_time = ros::Time::now().toSec();
  std::cout << "lidar_point_time: " << lidar_point_time - start_time << std::endl;
 search_and_pub_loop(init_tranform, current_cloud, current_pose);

 double end_time = ros::Time::now().toSec();
 std::cout << "matching time: " << end_time - start_time << std::endl;
 
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
  for(int i = 0; i < cloud->size(); i++)
  {
    auto &pt = whole_cloud.points[i];
    pt.x = cloud->points[i].x;
    pt.y = cloud->points[i].y;
    pt.z = cloud->points[i].z;
    pt.intensity = 100;
  }
  
  std::cout << "whole_cloud size: " << whole_cloud.size() << std::endl;

//检查全局坐标系
  pcl::PointXYZI min_pt, max_pt;
  pcl::getMinMax3D(whole_cloud, min_pt, max_pt);

  std::cout << "点云坐标范围:" << std::endl;
  std::cout << "X: " << min_pt.x << " 到 " << max_pt.x << std::endl;
  std::cout << "Y: " << min_pt.y << " 到 " << max_pt.y << std::endl;
  std::cout << "Z: " << min_pt.z << " 到 " << max_pt.z << std::endl;

  // 判断标准：
  // - 如果坐标值很小（如-100到100米）：局部坐标系
  // - 如果坐标值很大（如几十万米）：UTM全局坐标系
  // - 如果经纬度值（如116.xxx, 39.xxx）：经纬度坐标系

  if (max_pt.x < 1000 && min_pt.x > -1000) {
      std::cout << "这很可能是局部坐标系下的点云" << std::endl;
  } else if (max_pt.x > 100000) {
      std::cout << "这很可能是UTM全局坐标系下的点云" << std::endl;
  }
  
  sensor_msgs::PointCloud2 pub_cloud;
  pcl::toROSMsg(whole_cloud, pub_cloud);
  pub_cloud.header.frame_id = "base";
  
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
  while(ros::ok())
  {
    ros::spinOnce();
  } 
  return 0;
}
