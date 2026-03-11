#include "submap_match.hpp"
#include <pcl/filters/voxel_grid.h>  // 确保包含体素滤波头文件
/**
 * @brief 使用NDT算法进行点云配准
 * 
 * @param source_cloud 源点云（需要变换的点云）
 * @param target_cloud 目标点云（参考点云）
 * @param initial_guess 初始变换矩阵（4x4）
 * @return Eigen::Matrix4f 优化后的变换矩阵
 */


NDT_MATCH::NDT_MATCH()
{
}

Eigen::Matrix4f NDT_MATCH::ndt_match(
    const PointCloudXYZI::Ptr& source_cloud,  // 修改点类型
    const PointCloudXYZI::Ptr& target_cloud,  // 修改点类型
    const Eigen::Matrix4f& initial_guess)
{
    // 使用PointXYZNormal的NDT对象
    pcl::NormalDistributionsTransform<PointType, PointType> ndt;
    PointCloudXYZI::Ptr output_cloud(new PointCloudXYZI);
    // 设置NDT参数
    ndt.setTransformationEpsilon(0.01);   // 设置变换收敛阈值
    ndt.setStepSize(0.1);                 // 设置牛顿法优化的最大步长
    ndt.setResolution(1.0);               // 设置网格分辨率（米）
    ndt.setMaximumIterations(35);         // 设置最大迭代次数
    
    // 设置输入点云
    std::vector<double> res{10.0, 5.0, 4.0, 3.0};
    for(auto& r : res)
    {
        ndt.setResolution(r);
        // 下采样点云
        auto rough_map1 = VoxelCloud(source_cloud, r*0.1); // 使用0.1米体素
        auto rough_map2 = VoxelCloud(target_cloud, r*0.1);
        ndt.setInputSource(rough_map1);
        ndt.setInputTarget(rough_map2);
        ndt.align(*output_cloud, initial_guess);
        if(ndt.hasConverged())
        {
            std::cout << "NDT converged. Score: " << ndt.getFitnessScore() << std::endl;
        }
    }
    if (ndt.hasConverged()) {
        std::cout << "NDT converged. Score: " << ndt.getFitnessScore() << std::endl;
        return ndt.getFinalTransformation();
    } else {
        std::cerr << "NDT did not converge!" << std::endl;
        return initial_guess;  // 返回初始猜测
    }
}

// 实现VoxelCloud函数
PointCloudXYZI::Ptr NDT_MATCH::VoxelCloud(
    const PointCloudXYZI::Ptr& cloud, 
    float leaf_size) 
{
    PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
    return filtered;
}


IKD_MATCH::IKD_MATCH() : 
mtx(),
ikdtree(new KD_TREE<PointType>()),
ndt(new NDT_MATCH()),
// normvec(new PointCloudXYZI(100000, 1)),
// laserCloudOri(new PointCloudXYZI(100000, 1)),
// corr_normvect(new PointCloudXYZI(100000, 1)),
// cloud1_msg(new PointCloudXYZI(100000, 1)),
// cloud2_msg(new PointCloudXYZI(100000, 1))
normvec(new PointCloudXYZI()),
laserCloudOri(new PointCloudXYZI()),
corr_normvect(new PointCloudXYZI()),
cloud1_msg(new PointCloudXYZI()),
cloud2_msg(new PointCloudXYZI())
{
}  

IKD_MATCH::~IKD_MATCH()
{
    flg_exit = true;
    sig_buffer.notify_all();
    ROS_WARN("Forcefully releasing resources...");
    // 正确写法2：如果需要释放内存，可以遍历清空每个子向量
    for (auto& vec : Nearest_Points) {
        PointVector().swap(vec);
    }
    Nearest_Points.clear();
    if (ikdtree->Root_Node != nullptr) {
        ikdtree->~KD_TREE();
    }
    ros::shutdown();
    std::quick_exit(0);  // 绕过静态对象析构v
}
// void IKD_MATCH::SigHandle(int sig)
// {
//     flg_exit = true;
//     ROS_WARN("catch sig %d", sig);
//     sig_buffer.notify_all();
//     ROS_WARN("Forcefully releasing resources...");
//     // 正确写法2：如果需要释放内存，可以遍历清空每个子向量
//     for (auto& vec : Nearest_Points) {
//         PointVector().swap(vec);
//     }
//     Nearest_Points.clear();
//     if (ikdtree.Root_Node != nullptr) {
//         ikdtree.~KD_TREE();
//     }
//     ros::shutdown();
//     std::quick_exit(0);  // 绕过静态对象析构v
// }
// void IKD_MATCH::h_share_model_relative_pose(
//     // 输入点云
//     const PointCloudXYZI::Ptr& source_cloud,  // 源点云
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform, 
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform_loop,  // 位姿增量
//     Eigen::Matrix<double, 6, 1>& dx, // 位姿增量
//     bool& valid                              // 有效性标志
//     )
// {
//        if (!source_cloud || source_cloud->empty()) {
//             valid = false;
//             return;
//         }
//     PointCloudXYZI::Ptr target_cloud(new PointCloudXYZI());

//     pcl::copyPointCloud(*source_cloud, *target_cloud);
//     laserCloudOri->clear();
//     corr_normvect->clear();
//     double total_residual = 0.0;
//     std::vector<float> res_last(source_cloud->points.size());
//         /* transform to world frame */
//     Eigen::Vector3d translation = transform_loop.first;
//     Eigen::Matrix3d rotation = transform_loop.second;

//     std::vector<bool> point_selected_surf(source_cloud->points.size(), false);
//     /** 1. 最近邻搜索与平面拟合 **/
//     #ifdef MP_EN

//         omp_set_num_threads(MP_PROC_NUM);
//         #pragma omp parallel for

//     #endif
//     for (int i = 0; i < source_cloud->points.size(); i++) {
        
//         PointType& point_source = source_cloud->points[i];
//         PointType point_target; 
        
//         V3D p_source(point_source.x, point_source.y, point_source.z);
//         V3D p_target(rotation * p_source + translation);
//         //V3D p_target(rotation.transpose() * (p_source - translation));
//         point_target.x = p_target(0);
//         point_target.y = p_target(1);
//         point_target.z = p_target(2);
//         point_target.intensity = point_source.intensity;
   
//         vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
//         PointVector &points_near = Nearest_Points[i];

//         //std::cout<< "point"<< i <<":"<< source_cloud->points.size()<< std::endl;

//         ikdtree->Nearest_Search(point_target, NUM_MATCH_POINTS, points_near, pointSearchSqDis);

//         point_selected_surf[i] = (points_near.size() >= NUM_MATCH_POINTS) && 
//                                 (pointSearchSqDis[NUM_MATCH_POINTS-1] < 5.0);

//         if (!point_selected_surf[i]) continue;

//         VF(4) pabcd;
//         point_selected_surf[i] = false;

//         if (esti_plane(pabcd, points_near, 0.1f)) {

//             float pd2 = pabcd(0)*point_target.x + pabcd(1)*point_target.y + pabcd(2)*point_target.z + pabcd(3);
//             V3D p_body (transform.second.transpose()*( p_source - transform.first ) );
//             float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
//             if (s > 0.9) {
//                 point_selected_surf[i] = true;
//                 normvec->points[i].x = pabcd(0);
//                 normvec->points[i].y = pabcd(1);
//                 normvec->points[i].z = pabcd(2);
//                 normvec->points[i].intensity = pd2;

//                 res_last[i] = abs(pd2);

//             }

//         }

//     }
//     double source_size = source_cloud->points.size();
//     /** 2. 筛选有效点 **/
//     int effct_feat_num = 0;
//     for (int i = 0; i < source_cloud->points.size(); i++) {
//         if (point_selected_surf[i]) {
//             laserCloudOri->points[effct_feat_num] = source_cloud->points[i];
//             corr_normvect->points[effct_feat_num] = normvec->points[i];
//             total_residual += res_last[i];
//             effct_feat_num++;
//         }
//     }
//     float feat_rate = effct_feat_num / source_size;
    
//     if(feat_rate < 0.05) {
//         if(if_bad_loop)
//         {
//             valid = false;
//             return;
//         }
        
//         if (T_loop_in_all.block<3, 3>(0, 0) != Eigen::Matrix3d::Identity() && T_loop_in_all.block<3, 1>(0, 3) != Eigen::Vector3d::Zero())
//         {
//             ROS_WARN("No Efficient Points! changed to best the loop in history ");
//             transform_loop.second = T_loop_in_all.block<3, 3>(0, 0);
//             transform_loop.first = T_loop_in_all.block<3, 1>(0, 3);
//         }else{
//             ROS_WARN("No Efficient Points! ");
//         }
//         if_bad_loop = true;
//         return;
//     }else{
//         if_bad_loop = false;
//     }

//     // alpha = (2.5 - alpha) * exp(- feat_rate) * exp(- feat_rate);  // learning rate
//     alpha = 0.5;

//     //min(2.5, source_size /effect_feat_num )  
//     //std::cout << "alpha: " << alpha << std::endl;

//     // if (effct_feat_num < 1) {
//     //     valid = false;
//     //     ROS_WARN("No Effective Points!");
//     //     return;
//     // }

//     double res_mean_last = total_residual / effct_feat_num;
//     //std::cout << "effct_feat_num: " << effct_feat_num << std::endl;
//     //std::cout << "res_mean_last: " << res_mean_last << std::endl;

//     /** 3. 计算相对位姿增量的雅可比和残差 ***/
//     Eigen::MatrixXd h_x = Eigen::MatrixXd::Zero(effct_feat_num, 6);  // 6 = 3平移 + 3旋转
//     Eigen::VectorXd h = Eigen::VectorXd::Zero(effct_feat_num);
//     std::vector<double> weight(effct_feat_num, 0.0);
//     for (int i = 0; i < effct_feat_num; i++) {
//         const PointType& p_source = laserCloudOri->points[i];
//         const PointType& norm_p = corr_normvect->points[i];
        
//         V3D n(norm_p.x, norm_p.y, norm_p.z);
//         n.normalize(); // 归一化法向量
//         // 残差：点到平面的距离（取负）
//         h(i) = norm_p.intensity;
//         // 计算权重
//         weight[i] = 1.0 ;//- fabs(h(i)) / res_mean_last;
//         weight[i] = max ( 0.1 , weight[i] ); // 确保权重不小于0.1 
//         // 雅可比计算（对位姿增量ξ的导数）
//         V3D p(p_source.x, p_source.y, p_source.z);
//                       // 对平移ρ的导数
//                 // 1. 使用当前位姿变换点
//         V3D p_body (transform.second.transpose()*( p- transform.first ) );
//         V3D qs = rotation * p_body;


//         // 3. 计算雅可比矩阵
//         // Eigen::Matrix<double, 1, 6> J;
//         // J.block<1, 3>(0, 0) = n.transpose();             // 平移部分
//         // J.block<1, 3>(0, 3) = (qs.cross(n)).transpose() ; // 旋转部分
//         // // std::cout << "J:"<< J.transpose() << std::endl;
        
//         // // 4. 累积Hessian和梯度
//         // H += J.transpose() * J * weight[i];
//         // b += J.transpose() * h(i) * weight[i];
       
       
       
//         // J.block<1,3>(0,0) = (qs.cross(nt)).transpose(); 
//        // h_x.block<1, 3>(i, 0) = n.transpose().cross(rotation * p);
//         h_x.block<1, 3>(i, 0) = n.transpose(); 
//         h_x.block<1, 3>(i, 3) = (qs.cross(n)).transpose();
//     } 

    
//     Eigen::Matrix<double, 6, 6> H = h_x.transpose() * h_x;
//     Eigen::Matrix<double, 6, 1> b = h_x.transpose() * h;
    

    
//     // 3. 构建线性系统并求解增量
//     dx = H.ldlt().solve(-b);
//     dx = alpha * dx;
//     //std::cout << "dx"<< dx.norm()<< std::endl;
   
//     // 4. 李代数更新位姿
//     //使用指数映射更新旋转
    
//     Eigen::Vector3d upsilon = dx.head<3>();
//     Eigen::Vector3d omega = dx.tail<3>();

//     double theta = omega.norm();
//     Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
//     if (theta > 1e-12) {
//         Eigen::Vector3d axis = omega.normalized();
//         dR = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
//     }

//     // 更新位姿 (注意更新顺序) 
//     // transform_loop.second = dR * transform_loop.second;
//     // transform_loop.first = dR * transform_loop.first + upsilon;
//     transform_loop.second = transform_loop.second * dR;
//     transform_loop.first = transform_loop.first + upsilon;
    
//     if(feat_rate > best_feat_rate)
//     {
//         T_loop.block<3, 3>(0, 0) = transform_loop.second;
//         T_loop.block<3, 1>(0, 3) = transform_loop.first;
//         best_feat_rate = feat_rate;
//     }
//     if(best_feat_rate > best_feat_rate_in_all)
//     {
//         T_loop_in_all.block<3, 3>(0, 0) = T_loop.block<3, 3>(0, 0);
//         T_loop_in_all.block<3, 1>(0, 3) = T_loop.block<3, 1>(0, 3);
//         best_feat_rate_in_all = best_feat_rate;
//     }
// }
bool IKD_MATCH::plane_fit(Eigen::Matrix<float, 3, NUM_NEIGHBOR> A, Eigen::Vector4f &plane_coeff)
{
    Eigen::Vector3f centroid = A.rowwise().mean();
    Eigen::MatrixXf centered = A.colwise() - centroid;
    Eigen::Matrix3f cov = centered * centered.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    Eigen::Vector3f normal = eig.eigenvectors().col(0);
    float d = -normal.dot(centroid);
    Eigen::Vector4f plane_eq;
    plane_coeff << normal, d; // std::cout << "Equation of plane1: " << plane_coeff(0) << "x + " << plane_coeff(1) << "y + " << plane_coeff(2) << "z + " << plane_coeff(3) << " = 0" << std::endl;
    // Compute residuals
    Eigen::VectorXf residuals = centered.transpose() * normal;
    // Check for large residuals
    for (int i = 0; i < residuals.size(); i++)
    {
        if (fabs(residuals(i)) > 0.1f)
            return false;
    }
    return true;
}


// void IKD_MATCH::h_share_model_relative_pose(
//     const PointCloudXYZI::Ptr& source_cloud,
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform_loop,
//     Eigen::Matrix<double, 6, 1>& dx,
//     bool& valid)
// {
//     // 初始化变量
//     valid = false;
//     dx.setZero();
    
//     // 从输入参数中提取位姿信息
//     Eigen::Vector3d transition = transform.first;
//     Eigen::Matrix3d R = transform.second;
//     Eigen::Quaterniond _q(R);
    
//     // 保存原始位姿
//     Eigen::Vector3d transition_original = transition;
//     Eigen::Quaterniond _q_original = _q;
    
//     // 将点云变换到世界坐标系
//     pcl::PointCloud<PointType> laserScan_purtubation;
//     Eigen::Affine3d transform_L2W = Eigen::Affine3d::Identity();
//     transform_L2W.rotate(R);
//     transform_L2W.translation() = transition;
//     pcl::transformPointCloud(*source_cloud, laserScan_purtubation, transform_L2W);
    
//     // 保存原始点云（在Lidar坐标系下）
//     pcl::PointCloud<PointType> laserScan_Lidar;
//     pcl::transformPointCloud(*source_cloud, laserScan_Lidar, Eigen::Affine3d::Identity());
    
//     // 迭代优化
//     bool isconverge = false;
//     int iter_conter = 0;
//     float error_Rt = 999.9;
    
//     while(!isconverge)
//     {
//         // 平面匹配
//         int effct_feat_num = 0;
//         std::vector<Eigen::Vector4f> plane_seq;
//         std::vector<Eigen::Vector3f> world_point_seq;
//         std::vector<Eigen::Vector3f> plane_point_seq;
        
//         float max_square_distance = 5.0;
//         for (int i = 0; i < laserScan_purtubation.size(); i++)
//         {
//             PointType point_world = laserScan_purtubation.points[i];
            
//             PointVector points_near(NUM_NEIGHBOR);
//             vector<float> pointSearchSqDis(NUM_NEIGHBOR);
//             ikdtree->Nearest_Search(point_world, NUM_NEIGHBOR, points_near, pointSearchSqDis);
            
//             if (pointSearchSqDis[NUM_NEIGHBOR - 1] > max_square_distance)
//                 continue;
                
//             Eigen::Matrix<float, 3, NUM_NEIGHBOR> A;
//             for (int j = 0; j < NUM_NEIGHBOR; j++)
//             {
//                 PointType point = points_near.at(j);
//                 A(0, j) = point.x;
//                 A(1, j) = point.y;
//                 A(2, j) = point.z;
//             }
            
//             Eigen::Vector4f plane_coeff;
//             if (!plane_fit(A, plane_coeff))
//                 continue;
                
//             Eigen::Vector3f centroid = A.rowwise().mean();
//             plane_point_seq.push_back(centroid);
//             plane_seq.push_back(plane_coeff);
//             world_point_seq.push_back(Eigen::Vector3f(point_world.x, point_world.y, point_world.z));
//         }
        
//         effct_feat_num = world_point_seq.size();
//         if(effct_feat_num < 1)
//         {
//             valid = false;
//             return;
//         }
        
//         // 构建优化问题
//         Eigen::MatrixXf h_x = Eigen::MatrixXf::Zero(effct_feat_num, 1);
//         Eigen::MatrixXf J_x = Eigen::MatrixXf::Zero(effct_feat_num, 6);
        
//         for (int i = 0; i < effct_feat_num; i++)
//         {
//             Eigen::Vector3f norm = plane_seq[i].block<3, 1>(0, 0);
//             Eigen::Vector3f w_point = world_point_seq[i] - transition.cast<float>();
//             Eigen::Vector3f L_point = _q.inverse().cast<float>() * w_point;
            
//             Eigen::Matrix3f L_point_hat;
//             L_point_hat << 0, -L_point(2), L_point(1),
//                           L_point(2), 0, -L_point(0),
//                           -L_point(1), L_point(0), 0;
                          
//             Eigen::Matrix3f R_q(_q.matrix().cast<float>());
//             Eigen::Vector3f J_tmp_last3term = L_point_hat * R_q.transpose() * norm;
            
//             h_x(i) = norm.transpose() * (world_point_seq[i] - plane_point_seq[i]);
//             J_x.row(i) << norm(0), norm(1), norm(2), 
//                           J_tmp_last3term(0), J_tmp_last3term(1), J_tmp_last3term(2);
//         }
        
//         // 计算误差
//         float error_R = abs(float((_q.matrix().transpose() * _q_original.matrix() - Eigen::Matrix3d::Identity()).trace()));
//         float error_t = abs(float((transition - transition_original).norm()));
//         error_Rt = error_R + error_t;
        
//         // 求解增量
//         Eigen::MatrixXf JTJ = J_x.transpose() * J_x;
//         Eigen::Matrix<float, 6, 1> delta_x = JTJ.ldlt().solve(-J_x.transpose() * h_x);
        
//         // 更新位姿
//         //float alpha = 0.5;  // 学习率
//        // float alpha = 1 - 0.75 * iter_conter * max_iterations;  // 学习率
//         delta_x = alpha * delta_x;
        
//         Eigen::Vector3f delta_phi = delta_x.block<3, 1>(3, 0);
//         Eigen::AngleAxisf delta_phi_left(delta_phi.norm(), delta_phi.normalized());
//         Eigen::Matrix3f delta_R = delta_phi_left.matrix();
//         Eigen::Vector3f delta_t = delta_x.block<3, 1>(0, 0);
        
//         transition += delta_t.cast<double>();
//         _q = _q * delta_R.cast<double>();
//         _q = _q.normalized();
        
//         // 检查收敛条件
//         if(error_Rt > 10.0)
//         {
//             valid = false;
//             return;
//         }
        
//         if(error_Rt < 0.1 || delta_t.norm() < 0.01)
//         {
//             isconverge = true;
//             valid = true;
//         }
        
//         if(iter_conter > 13)
//         {
//             isconverge = true;
//             valid = true;
//         }
        
//         // 更新点云
//         laserScan_purtubation.clear();
//         for(int j = 0; j < laserScan_Lidar.size(); j++)
//         {
//             PointType pt = laserScan_Lidar.at(j);
//             Eigen::Vector3d ptt = {pt.x, pt.y, pt.z};
//             ptt = _q * ptt + transition;
//             PointType ptinworld;
//             ptinworld.x = ptt(0);
//             ptinworld.y = ptt(1);
//             ptinworld.z = ptt(2);
//             laserScan_purtubation.push_back(ptinworld);
//         }
        
//         iter_conter++;
//     }
    
//     // 计算最终的位姿增量
//     Eigen::Affine3d final_transform = Eigen::Affine3d::Identity();
//     final_transform.rotate(_q.matrix());
//     final_transform.translation() = transition;
    
//     Eigen::Affine3d original_transform = Eigen::Affine3d::Identity();
//     original_transform.rotate(_q_original.matrix());
//     original_transform.translation() = transition_original;
    
//     Eigen::Affine3d delta_transform = original_transform.inverse() * final_transform;
    
//     // 将变换增量转换为李代数表示
//     Eigen::AngleAxisd rotation_vector(delta_transform.rotation());
//     dx.head(3) = delta_transform.translation();
//     dx.tail(3) = rotation_vector.axis() * rotation_vector.angle();
    
//     // 设置输出参数
//     transform_loop.first = delta_transform.translation();
//     transform_loop.second = delta_transform.rotation();
    
//     valid = true;
// }


   
// void IKD_MATCH::optimizeLoopTransform(
//     const PointCloudXYZI::Ptr& source_cloud,    // 原世界坐标系下的点云
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,  // 位姿增量
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform, // 待优化的T_loop
//     int max_iterations = 20,
//     double epsilon = 0.01)
// {
//     Eigen::Matrix<double, 6, 1> dx;
//     alpha = 1;
//     for (int i = 0; i < 1; ++i){
//         if_bad_loop = false;
//         alpha = i;
//         for (int iter = 0; iter < max_iterations; ++iter) {
//             // 2. 调用残差计算函数（修改后的h_share_model_relative_pose）
//             bool valid = true;
//             // (初始化Nearest_Points等参数)
//             //std::cout << "00000000000000: "<< i << std::endl;
            
            
//             h_share_model_relative_pose(source_cloud, transform, loop_transform, dx, valid);
                
//             if (!valid) break;
            
//             // 6. 检查收敛条件s
//             if (dx.norm() < 1e-4) {
//                std::cout << "Converged in " << iter << " iterations." << std::endl;
//                 if_converged = true;
//                 break;
//             }
//         }
        
//         if (best_feat_rate< 0.1){
//             ROS_WARN("No Effective Points!");
//             if_converged = false;
//         } else{
//             loop_transform.second = T_loop.block<3, 3>(0, 0);
//             loop_transform.first = T_loop.block<3, 1>(0, 3);
//         }
//     }
//     if (dx.norm() < epsilon) {
 
//         if_converged = true;

//     }
//     if (best_feat_rate< 0.1){
//         ROS_WARN("No Effective Points!");
//         if_converged = false;
//     }

// }


void IKD_MATCH::optimizeLoopTransform(
    const PointCloudXYZI::Ptr& source_cloud,    // 原世界坐标系下的点云
    std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,  // 位姿增量
    std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform, // 待优化的T_loop
    int max_iterations = 20,
    double epsilon=0.001)
{
    // 初始化变量
    bool valid = false;
    double error_Rt = 999.9;
    
    // 从loop_transform中提取位姿信息
    Eigen::Vector3d transition = loop_transform.first;
    Eigen::Matrix3d R = loop_transform.second;
    Eigen::Quaterniond _q(R);
    
    // 保存原始位姿
    Eigen::Vector3d transition_original = transition;
    Eigen::Quaterniond _q_original = _q;
    
    // 将点云变换到世界坐标系
    pcl::PointCloud<PointType> laserScan_purtubation;
    Eigen::Affine3d transform_L2W = Eigen::Affine3d::Identity();
    transform_L2W.rotate(R);
    transform_L2W.translation() = transition;
    pcl::transformPointCloud(*source_cloud, laserScan_purtubation, transform_L2W);
    
    // 保存原始点云（在Lidar坐标系下）
    pcl::PointCloud<PointType> laserScan_Lidar;
    pcl::transformPointCloud(*source_cloud, laserScan_Lidar, Eigen::Affine3d::Identity());
    
    // 迭代优化
    bool isconverge = false;
    int iter_conter = 0;
    
    while(!isconverge && iter_conter < max_iterations)
    {
        // 平面匹配
        int effct_feat_num = 0;
        std::vector<Eigen::Vector4f> plane_seq;
        std::vector<Eigen::Vector3f> world_point_seq;
        std::vector<Eigen::Vector3f> plane_point_seq;
        
        float max_square_distance = 5.0;
        for (int i = 0; i < laserScan_purtubation.size(); i++)
        {
            PointType point_world = laserScan_purtubation.points[i];
            
            PointVector points_near(NUM_NEIGHBOR);
            vector<float> pointSearchSqDis(NUM_NEIGHBOR);
            ikdtree->Nearest_Search(point_world, NUM_NEIGHBOR, points_near, pointSearchSqDis);
            
            if (pointSearchSqDis[NUM_NEIGHBOR - 1] > max_square_distance)
                continue;
                
            Eigen::Matrix<float, 3, NUM_NEIGHBOR> A;
            for (int j = 0; j < NUM_NEIGHBOR; j++)
            {
                PointType point = points_near.at(j);
                A(0, j) = point.x;
                A(1, j) = point.y;
                A(2, j) = point.z;
            }
            
            Eigen::Vector4f plane_coeff;
            if (!plane_fit(A, plane_coeff))
                continue;
                
            Eigen::Vector3f centroid = A.rowwise().mean();
            plane_point_seq.push_back(centroid);
            plane_seq.push_back(plane_coeff);
            world_point_seq.push_back(Eigen::Vector3f(point_world.x, point_world.y, point_world.z));
        }
        
        effct_feat_num = world_point_seq.size();
        // ROS_WARN_STREAM("Effective Feature Number: " << effct_feat_num);
        if(effct_feat_num < 1)
        {
            // 没有找到足够的匹配平面，优化失败
            valid = false;
            break;
        }
        
        // 构建优化问题
        Eigen::MatrixXf h_x = Eigen::MatrixXf::Zero(effct_feat_num, 1);
        Eigen::MatrixXf J_x = Eigen::MatrixXf::Zero(effct_feat_num, 6);
        
        for (int i = 0; i < effct_feat_num; i++)
        {
            Eigen::Vector3f norm = plane_seq[i].block<3, 1>(0, 0);
            Eigen::Vector3f w_point = world_point_seq[i] - transition.cast<float>();
            Eigen::Vector3f L_point = _q.inverse().cast<float>() * w_point;
            
            Eigen::Matrix3f L_point_hat;
            L_point_hat << 0, -L_point(2), L_point(1),
                          L_point(2), 0, -L_point(0),
                          -L_point(1), L_point(0), 0;
                          
            Eigen::Matrix3f R_q(_q.matrix().cast<float>());
            Eigen::Vector3f J_tmp_last3term = L_point_hat * R_q.transpose() * norm;
            
            h_x(i) = norm.transpose() * (world_point_seq[i] - plane_point_seq[i]);
            J_x.row(i) << norm(0), norm(1), norm(2), 
                          J_tmp_last3term(0), J_tmp_last3term(1), J_tmp_last3term(2);
        }
        
        // 计算误差
        float error_R = abs(float((_q.matrix().transpose() * _q_original.matrix() - Eigen::Matrix3d::Identity()).trace()));
        float error_t = abs(float((transition - transition_original).norm()));
        error_Rt = error_R + error_t;
        
        // 求解增量
        Eigen::MatrixXf JTJ = J_x.transpose() * J_x;
        Eigen::Matrix<float, 6, 1> delta_x = JTJ.ldlt().solve(-J_x.transpose() * h_x);
        
        // 更新位姿
        //float alpha = 0.5;  // 学习率
        float alpha = 1 - 0.75 * iter_conter / max_iterations;  // 学习率
        delta_x = alpha * delta_x;
        
        Eigen::Vector3f delta_phi = delta_x.block<3, 1>(3, 0);
        Eigen::AngleAxisf delta_phi_left(delta_phi.norm(), delta_phi.normalized());
        Eigen::Matrix3f delta_R = delta_phi_left.matrix();
        Eigen::Vector3f delta_t = delta_x.block<3, 1>(0, 0);
        
        transition += delta_t.cast<double>();
        _q = _q * delta_R.cast<double>();
        _q = _q.normalized();
        
        // 检查收敛条件
        if(error_Rt > 10.0)
        {
            // 误差过大，优化失败
            valid = false;
            break;
        }
        
        if((delta_phi.norm() + delta_t.norm() < epsilon)|| iter_conter == max_iterations-1)
        {
            // 收敛，优化成功
            isconverge = true;
            ROS_WARN_STREAM("delta_phi.norm(): " << delta_phi.norm());
            ROS_WARN_STREAM("delta_t.norm(): " << delta_t.norm());
            ROS_WARN_STREAM("epsilon" << epsilon );
            ROS_WARN_STREAM("iter_conter: " << iter_conter);
            if((delta_phi.norm() + delta_t.norm() < 0.02))
            {
                if_converged = true;
            }
            valid = true;
        }
        
        // 更新点云
        laserScan_purtubation.clear();
        for(int j = 0; j < laserScan_Lidar.size(); j++)
        {
            PointType pt = laserScan_Lidar.at(j);
            Eigen::Vector3d ptt = {pt.x, pt.y, pt.z};
            ptt = _q * ptt + transition;
            PointType ptinworld;
            ptinworld.x = ptt(0);
            ptinworld.y = ptt(1);
            ptinworld.z = ptt(2);
            laserScan_purtubation.push_back(ptinworld);
        }
        // ROS_WARN_STREAM("isconverge: " << isconverge);
        // ROS_WARN_STREAM("iter_conter: " << iter_conter);
        
        iter_conter++;
    }
    
    // 计算最终的位姿增量
    if(valid)
    {
        // 计算相对于原始位姿的变换增量
        Eigen::Affine3d final_transform = Eigen::Affine3d::Identity();
        final_transform.rotate(_q.matrix());
        final_transform.translation() = transition;
        
        Eigen::Affine3d original_transform = Eigen::Affine3d::Identity();
        original_transform.rotate(_q_original.matrix());
        original_transform.translation() = transition_original;
        
        Eigen::Affine3d delta_transform = original_transform.inverse() * final_transform;
        
        // 将变换增量转换为李代数表示
        Eigen::AngleAxisd rotation_vector(delta_transform.rotation());
        Eigen::Matrix<double, 6, 1> dx;
        dx.head(3) = delta_transform.translation();
        dx.tail(3) = rotation_vector.axis() * rotation_vector.angle();
        
        // 更新输出参数
        transform.first = delta_transform.translation();
        transform.second = delta_transform.rotation();
        
        // 更新优化后的位姿
        loop_transform.first = transition;
        loop_transform.second = _q.matrix();
    }
    else
    {
        // 优化失败，保持原始位姿不变
        transform.first = Eigen::Vector3d::Zero();
        transform.second = Eigen::Matrix3d::Identity();
    }
}


PointCloudXYZI IKD_MATCH::convertToXYZINormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) {
    PointCloudXYZI output;
    output.reserve(input->size());
    
    for (const auto& pt : *input) {
        PointType new_pt;
        new_pt.x = pt.x;
        new_pt.y = pt.y;
        new_pt.z = pt.z;
        new_pt.intensity = pt.intensity;
        new_pt.normal_x = 0;
        new_pt.normal_y = 0;
        new_pt.normal_z = 0;
        new_pt.curvature = 0;
        output.push_back(new_pt);
    }
    return output;
}
void IKD_MATCH::cloud_matching_init(std::string pcd_file)
{
    std::cout << "cloud_matching pcd file path: " << pcd_file<< std::endl;
    if (ikdtree->Root_Node == nullptr) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(
            new pcl::PointCloud<pcl::PointXYZI>());
        
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud1);
        
        *cloud1_msg = convertToXYZINormal(cloud1);
        int feats_size = cloud1_msg->points.size();
        if (feats_size > 5) {
            ikdtree->set_downsample_param(0.5);
            ikdtree->Build(cloud1_msg->points);
            ROS_INFO("Built new IKD-Tree with %d points", feats_size);
        } else {
            ROS_WARN("Insufficient points to build IKD-Tree: %d", feats_size);
            return;
        }
    }
   // std::cout << "222222222222222222222222" << std::endl;
    int featsFromMapNum = ikdtree->validnum();
    int kdtree_size_st = ikdtree->size();
     // Load point cloud from pcd file
    
}
bool IKD_MATCH::cloud_matching(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> transform,  // 位姿增量
    std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform// 待优化的T_loop
    )
{
    best_feat_rate = 0.0;
    if_converged = false;
    T_loop = Eigen::Matrix4d::Identity();
   
    *cloud2_msg = convertToXYZINormal(cloud2);

    int feats_size = cloud2_msg->points.size();

    if (feats_size < 5)
    {
        ROS_WARN("No point, skip this scan!\n");

        return false;
    }

    //icp精匹配
    Nearest_Points.resize(cloud2_msg->points.size());
    normvec->resize(cloud2_msg->points.size());
    int rematch_num = 0;
    bool nearest_search_en = true; //
    std::cout << "icp_matching" << std::endl;
    /*** iterated state estimation ***/
    optimizeLoopTransform(cloud2_msg,    // 原世界坐标系下的点云
        transform,
        loop_transform);

    {
        std::unique_lock<std::mutex> lock(optimization_mutex);
        optimization_cv.wait(lock, [this] {
            return !optimization_active;
        });
    }
    std::cout<<"if_converged: "<<if_converged<<std::endl;
    if(if_converged)
    {
        if_converged = false;
        return true;
        
    }
    else
    {
        return false;
    } 
     
}


//    std::pair<Eigen::Vector3d, Eigen::Matrix3d> IKD_MATCH::cloud_matching(
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
//     const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,  // 位姿增量
//     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform// 待优化的T_loop
//     )
// {
//     best_feat_rate = 0.0;
//     if_converged = false;
//     T_loop = Eigen::Matrix4d::Identity();
//     std::cout << "cloud_matching" << std::endl;
//     *cloud1_msg = convertToXYZINormal(cloud1);
//     *cloud2_msg = convertToXYZINormal(cloud2);
//    // std::cout << "00000000000000000000000" << std::endl;
//     //重置IKD-Tree
//     if (ikdtree->Root_Node != nullptr) {
//         delete ikdtree;
//         ikdtree = new KD_TREE<PointType>(); 
//     }
//     /*** initialize the map kdtree ***/
//     int feats_size = cloud1_msg->points.size();
//    // std::cout << "111111111111111111111111" << std::endl;
//     if (ikdtree->Root_Node == nullptr) {
//         if (feats_size > 5) {
//             //ikdtree->set_downsample_param(filter_size_map_min);
//             ikdtree->Build(cloud1_msg->points);
//             ROS_INFO("Built new IKD-Tree with %d points", feats_size);
//         } else {
//             ROS_WARN("Insufficient points to build IKD-Tree: %d", feats_size);
//             return std::make_pair(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
//         }
//     }
//    // std::cout << "222222222222222222222222" << std::endl;
//     int featsFromMapNum = ikdtree->validnum();
//     int kdtree_size_st = ikdtree->size();

//     // cout << "[ mapping ]: In num: " << feats_undistort->points.size() << " downsamp " << feats_down_size << " Map num: " << featsFromMapNum << "effect num:" << effct_feat_num << endl;

//     if (feats_size < 5)
//     {
//         ROS_WARN("No point, skip this scan!\n");
//         loop_transform = std::pair<Eigen::Vector3d, Eigen::Matrix3d>(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());     
        

//         return loop_transform;
//     }
//   //  std::cout << "333333333333333333" << std::endl;
//     //  if (0) // If you need to see map point, change to "if(1)"
//     //  {
//     //      PointVector().swap(ikdtree.PCL_Storage);
//     //      ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
//     //      featsFromMap->clear();
//     //      featsFromMap->points = ikdtree.PCL_Storage;
//     //  }

//     Nearest_Points.resize(cloud2_msg->points.size());
//     normvec->resize(cloud2_msg->points.size());
//     int rematch_num = 0;
//     bool nearest_search_en = true; //
//    // std::cout << "4444444444444444444444444444" << std::endl;
//     /*** iterated state estimation ***/
//     optimizeLoopTransform(cloud2_msg,    // 原世界坐标系下的点云
//         transform,
//         loop_transform);
//     //    std::cout << "5555555555555555555555555" << std::endl;

//     {
//         std::unique_lock<std::mutex> lock(optimization_mutex);
//         optimization_cv.wait(lock, [this] {
//             return !optimization_active;
//         });
//     }
//     std::cout<<"if_converged: "<<if_converged<<std::endl;
//    // std::cout << "6666666666666666666666666666666" << std::endl;
//     if(if_converged)
//     {
//         if_converged = false;
        
//     }
//     else
//     {
//         loop_transform = std::pair<Eigen::Vector3d, Eigen::Matrix3d>(Eigen::Vector3d::Zero(), Eigen::Matrix3d::Identity());
//     } 
//     return loop_transform; 
// }