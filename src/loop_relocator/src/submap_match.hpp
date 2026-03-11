#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <iostream>
#include <condition_variable>
#include <sstream>
#include <vector>
#include <Eigen/Core>
#include "include/ikd-Tree/ikd_Tree.h" // Include the iKD-Tree library
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/conversions.h>
#include <ros/ros.h>
#include "include/so3_math.h"
#include <Eigen/Eigen>
#include <signal.h>
#include <csignal>
#include <setjmp.h>
#include <pcl_conversions/pcl_conversions.h>  // 必须添加
#include <pcl/registration/ndt.h>  // 包含NDT头文件
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;

#define MD(a,b)  Eigen::Matrix<double, (a), (b)>
#define VD(a)    Eigen::Matrix<double, (a), 1>
#define MF(a,b)  Eigen::Matrix<float, (a), (b)>
#define VF(a)    Eigen::Matrix<float, (a), 1>
#define NUM_MATCH_POINTS 5
class NDT_MATCH
{
    public:
    NDT_MATCH();
    ~NDT_MATCH();
    Eigen::Matrix4f ndt_match(
        const PointCloudXYZI::Ptr& source_cloud,  // 修改点类型
        const PointCloudXYZI::Ptr& target_cloud,  // 修改点类型
        const Eigen::Matrix4f& initial_guess);
    PointCloudXYZI::Ptr VoxelCloud(
        const PointCloudXYZI::Ptr& cloud, 
        float leaf_size);
};
#define NUM_NEIGHBOR    (5)
class IKD_MATCH
{

    public:

    condition_variable sig_buffer;

    IKD_MATCH();
    ~IKD_MATCH();
    // void h_share_model_relative_pose(
    //     // 输入点云
    //     const PointCloudXYZI::Ptr& source_cloud,  // 源点云
    //     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform, 
    //     std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform_loop,  // 位姿增量
    //     Eigen::Matrix<double, 6, 6>& H,             
    //     Eigen::Matrix<double, 6, 1>& b,
    //     bool& valid                              // 有效性标志
    //     );
    void cloud_matching_init(std::string pcd_file);
    void h_share_model_relative_pose(
        // 输入点云
        const PointCloudXYZI::Ptr& source_cloud,  // 源点云
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform, 
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform_loop,  // 位姿增量
        Eigen::Matrix<double, 6, 1>& dx, // 位姿增量
        bool& valid                              // 有效性标志
        );
    void optimizeLoopTransform(
        const PointCloudXYZI::Ptr& source_cloud,    // 原世界坐标系下的点云
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,  // 位姿增量
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform, // 待优化的T_loop
        int max_iterations,
        double epsilon);
    bool cloud_matching(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> transform,  // 位姿增量
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform// 待优化的T_loop
        );
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> cloud_matching(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud1,
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud2,
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform,  // 位姿增量
        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform// 待优化的T_loop
        );

    bool plane_fit(Eigen::Matrix<float, 3, NUM_NEIGHBOR> A, Eigen::Vector4f &plane_coeff);

    PointCloudXYZI convertToXYZINormal(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input);
    NDT_MATCH* ndt;
    
    
    
    private:
    
    std::mutex mtx;
    KD_TREE<PointType>* ikdtree;
    std::atomic<bool> optimization_active{false};
    std::mutex optimization_mutex;
    std::condition_variable optimization_cv;
    bool if_converged = false;
    bool if_bad_loop = false;
    PointCloudXYZI::Ptr cloud1_msg;
    PointCloudXYZI::Ptr cloud2_msg;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
    bool lidar_pushed, flg_first_scan = true, flg_exit = false;
    float alpha = 1.0;  // learning rate
    float best_feat_rate = 0.0;
    float best_feat_rate_in_all = 0.0;
    Eigen::Matrix4d T_loop = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_loop_in_all = Eigen::Matrix4d::Identity();
    vector<PointVector> Nearest_Points;
   //onst int NUM_MATCH_POINTS = 5; // Number of nearest points to match
    sigjmp_buf env;
 // void SigHandle(int sig);
    PointCloudXYZI::Ptr normvec;
    PointCloudXYZI::Ptr laserCloudOri;
    PointCloudXYZI::Ptr corr_normvect;
    template<typename T>
    bool esti_plane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
    {
        Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
        Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            A(j,0) = point[j].x;
            A(j,1) = point[j].y;
            A(j,2) = point[j].z;
        }

        Eigen::Matrix<T, 3, 1> normvector = A.colPivHouseholderQr().solve(b);

        T n = normvector.norm();
        pca_result(0) = normvector(0) / n;
        pca_result(1) = normvector(1) / n;
        pca_result(2) = normvector(2) / n;
        pca_result(3) = 1.0 / n;

        for (int j = 0; j < NUM_MATCH_POINTS; j++)
        {
            if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
            {
                return false;
            }
        }
        return true;
    }
   

    template <typename T>
    Eigen::Vector3d point2vec(const T &pi) {
    Eigen::Vector3d vec(pi.x, pi.y, pi.z);
    return vec;
    }

    pcl::PointXYZI vec2point(const Eigen::Vector3d &vec) {
        pcl::PointXYZI pi;
        pi.x = vec[0];
        pi.y = vec[1];
        pi.z = vec[2];
        return pi;
    }
    
};

