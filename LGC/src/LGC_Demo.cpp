#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>

// #include <vector>
// #include <unordered_map>
// #include <unordered_set>

// #include <boost/program_options.hpp>
#include <yaml-cpp/yaml.h>

#include "sensor_msgs/PointCloud2.h"

// #include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tools/data_loader.h"

#include "LGC/LGC.h"

using namespace std;
using namespace pcl;
using namespace Eigen;

using PointType = pcl::PointXYZI;

void get_normals1(pcl::PointCloud<PointType>::Ptr points, int neighbour_nums, std::vector<Eigen::Vector3f> &normals, std::vector<float> &curvature) {
    pcl::KdTreeFLANN<PointType> kdtree;  // 声明一个 k-d 树对象
    kdtree.setInputCloud(points);  // 将输入点云设置为 k-d 树的输入
    std::vector<int> point_KNN_idx(neighbour_nums);  // 存储邻近点索引的向量
    std::vector<float> point_KNN_dis(neighbour_nums);  // 存储邻近点距离平方的向量

    normals.resize(points->size());  // 调整法向量数组大小为点云大小
    curvature.resize(points->size());  // 调整曲率数组大小为点云大小

    for (size_t i = 0; i < points->size(); ++i) {  // 对于每个点
        kdtree.nearestKSearch(points->points[i], neighbour_nums + 1, point_KNN_idx, point_KNN_dis);  // 进行最近邻搜索

        Eigen::MatrixXf neighbors(neighbour_nums, 3);  // 声明一个矩阵来存储邻近点
        for (int j = 1; j <= neighbour_nums; ++j) {  // 从第二个邻近点开始，因为第一个是自身
            neighbors(j - 1, 0) = points->points[point_KNN_idx[j]].x - points->points[i].x;  // 计算邻近点与当前点的 x 坐标差
            neighbors(j - 1, 1) = points->points[point_KNN_idx[j]].y - points->points[i].y;  // 计算邻近点与当前点的 y 坐标差
            neighbors(j - 1, 2) = points->points[point_KNN_idx[j]].z - points->points[i].z;  // 计算邻近点与当前点的 z 坐标差
        }

        Eigen::Matrix3f covariance_matrix = neighbors.transpose() * neighbors / neighbour_nums;  // 计算协方差矩阵
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance_matrix, Eigen::ComputeEigenvectors);  // 对协方差矩阵进行特征值分解
        Eigen::Vector3f eigen_values = solver.eigenvalues();  // 获取特征值
        Eigen::Matrix3f eigen_vectors = solver.eigenvectors();  // 获取特征向量

        normals[i] = eigen_vectors.col(0);  // 法向量是最小特征值对应的特征向量
        curvature[i] = eigen_values(0) / eigen_values.sum();  // 曲率是最小特征值除以特征值的总和
    }
}

void pcsegdist1(pcl::PointCloud<PointType>::Ptr points, float minDistance, int minPoints, int maxPoints, std::vector<int> &labels, std::vector<float> &score)
{
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(points);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(minDistance);
    ec.setMinClusterSize(minPoints);
    ec.setMaxClusterSize(maxPoints);
    ec.setSearchMethod(tree);
    ec.setInputCloud(points);
    ec.extract(cluster_indices);

    labels.resize(points->points.size(), -1);
    score.clear();

    int label = 0;
    for (const auto &indices : cluster_indices)
    {
        score.push_back(indices.indices.size());
        for (const int &idx : indices.indices)
        {
            labels[idx] = label;
        }
        ++label;
    }
}


Eigen::MatrixXf my_circshift1(const Eigen::MatrixXf& matrix, int shift) {
    // 获取矩阵的列数
    int num_cols = matrix.cols();
    
    // 如果shift为0，直接返回原矩阵
    if (shift == 0) {
        return matrix;
    }
    
    // 计算有效的移位数
    shift = shift % num_cols;
    
    // 分割矩阵并合并形成移位后的矩阵
    Eigen::MatrixXf shifted_matrix(matrix.rows(), num_cols);
    shifted_matrix << matrix.rightCols(shift), matrix.leftCols(num_cols - shift);
    
    return shifted_matrix;
}



float corr_sim_P1(const MatrixXf& ndd1_input, const MatrixXf& ndd2) {
    // Get dimensions of ndd1
    int num_rings = ndd1_input.rows();
    int num_sectors = ndd1_input.cols();

    // Create a copy of ndd1_input to work with
    MatrixXf ndd1 = ndd1_input;

    // Initialize array to store geometric similarities
    VectorXf geo_sim(num_sectors);
    geo_sim.setZero();

 
    // Compute row vector sum for ndd2
    VectorXf row_vec2 = ndd2.colwise().sum();

    for (int i = 0; i < num_sectors; ++i) {
        // Perform circular shift on ndd1
        int one_step = 1; // const
        ndd1 = my_circshift1(ndd1, one_step);

        // Compute row vector sum for shifted ndd1
        VectorXf row_vec1 = ndd1.colwise().sum();
        // Calculate geometric similarity
        geo_sim(i) = row_vec1.dot(row_vec2) / (row_vec1.norm() * row_vec2.norm());
    }
  

    cout<<geo_sim.transpose()<<endl;
    // Find index of maximum geometric similarity
    int shiftcol;
    geo_sim.maxCoeff(&shiftcol);
    
    // Perform circular shift on ndd1 based on optimal shift
    ndd1 = my_circshift1(ndd1, shiftcol+1);

    // Calculate correlation similarity
    MatrixXf a = ndd1;
    MatrixXf b = ndd2;
    a.array() -= a.mean();
    b.array() -= b.mean();
    float corr_similarity = (a.array() * b.array()).sum() / sqrt((a.array() * a.array()).sum() * (b.array() * b.array()).sum());

    return corr_similarity;
}






void getIMG1(const pcl::PointCloud<PointType>::Ptr &cloud_in, Eigen::MatrixXf &img)
{
    pcl::PointCloud<PointType>::Ptr ptcloud1(new pcl::PointCloud<PointType>);
    *ptcloud1 = *cloud_in;

    // 调用函数 get_normals 计算点云的法线和曲率，并将结果存储在 Normal_cpu 和 Curvature 中
    std::vector<Eigen::Vector3f> Normal_cpu;
    std::vector<float> Curvature;
    get_normals1(ptcloud1, 10, Normal_cpu, Curvature);

/*
    for (size_t i = 0; i < Normal_cpu.size(); ++i) {
        std::cout << "Normal " << i << ": [" 
                  << Normal_cpu[i][0] << ", " 
                  << Normal_cpu[i][1] << ", " 
                  << Normal_cpu[i][2] << "]" << std::endl;
    }
*/
    Eigen::Vector3f z(0, 0, 1);
    float height = 30;
    Eigen::MatrixXf A(ptcloud1->points.size(), 3);

    for (size_t i = 0; i < ptcloud1->points.size(); ++i)
    {
        A.row(i) << ptcloud1->points[i].x, ptcloud1->points[i].y, ptcloud1->points[i].z;
    }

    float verticality = 0.7;
    Eigen::VectorXf v(Normal_cpu.size());
    for (size_t i = 0; i < Normal_cpu.size(); ++i)
    {
        v[i] = 1 - abs(Normal_cpu[i].dot(z));
    }
    //std::cout << "v: " << v.transpose() << std::endl;


    pcl::PointCloud<PointType>::Ptr tree(new pcl::PointCloud<PointType>);
    for (size_t i = 0; i < v.size(); ++i)
    {
        if (v[i] > verticality)
        {
            tree->points.push_back(ptcloud1->points[i]);
        }
    }

    /*
    std::cout << "Filtered tree point cloud:" << std::endl;
    for (size_t i = 0; i < tree->points.size(); ++i) {
        std::cout << "Point " << i << ": [" 
                  << tree->points[i].x << ", " 
                  << tree->points[i].y << ", " 
                  << tree->points[i].z << ", "
                  << tree->points[i].intensity << "]" << std::endl;
    }
    */

    // 对过滤后的点云进行主成分分析（PCA），得到特征向量和特征值
    pcl::PCA<PointType> pca;
    pca.setInputCloud(tree);
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector3f eigenValues = pca.getEigenValues();

    // 根据特征值确定方向向量 fd
    Eigen::Vector3f fd;
    if (eigenValues(1) / 3 > eigenValues(2))
    {
        fd = eigenVectors.col(2);
    }
    else if (eigenValues(0) / 3 > eigenValues(1))
    {
        fd = eigenVectors.col(0);
    }
    else
    {
        fd = Eigen::Vector3f(1, 0, 0);
    }
/*
    std::cout << "Direction vector fd: [" 
              << fd[0] << ", " 
              << fd[1] << ", " 
              << fd[2] << "]" << std::endl;
*/
    // 使用特征向量构造旋转矩阵 R，并调整方向向量 fd
    Eigen::Matrix3f R;
    R.col(0) = eigenVectors.col(0);
    R.col(1) = eigenVectors.col(1);
    R.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    fd = R * fd;

    
    // 计算偏航角并构造相应的旋转矩阵 A_rotation
    float yaw = std::atan(fd(1)/fd(0)) + M_PI / 2;

    Eigen::Matrix3f A_rotation;
    A_rotation << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;

    // 使用旋转矩阵对过滤后的点云 tree 进行变换
    Eigen::MatrixXf ptcloud_tran(tree->points.size(), 3);
    for (size_t i = 0; i < tree->points.size(); ++i)
    {
        ptcloud_tran.row(i) << tree->points[i].x, tree->points[i].y, tree->points[i].z;
    }



    ptcloud_tran = ptcloud_tran * A_rotation;
    

/*
    

*/

    // 定义最大范围、扇区数和环数。计算每个点的极坐标 theta 和 r
    float max_range = 80; // meter
    int num_sector = 60;
    int num_ring = 20;

    Eigen::VectorXf theta(ptcloud_tran.rows());
    Eigen::VectorXf r(ptcloud_tran.rows());
    pcl::PointCloud<pcl::PointXYZI> ptcloud_tran_pcl;
    for (int i = 0; i < ptcloud_tran.rows(); ++i)
    {
        pcl::PointXYZI point;
        point.x = ptcloud_tran(i, 0);
        point.y = ptcloud_tran(i, 1);
        point.z = ptcloud_tran(i, 2);
        ptcloud_tran_pcl.push_back(point);

        float x = ptcloud_tran(i, 0);
        float y = ptcloud_tran(i, 1);
        theta[i] = atan2(y, x) + M_PI;
        r[i] = sqrt(x * x + y * y);
       // std::cout << "Point " << i << ": theta = " << theta[i] << ", r = " << r[i] << std::endl;
    }


    // 根据极坐标将点分组到环和扇区中，得到环索引 r_index 和扇区索引 s_index
    float r_step = max_range / num_ring;
    Eigen::VectorXi r_index = ((r / r_step).array().ceil()).matrix().cast<int>();
    r_index = r_index.cwiseMin(num_ring);

    float s_step = (360.0 / num_sector) * M_PI / 180.0;
    float eps = 1e-6; // Define epsilon for floating-point comparisons
    theta = (theta.array() > 2 * M_PI - eps).select(0, theta);
    Eigen::VectorXi s_index = ((theta / s_step).array().ceil()).matrix().cast<int>();
    s_index = s_index.cwiseMax(1);



    // 创建一个全零初始化的图像矩阵 img，将点云分割到环和扇区中，并根据每个区域的点数计算一个得分
    //Eigen::MatrixXf img = Eigen::MatrixXf::Zero(num_ring, num_sector);
    //std::vector<std::vector<pcl::PointCloud<PointType>::Ptr>> segPoint(num_ring, std::vector<pcl::PointCloud<PointType>::Ptr>(num_sector, pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>)));
    for (int i = 1; i <= num_ring; ++i)
    {
        for (int j = 1; j <= num_sector; ++j)
        {
            std::vector<int> idx;
            for (int k = 0; k < r_index.size(); ++k) {
                if (r_index(k) == i && s_index(k) == j) {
                    idx.push_back(k);
                }
            }
            pcl::PointCloud<PointType>::Ptr point(new pcl::PointCloud<PointType>);
            for (int k : idx) {
                point->points.push_back(ptcloud_tran_pcl.points[k]);
            }


/*
            // 打印 point
            std::cout << "Ring " << i << ", Sector " << j << " has " << point->points.size() << " points." << std::endl;
            for (size_t k = 0; k < point->points.size(); ++k) {
                std::cout << "Point " << k << ": ["
                          << point->points[k].x << ", "
                          << point->points[k].y << ", "
                          << point->points[k].z << ", "
                          << point->points[k].intensity << "]" << std::endl;
            }
*/
            if (point->size() > 4)
            {
                float minDistance = 1.0f; // 聚类的最大距离
                int minPoints = 1;   // 聚类的最小点数
                int maxPoints = 1000;  // 聚类的最大点数

                                std::vector<int> labels;
                std::vector<float> score;
                pcsegdist1(point, minDistance, minPoints, maxPoints, labels, score); // You need to implement pcsegdist function
                img(i - 1, j - 1) = score.size();
/*
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
                tree->setInputCloud(point);

                std::vector<pcl::PointIndices> cluster_indices;
                pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
                ec.setClusterTolerance(minDistance);
                ec.setMinClusterSize(minPoints);
                ec.setMaxClusterSize(maxPoints);
                ec.setSearchMethod(tree);
                ec.setInputCloud(point);
                ec.extract(cluster_indices);

                int score = cluster_indices.size();

                img(i - 1, j - 1) = score;
*/
            }
            else
            {
                img(i - 1, j - 1) = 0;
            }
        }
    }

/*
    for (int i = 0; i < img.rows(); ++i) {
        for (int j = 0; j < img.cols(); ++j) {
            std::cout << img(i, j) << " ";
        }
        std::cout << std::endl;
    }

    std::ofstream file("img_output.txt");
    if (file.is_open()) {
        for (int i = 0; i < img.rows(); ++i) {
            for (int j = 0; j < img.cols(); ++j) {
                file << img(i, j);
                if (j < img.cols() - 1) {
                    file << " ";
                }
            }
            file << std::endl;
        }
        file.close();
    } else {
        std::cerr << "Unable to open file " << std::endl;
    }
*/


}

int main_test()
{
    LGC::ConfigSetting config_setting;

    std::string config_path = "/home/zw/桌面/LGC(0710)/catkin_ws/src/LGC/config/config_ford.yaml";
    try
    {
        // 加载 YAML 文件
        YAML::Node config_file = YAML::LoadFile(config_path);

        // 获取参数
        config_setting.DATASET_NAME = config_file["dataset_name"].as<std::string>();
        config_setting.DATASET_SEQUENCE = config_file["dataset_sequence"].as<std::string>();
        config_setting.DATA_FORMAT = config_file["data_format"].as<std::string>();

        config_setting.DATA_ROOT = config_file["data_root"].as<std::string>();
        config_setting.OUTPUT_ROOT = config_file["output_root"].as<std::string>();

        config_setting.LIDAR_MAX_VIEW = config_file["lidar_max_view"].as<double>(); // 激光雷达最大视距
        config_setting.LIDAR_HEIGHT = config_file["lidar_height"].as<double>();     // 激光雷达高度
        config_setting.GRID_SIZE = config_file["grid_size"].as<double>();           // 网格边长大小

        config_setting.GROUND_SELECT_THRE = config_file["ground_select_thre"].as<double>(); // 区分高低点集的高度差
        config_setting.LOW_NEIGHBOUR_NUM = config_file["low_neighbour_num"].as<int>();
        config_setting.GROUND_FITTING_HEIGHT = config_file["ground_fitting_height"].as<double>(); // 地面分割高度阈值
        config_setting.GROUND_FITTING_DEGREE = config_file["ground_fitting_degree"].as<double>(); // 地面分割角度阈值
        config_setting.HIGH_NEIGHBOUR_NUM = config_file["high_neighbour_num"].as<int>();
        config_setting.TREE_VERTICALITY_THRE = config_file["tree_verticality_thre"].as<double>(); // 树的垂直性阈值
        config_setting.CLUSTER_TOLERANCE = config_file["cluster_tolerance"].as<double>();         // 聚类的最小距离
        config_setting.MIN_CLUSTER_SIZE = config_file["min_cluster_size"].as<int>();              // 最小聚类点数

        config_setting.DBSCAN_MINPTS = config_file["dbscan_minpts"].as<int>();      // 密度聚类定义为核心点所需的最少邻居数量
        config_setting.DBSCAN_EPSILON = config_file["dbscan_epsilon"].as<double>(); // 密度聚类的邻域半径

        config_setting.DESCRIPTOR_NEAR_NUM = config_file["descriptor_near_num"].as<int>(); // 用于构造三角形的最近点的个数

        config_setting.HASH_DELTA_L = config_file["hash_delta_l"].as<double>(); // 哈希函数中边长归一化的精度
        config_setting.HASH_P = config_file["hash_p"].as<int>();                // 哈希函数中较大的素数
        config_setting.HASH_MAXN = config_file["hash_maxn"].as<int>();          // 哈希表的大小

        config_setting.SKIP_BEGIN_NUM = config_file["skip_begin_num"].as<int>(); // 跳过最开始的帧数
        config_setting.SKIP_NEAR_NUM = config_file["skip_near_num"].as<int>();   // 搜索时跳过前多少帧
        config_setting.CANDIDATE_NUM = config_file["candidate_num"].as<int>();   // 候选帧数量

        config_setting.DIS_THRESHOLD = config_file["dis_threshold"].as<double>();
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
        return 1;
    }

    using PointType = pcl::PointXYZI;

    std::string pc_path_1 = "/home/zw/桌面/FORD2/FORD01/velodyne/222.pcd";
    std::string pc_path_2 = "/home/zw/桌面/FORD2/FORD01/velodyne/2348.pcd";
    pcl::PointCloud<PointType>::Ptr pc_1(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr pc_2(new pcl::PointCloud<PointType>);

    if (pcl::io::loadPCDFile<PointType>(pc_path_1, *pc_1) == -1)
    {
        PCL_ERROR("Couldn't read PCD file.\n");
        return -1;
    }
    if (pcl::io::loadPCDFile<PointType>(pc_path_2, *pc_2) == -1)
    {
        PCL_ERROR("Couldn't read PCD file.\n");
        return -1;
    }
    Eigen::MatrixXf img1 = Eigen::MatrixXf::Zero(20, 60);
    Eigen::MatrixXf img2 = Eigen::MatrixXf::Zero(20, 60);
    LGC::LGCManager LGC(config_setting);

    LGC.current_frame_id_ = 1759;
    // LGC.GenerateDescriptors(pc_1);
    getIMG1(pc_1, img1);
    //cout << 1 << endl;
    // LGC::LGCManager LGC1(config_setting);
    // LGC1.current_frame_id_ = 2348;
    // LGC1.GenerateDescriptors(pc_2);
    //LGC.current_frame_id_ = 2348;
    // LGC.GenerateDescriptors(pc_2);
     getIMG1(pc_2, img2);
   
     float dis = corr_sim_P1(img2, img1);
     cout << dis << endl;

    return 0;
}

// 测量两个时间点之间的时间间隔，返回以毫秒为单位的时间差
double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
                std::chrono::_V2::system_clock::time_point &t_begin)
{
    // std::chrono::duration_cast<std::chrono::duration<double>> 将时间点之间的持续时间（时间差）转换为一个以双精度浮点数表示的持续时间。
    // t_end - t_begin 计算了两个时间点之间的持续时间，.count()返回持续时间的计数（数量），然后乘以 1000，将持续时间转换为毫秒。
    return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count() * 1000;
}
/**/
int main_LCD()
{

    LGC::ConfigSetting config_setting;
    std::string config_path = "/home/zw/桌面/LGC(0710)/catkin_ws/src/LGC/config/config_ford.yaml";
    try
    {
        // 加载 YAML 文件
        YAML::Node config_file = YAML::LoadFile(config_path);

        // 获取参数
        config_setting.DATASET_NAME = config_file["dataset_name"].as<std::string>();
        config_setting.DATASET_SEQUENCE = config_file["dataset_sequence"].as<std::string>();
        config_setting.DATA_FORMAT = config_file["data_format"].as<std::string>();

        config_setting.DATA_ROOT = config_file["data_root"].as<std::string>();
        config_setting.OUTPUT_ROOT = config_file["output_root"].as<std::string>();

        config_setting.LIDAR_MAX_VIEW = config_file["lidar_max_view"].as<double>(); // 激光雷达最大视距
        config_setting.LIDAR_HEIGHT = config_file["lidar_height"].as<double>();     // 激光雷达高度
        config_setting.GRID_SIZE = config_file["grid_size"].as<double>();           // 网格边长大小

        config_setting.GROUND_SELECT_THRE = config_file["ground_select_thre"].as<double>(); // 区分高低点集的高度差
        config_setting.LOW_NEIGHBOUR_NUM = config_file["low_neighbour_num"].as<int>();
        config_setting.GROUND_FITTING_HEIGHT = config_file["ground_fitting_height"].as<double>(); // 地面分割高度阈值
        config_setting.GROUND_FITTING_DEGREE = config_file["ground_fitting_degree"].as<double>(); // 地面分割角度阈值
        config_setting.HIGH_NEIGHBOUR_NUM = config_file["high_neighbour_num"].as<int>();
        config_setting.TREE_VERTICALITY_THRE = config_file["tree_verticality_thre"].as<double>(); // 树的垂直性阈值
        config_setting.CLUSTER_TOLERANCE = config_file["cluster_tolerance"].as<double>();         // 聚类的最小距离
        config_setting.MIN_CLUSTER_SIZE = config_file["min_cluster_size"].as<int>();              // 最小聚类点数

        config_setting.DBSCAN_MINPTS = config_file["dbscan_minpts"].as<int>();      // 密度聚类定义为核心点所需的最少邻居数量
        config_setting.DBSCAN_EPSILON = config_file["dbscan_epsilon"].as<double>(); // 密度聚类的邻域半径

        config_setting.DESCRIPTOR_NEAR_NUM = config_file["descriptor_near_num"].as<int>(); // 用于构造三角形的最近点的个数

        config_setting.HASH_DELTA_L = config_file["hash_delta_l"].as<double>(); // 哈希函数中边长归一化的精度
        config_setting.HASH_P = config_file["hash_p"].as<int>();                // 哈希函数中较大的素数
        config_setting.HASH_MAXN = config_file["hash_maxn"].as<int>();          // 哈希表的大小

        config_setting.SKIP_BEGIN_NUM = config_file["skip_begin_num"].as<int>(); // 跳过最开始的帧数
        config_setting.SKIP_NEAR_NUM = config_file["skip_near_num"].as<int>();   // 搜索时跳过前多少帧
        config_setting.CANDIDATE_NUM = config_file["candidate_num"].as<int>();   // 候选帧数量

        config_setting.DIS_THRESHOLD = config_file["dis_threshold"].as<double>();
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Failed to load YAML file: " << e.what() << std::endl;
        return 1;
    }

    // 使用参数
    std::string dataset_name = config_setting.DATASET_NAME;
    std::string dataset_sequence = config_setting.DATASET_SEQUENCE;
    std::string data_format = config_setting.DATA_FORMAT;
    std::string data_root = config_setting.DATA_ROOT;
    std::string output_root = config_setting.OUTPUT_ROOT;

    std::cout << "dataset_name: " << dataset_name << std::endl;
    std::cout << "dataset_sequence: " << dataset_sequence << std::endl;
    std::cout << "data_format: " << data_format << std::endl;
    std::cout << "data_root: " << data_root << std::endl;
    std::cout << "output_root: " << output_root << std::endl;

    using PointType = pcl::PointXYZI;

    std::string data_path = data_root + dataset_sequence + "/";
    std::cout << "data_path: " << data_path << std::endl;

    vector<DataFile> file_list;
    DataLoader data_loader;

    if (data_format == "bin")
    {
        if (data_loader.kitti_bin_loader(data_path, dataset_sequence, file_list) == -1)
        {
            printf("No file loaded.\n");
            return -1;
        }
    }
    else if (data_format == "pcd")
    {
        if (data_loader.jord_pcd_loader(data_path, file_list) == -1)
        {
            printf("No file loaded.\n");
            return -1;
        }
    }

    std::string loop_file_name = output_root + "LGC" + "_" + dataset_name + "_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);
    if (!loop_file.is_open())
    {
        printf("Outputfile can't open.\n");
        return -1;
    }

    std::cout << file_list.size() << std::endl;

    // 创建计时器容器
    std::vector<double> descriptor_time;
    std::vector<double> querying_time;
    std::vector<double> update_time;
    int loop_num = 0; // 回环数量

    LGC::LGCManager LGC(config_setting);

    for (size_t cloud_i = 0; cloud_i < file_list.size(); ++cloud_i)
    {
        std::string cloud_path = file_list[cloud_i].file_name;
        int cloud_number = file_list[cloud_i].order_number;

        pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 定义为Ptr，以便在两种情况中都适用
        if (data_format == "bin")
        {
            pc_to_read = readKITTIPointCloudBin<PointType>(cloud_path);
        }
        else if (data_format == "pcd")
        {
            if (pcl::io::loadPCDFile<PointType>(cloud_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                return -1;
            }
        }
        else
        {
            PCL_ERROR("Unsupported data format: %s\n", data_format.c_str());
            return -1;
        }
        pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
        // pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

        // std::cout << "Successfully read the No." << cloud_i << " point cloud, cloud size: " << pc_to_read->points.size() << std::endl;
        std::cout << "Frame id:" << cloud_i << ", cloud size: " << pc_to_read->points.size() << std::endl;

        // step1: 提取描述符
        auto t_descriptor_begin = std::chrono::high_resolution_clock::now();
        LGC.GenerateDescriptors(pc_to_read);
        auto t_descriptor_end = std::chrono::high_resolution_clock::now();
        descriptor_time.push_back(time_inc(t_descriptor_end, t_descriptor_begin));

        // step2: 搜索回环
        double min_dis = 1000.00;
        int loop_frame_id = -1;
        auto t_query_begin = std::chrono::high_resolution_clock::now();
        LGC::LoopResults search_result(cloud_number);
        if ((int)cloud_i >= config_setting.SKIP_BEGIN_NUM)
        { // 前50帧不考虑回环
            // search_result = LGC.SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair);
            LGC::LoopResults search_result = LGC.SearchLoop();
            loop_frame_id = search_result.loop_id;
            min_dis = search_result.similarity;
        }
        // 如果找到回环
        /*
        if (loop_frame_id >= 0) {
            std::cout << "[Loop Detection] Frame id: " << cloud_i << " --- Find loop: "
                    << search_result.loop_id << ", score: " << search_result.similarity
                    << std::endl;
        } else {
            min_dis = 0.0;
        }
        */
        if (cloud_number < config_setting.SKIP_BEGIN_NUM)
        {
            min_dis = 0.0;
        }
        auto t_query_end = std::chrono::high_resolution_clock::now();
        querying_time.push_back(time_inc(t_query_end, t_query_begin));

        // step3: 更新数据库
        auto t_update_begin = std::chrono::high_resolution_clock::now();
        LGC.UpdateDatabase();
        auto t_update_end = std::chrono::high_resolution_clock::now();
        update_time.push_back(time_inc(t_update_end, t_update_begin));

        // 打印时间信息
        // std::cout << "[Time] descriptor extraction: "
        //             << time_inc(t_descriptor_end, t_descriptor_begin) << "ms, "
        //             << "query: " << time_inc(t_query_end, t_query_begin) << "ms, "
        //             << "update: "
        //             << time_inc(t_update_end, t_update_begin) << "ms"
        //             << std::endl;
        // std::cout << std::endl;

        std::cout << "No." << cloud_number << ", loop_frame_id: " << loop_frame_id << ", dist: " << min_dis << std::endl;
        loop_file << cloud_number << " " << loop_frame_id << " " << (float)min_dis << std::endl;
    }

    loop_file.close();

    // 计算平均时间
    double mean_descriptor_time =
        std::accumulate(descriptor_time.begin(), descriptor_time.end(), 0) * 1.0 /
        descriptor_time.size();
    double mean_query_time =
        std::accumulate(querying_time.begin(), querying_time.end(), 0) * 1.0 /
        querying_time.size();
    double mean_update_time =
        std::accumulate(update_time.begin(), update_time.end(), 0) * 1.0 /
        update_time.size();

    // 打印结果
    std::cout << "Total key frame number:" << file_list.size()
              << ", loop number:" << loop_num << std::endl;
    std::cout << "Mean time for descriptor extraction: " << mean_descriptor_time
              << "ms, query: " << mean_query_time
              << "ms, update: " << mean_update_time << "ms, total: "
              << mean_descriptor_time + mean_query_time + mean_update_time << "ms"
              << std::endl;

    return 0;
}

int main(int argc, char const *argv[])
{
   // main_test();
    main_LCD();
    return 0;
}
