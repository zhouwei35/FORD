#ifndef __LGC__
#define __LGC__


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <map>

#include <sstream>
#include <fstream>
#include <mutex>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


#include <ceres/ceres.h>
#include <ceres/rotation.h>


#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/pca.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include "tools/nanoflann.hpp"
#include "tools/KDTreeVectorOfVectorsAdaptor.h"


namespace LGC {


using PointType = pcl::PointXYZI;



// 超参数 hyper parameters
typedef struct ConfigSetting {

    // data
    std::string DATASET_NAME = "FORD";
    std::string DATASET_SEQUENCE = "01";
    std::string DATA_FORMAT = "pcd";

    // path
    std::string DATA_ROOT = "/mnt/1t/dataset/FORD/jord_pcd/";
    std::string OUTPUT_ROOT = "/home/jlurobot/桌面/test_lcd/src/LGC/output/";

    // 
    double LIDAR_MAX_VIEW = 80.0;  // 激光雷达最大视距
    double LIDAR_HEIGHT = 1.73;  // 激光雷达高度
    double GRID_SIZE = 1.0;  // 网格边长大小

   

    // for point cloud pre-preocess
    double GROUND_SELECT_THRE = 0.2;  // 区分高低点集的高度差
    int LOW_NEIGHBOUR_NUM = 10;
    double GROUND_FITTING_HEIGHT = 0.2;  // 地面分割高度阈值
    double GROUND_FITTING_DEGREE = 0.9;  // 地面分割角度阈值
    int HIGH_NEIGHBOUR_NUM = 20;
    double TREE_VERTICALITY_THRE = 0.7;  // 树的垂直性阈值
    double CLUSTER_TOLERANCE = 0.6;  // 聚类的最小距离
    int MIN_CLUSTER_SIZE = 10;  // 最小聚类点数

    int DBSCAN_MINPTS = 30;  // 密度聚类定义为核心点所需的最少邻居数量
    double DBSCAN_EPSILON = 0.3;  // 密度聚类的邻域半径

    // for LGC
    int DESCRIPTOR_NEAR_NUM = 5;  // 用于构造三角形的最近点的个数

    double HASH_DELTA_L = 0.1;  // 哈希函数中边长归一化的精度
    int HASH_P = 116101;  // 哈希函数中较大的素数
    int HASH_MAXN = 1000000000;  // 哈希表的大小


    // candidate search
    int SKIP_BEGIN_NUM = 50;  // 跳过最开始的帧数
    int SKIP_NEAR_NUM = 50;  // 搜索时跳过前多少帧
    int CANDIDATE_NUM = 10;  // 候选帧数量

    // for place recognition
    double DIS_THRESHOLD = 0.3;

} ConfigSetting;



// LoopResults结构体，用于存储回环检测的结果
struct LoopResults {
    LoopResults() : loop_id(-1), similarity(1000.0) {};
    LoopResults(int f_id, int l_id = -1, double sim = 1000.0) : frame_id(f_id), loop_id(l_id), similarity(sim) { }
    int frame_id;
    std::vector<int> candidate_list;
    int loop_id;
    double similarity;
    // int shift;
};

// Structure for Stabel Triangle Descriptor
// 定义了一个名为 LGCesc 的结构体，用于存储稳定三角形描述符的相关信息
typedef struct LGCesc {
    unsigned int frame_id;  // 所属帧的ID
    
    Eigen::Vector3d vertex_A;  // 顶点A
    Eigen::Vector3d vertex_B;  // 顶点B
    Eigen::Vector3d vertex_C;  // 顶点C
    
    Eigen::Vector3d side_length;  // LGCesc 的边长，从短到长排列

    Eigen::VectorXd signature;  // 签名
    
    // Eigen::Vector3d angle;  // 顶点之间的投影角度
    // Eigen::Vector3d center;  // 中心坐标
    // Eigen::Vector3d vertex_attached;  // 附加到每个顶点的其他信息，例如强度
} LGCesc;


typedef struct LGCMatchList {
  std::vector<std::pair<LGCesc, LGCesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
} LGCMatchList;


class LGCManager
{

public:
    // ------------------------------------------初始化------------------------------------------
    LGCManager() = default;
    ~LGCManager() = default;

    ConfigSetting config_setting_;  // 配置器

    unsigned int current_frame_id_;  // 帧ID计数器

    LGCManager(ConfigSetting &config_setting) : config_setting_(config_setting) {
        current_frame_id_ = 0;
    };

    // -----------------------------------------对外API接口---------------------------------------
    // --------生成描述符--------
    void GenerateDescriptors(pcl::PointCloud<PointType>::Ptr &input_cloud);

    // --------回环检测--------
    LoopResults SearchLoop();

    // --------将描述符添加到数据库--------
    void UpdateDatabase();


    // ------------------------------------------内部函数-----------------------------------------
    void segmentTrees(pcl::PointCloud<PointType>::Ptr &input_cloud,
                    pcl::PointCloud<PointType>::Ptr &trunk_points,
                    pcl::PointCloud<PointType>::Ptr &ground_points);
    
    void extractClusters(pcl::PointCloud<PointType>::Ptr &input_cloud,
                        std::vector<pcl::PointIndices> &clusters,
                        std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds,
                        std::vector<PointType> &cluster_means);

    void buildCtdAndKey(std::vector<PointType> &cluster_means, std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds);

    void buildAndSaveHashKey(std::vector<LGCesc> &triangles);

    void getIMG(const pcl::PointCloud<PointType>::Ptr &cloud_in);

    void buildAndSaveSnapshot(std::vector<PointType> &cluster_means, std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds);

    void candidateSelector(std::vector<int> &candidate_list);

    void sortAndSelectTopN(const std::vector<int>& unique_indices, const std::vector<int>& repeated_counts, std::vector<int>& top_unique_indices, std::vector<int>& top_repeated_counts, int top_n);

    // ------------------------------------------工具函数-----------------------------------------
    void GetConfigFromYAML(const std::string& config_path);  // 获取参数信息

    void visualizeTriangles(const std::vector<LGCesc> &triangles);  // 可视化生成的三角形


    // ------------------------------------------辅助函数-----------------------------------------


private:
    /* data */
    std::vector<LGCesc> ctds_vec_;
    std::unordered_map<int, std::vector<LGCesc>> hash_table_;
    std::unordered_map<int, std::vector<int>> hash_tables_;
    std::vector<Eigen::MatrixXf> snapshot_mat_;
    std::queue<std::vector<LGCesc>> qe_desc_;
    
    std::vector<Eigen::MatrixXf> imgs_mat_;
    
};




}  // namespace LGC


#endif  // !__LGC__