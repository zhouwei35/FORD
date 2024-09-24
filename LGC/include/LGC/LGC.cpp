#include "LGC.h"



using namespace LGC;
using namespace std;
using namespace pcl;
using namespace Eigen;

void visualizeClusters(const std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds) {
    // 创建 PCLVisualizer 对象
    pcl::visualization::PCLVisualizer viewer("Cluster visualization");

    // 设置不同颜色
    std::vector<std::string> colors = {"red", "green", "blue", "yellow", "cyan", "magenta"};

    // 可视化每个点云簇
    for (size_t i = 0; i < clustered_clouds.size(); ++i) {
        pcl::PointCloud<PointType>::Ptr cluster = clustered_clouds[i];
        std::string cloud_name = "cluster_" + std::to_string(i);

        // 给每个点云簇指定不同的颜色
        pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(
            cluster, 
            (i * 50) % 256,  // 红色分量
            (i * 80) % 256,  // 绿色分量
            (i * 110) % 256  // 蓝色分量
        );

        viewer.addPointCloud<PointType>(cluster, color_handler, cloud_name);
    }

    // 设置点云属性
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    // 运行视图
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}


void LGCManager::GenerateDescriptors(pcl::PointCloud<PointType>::Ptr &input_cloud)
{
    getIMG(input_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr trunk_points(new pcl::PointCloud<PointType>);  // 树干点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<PointType>);  // 地面点云
    // 执行树的分割操作，并获取分割结果
    segmentTrees(input_cloud, trunk_points, ground_points);
    
    // 提取聚类点云
    std::vector<pcl::PointIndices> trunk_clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> trunk_clustered_clouds;
    std::vector<pcl::PointXYZI> cluster_means;
    extractClusters(trunk_points, trunk_clusters, trunk_clustered_clouds, cluster_means);
    //std::cout << "Size of trunk_clusters: " << trunk_clusters.size() << std::endl;
    //std::cout << "Size of trunk_clustered_clouds: " << trunk_clustered_clouds.size() << std::endl;
    //std::cout << "Size of cluster_means: " << cluster_means.size() << std::endl;

    // pcl::io::savePCDFileASCII("ground_points.pcd", *ground_points);
    // std::cout << "trunk_points have been saved to trunk_points.pcd" << std::endl;

        // 保存每个 trunk_clustered_cloud 到单独的 PCD 文件
        // 合并所有 trunk_clustered_clouds 到一个点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& cluster_cloud : trunk_clustered_clouds) {
        *combined_cloud += *cluster_cloud;
    }

    // 保存合并后的点云到 PCD 文件
    //pcl::io::savePCDFileASCII("combined_trunk_clusters.pcd", *combined_cloud);
    //std::cout << "All trunk_clustered_clouds have been saved to combined_trunk_clusters.pcd" << std::endl;


    /*
        // 打印 cluster_means
    std::cout << "Cluster Means:" << std::endl;
    for (const auto& point : cluster_means) {
        std::cout << point.x << ", " << point.y << ", " << point.z << std::endl;
    }

    // 将 cluster_means 写入到 txt 文件中

    std::ofstream file("cluster_means.txt");
    if (file.is_open()) {
        for (const auto& point : cluster_means) {
            file << point.x << ", " << point.y << ", " << point.z << std::endl;
        }
        file.close();
        std::cout << "Cluster means have been written to cluster_means.txt" << std::endl;
    } else {
        std::cerr << "Unable to open file for writing" << std::endl;
    }
    */

/*
    std::string output_path = config_setting_.OUTPUT_ROOT;
    if (pcl::io::savePCDFile(output_path + "trunk_points/trunk_points_" + std::to_string(current_frame_id_) + ".pcd", *trunk_points) == -1)
    {
        PCL_ERROR("Error saving point cloud to file %s.\n", (output_path + "trunk_points/trunk_points_" + std::to_string(current_frame_id_) + ".pcd").c_str());
    } else {
        std::cout << "Successfully saved point cloud to file " << output_path + "trunk_points/trunk_points_" + std::to_string(current_frame_id_) + ".pcd" << std::endl;
    }
*/
    // 可视化点云聚类结果
    //visualizeClusters(trunk_clustered_clouds);

    // 构造描述符和Key
    buildCtdAndKey(cluster_means, trunk_clustered_clouds);
}

void get_normals(pcl::PointCloud<PointType>::Ptr points, int neighbour_nums, std::vector<Eigen::Vector3f> &normals, std::vector<float> &curvature)
{
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



void LGCManager::segmentTrees(pcl::PointCloud<PointType>::Ptr &input_cloud,
                    pcl::PointCloud<PointType>::Ptr &trunk_points,
                    pcl::PointCloud<PointType>::Ptr &ground_points)
{
    // 读取并转换点云数据
    // int points_num = input_cloud->size();
    const double lidar_max_view = config_setting_.LIDAR_MAX_VIEW;  // 激光雷达最大视距
    const double grid_size = config_setting_.GRID_SIZE;  // 网格大小

    int num_grids = static_cast<int>(std::ceil((lidar_max_view * 2) / grid_size));
    std::vector<std::vector<std::vector<PointType>>> grid_set(num_grids, std::vector<std::vector<PointType>>(num_grids));

    // 初始化低点集和高点集
    pcl::PointCloud<PointType>::Ptr low_points(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr high_points(new pcl::PointCloud<PointType>);

    // 计算每个点的网格索引并分配到对应的网格中
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        const auto& point = input_cloud->points[i];

        if (point.x < -lidar_max_view || point.x >= lidar_max_view || point.y < -lidar_max_view || point.y >= lidar_max_view) {
            continue;  // 过滤掉超出视距范围的点
        }
        
        int row_index = static_cast<int>((point.x + lidar_max_view) / grid_size);
        int col_index = static_cast<int>((point.y + lidar_max_view) / grid_size);
        
        // 将点分配到对应的网格中
        grid_set[row_index][col_index].push_back(point);
    }

    // 分析每个网格中的点，根据 z 坐标范围将点分类为地面点或树木点
    const double ground_select_thre = config_setting_.GROUND_SELECT_THRE;
    for (int i = 0; i < num_grids; ++i) {
        for (int j = 0; j < num_grids; ++j) {
            if (!grid_set[i][j].empty()) {
                double z_min = std::numeric_limits<double>::max();  // double类型的最大值
                double z_max = std::numeric_limits<double>::lowest();  // double类型的最小值
                
                for (const auto& point : grid_set[i][j]) {
                    if (point.z < z_min) z_min = point.z;
                    if (point.z > z_max) z_max = point.z;
                }
                
                if ((z_max - z_min) <= ground_select_thre) {
                    for (const auto& point : grid_set[i][j]) {
                        low_points->push_back(point);
                    }
                } else {
                    for (const auto& point : grid_set[i][j]) {
                        high_points->push_back(point);
                    }
                }
            }
        }
    }

    // 从low_points继续中分离出地面点和非地面点
    // 判断地面点的标准：点的高度要非常接近地面 + 点的表面几乎是水平的（法向量与z轴几乎平行）
    std::vector<Eigen::Vector3f> low_normals;  // 法向量的向量
    std::vector<float> low_curvature;  // 曲率的向量
    const int low_neighbour_num = config_setting_.LOW_NEIGHBOUR_NUM;
    // 调用 get_normals 函数计算地面点的法向量和曲率
    get_normals(low_points, low_neighbour_num, low_normals, low_curvature);
    // Eigen::Vector3f z_unit(0, 0, 1);  // 定义一个z轴方向（垂直向上）的单位向量
    Eigen::Vector3f z_unit = Eigen::Vector3f::UnitZ(); // 创建一个单位z轴向量 (0, 0, 1)
    const double ground_fitting_height = config_setting_.GROUND_FITTING_HEIGHT;
    const double ground_fitting_degree = config_setting_.GROUND_FITTING_DEGREE;
    for (size_t i = 0; i < low_points->size(); ++i) {
        // 计算法向量与z轴（即垂直方向）点积的绝对值，该值反映了法向量与z轴之间的角度
        // 值接近1表示法向量几乎与z轴对齐，而接近0则表示法向量与z轴几乎垂直
        double levelDegree = abs(z_unit.dot(low_normals[i]));
        // 如果 z 坐标小于等于 0.2 且法向量几乎垂直于 z 轴，则认为是地面点
        if (low_points->points[i].z <= ground_fitting_height && levelDegree >= ground_fitting_degree) {
            ground_points->push_back(low_points->points[i]);
        } else {
            high_points->push_back(low_points->points[i]);
        }
    }

    // 提取树干
    pcl::PointCloud<PointType>::Ptr tree_points(new pcl::PointCloud<PointType>);

    std::vector<Eigen::Vector3f> high_normals;  // 法向量的向量
    std::vector<float> high_curvature;  // 曲率的向量
    const int high_neighbour_num = config_setting_.HIGH_NEIGHBOUR_NUM;
    get_normals(high_points, high_neighbour_num, high_normals, high_curvature);
    // 初始化树的垂直性参数
    const double tree_verticality_thre = config_setting_.TREE_VERTICALITY_THRE;

    // 计算所有点的垂直性
    for (size_t i = 0; i < high_points->size(); ++i) {
        double verticality = 1.0 - abs(z_unit.dot(high_normals[i]));
        if (verticality >= tree_verticality_thre) {
            tree_points->push_back(high_points->points[i]);
        }
    }
    
    // 使用欧式距离聚类进行树干提取
    pcl::search::KdTree<PointType>::Ptr tree_search(new pcl::search::KdTree<PointType>);
    tree_search->setInputCloud(tree_points);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    const double cluster_tolerance = config_setting_.CLUSTER_TOLERANCE;  // 聚类的最小距离
    const int min_cluster_size = config_setting_.MIN_CLUSTER_SIZE;  // 最小聚类点数
    ec.setClusterTolerance(cluster_tolerance);  // 设置聚类的容差
    ec.setMinClusterSize(min_cluster_size);  // 设置最小聚类尺寸
    // ec.setMaxClusterSize(250);   // 设置最大聚类尺寸
    ec.setSearchMethod(tree_search);
    ec.setInputCloud(tree_points);
    ec.extract(cluster_indices);

    // 筛选有效的点集
    pcl::PointIndices::Ptr valid_points(new pcl::PointIndices);
    for (const auto &indices : cluster_indices) {
        for (const auto &index : indices.indices) {
            valid_points->indices.push_back(index);
        }
    }

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(tree_points);
    extract.setIndices(valid_points);
    extract.filter(*trunk_points);

    /*
    std::string output_path = config_setting_.OUTPUT_ROOT;
    if (pcl::io::savePCDFile(output_path + "trunk_points/trunk_points_" + std::to_string(current_frame_id_) + ".pcd", *trunk_points) == -1)
    {
        PCL_ERROR("Error saving point cloud to file %s.\n", (output_path + "trunk_points/trunk_points_" + std::to_string(current_frame_id_) + ".pcd").c_str());
    } else {
        std::cout << "Successfully saved point cloud to file " << output_path + "trunk_points/trunk_points_" + std::to_string(current_frame_id_) + ".pcd" << std::endl;
    }

    if (pcl::io::savePCDFile(output_path + "ground_points/ground_points_" + std::to_string(current_frame_id_) + ".pcd", *ground_points) == -1)
    {
        PCL_ERROR("Error saving point cloud to file %s.\n", (output_path + "ground_points/ground_points_" + std::to_string(current_frame_id_) + ".pcd").c_str());
    } else {
        std::cout << "Successfully saved point cloud to file " << output_path + "ground_points/ground_points_" + std::to_string(current_frame_id_) + ".pcd" << std::endl;
    }
    */
    
}


void LGCManager::extractClusters(pcl::PointCloud<PointType>::Ptr &input_cloud,
                      std::vector<pcl::PointIndices> &clusters,
                      std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds,
                      std::vector<PointType> &cluster_means) {
    
    clusters.clear();
    clustered_clouds.clear();
    cluster_means.clear();

    int points_num = input_cloud->size();

    // 创建只包含x和y维度的临时点云
    pcl::PointCloud<PointType>::Ptr xy_cloud(new pcl::PointCloud<PointType>(*input_cloud));
    for (auto& point : xy_cloud->points) {
        point.z = 0;
    }

    // DBSCAN 聚类
    // 初始化点云和聚类参数
    const int dbscan_MinPts = config_setting_.DBSCAN_MINPTS;
    const double dbscan_epsilon = config_setting_.DBSCAN_EPSILON;

    std::vector<int> cluster_ids(points_num, -1);  // 每个店对应的聚类id（-1表示未分类，-2表示噪音点）
    // pcl::search::KdTree<PointType> kd_tree;
    pcl::KdTreeFLANN<PointType> kd_tree;
    kd_tree.setInputCloud(xy_cloud);

    int cluster_num = 0;  // 当前聚类编号
    for (int i = 0; i < points_num; ++i) {
        if (cluster_ids[i] == -1) {
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_sqr_dis;
            kd_tree.radiusSearch(xy_cloud->points[i], dbscan_epsilon, neighbor_indices, neighbor_sqr_dis);

            if ((int)neighbor_indices.size() < dbscan_MinPts) {  // 噪音点
                cluster_ids[i] = -2;
                continue;
            }

            cluster_ids[i] = cluster_num;

            std::queue<int> search_qe;
            for (const int &neighbor : neighbor_indices) {
                if (neighbor != i) {
                    search_qe.push(neighbor);
                }
            }

            while (!search_qe.empty()) {
                int curr_idx = search_qe.front();
                search_qe.pop();

                if (cluster_ids[curr_idx] == -1) {
                    kd_tree.radiusSearch(xy_cloud->points[curr_idx], dbscan_epsilon, neighbor_indices, neighbor_sqr_dis);

                    if ((int)neighbor_indices.size() >= dbscan_MinPts) {
                        for (const int &next_idx : neighbor_indices) {
                            if (next_idx != curr_idx && cluster_ids[next_idx] == -1) {
                                search_qe.push(next_idx);
                            }
                        }
                        cluster_ids[curr_idx] = cluster_num;
                    } else {  // 噪音点
                        cluster_ids[curr_idx] = -2;
                    }
                }
            }
            ++cluster_num;
        }
    }

    clusters.resize(cluster_num);
    for (int i = 0; i < points_num; ++i) {
        if (cluster_ids[i] >= 0) {
            clusters[cluster_ids[i]].indices.push_back(i);
        }
    }

    for (const auto &cluster_indices : clusters) {
        pcl::PointCloud<PointType>::Ptr clustered_cloud(new pcl::PointCloud<PointType>);
        double sum_x = 0;
        double sum_y = 0;
        double sum_z = 0;
        for (const auto &index : cluster_indices.indices) {
            clustered_cloud->push_back(input_cloud->points[index]);
            sum_x += input_cloud->points[index].x;
            sum_y += input_cloud->points[index].y;
            sum_z += input_cloud->points[index].z;
        }
        PointType mean_point;
        mean_point.x = sum_x / cluster_indices.indices.size();
        mean_point.y = sum_y / cluster_indices.indices.size();
        mean_point.z = sum_z / cluster_indices.indices.size();
        cluster_means.push_back(mean_point);
        clustered_clouds.push_back(clustered_cloud);
    }
}

// 可视化生成的三角形
void LGCManager::visualizeTriangles(const std::vector<LGCesc> &triangles)
{
    // 保存三角形数据到 txt 文件
    std::string file_path = config_setting_.OUTPUT_ROOT + "triangles/triangles_" + std::to_string(current_frame_id_) + ".txt";
    std::ofstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << file_path << std::endl;
        return;
    }
    for (const auto& triangle : triangles) {
        file << triangle.vertex_A.x() << " " << triangle.vertex_A.y() << " " << triangle.vertex_A.z() << " "
             << triangle.vertex_B.x() << " " << triangle.vertex_B.y() << " " << triangle.vertex_B.z() << " "
             << triangle.vertex_C.x() << " " << triangle.vertex_C.y() << " " << triangle.vertex_C.z() << "\n";
    }
    file.close();

    // 生成Python脚本调用命令
    std::string script_path = "/home/zw/桌面/LGC(0710)/catkin_ws/src/LGC/scripts/plot_triangles.py"; // 根据实际路径设置
    std::string command = "python3 " + script_path + " " + file_path;

    // 在C++中调用Python脚本来绘制三角形
    std::system(command.c_str());
}

void pcsegdist(pcl::PointCloud<PointType>::Ptr points, float minDistance, int minPoints, int maxPoints, std::vector<int> &labels, std::vector<float> &score)
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
    for (const auto &indices : cluster_indices) {
        score.push_back(indices.indices.size());
        for (const int &idx : indices.indices) {
            labels[idx] = label;
        }
        ++label;
    }
}

void LGCManager::getIMG(const pcl::PointCloud<PointType>::Ptr &cloud_in)
{
    pcl::PointCloud<PointType>::Ptr ptcloud2(new pcl::PointCloud<PointType>);
    *ptcloud2 = *cloud_in;
   //  降采样  垂直度   F1-score
   //  0.1  0  94%
   //  0.3  0.5 92%
   //  0.7  0   94%
   //  0.9  0.7  90%
     pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(ptcloud2);
    sor.setLeafSize(0.9f, 0.9f, 0.9f); // 设置体素大小
    pcl::PointCloud<PointType>::Ptr ptcloud1(new pcl::PointCloud<PointType>);
    sor.filter(*ptcloud1);
    //cout<<"-------------"<<endl;
   // cout<<"原来："<<ptcloud2->size()<<"采样后："<<ptcloud1->size()<<endl;

    // 调用函数 get_normals 计算点云的法线和曲率，并将结果存储在 Normal_cpu 和 Curvature 中
    std::vector<Eigen::Vector3f> Normal_cpu;
    std::vector<float> Curvature;
    get_normals(ptcloud1, 10, Normal_cpu, Curvature);



    Eigen::Vector3f z(0, 0, 1);
    float height = 30;
    Eigen::MatrixXf A(ptcloud1->points.size(), 3);

    for (size_t i = 0; i < ptcloud1->points.size(); ++i) {
        A.row(i) << ptcloud1->points[i].x, ptcloud1->points[i].y, ptcloud1->points[i].z;
    }

    
    float verticality = 0.7;
    Eigen::VectorXf v(Normal_cpu.size());
    for (size_t i = 0; i < Normal_cpu.size(); ++i) {
        v[i] = 1 - abs(Normal_cpu[i].dot(z));
    }

    pcl::PointCloud<PointType>::Ptr tree(new pcl::PointCloud<PointType>);
    for (size_t i = 0; i < v.size(); ++i) {
        if (v[i] > verticality) {
            tree->points.push_back(ptcloud1->points[i]);
        }
    }

    // 对过滤后的点云进行主成分分析（PCA），得到特征向量和特征值
    pcl::PCA<PointType> pca;
    pca.setInputCloud(tree);
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector3f eigenValues = pca.getEigenValues();

    // 根据特征值确定方向向量 fd
    Eigen::Vector3f fd;
    if (eigenValues(1) / 3 > eigenValues(2)) {
        fd = eigenVectors.col(2);
    } else if (eigenValues(0) / 3 > eigenValues(1)) {
        fd = eigenVectors.col(0);
    } else {
        fd = Eigen::Vector3f(1, 0, 0);
    }

    // 使用特征向量构造旋转矩阵 R，并调整方向向量 fd
    Eigen::Matrix3f R;
    R.col(0) = eigenVectors.col(0);
    R.col(1) = eigenVectors.col(1);
    R.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    fd = R * fd;

    // 计算偏航角并构造相应的旋转矩阵 A_rotation
    float yaw = atan2(fd(1), fd(0)) + M_PI / 2;
    Eigen::Matrix3f A_rotation;
    A_rotation << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;

    // 使用旋转矩阵对过滤后的点云 tree 进行变换
    Eigen::MatrixXf ptcloud_tran(tree->points.size(), 3);
    for (size_t i = 0; i < tree->points.size(); ++i) {
        ptcloud_tran.row(i) << tree->points[i].x, tree->points[i].y, tree->points[i].z;
    }
    ptcloud_tran = ptcloud_tran * A_rotation.transpose();

    // 定义最大范围、扇区数和环数。计算每个点的极坐标 theta 和 r
    float max_range = 80; // meter
    int num_sector = 60;
    int num_ring = 20;

    Eigen::VectorXf theta(ptcloud_tran.rows());
    Eigen::VectorXf r(ptcloud_tran.rows());
    for (int i = 0; i < ptcloud_tran.rows(); ++i)
    {
        float x = ptcloud_tran(i, 0);
        float y = ptcloud_tran(i, 1);
        theta[i] = atan2(y, x) + M_PI;
        r[i] = sqrt(x * x + y * y);
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
    Eigen::MatrixXf img = Eigen::MatrixXf::Zero(num_ring, num_sector);
    std::vector<std::vector<pcl::PointCloud<PointType>::Ptr>> segPoint(num_ring, std::vector<pcl::PointCloud<PointType>::Ptr>(num_sector, pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>)));
    for (int i = 1; i <= num_ring; ++i) {
        for (int j = 1; j <= num_sector; ++j) {
            std::vector<int> idx;
            for (int k = 0; k < r_index.size(); ++k) {
                if (r_index(k) == i && s_index(k) == j) {
                    idx.push_back(k);
                }
            }
            pcl::PointCloud<PointType>::Ptr point(new pcl::PointCloud<PointType>);
            for (int k : idx) {
                point->points.push_back(ptcloud1->points[k]);
            }
            if (point->size() > 4) {
                float minDistance = 1; // clustering max distance
                int minPoints = 1;       // clustering min points
                int maxPoints = 1000;    // clustering max points
                std::vector<int> labels;
                std::vector<float> score;
                pcsegdist(point, minDistance, minPoints, maxPoints, labels, score); // You need to implement pcsegdist function
                img(i - 1, j - 1) = score.size();
            } else {
                img(i - 1, j - 1) = 0;
            }
        }
    }
    snapshot_mat_.push_back(img);
}


Eigen::MatrixXf my_circshift(const Eigen::MatrixXf& matrix, int shift) {
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


float corr_sim_P(const Eigen::MatrixXf& ndd1_input, const Eigen::MatrixXf& ndd2)
{
    // Get dimensions of ndd1
    int num_rings = ndd1_input.rows();
    int num_sectors = ndd1_input.cols();

    // Create a copy of ndd1_input to work with
    Eigen::MatrixXf ndd1 = ndd1_input;

    // Initialize array to store geometric similarities
    VectorXf geo_sim(num_sectors);
    geo_sim.setZero();

 
    // Compute row vector sum for ndd2
    VectorXf row_vec2 = ndd2.colwise().sum();

    for (int i = 0; i < num_sectors; ++i) {
        // Perform circular shift on ndd1
        int one_step = 1; // const
        ndd1 = my_circshift(ndd1, one_step);

        // Compute row vector sum for shifted ndd1
        VectorXf row_vec1 = ndd1.colwise().sum();
        // Calculate geometric similarity
        geo_sim(i) = row_vec1.dot(row_vec2) / (row_vec1.norm() * row_vec2.norm());
    }
  

    // cout<<geo_sim.transpose()<<endl;
    // Find index of maximum geometric similarity
    int shiftcol;
    geo_sim.maxCoeff(&shiftcol);
    
    // Perform circular shift on ndd1 based on optimal shift
    ndd1 = my_circshift(ndd1, shiftcol+1);

    // Calculate correlation similarity
    MatrixXf a = ndd1;
    MatrixXf b = ndd2;
    a.array() -= a.mean();
    b.array() -= b.mean();
    float corr_similarity = (a.array() * b.array()).sum() / sqrt((a.array() * a.array()).sum() * (b.array() * b.array()).sum());

    return 1-corr_similarity;
}

Eigen::Vector3d computeSideLengths(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& C) {
    double AB = (A - B).norm();
    double BC = (B - C).norm();
    double CA = (C - A).norm();

    Eigen::Vector3d side_length(AB, BC, CA);
    std::sort(side_length.data(), side_length.data() + 3);

    return side_length;
}

bool isValidTriangle(const LGCesc& triangle) {
    const Eigen::Vector3d& side_length = triangle.side_length;

    if (side_length[0] == side_length[1] || side_length[1] == side_length[2] || side_length[0] == side_length[2]) {
        return false;
    }

    if (side_length[0] == 0 || side_length[1] == 0 || side_length[2] == 0) {
        return false;
    }

    return true;
}

// 计算哈希键
int computeHashKey(const LGCesc &ctdesc, const double &delta_l, const int &p, const int &B)
{
    // 获取三角形的边长
    double l1 = ctdesc.side_length(0);
    double l2 = ctdesc.side_length(1);
    double l3 = ctdesc.side_length(2);
    int bar_l1 = round(l1 / delta_l);
    int bar_l2 = round(l2 / delta_l);
    int bar_l3 = round(l3 / delta_l);
    return ((bar_l3 * p + bar_l2) * p % B + bar_l1) % B;
}

void LGCManager::buildAndSaveSnapshot(std::vector<PointType> &cluster_means, std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds)
{
    
}

void LGCManager::buildCtdAndKey(std::vector<PointType> &cluster_means, std::vector<pcl::PointCloud<PointType>::Ptr> &clustered_clouds)
{
    // 极角排序
    std::sort(cluster_means.begin(), cluster_means.end(), [&](const PointType& a, const PointType& b) {
        if (atan2(a.y, a.x) != atan2(b.y,b.x))
            return atan2(a.y, a.x) < atan2(b.y, b.x);
        else
            return a.x < b.x;
    });


    // 构建KD树
    Eigen::MatrixXd points(cluster_means.size(), 2);
    for (size_t i = 0; i < cluster_means.size(); ++i) {
        points(i, 0) = cluster_means[i].x;
        points(i, 1) = cluster_means[i].y;
        // points(i, 2) = cluster_means[i].z;
    }

    // KD树索引类型
    typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, -1, nanoflann::metric_L2> KDTree;
    KDTree kd_tree(2, points);
    kd_tree.index->buildIndex();

    const int descriptor_near_num = config_setting_.DESCRIPTOR_NEAR_NUM;

    ctds_vec_.clear();  // std::vector<LGCesc> triangles;
    nanoflann::KNNResultSet<double> result_set(descriptor_near_num);
    std::vector<size_t> ret_index(descriptor_near_num);
    std::vector<double> out_dist_sqr(descriptor_near_num);

    std::set<std::array<size_t, 3>> unique_triangle_ids;
    for (size_t i = 0; i < cluster_means.size(); ++i) {
        Eigen::Vector2d current_point = points.row(i).transpose();
        // const Eigen::Vector3d& current_point = points.row(i).transpose();
        // Eigen::Vector3d current_point(cluster_means[i].x, cluster_means[i].y, cluster_means[i].z);
        result_set.init(&ret_index[0], &out_dist_sqr[0]);

        kd_tree.index->findNeighbors(result_set, current_point.data(), nanoflann::SearchParams(10));

        for (int j = 0; j < descriptor_near_num; ++j) {
            for (int k = j + 1; k < descriptor_near_num; ++k) {
                // Eigen::Vector2d neighbor1 = points.row(ret_index[j]).transpose();
                // Eigen::Vector2d neighbor2 = points.row(ret_index[k]).transpose();
                std::array<size_t, 3> triangle_id = {i, ret_index[j], ret_index[k]};
                std::sort(triangle_id.begin(), triangle_id.end());

                if (unique_triangle_ids.count(triangle_id)) {
                    continue;
                }
                unique_triangle_ids.insert(triangle_id);

                LGCesc triangle;
                triangle.vertex_A = Eigen::Vector3d(cluster_means[triangle_id[0]].x, cluster_means[triangle_id[0]].y, cluster_means[triangle_id[0]].z);
                triangle.vertex_B = Eigen::Vector3d(cluster_means[triangle_id[1]].x, cluster_means[triangle_id[1]].y, cluster_means[triangle_id[1]].z);
                triangle.vertex_C = Eigen::Vector3d(cluster_means[triangle_id[2]].x, cluster_means[triangle_id[2]].y, cluster_means[triangle_id[2]].z);
                triangle.side_length = computeSideLengths(triangle.vertex_A, triangle.vertex_B, triangle.vertex_C);
                triangle.frame_id = current_frame_id_;

                if (isValidTriangle(triangle)) {
                    ctds_vec_.push_back(triangle);
                    // std::cout << triangle_id[0] << " " << triangle_id[1] << " " << triangle_id[2] << std::endl;
                }
            }
        }
    }
    
    std::cout << "Size of triangles: " << ctds_vec_.size() << std::endl;
    // for (const auto& triangle : ctds_vec_) {
    //     std::cout << "Triangle: "
    //               << "A(" << triangle.vertex_A.transpose() << "), "
    //               << "B(" << triangle.vertex_B.transpose() << "), "
    //               << "C(" << triangle.vertex_C.transpose() << ") "
    //               << "Side Lengths: " << triangle.side_length.transpose() << std::endl;
    // }

    // 可视化生成的三角形
    // visualizeTriangles(ctds_vec_);

    qe_desc_.push(ctds_vec_);

    buildAndSaveSnapshot(cluster_means, clustered_clouds);
}

/*
void LGCManager::candidateSelector(std::vector<int> &candidate_list)
{
    std::unordered_map<int, int> frame_vote;
    for (const auto& ctd : ctds_vec_) {
        int hash_key = computeHashKey(ctd, config_setting_.HASH_DELTA_L, config_setting_.HASH_P, config_setting_.HASH_MAXN);
        std::vector<int> unique_indices;
        int count;
        std::vector<int> repeated_count;
        getKeyIndicesAndCount(global_hash_table, src_keys, unique_indices, count, repeated_count);

    }

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> frame_pq;
    for (const auto& elem : frame_vote) {
        frame_pq.push({elem.second, elem.first});
        if (frame_pq.size() > config_setting_.CANDIDATE_NUM) {
            frame_pq.pop();
        }
    }

    while (!frame_pq.empty()) {
        candidate_list.push_back(frame_pq.top().second);
        frame_pq.pop();
    }
}
*/



void LGCManager::candidateSelector(std::vector<int> &candidate_list)
{
    std::unordered_map<int, int> frame_vote;
    for (const auto& ctd : ctds_vec_) {
        int hash_key = computeHashKey(ctd, config_setting_.HASH_DELTA_L, config_setting_.HASH_P, config_setting_.HASH_MAXN);
        if (hash_table_.count(hash_key)) {
            for (const auto& triangle : hash_table_[hash_key]) {
                ++frame_vote[triangle.frame_id];
            }
        }
    }

    std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> frame_pq;
    for (const auto& elem : frame_vote) {
        frame_pq.push({elem.second, elem.first});
        if (frame_pq.size() > config_setting_.CANDIDATE_NUM) {
            frame_pq.pop();
        }
    }

    while (!frame_pq.empty()) {
        candidate_list.push_back(frame_pq.top().second);
        frame_pq.pop();
    }
}


/*  
void LGCManager::candidateSelector(std::vector<int> &candidate_list)
{

    std::unordered_map<int, int> frame_vote;

    std::vector<int> all_unique_indices;
    std::vector<int> all_repeated_counts;   

    for (const auto& ctd : ctds_vec_) {
        std::vector<int> unique_indices;
        std::vector<int> repeated_count;
        int key;int count;
        std::vector<int> indices;
        int hash_key = computeHashKey(ctd, config_setting_.HASH_DELTA_L, config_setting_.HASH_P, config_setting_.HASH_MAXN);
        auto it =  hash_table_.find(hash_key);
        if (it != hash_table_.end()) {
        //if (hash_table_.count(hash_key)) {
            for (const auto& triangle : hash_table_[hash_key]) {
                indices.push_back(triangle.frame_id);
                ++frame_vote[triangle.frame_id];
            }
            std::sort(indices.begin(), indices.end());
            auto last = std::unique(indices.begin(), indices.end());
            unique_indices.assign(indices.begin(), last);
            count = unique_indices.size();
            repeated_count.resize(count);
            for (int j = 0; j < count; ++j) {
                repeated_count[j] = std::count(indices.begin(), indices.end(), unique_indices[j]);
            }

        }else {
            unique_indices.clear();
            count = 0;
            repeated_count.clear();

        }

        if (unique_indices.empty()) continue;

        for (int ii = 0; ii < count; ++ii) {
            int index = unique_indices[ii];
            int repeat_count = repeated_count[ii];

            auto it = std::find(all_unique_indices.begin(), all_unique_indices.end(), index);
            if (it == all_unique_indices.end()) {
                all_unique_indices.push_back(index);
                all_repeated_counts.push_back(repeat_count);
            } else {
                int existing_index = std::distance(all_unique_indices.begin(), it);
                all_repeated_counts[existing_index] += repeat_count;
            }
        }
    }

    std::vector<int> top_unique_indices;
    std::vector<int> top_repeated_counts;
    sortAndSelectTopN(all_unique_indices, all_repeated_counts, top_unique_indices, top_repeated_counts, 10);

    candidate_list.insert(candidate_list.end(), top_unique_indices.begin(), top_unique_indices.end());
    std::cout <<"-----------------"<<std::endl;


    std::cout << "candidate_list:" << std::endl;
    for (const auto& index : candidate_list) {
        std::cout << index << " ";
    }
    std::cout <<" "<<std::endl;



        // 打印top_unique_indices
/*
    std::cout << "all_unique_indices:" << std::endl;
    for (const auto& index : all_unique_indices) {
        std::cout << index << " ";
    }
    std::cout <<"-----------------"<<std::endl;
    std::cout << "all_repeated_counts:" << std::endl;
    for (const auto& index : all_repeated_counts) {
        std::cout << index << " ";
    }



}
*/

/*
void LGCManager::sortAndSelectTopN(const std::vector<int>& unique_indices, const std::vector<int>& repeated_counts, std::vector<int>& top_unique_indices, std::vector<int>& top_repeated_counts, int top_n) {
    std::vector<std::pair<int, int>> index_count_pairs;
    for (size_t i = 0; i < unique_indices.size(); ++i) {
        index_count_pairs.emplace_back(unique_indices[i], repeated_counts[i]);
    }

    std::sort(index_count_pairs.begin(), index_count_pairs.end(), [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
        return b.second < a.second;
    });


    top_unique_indices.clear();
    top_repeated_counts.clear();
    for (int i = 0; i < top_n && i < static_cast<int>(index_count_pairs.size()); ++i) {
        top_unique_indices.push_back(index_count_pairs[i].first);
        top_repeated_counts.push_back(index_count_pairs[i].second);
    }

    std::cout << "top_unique_indices:" << std::endl;
    for (const auto& index : top_unique_indices) {
        std::cout << index << " ";
    }
    std::cout <<" "<<std::endl;
    std::cout << "top_repeated_counts:" << std::endl;
    for (const auto& index : top_repeated_counts) {
        std::cout << index << " ";
    }
    std::cout <<" "<<std::endl;



}
*/

LoopResults LGCManager::SearchLoop()
{
    LoopResults loop_result(current_frame_id_);

    candidateSelector(loop_result.candidate_list);


    // 测试用
    if (loop_result.candidate_list.size() == 0) {
        loop_result.candidate_list.push_back(std::max(0, (int)current_frame_id_ - config_setting_.SKIP_NEAR_NUM - 1));
    }

    for (const auto& frame_id : loop_result.candidate_list) {
        float dis = corr_sim_P(snapshot_mat_[current_frame_id_], snapshot_mat_[frame_id]);
        if(dis < loop_result.similarity) {
            loop_result.similarity = dis;
            loop_result.loop_id = frame_id;
        }
    }

    return loop_result;
}

void LGCManager::buildAndSaveHashKey(std::vector<LGCesc> &triangles)
{
    const double hash_delta_l = config_setting_.HASH_DELTA_L;  // 哈希函数中边长归一化的精度
    const int hash_p = config_setting_.HASH_P;  // 哈希函数中较大的素数
    const int hash_maxn = config_setting_.HASH_MAXN;  // 哈希表的大小
    for (const auto& triangle : triangles) {
        int hash_key = computeHashKey(triangle, hash_delta_l, hash_p, hash_maxn);
        // std::cout << "The key of triangle: " << hash_key << std::endl;
        if (hash_table_.count(hash_key)) {
            hash_table_[hash_key].push_back(triangle);
            hash_tables_[hash_key].push_back(triangle.frame_id);
        } else {
            hash_table_[hash_key] = {triangle};
            hash_tables_[hash_key] = {triangle.frame_id};
        }
    }
    /*
    for (const auto& pair:hash_tables_){
        std::cout<<"Key: "<<pair.first<<",Value: ";
        for (const auto& value : pair.second){
            cout<< value << ",";
        }
        cout << endl;
    }   
    */
    // std::cout << "The Size of hash_table: " << hash_table_.size() << std::endl;
}

void LGCManager::UpdateDatabase()
{
    if (current_frame_id_ >= config_setting_.SKIP_NEAR_NUM) {
        std::vector<LGCesc> triangles = qe_desc_.front();
        qe_desc_.pop();
        buildAndSaveHashKey(triangles);
    }
    ++current_frame_id_;
}

