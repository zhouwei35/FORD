#include "lcd_manager.h"

using namespace std;
using namespace Eigen;
using namespace pcl;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Vector3d Vector3d;

struct SegmentedResult
{
    PointCloudT::Ptr segmentedPtCloud;
    PointCloudT::Ptr extraGround;
};

string createKey(double a, double b, double c)
{
    vector<double> sides = {a, b, c};
    sort(sides.begin(), sides.end());
    ostringstream oss;
    oss << fixed << setprecision(2);
    for (double side : sides)
    {
        oss << side << "-";
    }
    string key = oss.str();
    key.pop_back(); // Remove trailing hyphen
    return key;
}

double calculateDistance(const pair<double, double> &point1, const pair<double, double> &point2)
{
    return sqrt(pow(point1.first - point2.first, 2) + pow(point1.second - point2.second, 2));
}

void get_normals(PointCloudT::Ptr points, int numNeighbours, vector<Vector3f> &normals, vector<float> &curvature)
{
    pcl::KdTreeFLANN<PointT> kdtree;  // 声明一个 k-d 树对象
    kdtree.setInputCloud(points);  // 将输入点云设置为 k-d 树的输入
    vector<int> pointIdxNKNSearch(numNeighbours);  // 存储邻近点索引的向量
    vector<float> pointNKNSquaredDistance(numNeighbours);  // 存储邻近点距离平方的向量

    normals.resize(points->size());  // 调整法向量数组大小为点云大小
    curvature.resize(points->size());  // 调整曲率数组大小为点云大小

    for (size_t i = 0; i < points->size(); ++i)  // 对于每个点
    {
        kdtree.nearestKSearch(points->points[i], numNeighbours + 1, pointIdxNKNSearch, pointNKNSquaredDistance);  // 进行最近邻搜索

        MatrixXf neighbors(numNeighbours, 3);  // 声明一个矩阵来存储邻近点
        for (int j = 1; j <= numNeighbours; ++j)  // 从第二个邻近点开始，因为第一个是自身
        {
            neighbors(j - 1, 0) = points->points[pointIdxNKNSearch[j]].x - points->points[i].x;  // 计算邻近点与当前点的 x 坐标差
            neighbors(j - 1, 1) = points->points[pointIdxNKNSearch[j]].y - points->points[i].y;  // 计算邻近点与当前点的 y 坐标差
            neighbors(j - 1, 2) = points->points[pointIdxNKNSearch[j]].z - points->points[i].z;  // 计算邻近点与当前点的 z 坐标差
        }

        Matrix3f covarianceMatrix = neighbors.transpose() * neighbors / numNeighbours;  // 计算协方差矩阵
        SelfAdjointEigenSolver<Matrix3f> solver(covarianceMatrix, ComputeEigenvectors);  // 对协方差矩阵进行特征值分解
        Vector3f eigenValues = solver.eigenvalues();  // 获取特征值
        Matrix3f eigenVectors = solver.eigenvectors();  // 获取特征向量

        normals[i] = eigenVectors.col(0);  // 法向量是最小特征值对应的特征向量
        curvature[i] = eigenValues(0) / eigenValues.sum();  // 曲率是最小特征值除以特征值的总和
    }
}

SegmentedResult segmentTree(PointCloudT::Ptr ptCloud)
{
    // 读取并转换点云数据
    int pointNum = ptCloud->size();
    const float LidarMaxView = 80.0f; // 激光雷达最大视距
    const float gridSize = 1.0f;      // 网格大小

    const int gridSizeDouble = static_cast<int>(LidarMaxView * 2);
    vector<vector<vector<PointT>>> gridSet(gridSizeDouble, vector<vector<PointT>>(gridSizeDouble));
    vector<vector<vector<int>>> gridSetIndex(gridSizeDouble, vector<vector<int>>(gridSizeDouble));

    // 对坐标进行预处理，避免循环中的重复计算
    vector<float> processedX(pointNum), processedY(pointNum);
    // 对输入点云的 x 和 y 坐标进行范围限制，确保在 [-LidarMaxView, LidarMaxView) 之间
    for (int i = 0; i < pointNum; ++i)
    {
        processedX[i] = min(max(ptCloud->points[i].x, -LidarMaxView), LidarMaxView - 0.001f);
        processedY[i] = min(max(ptCloud->points[i].y, -LidarMaxView), LidarMaxView - 0.001f);
    }
    // cout << "HELLO" << endl;
    // 计算网格索引
    vector<int> rowIndex(pointNum), colIndex(pointNum);
    // 计算每个点在网格中的行索引 rowIndex 和列索引 colIndex
    for (int i = 0; i < pointNum; ++i)
    {
        rowIndex[i] = static_cast<int>((processedX[i] + LidarMaxView) / gridSize);

        colIndex[i] = static_cast<int>((processedY[i] + LidarMaxView) / gridSize);
    }
    // cout << "World" << endl;
    // 分配点到网格中
    // 将每个点分配到对应的网格中，并记录其索引
    for (int i = 0; i < pointNum; ++i)
    {
        // cout<<rowIndex[i]<<" "<<colIndex[i]<<endl;
        gridSet[rowIndex[i]][colIndex[i]].push_back(ptCloud->points[i]);
        gridSetIndex[rowIndex[i]][colIndex[i]].push_back(i);
    }
    // cout << "JLIN" << endl;

    
    // 初始化地面和树木的点集
    PointCloudT::Ptr groundPoints(new PointCloudT);
    PointCloudT::Ptr treePoints(new PointCloudT);
    PointCloudT::Ptr noisePoints(new PointCloudT);

    // 分析每个网格中的点，根据 z 坐标范围将点分类为地面点或树木点
    for (int i = 0; i < gridSizeDouble; ++i)
    {
        for (int j = 0; j < gridSizeDouble; ++j)
        {
            if (!gridSet[i][j].empty())
            {
                vector<int> idx = gridSetIndex[i][j];
                float zRange = 0.0f;
                for (const int &id : idx)
                {
                    zRange = max(zRange, ptCloud->points[id].z) - min(zRange, ptCloud->points[id].z);
                }

                if (zRange <= 0.2)
                {
                    for (const auto &point : gridSet[i][j])
                    {
                        groundPoints->push_back(point);
                    }
                }
                else
                {
                    for (const auto &point : gridSet[i][j])
                    {
                        treePoints->push_back(point);
                    }
                }
            }
        }
    }
    // cout << "ZWW" << endl;




    //  过滤地面点
    vector<Vector3f> normals;  // 法向量的向量
    vector<float> curvature;  // 曲率的向量
    // 调用 get_normals 函数计算地面点的法向量和曲率
    get_normals(groundPoints, 10, normals, curvature);

    Vector3f z(0, 0, 1);  // 定义一个z轴方向的向量
    vector<bool> logicGround(groundPoints->size(), false);  // 初始化一个逻辑向量，用于标记地面点
    for (size_t i = 0; i < groundPoints->size(); ++i)
    {
        float levelDegree = abs(z.dot(normals[i]));  // 计算法向量与 z 轴的点积的绝对值（在Z轴上的投影）
        logicGround[i] = (groundPoints->points[i].z <= 0.2f) && (levelDegree > 0.9f);  // 如果 z 坐标小于等于 0.2 且法向量几乎垂直于 z 轴，则认为是地面点
    }

    PointCloudT::Ptr extraGround(new PointCloudT);  // 创建一个点云来存储额外的地面点
    PointCloudT::Ptr remainingPoints(new PointCloudT);  // 创建一个点云来存储剩余的点
    for (size_t i = 0; i < groundPoints->size(); ++i)
    {
        if (logicGround[i])
        {
            extraGround->push_back(groundPoints->points[i]);  // 如果是地面点，加入到 extraGround
        }
        else
        {
            remainingPoints->push_back(groundPoints->points[i]);  // 否则，加入到 remainingPoints
        }
    }
    for (const auto &point : treePoints->points)
    {
        remainingPoints->push_back(point);
    }







    // 提取树干
    get_normals(remainingPoints, 20, normals, curvature);

    // 初始化树干的垂直性参数
    const float verticalityThreshold = 0.7f; // 树干的垂直性阈值
    Vector3f zUnit(0, 0, 1);                 // 垂直向上的单位向量

    // 计算所有点的垂直性
    vector<bool> treeIndices(remainingPoints->size(), false);
    for (size_t i = 0; i < normals.size(); ++i)
    {
        float verticality = 1.0f - abs(zUnit.dot(normals[i]));
        treeIndices[i] = (verticality > verticalityThreshold);
    }

    PointCloudT::Ptr tree(new PointCloudT);
    for (size_t i = 0; i < treeIndices.size(); ++i)
    {
        if (treeIndices[i])
        {
            tree->push_back(remainingPoints->points[i]);
        }
    }

    // 使用欧式距离聚类进行树干提取
    pcl::search::KdTree<PointT>::Ptr treeSearch(new pcl::search::KdTree<PointT>);
    treeSearch->setInputCloud(tree);
    vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(0.6); // 聚类的最小距离
    ec.setMinClusterSize(10);    // 最小聚类点数
    ec.setSearchMethod(treeSearch);
    ec.setInputCloud(tree);
    ec.extract(clusterIndices);

    // 筛选有效的点集
    pcl::PointIndices::Ptr validPoints(new pcl::PointIndices);
    for (const auto &indices : clusterIndices)
    {
        for (const auto &index : indices.indices)
        {
            validPoints->indices.push_back(index);
        }
    }

    PointCloudT::Ptr segmentedPtCloud(new PointCloudT);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(tree);
    extract.setIndices(validPoints);
    extract.filter(*segmentedPtCloud);

    SegmentedResult result;
    result.segmentedPtCloud = segmentedPtCloud;
    result.extraGround = extraGround;
    return result;
}

vector<vector<int>> nChooseK(int n, int k)
{
    vector<vector<int>> combinations;  // 存储组合结果
    string bitmask(k, 1);  // 创建一个包含 k 个 1 的字符串，用于选择 k 个元素
    bitmask.resize(n, 0);  // 将 bitmask 扩展到长度为 n，扩展部分用 0 填充
    do
    {
        vector<int> combination;
        for (int i = 0; i < n; ++i)
        {
            if (bitmask[i])
                combination.push_back(i);
        }
        combinations.push_back(combination);  // 添加组合到结果
    } while (prev_permutation(bitmask.begin(), bitmask.end()));
    return combinations;  // 返回所有组合
}

class KDTree1
{
public:
    KDTree1(const vector<pair<double, double>> &points)
    {
        build(points);  // 构建 KD 树
    }

    vector<int> knnSearch(const pair<double, double> &point, int k)
    {
        priority_queue<pair<double, int>> pq;  // 优先队列存储 k 个最近邻
        knnSearch(root, point, k, pq);  // 搜索 k 近邻
        vector<int> result;
        while (!pq.empty())
        {
            result.push_back(pq.top().second);
            pq.pop();
        }
        reverse(result.begin(), result.end());  // 结果按距离升序排列
        return result;  // 返回 k 近邻
    }

private:
    struct Node
    {
        pair<double, double> point;
        int index;
        Node *left;
        Node *right;
    };

    Node *root;

    Node *buildTree(vector<pair<double, double>> &points, vector<int> &indices, int depth)
    {
        if (points.empty())
            return nullptr;

        int axis = depth % 2;
        auto comparator = [axis](const pair<double, double> &a, const pair<double, double> &b)
        {
            return axis == 0 ? a.first < b.first : a.second < b.second;
        };

        int median = points.size() / 2;
        nth_element(points.begin(), points.begin() + median, points.end(), comparator);

        Node *node = new Node{points[median], indices[median], nullptr, nullptr};
        vector<pair<double, double>> left_points(points.begin(), points.begin() + median);
        vector<pair<double, double>> right_points(points.begin() + median + 1, points.end());
        vector<int> left_indices(indices.begin(), indices.begin() + median);
        vector<int> right_indices(indices.begin() + median + 1, indices.end());

        node->left = buildTree(left_points, left_indices, depth + 1);
        node->right = buildTree(right_points, right_indices, depth + 1);

        return node;
    }

    void knnSearch(Node *node, const pair<double, double> &target, int k, priority_queue<pair<double, int>> &pq, int depth = 0)
    {
        if (!node)
            return;

        double distance = calculateDistance(node->point, target);
        if (pq.size() < k || distance < pq.top().first)
        {
            pq.push({distance, node->index});
            if (pq.size() > k)
                pq.pop();
        }

        int axis = depth % 2;
        double diff = (axis == 0 ? target.first - node->point.first : target.second - node->point.second);

        knnSearch(diff < 0 ? node->left : node->right, target, k, pq, depth + 1);
        if (fabs(diff) < pq.top().first)
        {
            knnSearch(diff < 0 ? node->right : node->left, target, k, pq, depth + 1);
        }
    }

    void build(const vector<pair<double, double>> &points)
    {
        vector<int> indices(points.size());
        iota(indices.begin(), indices.end(), 0);
        root = buildTree(const_cast<vector<pair<double, double>> &>(points), indices, 0);
    }
};

// Define a structure to represent a triangle
struct Triangle
{
    int vertex_a, vertex_b, vertex_c;
    double vertex_a_opposite_side, vertex_b_opposite_side, vertex_c_opposite_side;
};

// 定义点云数据结构
struct PointCloud1
{
    MatrixXd Location; // 点云的XYZ坐标
};

// 计算两点之间的距离
double distance(const Vector2d &a, const Vector2d &b)
{
    return sqrt(pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2));
}

// 计算二维凸包
void ConvexHull2D(const MatrixXd &points, MatrixXd &hull)
{
    vector<Vector2d> inputPoints;
    for (int i = 0; i < points.rows(); ++i)
    {
        inputPoints.push_back(points.row(i));
    }

    auto cross = [](const Vector2d &O, const Vector2d &A, const Vector2d &B)
    {
        return (A(0) - O(0)) * (B(1) - O(1)) - (A(1) - O(1)) * (B(0) - O(0));
    };

    sort(inputPoints.begin(), inputPoints.end(), [](const Vector2d &a, const Vector2d &b)
         { return a(0) < b(0) || (a(0) == b(0) && a(1) < b(1)); });

    vector<Vector2d> lower, upper;
    for (const auto &p : inputPoints)
    {
        while (lower.size() >= 2 && cross(lower[lower.size() - 2], lower.back(), p) <= 0)
        {
            lower.pop_back();
        }
        lower.push_back(p);
    }

    for (int i = inputPoints.size() - 1; i >= 0; --i)
    {
        const auto &p = inputPoints[i];
        while (upper.size() >= 2 && cross(upper[upper.size() - 2], upper.back(), p) <= 0)
        {
            upper.pop_back();
        }
        upper.push_back(p);
    }

    upper.pop_back();
    lower.pop_back();

    vector<Vector2d> result = lower;
    result.insert(result.end(), upper.begin(), upper.end());

    hull.resize(result.size(), 2);
    for (int i = 0; i < result.size(); ++i)
    {
        hull.row(i) = result[i];
    }
}

// 计算凸包面积
double ConvexHullArea(const MatrixXd &hull)
{
    double area = 0.0;
    for (int i = 0; i < hull.rows(); ++i)
    {
        Vector2d p1 = hull.row(i);
        Vector2d p2 = hull.row((i + 1) % hull.rows());
        area += p1(0) * p2(1) - p2(0) * p1(1);
    }
    return abs(area) / 2.0;
}

// DBSCAN 聚类算法
pair<vector<Vector3d>, vector<vector<Vector3d>>> clusterXY(const PointCloud1 &segmentedPtCloud)
{
    // 提取 X, Y 坐标
    MatrixXd X = segmentedPtCloud.Location.leftCols(2);

    int MinPts = 30;
    double epsilon = 0.3;
    int C = 0;  // 聚类数量
    int n = X.rows();  // 点的数量
    vector<int> IDX(n, 0);             // 聚类标记
    MatrixXd D = MatrixXd::Zero(n, n); // 距离矩阵
    vector<bool> visited(n, false);    // 访问标记
    vector<bool> isnoise(n, false);    // 噪声标记

    // 计算距离矩阵
    for (int i = 0; i < n; ++i)
    {
        for (int j = i + 1; j < n; ++j)
        {
            D(i, j) = D(j, i) = distance(X.row(i), X.row(j));
        }
    }

    for (int i = 0; i < n; ++i)
    {
        if (!visited[i])
        {
            visited[i] = true;
            vector<int> Neighbors;
            for (int j = 0; j < n; ++j)
            {
                if (D(i, j) <= epsilon)
                {
                    Neighbors.push_back(j);
                }
            }

            if (Neighbors.size() < MinPts)
            {
                isnoise[i] = true;
            }
            else
            {
                C++;
                IDX[i] = C;
                int k = 0;
                while (k < Neighbors.size())
                {
                    int j = Neighbors[k];
                    if (!visited[j])
                    {
                        visited[j] = true;
                        vector<int> Neighbors2;
                        for (int m = 0; m < n; ++m)
                        {
                            if (D(j, m) <= epsilon)
                            {
                                Neighbors2.push_back(m);
                            }
                        }
                        if (Neighbors2.size() >= MinPts)
                        {
                            Neighbors.insert(Neighbors.end(), Neighbors2.begin(), Neighbors2.end());
                        }
                    }
                    if (IDX[j] == 0)
                    {
                        IDX[j] = C;
                    }
                    k++;
                }
            }
        }
    }

    // 初始化输出变量
    vector<Vector3d> arr;  // 存储所有聚类的点
    vector<vector<Vector3d>> ClusterXYZ(C);  // 存储每个聚类的点
    vector<double> areas(C, 0.0);  // 存储每个聚类的凸包面积

    // 计算所有聚类的凸包面积
    for (int i = 1; i <= C; ++i)
    {
        vector<Vector2d> ClusterXY;
        for (int j = 0; j < n; ++j)
        {
            if (IDX[j] == i)
            {
                ClusterXY.push_back(X.row(j));
            }
        }
        if (ClusterXY.size() >= 30)
        {
            MatrixXd points(ClusterXY.size(), 2);
            for (int j = 0; j < ClusterXY.size(); ++j)
            {
                points.row(j) = ClusterXY[j];
            }
            MatrixXd hull;
            ConvexHull2D(points, hull);
            areas[i - 1] = ConvexHullArea(hull);
        }
    }

    // 过滤异常聚类
    vector<bool> validClusters(C, true);
    double mean_area = accumulate(areas.begin(), areas.end(), 0.0) / C;
    double sq_sum = inner_product(areas.begin(), areas.end(), areas.begin(), 0.0);
    double stdev = sqrt(sq_sum / C - mean_area * mean_area);
    for (int i = 0; i < C; ++i)
    {
        if (abs(areas[i] - mean_area) > 2 * stdev)
        {
            validClusters[i] = false;
        }
    }

    // 保存和返回有效聚类的信息
    for (int i = 1; i <= C; ++i)
    {
        if (validClusters[i - 1])
        {
            vector<Vector3d> cluster;
            for (int j = 0; j < n; ++j)
            {
                if (IDX[j] == i)
                {
                    Vector3d point = segmentedPtCloud.Location.row(j);
                    cluster.push_back(point);
                    arr.push_back(point);
                }
            }
            ClusterXYZ[i - 1] = cluster;
        }
    }

    // 移除空的聚类
    ClusterXYZ.erase(remove_if(ClusterXYZ.begin(), ClusterXYZ.end(), [](const vector<Vector3d> &c)
                               { return c.empty(); }),
                     ClusterXYZ.end());

    return {arr, ClusterXYZ};
}



// Function to get the sorted triangles
vector<Triangle> getSortTriangles(const vector<pair<double, double>> &stem_positions, int max_stems_for_exhaustive_search, int knn)
{
    int num_stem = stem_positions.size();  // 获取点的数量
    vector<vector<int>> set_triangle_vertices;  // 存储三角形顶点集合

    // 如果点数量小于等于 max_stems_for_exhaustive_search，则对所有点进行三点组合
    if (num_stem <= max_stems_for_exhaustive_search)
    {
        set_triangle_vertices = nChooseK(num_stem, 3);
    }
    else
    {
        KDTree1 tree(stem_positions);  // 构建 KD 树
        for (int i = 0; i < num_stem; ++i)
        {
            vector<int> idx = tree.knnSearch(stem_positions[i], knn + 1);  // 对每个点进行 k 近邻搜索
            vector<vector<int>> neighbor_combs = nChooseK(knn, 2);  // 获取邻近点的所有二点组合
            for (const auto &comb : neighbor_combs)
            {
                set_triangle_vertices.push_back({i, idx[comb[0] + 1], idx[comb[1] + 1]});  // 形成三角形顶点集合
            }
        }
    }

    vector<Triangle> triangles;  // 存储三角形

    // Construct triangles
    // 构建三角形
    for (const auto &vertices : set_triangle_vertices)
    {
        int vertex_a = vertices[0];
        int vertex_b = vertices[1];
        int vertex_c = vertices[2];

        double side_ab = calculateDistance(stem_positions[vertex_a], stem_positions[vertex_b]);  // 计算边长
        double side_ac = calculateDistance(stem_positions[vertex_a], stem_positions[vertex_c]);
        double side_bc = calculateDistance(stem_positions[vertex_b], stem_positions[vertex_c]);

        Triangle triangle;  // 创建三角形对象
        triangle.vertex_a = vertex_a;
        triangle.vertex_a_opposite_side = side_bc;
        triangle.vertex_b = vertex_b;
        triangle.vertex_b_opposite_side = side_ac;
        triangle.vertex_c = vertex_c;
        triangle.vertex_c_opposite_side = side_ab;

        triangles.push_back(triangle);  // 添加三角形到集合
    }

    // Sort edges by length
    // 按边长排序
    for (auto &triangle : triangles)
    {
        vector<pair<int, double>> vertices = {
            {triangle.vertex_a, triangle.vertex_a_opposite_side},
            {triangle.vertex_b, triangle.vertex_b_opposite_side},
            {triangle.vertex_c, triangle.vertex_c_opposite_side}};

        sort(vertices.begin(), vertices.end(), [](const pair<int, double> &a, const pair<int, double> &b)
             { return a.second > b.second; });  // 按边长降序排序

        triangle.vertex_a = vertices[0].first;  // 更新顶点顺序
        triangle.vertex_a_opposite_side = vertices[0].second;
        triangle.vertex_b = vertices[1].first;
        triangle.vertex_b_opposite_side = vertices[1].second;
        triangle.vertex_c = vertices[2].first;
        triangle.vertex_c_opposite_side = vertices[2].second;
    }

    // Ensure clockwise order
    // 保证顶点顺时针顺序
    for (auto &triangle : triangles)
    {
        const auto &ver_a = stem_positions[triangle.vertex_a];
        const auto &ver_b = stem_positions[triangle.vertex_b];
        const auto &ver_c = stem_positions[triangle.vertex_c];

        double cp = (ver_b.first - ver_a.first) * (ver_c.second - ver_a.second) - (ver_b.second - ver_a.second) * (ver_c.first - ver_a.first);

        if (cp > 0)  // 如果叉积大于0，交换顶点 b 和 c
        {
            swap(triangle.vertex_b, triangle.vertex_c);
        }
    }

    // Remove duplicate triangles
    // 移除重复三角形
    map<string, Triangle> uniqueTriangles;
    for (const auto &triangle : triangles)
    {
        string key = createKey(triangle.vertex_a_opposite_side, triangle.vertex_b_opposite_side, triangle.vertex_c_opposite_side);
        uniqueTriangles[key] = triangle;  // 以三角形边长创建唯一键
    }

    vector<Triangle> result;
    for (const auto &pair : uniqueTriangles)
    {
        result.push_back(pair.second);  // 返回唯一三角形集合
    }

    return result;  // 返回排序后的三角形
}

// 计算哈希键
int compute_hash_key(double l1, double l2, double l3, double delta_l, int p, int B)
{
    int bar_l1 = round(l1 / delta_l);
    int bar_l2 = round(l2 / delta_l);
    int bar_l3 = round(l3 / delta_l);
    return (bar_l3 * p + bar_l2) % B + bar_l1 % B;
}

//
void pcsegdist(PointCloudT::Ptr points, float minDistance, int minPoints, int maxPoints, vector<int> &labels, vector<float> &score)
{
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(points);

    vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
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
// 定义哈希表中存储的三角形信息
struct TriangleInfo
{
    Triangle triangle;
    int index;
};

using HashTable = std::unordered_map<std::string, std::vector<TriangleInfo>>;
typedef std::unordered_map<std::string, std::vector<TriangleInfo>> GlobalHashTable;
// 构建哈希表
HashTable constructHashTable(const std::vector<Triangle> &triangles, double delta_l, int p, int B, int i)
{
    HashTable hash_table;
    for (const auto &triangle : triangles)
    {
        // 获取三角形的边长
        double l1 = triangle.vertex_a_opposite_side;
        double l2 = triangle.vertex_b_opposite_side;
        double l3 = triangle.vertex_c_opposite_side;

        // 计算哈希键
        int hash_key = compute_hash_key(l1, l2, l3, delta_l, p, B);
        std::string hash_key_str = std::to_string(hash_key);

        // 将三角形信息存储在哈希表中
        TriangleInfo triangle_info;
        triangle_info.triangle = triangle;
        triangle_info.index = i;

        if (hash_table.find(hash_key_str) != hash_table.end())
        {
            hash_table[hash_key_str].push_back(triangle_info);
        }
        else
        {
            hash_table[hash_key_str] = {triangle_info};
        }
    }
    return hash_table;
}

// 打印哈希表
void printHashTable(const HashTable &hash_table)
{
    for (const auto &pair : hash_table)
    {
        std::cout << "Key: " << pair.first << "\n";
        for (const auto &triangle_info : pair.second)
        {
            std::cout << "  Triangle with vertices ("
                      << triangle_info.triangle.vertex_a << ", "
                      << triangle_info.triangle.vertex_b << ", "
                      << triangle_info.triangle.vertex_c << ") and sides ("
                      << triangle_info.triangle.vertex_a_opposite_side << ", "
                      << triangle_info.triangle.vertex_b_opposite_side << ", "
                      << triangle_info.triangle.vertex_c_opposite_side << ") and index "
                      << triangle_info.index << "\n";
        }
    }
}

std::vector<int> unique(const std::vector<int> &vec)
{
    std::unordered_set<int> unique_set(vec.begin(), vec.end());
    return std::vector<int>(unique_set.begin(), unique_set.end());
}

std::tuple<std::vector<int>, int, std::unordered_map<int, int>> getKeyIndicesAndCount1(
    const std::unordered_map<std::string, std::vector<int>> &globalHashTable, const std::vector<std::string> &srcHashKeys)
{

    std::vector<int> uniqueIndices;
    std::unordered_map<int, int> repeatedCount;

    for (const auto &key : srcHashKeys)
    {
        if (globalHashTable.find(key) != globalHashTable.end())
        {
            const std::vector<int> &trianglesInfo = globalHashTable.at(key);
            uniqueIndices.insert(uniqueIndices.end(), trianglesInfo.begin(), trianglesInfo.end());
        }
    }

    std::sort(uniqueIndices.begin(), uniqueIndices.end());
    uniqueIndices.erase(std::unique(uniqueIndices.begin(), uniqueIndices.end()), uniqueIndices.end());

    for (const auto &index : uniqueIndices)
    {
        repeatedCount[index] = std::count(uniqueIndices.begin(), uniqueIndices.end(), index);
    }

    int count = uniqueIndices.size();

    return std::make_tuple(uniqueIndices, count, repeatedCount);
}

// Function to get histogram count of unique elements
std::vector<int> histc(const std::vector<int> &vec, const std::vector<int> &unique_vec)
{
    std::unordered_map<int, int> count_map;
    for (const auto &val : vec)
    {
        count_map[val]++;
    }
    std::vector<int> counts;
    for (const auto &val : unique_vec)
    {
        counts.push_back(count_map[val]);
    }
    return counts;
}

// Function to get key indices and count
std::tuple<std::vector<int>, int, std::vector<int>> getKeyIndicesAndCount1(GlobalHashTable &global_hash_table, const std::vector<std::string> &src_hash_keys)
{
    std::vector<int> unique_indices;
    std::vector<int> repeated_count;

    for (const auto &key : src_hash_keys)
    {
        if (global_hash_table.find(key) != global_hash_table.end())
        {
            const auto &triangles_info = global_hash_table[key];
            std::vector<int> indices;
            for (const auto &info : triangles_info)
            {
                indices.push_back(info.index);
            }
            auto unique_ind = unique(indices);
            unique_indices.insert(unique_indices.end(), unique_ind.begin(), unique_ind.end());
        }
    }
    auto all_unique_indices = unique(unique_indices);
    auto repeated_counts = histc(unique_indices, all_unique_indices);
    cout << static_cast<int>(all_unique_indices.size()) << endl;
    return {all_unique_indices, static_cast<int>(all_unique_indices.size()), repeated_counts};
}

class KDTree2
{
public:
    KDTree2(const vector<vector<double>> &points)
    {
        build(points);
    }

    vector<int> knnSearch(const vector<double> &point, int k)
    {
        priority_queue<pair<double, int>> pq;
        knnSearch(root, point, k, pq);
        vector<int> result;
        while (!pq.empty())
        {
            result.push_back(pq.top().second);
            pq.pop();
        }
        reverse(result.begin(), result.end());
        return result;
    }

private:
    struct Node
    {
        vector<double> point;
        int index;
        Node *left;
        Node *right;
    };

    Node *root;

    Node *buildTree(vector<vector<double>> &points, vector<int> &indices, int depth)
    {
        if (points.empty())
            return nullptr;

        int axis = depth % 2;
        auto comparator = [axis](const vector<double> &a, const vector<double> &b)
        {
            return axis == 0 ? a[0] < b[0] : a[1] < b[1];
        };

        int median = points.size() / 2;
        nth_element(points.begin(), points.begin() + median, points.end(), comparator);

        Node *node = new Node{points[median], indices[median], nullptr, nullptr};
        vector<vector<double>> left_points(points.begin(), points.begin() + median);
        vector<vector<double>> right_points(points.begin() + median + 1, points.end());
        vector<int> left_indices(indices.begin(), indices.begin() + median);
        vector<int> right_indices(indices.begin() + median + 1, indices.end());

        node->left = buildTree(left_points, left_indices, depth + 1);
        node->right = buildTree(right_points, right_indices, depth + 1);

        return node;
    }

    void knnSearch(Node *node, const vector<double> &target, int k, priority_queue<pair<double, int>> &pq, int depth = 0)
    {
        if (!node)
            return;

        double distance = calculateDistance(node->point, target);
        if (pq.size() < k || distance < pq.top().first)
        {
            pq.push({distance, node->index});
            if (pq.size() > k)
                pq.pop();
        }

        int axis = depth % 2;
        double diff = (axis == 0 ? target[0] - node->point[0] : target[1] - node->point[1]);

        knnSearch(diff < 0 ? node->left : node->right, target, k, pq, depth + 1);
        if (fabs(diff) < pq.top().first)
        {
            knnSearch(diff < 0 ? node->right : node->left, target, k, pq, depth + 1);
        }
    }

    double calculateDistance(const vector<double> &a, const vector<double> &b)
    {
        return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2));
    }

    void build(const vector<vector<double>> &points)
    {
        vector<int> indices(points.size());
        iota(indices.begin(), indices.end(), 0);
        root = buildTree(const_cast<vector<vector<double>> &>(points), indices, 0);
    }
};

// 定义点和三角形的数据结构
struct Point
{
    double x, y;
};

// 计算两个点之间的欧几里得距离
double euclideanDistance(const std::pair<double, double> &p1, const std::pair<double, double> &p2)
{
    return sqrt(pow(p1.first - p2.first, 2) + pow(p2.second - p2.second, 2));
}

// 使用knnsearch函数查找最近邻
std::vector<int> knnsearch(const std::vector<std::vector<double>> &src, const std::vector<std::vector<double>> &tgt, double threshold)
{
    std::vector<int> idx(tgt.size(), -1);
    for (size_t i = 0; i < tgt.size(); ++i)
    {
        double minDist = std::numeric_limits<double>::max();
        for (size_t j = 0; j < src.size(); ++j)
        {
            double dist = 0;
            for (size_t k = 0; k < src[j].size(); ++k)
            {
                dist += pow(src[j][k] - tgt[i][k], 2);
            }
            dist = sqrt(dist);
            if (dist < minDist && dist <= threshold)
            {
                minDist = dist;
                idx[i] = j;
            }
        }
    }
    return idx;
}

// 全局一致性检查
bool satisfyGlobalConsistency(const Triangle &tri1, const Triangle &tri2, const std::vector<std::pair<double, double>> &stem_positions_src, const std::vector<std::pair<double, double>> &stem_positions_tar, double edge_diff)
{
    std::vector<int> src_vertices = {tri1.vertex_a, tri1.vertex_b, tri1.vertex_c};
    std::vector<int> tgt_vertices = {tri2.vertex_a, tri2.vertex_b, tri2.vertex_c};

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            double dist_src = euclideanDistance(stem_positions_src[src_vertices[i]], stem_positions_src[src_vertices[j]]);
            double dist_tgt = euclideanDistance(stem_positions_tar[tgt_vertices[i]], stem_positions_tar[tgt_vertices[j]]);
            if (std::abs(dist_src - dist_tgt) > edge_diff)
            {
                return false;
            }
        }
    }
    return true;
}

/*
void getTraAndQuad(const vector<Triangle>& triangles_src, const vector<Triangle>& triangles_tgt, int& NumTra, int& NumQuad) {
    if (triangles_src.empty() || triangles_tgt.empty()) {
        NumTra = 0;
        NumQuad = 0;
        return;
    }
    // Extract source and target feature vectors and convert them to matrix form
    MatrixXd features_src(triangles_src.size(), 3);
    MatrixXd features_tgt(triangles_tgt.size(), 3);

    for (size_t i = 0; i < triangles_src.size(); ++i) {
        features_src(i, 0) = triangles_src[i].vertex_a_opposite_side;
        features_src(i, 1) = triangles_src[i].vertex_b_opposite_side;
        features_src(i, 2) = triangles_src[i].vertex_c_opposite_side;
    }
    for (size_t i = 0; i < triangles_tgt.size(); ++i) {
        features_tgt(i, 0) = triangles_tgt[i].vertex_a_opposite_side;
        features_tgt(i, 1) = triangles_tgt[i].vertex_b_opposite_side;
        features_tgt(i, 2) = triangles_tgt[i].vertex_c_opposite_side;
    }
    // Nearest neighbor search using Euclidean distance
    double edge_diff = 0.1;
    vector<pair<int, int>> locally_matched_pairs;
    for (size_t i = 0; i < features_tgt.rows(); ++i) {
        //cout<<"features_tgt.rows()"<<features_tgt.rows()<<endl;
        double min_dist = numeric_limits<double>::max();
        int min_index = -1;
        for (size_t j = 0; j < features_src.rows(); ++j) {
          //  cout<<"features_src.rows()"<<features_tgt.rows()<<endl;
            double dist = (features_tgt.row(i) - features_src.row(j)).norm();
            if (dist < min_dist) {
                min_dist = dist;
                min_index = j;
            }
        }

        if (min_dist <= edge_diff) {
            locally_matched_pairs.emplace_back(min_index, i);
        }
    }
    if (locally_matched_pairs.empty()) {
        NumTra = 0;
        NumQuad = 0;
        return;
    }

   // NumTra = locally_matched_pairs.size();
    //cout<<NumTra<<endl;
   // NumQuad = 0; // Adjust as needed based on the specific criteria for counting quads

    std::vector<std::vector<int>> groups(locally_matched_pairs.size());
    std::vector<int> group_size(locally_matched_pairs.size(), 0);

    std::vector<std::array<int, 6>> vertex_pairs;
    for (const auto &pair : locally_matched_pairs)
    {
        vertex_pairs.push_back({triangles_src[pair.first].vertex_a, triangles_src[pair.first].vertex_b, triangles_src[pair.first].vertex_c, triangles_tgt[pair.second].vertex_a, triangles_tgt[pair.second].vertex_b, triangles_tgt[pair.second].vertex_c});
    }

    for (size_t i_grp = 0; i_grp < vertex_pairs.size(); ++i_grp)
    {
        groups[i_grp].push_back(i_grp);
        auto pair_init = vertex_pairs[i_grp];

        for (size_t i_pair = 0; i_pair < vertex_pairs.size(); ++i_pair)
        {
            if (i_grp != i_pair)
            {
                auto pair_cand = vertex_pairs[i_pair];
                if (satisfyGlobalConsistency(triangles_src[pair_init[0] / 3], triangles_tgt[pair_init[3] / 3], stem_positions_src, stem_positions_tar, 0.1))
                {
                    groups[i_grp].push_back(i_pair);
                }
            }
        }
        group_size[i_grp] = groups[i_grp].size();
    }

    auto max_group_it = std::max_element(group_size.begin(), group_size.end());
    int id_max_group = std::distance(group_size.begin(), max_group_it);
    auto globally_matched_pairs = groups[id_max_group];

    std::set<int> unique_src, unique_tgt;
    for (auto idx : globally_matched_pairs)
    {
        for (int i = 0; i < 3; ++i)
        {
            unique_src.insert(vertex_pairs[idx][i]);
            unique_tgt.insert(vertex_pairs[idx][i + 3]);
        }
    }

    numTra = locally_matched_pairs.size();

    std::set<std::pair<int, int>> src_quad, tgt_quad;
    for (size_t i = 0; i < globally_matched_pairs.size(); ++i)
    {
        for (size_t j = i + 1; j < globally_matched_pairs.size(); ++j)
        {
            std::set<int> src_set = {vertex_pairs[i][0], vertex_pairs[i][1], vertex_pairs[i][2], vertex_pairs[j][0], vertex_pairs[j][1], vertex_pairs[j][2]};
            std::set<int> tgt_set = {vertex_pairs[i][3], vertex_pairs[i][4], vertex_pairs[i][5], vertex_pairs[j][3], vertex_pairs[j][4], vertex_pairs[j][5]};
            if (src_set.size() == 4 && tgt_set.size() == 4)
            {
                src_quad.insert({*src_set.begin(), *(++src_set.begin())});
                tgt_quad.insert({*tgt_set.begin(), *(++tgt_set.begin())});
            }
        }
    }
    numQuad = src_quad.size();
}
*/
// 获取匹配的三角形和四边形数量
void getTraAndQuad(const std::vector<Triangle> &triangles_src, const std::vector<Triangle> &triangles_tgt, const std::vector<std::pair<double, double>> &stem_positions_src, const std::vector<std::pair<double, double>> &stem_positions_tar, int &numTra, int &numQuad)
{
    if (triangles_src.empty() || triangles_tgt.empty())
    {
        numTra = 0;
        numQuad = 0;
        return;
    }

    // 提取源和目标特征向量并转换为矩阵形式
    std::vector<std::vector<double>> features_src, features_tgt;
    for (const auto &tri : triangles_src)
    {
        features_src.push_back({tri.vertex_a_opposite_side, tri.vertex_b_opposite_side, tri.vertex_c_opposite_side});
    }
    for (const auto &tri : triangles_tgt)
    {
        features_tgt.push_back({tri.vertex_a_opposite_side, tri.vertex_b_opposite_side, tri.vertex_c_opposite_side});
    }

    // 使用knnsearch查找最近邻
    std::vector<int> idx = knnsearch(features_src, features_tgt, 0.1);

    // 构造匹配对数组
    std::vector<std::pair<int, int>> locally_matched_pairs;
    for (size_t i = 0; i < idx.size(); ++i)
    {
        if (idx[i] != -1)
        {
            locally_matched_pairs.emplace_back(idx[i], i);
        }
    }

    // cout << locally_matched_pairs.size() << endl;

    if (locally_matched_pairs.empty())
    {
        numTra = 0;
        numQuad = 0;
        return;
    }

    std::vector<std::vector<int>> groups(locally_matched_pairs.size());
    std::vector<int> group_size(locally_matched_pairs.size(), 0);

    std::vector<std::array<int, 6>> vertex_pairs;
    for (const auto &pair : locally_matched_pairs)
    {
        vertex_pairs.push_back({triangles_src[pair.first].vertex_a, triangles_src[pair.first].vertex_b, triangles_src[pair.first].vertex_c, triangles_tgt[pair.second].vertex_a, triangles_tgt[pair.second].vertex_b, triangles_tgt[pair.second].vertex_c});
    }

    for (size_t i_grp = 0; i_grp < vertex_pairs.size(); ++i_grp)
    {
        groups[i_grp].push_back(i_grp);
        auto pair_init = vertex_pairs[i_grp];

        for (size_t i_pair = 0; i_pair < vertex_pairs.size(); ++i_pair)
        {
            if (i_grp != i_pair)
            {
                auto pair_cand = vertex_pairs[i_pair];
                if (satisfyGlobalConsistency(triangles_src[pair_init[0] / 3], triangles_tgt[pair_init[3] / 3], stem_positions_src, stem_positions_tar, 0.1))
                {
                    groups[i_grp].push_back(i_pair);
                }
            }
        }
        group_size[i_grp] = groups[i_grp].size();
    }

    auto max_group_it = std::max_element(group_size.begin(), group_size.end());
    int id_max_group = std::distance(group_size.begin(), max_group_it);
    auto globally_matched_pairs = groups[id_max_group];

    std::set<int> unique_src, unique_tgt;
    for (auto idx : globally_matched_pairs)
    {
        for (int i = 0; i < 3; ++i)
        {
            unique_src.insert(vertex_pairs[idx][i]);
            unique_tgt.insert(vertex_pairs[idx][i + 3]);
        }
    }

    numTra = locally_matched_pairs.size();

    std::set<std::pair<int, int>> src_quad, tgt_quad;
    for (size_t i = 0; i < globally_matched_pairs.size(); ++i)
    {
        for (size_t j = i + 1; j < globally_matched_pairs.size(); ++j)
        {
            std::set<int> src_set = {vertex_pairs[i][0], vertex_pairs[i][1], vertex_pairs[i][2], vertex_pairs[j][0], vertex_pairs[j][1], vertex_pairs[j][2]};
            std::set<int> tgt_set = {vertex_pairs[i][3], vertex_pairs[i][4], vertex_pairs[i][5], vertex_pairs[j][3], vertex_pairs[j][4], vertex_pairs[j][5]};
            if (src_set.size() == 4 && tgt_set.size() == 4)
            {
                src_quad.insert({*src_set.begin(), *(++src_set.begin())});
                tgt_quad.insert({*tgt_set.begin(), *(++tgt_set.begin())});
            }
        }
    }
    numQuad = src_quad.size();
}

MatrixXf getIMG(const PointCloudT::Ptr &cloud)
{
    PointCloudT::Ptr ptcloud1(new PointCloudT);
    *ptcloud1 = *cloud;

    // 调用函数 get_normals 计算点云的法线和曲率，并将结果存储在 Normal_cpu 和 Curvature 中
    vector<Vector3f> Normal_cpu;
    vector<float> Curvature;
    get_normals(ptcloud1, 10, Normal_cpu, Curvature);

    Vector3f z(0, 0, 1);
    float height = 30;
    MatrixXf A(ptcloud1->points.size(), 3);

    for (size_t i = 0; i < ptcloud1->points.size(); ++i)
    {
        A.row(i) << ptcloud1->points[i].x, ptcloud1->points[i].y, ptcloud1->points[i].z;
    }

    float verticality = 0.7;
    VectorXf v(Normal_cpu.size());
    for (size_t i = 0; i < Normal_cpu.size(); ++i)
    {
        v[i] = 1 - abs(Normal_cpu[i].dot(z));
    }

    PointCloudT::Ptr tree(new PointCloudT);
    for (size_t i = 0; i < v.size(); ++i)
    {
        if (v[i] > verticality)
        {
            tree->points.push_back(ptcloud1->points[i]);
        }
    }

    // 对过滤后的点云进行主成分分析（PCA），得到特征向量和特征值
    pcl::PCA<PointT> pca;
    pca.setInputCloud(tree);
    Eigen::Matrix3f eigenVectors = pca.getEigenVectors();
    Eigen::Vector3f eigenValues = pca.getEigenValues();

    // 根据特征值确定方向向量 fd
    Vector3f fd;
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
        fd = Vector3f(1, 0, 0);
    }

    // 使用特征向量构造旋转矩阵 R，并调整方向向量 fd
    Matrix3f R;
    R.col(0) = eigenVectors.col(0);
    R.col(1) = eigenVectors.col(1);
    R.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    fd = R * fd;

    // 计算偏航角并构造相应的旋转矩阵 A_rotation
    float yaw = atan2(fd(1), fd(0)) + M_PI / 2;
    Matrix3f A_rotation;
    A_rotation << cos(yaw), sin(yaw), 0, -sin(yaw), cos(yaw), 0, 0, 0, 1;

    // 使用旋转矩阵对过滤后的点云 tree 进行变换
    MatrixXf ptcloud_tran(tree->points.size(), 3);
    for (size_t i = 0; i < tree->points.size(); ++i)
    {
        ptcloud_tran.row(i) << tree->points[i].x, tree->points[i].y, tree->points[i].z;
    }
    ptcloud_tran = ptcloud_tran * A_rotation.transpose();

    // 定义最大范围、扇区数和环数。计算每个点的极坐标 theta 和 r
    float max_range = 80; // meter
    int num_sector = 60;
    int num_ring = 20;

    VectorXf theta(ptcloud_tran.rows());
    VectorXf r(ptcloud_tran.rows());
    for (int i = 0; i < ptcloud_tran.rows(); ++i)
    {
        float x = ptcloud_tran(i, 0);
        float y = ptcloud_tran(i, 1);
        theta[i] = atan2(y, x) + M_PI;
        r[i] = sqrt(x * x + y * y);
    }

    // 根据极坐标将点分组到环和扇区中，得到环索引 r_index 和扇区索引 s_index
    float r_step = max_range / num_ring;
    VectorXi r_index = ((r / r_step).array().ceil()).matrix().cast<int>();
    r_index = r_index.cwiseMin(num_ring);

    float s_step = (360.0 / num_sector) * M_PI / 180.0;
    float eps = 1e-6; // Define epsilon for floating-point comparisons
    theta = (theta.array() > 2 * M_PI - eps).select(0, theta);
    VectorXi s_index = ((theta / s_step).array().ceil()).matrix().cast<int>();
    s_index = s_index.cwiseMax(1);

    // 创建一个全零初始化的图像矩阵 img，将点云分割到环和扇区中，并根据每个区域的点数计算一个得分
    MatrixXf img = MatrixXf::Zero(num_ring, num_sector);
    vector<vector<PointCloudT::Ptr>> segPoint(num_ring, vector<PointCloudT::Ptr>(num_sector, PointCloudT::Ptr(new PointCloudT)));
    for (int i = 1; i <= num_ring; ++i)
    {
        for (int j = 1; j <= num_sector; ++j)
        {
            vector<int> idx;
            for (int k = 0; k < r_index.size(); ++k)
            {
                if (r_index(k) == i && s_index(k) == j)
                {
                    idx.push_back(k);
                }
            }
            PointCloudT::Ptr point(new PointCloudT);
            for (int k : idx)
            {
                point->points.push_back(ptcloud1->points[k]);
            }
            if (point->size() > 1)
            {
                float minDistance = 1.2; // clustering max distance
                int minPoints = 1;       // clustering min points
                int maxPoints = 1000;    // clustering max points
                vector<int> labels;
                vector<float> score;
                pcsegdist(point, minDistance, minPoints, maxPoints, labels, score); // You need to implement pcsegdist function
                img(i - 1, j - 1) = score.size();
            }
            else
            {
                img(i - 1, j - 1) = 0;
            }
        }
    }
    return img;
}

float mean2(const MatrixXf &mat)
{
    return mat.mean();
}

float corr_sim_P(const MatrixXf &ndd1, const MatrixXf &ndd2)
{
    int num_sectors = ndd1.cols();
    VectorXf geo_sim(num_sectors);
    VectorXf row_vec2 = ndd2.colwise().sum();

    for (int i = 0; i < num_sectors; ++i)
    {
        int one_step = 1; // const
        MatrixXf shifted_ndd1 = ndd1;
        std::rotate(shifted_ndd1.data(), shifted_ndd1.data() + one_step, shifted_ndd1.data() + shifted_ndd1.size());
        VectorXf row_vec1 = shifted_ndd1.colwise().sum();
        float norm_row_vec1 = row_vec1.norm();
        float norm_row_vec2 = row_vec2.norm();
        float dot_product = row_vec1.dot(row_vec2);
        geo_sim(i) = dot_product / (norm_row_vec1 * norm_row_vec2);
    }

    int shiftcol;
    geo_sim.maxCoeff(&shiftcol);

    MatrixXf shifted_ndd1 = ndd1;
    std::rotate(shifted_ndd1.data(), shifted_ndd1.data() + shiftcol, shifted_ndd1.data() + shifted_ndd1.size());

    // sim calc.
    MatrixXf a = shifted_ndd1;
    MatrixXf b = ndd2;
    a.array() -= mean2(a);
    b.array() -= mean2(b);
    float corr_similarity = (a.array() * b.array()).sum() / sqrt((a.array() * a.array()).sum() * (b.array() * b.array()).sum());

    return corr_similarity;
}

template <typename Point, size_t DIM>
struct KDTree
{
public:
    void insert(const Point &point)
    {
        values.push_back(point);
        rebuild_tree();
    }

    void search_knn(const Point &point, int k, std::vector<Point> &result, std::vector<size_t> &result_index) const
    {
        result_index.resize(k);
        result.resize(k);
        if (tree == nullptr)
        {
            result_index.clear();
            result.clear();
            return;
        }
        std::vector<double> out_dists_sqr(k);
        nanoflann::KNNResultSet<double> knnsearch_result(k);        // 预定义
        knnsearch_result.init(&result_index[0], &out_dists_sqr[0]); // 初始化
        tree->index->findNeighbors(knnsearch_result, point.data() /* query */, nanoflann::SearchParams(10));
    }

    void rebuild_tree()
    {
        if (values.size() > last_count + 50)
        {
            if (tree != nullptr)
                delete tree;
            tree = new KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L2>(DIM, values, 10);
            last_count = values.size();
        }
    }

    size_t last_count = 0;
    KDTreeVectorOfVectorsAdaptor<std::vector<Point>, double, DIM, nanoflann::metric_L2> *tree = nullptr;
    std::vector<Point> values;
};

// ============================= KITTI =============================

void LCDManager::KITTI_ESF(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/ESF_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        KDTree<std::array<double, 640>, 640> kdtree;
        std::vector<std::array<double, 640>> history_points;
        std::vector<std::array<double, 640>> result;
        std::vector<size_t> result_index;
        std::queue<std::array<double, 640>> qe; // 缓冲队列，不考虑前50帧
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
            pcl::ESFEstimation<PointType, pcl::ESFSignature640> ESF;
            ESF.setInputCloud(pc_to_read);
            ESF.compute(*esf_descriptor);
            // 将esf_descriptor转换为y方向的vector
            std::vector<float> esf_descriptor_y;
            for (int i = 0; i < 640; i++)
            {
                esf_descriptor_y.push_back(esf_descriptor->points[0].histogram[i]);
            }
            // 将esf_descriptor_x转换为point_type
            std::array<double, 640> point;
            for (int i = 0; i < 640; i++)
            {
                point[i] = esf_descriptor_y[i];
            }
            qe.push(point);
            history_points.push_back(point);
            if (qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if (cloud_i < 50) // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(point, 1, result, result_index);
            int loopFrameId = result_index[0];
            double dist = 0;
            // 计算两个向量的L1距离
            std::array<double, 640> history = history_points[loopFrameId];
            for (int i = 0; i < 640; i++)
            {
                dist += fabs(point[i] - history[i]);
            }
            std::cout << "No." << bin_number << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_M2DP(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/M2DP_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        KDTree<std::array<double, 192>, 192> kdtree;
        std::vector<std::array<double, 192>> history_points;
        std::vector<std::array<double, 192>> result;
        std::vector<size_t> result_index;
        std::queue<std::array<double, 192>> qe; // 缓冲队列，不考虑前50帧
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            M2DP m2dp(current_cloud);
            Eigen::Matrix<double, 1, 192> desM2dp;
            desM2dp = m2dp.get_m2dp_result();
            std::array<double, 192> A_m2dp_point;
            for (int i = 0; i < 192; i++)
            {
                A_m2dp_point[i] = desM2dp(0, i);
            }
            qe.push(A_m2dp_point);
            history_points.push_back(A_m2dp_point);
            if (qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if (cloud_i < 50) // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(A_m2dp_point, 1, result, result_index);
            int loopFrameId = result_index[0];
            // 计算两个向量的L2距离
            double dist = 0;
            std::array<double, 192> history = history_points[loopFrameId];
            for (int i = 0; i < 192; i++)
            {
                dist += pow(A_m2dp_point[i] - history[i], 2);
            }
            dist = sqrt(dist);

            std::cout << "No." << cloud_i << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << bin_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_NDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/home/jm/catkin_ws/data/kitti_bin/00/";
    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // 输出所有结果

    /*
    static std::once_flag flag;
    std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("../results/SC.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    */

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/NDD_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file;
    // loop_file.open("../results/SC.txt", std::ios::app);
    loop_file.open(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        myNDD::NDDManager NDD_demo;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            NDD_demo.makeAndSaveNDDAndKeys(laserCloudIn);
            std::pair<int, float> result = NDD_demo.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_Iris_Force(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/Iris_KITTI_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        int loop_event = 2;
        if (dataset_sequence == "00" || dataset_sequence == "05")
        {
            loop_event = 0;
        }
        else if (dataset_sequence == "08" || dataset_sequence == "051" || dataset_sequence == "052" || dataset_sequence == "061" || dataset_sequence == "0621" || dataset_sequence == "0622")
        {
            loop_event = 1;
        }
        else if (dataset_sequence == "02")
        {
            loop_event = 2;
        }
        LidarIris iris(4, 18, 1.6, 0.75, loop_event);
        std::vector<LidarIris::FeatureDesc> dataset(file_list.size());
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;

            cv::Mat1b li1 = LidarIris::GetIris(laserCloudIn);
            LidarIris::FeatureDesc fd1 = iris.GetFeature(li1);
            dataset[cloud_i] = fd1;

            if (cloud_i < 50) // 前50帧不考虑回环
            {
                loop_file << bin_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            float min_dis = 100000.0;
            int loop_id = -1;
            for (size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
            {
                LidarIris::FeatureDesc fd2 = dataset[cloud_j];

                int bias;
                auto dis = iris.Compare(fd1, fd2, &bias);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    loop_id = cloud_j;
                }
            }
            if (loop_id == -1)
                min_dis = 0.0;

            std::cout << "No." << bin_number << ", loopFrameId: " << loop_id << ", dist: " << min_dis << std::endl;
            loop_file << bin_number << " " << loop_id << " " << (float)min_dis << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_Iris(std::string dataset_sequence)
{

    return;
}

void LCDManager::KITTI_SC_Intensity(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_Intensity_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        ISCManager SC_Intensity;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC_Intensity.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC_Intensity.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_ISC(std::string dataset_sequence)
{
    return;
}

void LCDManager::KITTI_IDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/home/jm/catkin_ws/data/kitti_bin/00/";
    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // 输出所有结果

    /*
    static std::once_flag flag;
    std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("../results/SC.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    */

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/IDD_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file;
    // loop_file.open("../results/SC.txt", std::ios::app);
    loop_file.open(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        myIDD::SCManager IDD_demo;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            IDD_demo.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = IDD_demo.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_CSSC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_KITTI_051.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_KITTI_052.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_KITTI_061.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_KITTI_0621.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_KITTI_0622.txt", std::ios::out);
    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        csscM CSSC;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr cloudPtr = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            CSSC.makeAndSaveCSSCAndFingerprint(cloudPtr, 64);
            std::pair<int, float> result = CSSC.detectLoopClosureIDAndDis();
            std::cout << "No." << bin_number << ": " << result.first << " " << result.second << std::endl;
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_BoW3D(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/BoW3D_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {

        // 设置 LinK3D 的参数
        int nScans = 64;          // 激光雷达扫描线的数量
        float scanPeriod = 0.1;   // 激光雷达扫描周期
        float minimumRange = 0.1; // 最小测距范围
        float distanceTh = 0.4;   // 距离阈值
        int matchTh = 6;          // 匹配阈值

        // 设置 BoW3D 的参数
        float thr = 3.5;                   // 阈值
        int thf = 5;                       // 阈值
        int num_add_retrieve_features = 5; // 添加和检索特征的数量

        // 创建 LinK3D_Extractor 和 BoW3D 对象
        BoW3D::LinK3D_Extractor *pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
        BoW3D::BoW3D *pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            pcl::PointCloud<PointType>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<PointType>>(pc_to_read);

            // 创建当前帧的Frame对象，并更新BoW3D
            BoW3D::Frame *pCurrentFrame = new BoW3D::Frame(pLinK3dExtractor, current_cloud);

            int loopFrameId = -1;      // 回环帧ID
            int matchKeypointNums = 0; // 匹配到的特征关键点数量
            Eigen::Matrix3d loopRelR;  // 旋转
            Eigen::Vector3d loopRelt;  // 平移

            if (pCurrentFrame->mnId < 50) // 前50帧不考虑回环，如考虑，将50换成2
            {
                // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
                pBoW3D->update(pCurrentFrame);
                loop_file << bin_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
                continue;
            }

            clock_t start, end;
            double time;
            start = clock();
            // 检索回环帧，并得到相对姿态信息
            // pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, matchKeypointNums, loopRelR, loopRelt);

            end = clock();
            time = ((double)(end - start)) / CLOCKS_PER_SEC;

            // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
            pBoW3D->update(pCurrentFrame);

            if (loopFrameId == -1)
            {
                cout << "-------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl;
            }
            else
            {
                cout << "--------------------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;

                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;

                cout << "Loop Relative t: " << endl;
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
            }

            loop_file << bin_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::KITTI_Scan_Context(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/home/jm/catkin_ws/data/kitti_bin/00/";
    std::string data_root = "/home/zw/桌面/KITTI/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.kitti_bin_loader(data_root, dataset_sequence, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // 输出所有结果

    /*
    static std::once_flag flag;
    std::call_once(flag, []()
        {
            //delete file loop.txt
            std::ofstream loop_file("../results/SC.txt", std::ios::out);
            loop_file.close();
        });
    //输出当前帧ID和min_dist到文件中
    */

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_KITTI_" + dataset_sequence + ".txt";
    std::ofstream loop_file;
    // loop_file.open("../results/SC.txt", std::ios::app);
    loop_file.open(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        SCManager SC;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string bin_path = file_list[cloud_i].file_name;
            int bin_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::ConstPtr pc_to_read = readKITTIPointCloudBin<PointType>(bin_path);
            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << bin_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

// ============================= JORD =============================

void LCDManager::JORD_ESF(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/ESF_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        KDTree<std::array<double, 640>, 640> kdtree;
        std::vector<std::array<double, 640>> history_points;
        std::vector<std::array<double, 640>> result;
        std::vector<size_t> result_index;
        std::queue<std::array<double, 640>> qe; // 缓冲队列，不考虑前50帧
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            // pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(pc_to_read);

            pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
            pcl::ESFEstimation<PointType, pcl::ESFSignature640> ESF;
            ESF.setInputCloud(pc_to_read);
            ESF.compute(*esf_descriptor);
            // 将esf_descriptor转换为y方向的vector
            std::vector<float> esf_descriptor_y;
            for (int i = 0; i < 640; i++)
            {
                esf_descriptor_y.push_back(esf_descriptor->points[0].histogram[i]);
            }
            // 将esf_descriptor_x转换为point_type
            std::array<double, 640> point;
            for (int i = 0; i < 640; i++)
            {
                point[i] = esf_descriptor_y[i];
            }
            qe.push(point);
            history_points.push_back(point);
            if (qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if (cloud_i < 50) // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(point, 1, result, result_index);
            int loopFrameId = result_index[0];
            double dist = 0;
            // 计算两个向量的L1距离
            std::array<double, 640> history = history_points[loopFrameId];
            for (int i = 0; i < 640; i++)
            {
                dist += fabs(point[i] - history[i]);
            }
            std::cout << "No." << pcd_number << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_M2DP(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/M2DP_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        KDTree<std::array<double, 192>, 192> kdtree;
        std::vector<std::array<double, 192>> history_points;
        std::vector<std::array<double, 192>> result;
        std::vector<size_t> result_index;
        std::queue<std::array<double, 192>> qe; // 缓冲队列，不考虑前50帧
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            // pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud = boost::const_pointer_cast<pcl::PointCloud<pcl::PointXYZ>>(pc_to_read);

            M2DP m2dp(pc_to_read);
            Eigen::Matrix<double, 1, 192> desM2dp;
            desM2dp = m2dp.get_m2dp_result();
            std::array<double, 192> A_m2dp_point;
            for (int i = 0; i < 192; i++)
            {
                A_m2dp_point[i] = desM2dp(0, i);
            }
            qe.push(A_m2dp_point);
            history_points.push_back(A_m2dp_point);
            if (qe.size() > 50)
            {
                kdtree.insert(qe.front());
                qe.pop();
            }

            if (cloud_i < 50) // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            // 搜索最近的候选帧
            kdtree.search_knn(A_m2dp_point, 1, result, result_index);
            int loopFrameId = result_index[0];
            // 计算两个向量的L2距离
            double dist = 0;
            std::array<double, 192> history = history_points[loopFrameId];
            for (int i = 0; i < 192; i++)
            {
                dist += pow(A_m2dp_point[i] - history[i], 2);
            }
            dist = sqrt(dist);

            std::cout << "No." << cloud_i << ", loopFrameId: " << loopFrameId << ", dist: " << dist << std::endl;
            loop_file << pcd_number << " " << loopFrameId << " " << (float)dist << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_Scan_Context(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/051/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/052/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/061/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0621/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0622/";
    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_051.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_052.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_061.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0621.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0622.txt", std::ios::out);
    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        SCManager SC;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_Iris_Force(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/Iris_JORD_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        int loop_event = 2;
        if (dataset_sequence == "00" || dataset_sequence == "05")
        {
            loop_event = 0;
        }
        else if (dataset_sequence == "08" || dataset_sequence == "051" || dataset_sequence == "052" || dataset_sequence == "061" || dataset_sequence == "0621" || dataset_sequence == "0622")
        {
            loop_event = 1;
        }
        else if (dataset_sequence == "02")
        {
            loop_event = 2;
        }
        LidarIris iris(4, 18, 1.6, 0.75, loop_event);
        std::vector<LidarIris::FeatureDesc> dataset(file_list.size());
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {

            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;

            cv::Mat1b li1 = LidarIris::GetIris(laserCloudIn);
            LidarIris::FeatureDesc fd1 = iris.GetFeature(li1);
            dataset[cloud_i] = fd1;

            if (cloud_i < 50) // 前50帧不考虑回环
            {
                loop_file << pcd_number << " " << (int)-1 << " " << (float)0.00 << std::endl;
                continue;
            }

            float min_dis = 100000.0;
            int loop_id = -1;
            for (size_t cloud_j = 0; cloud_j <= cloud_i - 50; cloud_j++)
            {
                LidarIris::FeatureDesc fd2 = dataset[cloud_j];

                int bias;
                auto dis = iris.Compare(fd1, fd2, &bias);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    loop_id = cloud_j;
                }
            }
            if (loop_id == -1)
                min_dis = 0.0;

            std::cout << "No." << pcd_number << ", loopFrameId: " << loop_id << ", dist: " << min_dis << std::endl;
            loop_file << pcd_number << " " << loop_id << " " << (float)min_dis << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_Iris(std::string dataset_sequence)
{
    return;
}

void LCDManager::JORD_SC_Intensity(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_Intensity_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        ISCManager SC_Intensity;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            SC_Intensity.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = SC_Intensity.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();
    return;
}

void LCDManager::JORD_ISC(std::string dataset_sequence)
{

    return;
}

void LCDManager::JORD_Seed(std::string dataset_sequence)
{

    return;
}

void LCDManager::JORD_CSSC_Force(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_Force_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path_now = file_list[cloud_i].file_name;
            int pcd_number_now = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read_now(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path_now, *pc_to_read_now) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudInNow = *pc_to_read_now;

            double min_dis = 2.00;
            int index = -1;
            if (cloud_i >= 50)
            { // 跳过前50帧
                for (size_t cloud_j = 0; cloud_j < cloud_i - 50; cloud_j++)
                {

                    // std::cout << cloud_j << std::endl;
                    string pcd_path_pre = file_list[cloud_j].file_name;
                    // int pcd_number_pre = file_list[cloud_j].order_number;
                    pcl::PointCloud<PointType>::Ptr pc_to_read_pre(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

                    // 加载PCD文件
                    if (pcl::io::loadPCDFile<PointType>(pcd_path_pre, *pc_to_read_pre) == -1)
                    {
                        PCL_ERROR("Couldn't read PCD file.\n");
                        // return -1;
                        return;
                    }

                    // pcl::PointCloud<pcl::PointXYZ> laserCloudInPre = *pc_to_read_pre;

                    csscM CSSC; // 创建了一个CSSC对象，用于计算LiDAR点云之间的相似性
                    // 调用CSSC对象的 calculateDistanceBtnPC 函数，传入两个点云数据 cloud1 和 cloud2，计算它们之间的相似性。
                    // 该函数返回一个 std::pair<float, float> 类型的结果，其中first表示计算得到的相似性值，second表示对应的下标。

                    std::pair<double, int> result = CSSC.calculateDistanceBtnPC(pc_to_read_pre, pc_to_read_now, 16);
                    if (result.first < min_dis)
                    {
                        min_dis = result.first;
                        index = cloud_j;
                    }
                }
            }
            if (index == -1)
                min_dis = 0.0;
            std::cout << "No. " << pcd_number_now << " " << index << " " << min_dis << std::endl;
            loop_file << pcd_number_now << " " << index << " " << min_dis << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_CSSC(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_051.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_052.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_061.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_0621.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_0622.txt", std::ios::out);
    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/CSSC_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        csscM CSSC;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;
            CSSC.makeAndSaveCSSCAndFingerprint(pc_to_read, 16);
            std::pair<int, float> result = CSSC.detectLoopClosureIDAndDis();
            std::cout << "No." << pcd_number << ": " << result.first << " " << result.second << std::endl;
            // loop_file << pcd_number + 1 << " " << result.first << " " << result.second << std::endl;
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_BoW3D(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZ;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/BoW3D_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {

        // 设置 LinK3D 的参数
        int nScans = 16;          // 激光雷达扫描线的数量
        float scanPeriod = 0.1;   // 激光雷达扫描周期
        float minimumRange = 0.1; // 最小测距范围
        float distanceTh = 0.4;   // 距离阈值
        int matchTh = 6;          // 匹配阈值

        // 设置 BoW3D 的参数
        float thr = 3.5;                   // 阈值
        int thf = 5;                       // 阈值
        int num_add_retrieve_features = 5; // 添加和检索特征的数量

        // 创建 LinK3D_Extractor 和 BoW3D 对象
        BoW3D::LinK3D_Extractor *pLinK3dExtractor = new BoW3D::LinK3D_Extractor(nScans, scanPeriod, minimumRange, distanceTh, matchTh);
        BoW3D::BoW3D *pBoW3D = new BoW3D::BoW3D(pLinK3dExtractor, thr, thf, num_add_retrieve_features);

        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {

            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;

            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            // pcl::PointCloud<pcl::PointXYZ> laserCloudIn = *pc_to_read;

            // 创建当前帧的Frame对象，并更新BoW3D
            BoW3D::Frame *pCurrentFrame = new BoW3D::Frame(pLinK3dExtractor, pc_to_read);

            int loopFrameId = -1;      // 回环帧ID
            int matchKeypointNums = 0; // 匹配到的特征关键点数量
            Eigen::Matrix3d loopRelR;  // 旋转
            Eigen::Vector3d loopRelt;  // 平移

            if (pCurrentFrame->mnId < 50) // 前50帧不考虑回环，如考虑，将50换成2
            {
                // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
                pBoW3D->update(pCurrentFrame);
                loop_file << pcd_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
                continue;
            }

            clock_t start, end;
            double time;
            start = clock();
            // 检索回环帧，并得到相对姿态信息
            // pBoW3D->retrieve(pCurrentFrame, loopFrameId, loopRelR, loopRelt);
            pBoW3D->retrieve(pCurrentFrame, loopFrameId, matchKeypointNums, loopRelR, loopRelt);

            end = clock();
            time = ((double)(end - start)) / CLOCKS_PER_SEC;

            // 将当前帧的信息更新到Bow3D的map映射中，更新N_nw_ofRatio，以便求取论文中的比率ratio
            pBoW3D->update(pCurrentFrame);

            if (loopFrameId == -1)
            {
                cout << "-------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has No Loop..." << endl;
            }
            else
            {
                cout << "--------------------------------------" << endl;
                cout << "Detection Time: " << time << "s" << endl;
                cout << "Frame" << pCurrentFrame->mnId << " Has Loop Frame" << loopFrameId << endl;

                cout << "Loop Relative R: " << endl;
                cout << loopRelR << endl;

                cout << "Loop Relative t: " << endl;
                cout << "   " << loopRelt.x() << " " << loopRelt.y() << " " << loopRelt.z() << endl;
            }

            loop_file << pcd_number << " " << loopFrameId << " " << matchKeypointNums << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_STD(std::string dataset_sequence)
{

    return;
}

void LCDManager::JORD_Contour_Context(std::string dataset_sequence)
{

    return;
}

void LCDManager::JORD_NDT_Map_Code(std::string dataset_sequence)
{

    return;
}

void LCDManager::JORD_NDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/051/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/052/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/061/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0621/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0622/";
    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_051.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_052.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_061.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0621.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0622.txt", std::ios::out);
    std::string loop_file_name = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/NDD_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        myNDD::NDDManager NDD_demo;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            NDD_demo.makeAndSaveNDDAndKeys(laserCloudIn);
            std::pair<int, float> result = NDD_demo.detectLoopClosureID();
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

void LCDManager::JORD_IDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";
    std::string loop_file_name1 = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/LGD_Time_051_ED1.txt";
    std::string loop_file_name2 = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/LGD_Time_051_PC.txt";
    std::string loop_file_name3 = "/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/LGD_Time_051_MQ.txt";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }
    std::ofstream time_file1(loop_file_name1,std::ios::out);
    std::ofstream time_file2(loop_file_name2,std::ios::out);
    std::ofstream time_file3(loop_file_name3,std::ios::out);
    std::string loop_file_name = "/home/zw/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/LGD_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        // myIDD::SCManager IDD_demo;
        // 声明一个二维向量（嵌套向量），用于存储三角形数据
        vector<vector<Triangle>> trianglesDS;
        // 声明一个二维向量，存储成对的双精度浮点数，表示树的点
        vector<vector<pair<double, double>>> treePoints;
        double delta_l = 1;
        const int skip = 300;
        int p = 1000007;
        int B = 5000000;
        int num_pc = file_list.size();

        // std::vector<Eigen::MatrixXd> treePoints(num_pc);
        std::vector<HashTable> all_hash_table(num_pc);
        std::vector<std::vector<std::string>> each_pc_hash_key(num_pc);
        std::vector<std::vector<int>> Candidate(num_pc);
        GlobalHashTable global_hash_table;  // 声明一个全局哈希表 GlobalHashTable
        std::vector<std::tuple<int, int, float>> xx;  // 声明一个元组向量，存储三个值（两个整数和一个浮点数）
        std::unordered_map<std::string, std::vector<int>> globalHashTable;  // 声明一个无序映射，键为字符串，值为整数向量
        std::vector<MatrixXf> AllIMG;
        for (size_t cloud_i = 0; cloud_i < 3508; cloud_i++)
        {
            cout << "----------" << cloud_i << "----------" << endl;
            //  cout<<"----------"<<endl;
            // 获取当前点云文件的路径和序号
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            // 声明并初始化两个点云指针
            PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
            PointCloud<PointXYZ>::Ptr segmentedPtCloud(new PointCloud<PointXYZ>);

            if (io::loadPCDFile<PointXYZ>(pcd_path, *cloud) == -1)
            {
                PCL_ERROR("Couldn't read file 0.pcd \n");
            }
            else
            {
                // 声明一个成对的双精度浮点数向量 clusterMeans
                vector<pair<double, double>> clusterMeans;
                // 获取图片数据并存入 AllIMG 向量
                MatrixXf img = getIMG(cloud);
                AllIMG.push_back(img);
                
                // 执行树的分割操作，并获取分割结果
                SegmentedResult result = segmentTree(cloud);
                segmentedPtCloud = result.segmentedPtCloud;
                PointCloud<PointXYZ>::Ptr extraGround = result.extraGround;

                // 声明一个 PointCloud1 类型的对象，并调整其大小
                PointCloud1 pc;
                pc.Location.resize(segmentedPtCloud->size(), 3);
                // 将分割点云数据复制到 pc 对象
                for (size_t i = 0; i < segmentedPtCloud->size(); ++i)
                {
                    pc.Location(i, 0) = segmentedPtCloud->points[i].x;
                    pc.Location(i, 1) = segmentedPtCloud->points[i].y;
                    pc.Location(i, 2) = segmentedPtCloud->points[i].z;
                }
                // 记录当前时间
                auto start_time = std::chrono::high_resolution_clock::now();
                // 聚类操作并获取结果
                auto result1 = clusterXY(pc);
                vector<vector<Vector3d>> ClusterXYZ = result1.second;
                // pcl::visualization::PCLVisualizer viewer("Cluster visualization");
/*
                pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
                for (const auto &point : arr)
                {
                    PointT pcl_point;
                    pcl_point.x = point(0);
                    pcl_point.y = point(1);
                    pcl_point.z = point(2);
                    cloud->push_back(pcl_point);
                }
*/
                // 遍历聚类结果，计算每个聚类的中心点
                for (const auto &cluster : ClusterXYZ)
                {
                    double sumX = 0.0, sumY = 0.0;
                    for (const auto &point : cluster)
                    {
                        sumX += point(0);
                        sumY += point(1);
                    }
                    double meanX = sumX / cluster.size();
                    double meanY = sumY / cluster.size();
                    //   std::cout << "meanX: " << meanX << ", meanY: " << meanY << std::endl; // Output meanX and meanY
                    clusterMeans.push_back({meanX, meanY});
                } 

                // 将聚类中心点加入 treePoints 向量
                treePoints.push_back(clusterMeans);
                // 定义用于三角形排序的参数
                int max_stems_for_exhaustive_search = 5;
                int knn = 2;
                
                // 获取排序后的三角形并记录结束时间
                vector<Triangle> triangles = getSortTriangles(clusterMeans, max_stems_for_exhaustive_search, knn);
                auto end_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> exec_time1 = end_time - start_time;
                time_file1 << exec_time1.count() << std::endl;

                // 将三角形数据加入 trianglesDS 向量
                trianglesDS.push_back(triangles);

                // 构建哈希表并获取对应的键值
                HashTable hash_table = constructHashTable(triangles, delta_l, p, B, cloud_i);

                std::vector<std::string> keys;
                for (const auto &kv : hash_table)
                {
                    keys.push_back(kv.first);
                }
                each_pc_hash_key[cloud_i] = keys;
                all_hash_table[cloud_i] = hash_table;
                // printHashTable(hash_table);
            }

            // 如果当前索引大于等于 skip
            if (cloud_i >= skip)
            {

                // 获取之前的哈希表和键值
                HashTable hash_tables = all_hash_table[cloud_i - skip];
                auto keys = each_pc_hash_key[cloud_i - skip];

                // 合并本地和全局哈希表
                for (const auto &pair : hash_tables)
                {
                    const std::string &ckey = pair.first;
                    std::vector<int> localValues;
                    for (const auto &triangle_info : pair.second)
                    {
                        localValues.push_back(triangle_info.index);
                    }
                    std::sort(localValues.begin(), localValues.end());
                    localValues.erase(std::unique(localValues.begin(), localValues.end()), localValues.end());

                    if (globalHashTable.find(ckey) != globalHashTable.end())
                    {
                        std::vector<int> &globalValues = globalHashTable[ckey];
                        globalValues.insert(globalValues.end(), localValues.begin(), localValues.end());
                    }
                    else
                    {
                        globalHashTable[ckey] = localValues;
                    }
                }
                // cout<<"正常"<<endl;
                // 获取当前点云的哈希键值
                std::vector<std::string> src_hash_keys = each_pc_hash_key[cloud_i];
                auto [uniqueIndices, count, repeatedCount] = getKeyIndicesAndCount1(globalHashTable, src_hash_keys);

                // 获取唯一索引和对应的三角形数据
                std::vector<int> hash_index = uniqueIndices;
                vector<Triangle> triangles_src = trianglesDS[cloud_i];
                vector<pair<double, double>> stem_positions1 = treePoints[cloud_i];

                // 声明 TraMatrix 和 QuadMatrix 向量
                std::vector<int> TraMatrix(hash_index.size());
                std::vector<int> QuadMatrix(hash_index.size());

                //cout << hash_index.size() << endl;
                // 记录当前时间
                auto start_time3 = std::chrono::high_resolution_clock::now();

                // 遍历每个索引，获取对应的三角形和位置数据，并计算 Tra 和 Quad
                for (int i = 0; i < hash_index.size(); i++)
                {
                    auto new_ind = hash_index[i];
                    vector<Triangle> triangles_tgt = trianglesDS[new_ind];
                    vector<pair<double, double>> stem_positions2 = treePoints[new_ind];

                    int numTra, numQuad;
                    getTraAndQuad(triangles_src, triangles_tgt, stem_positions1, stem_positions2, numTra, numQuad);
                    // getTraAndQuad(triangles_tgt, triangles_src, numTra, numQuad);
                    TraMatrix[i] = numTra;
                    // cout<<numTra<<endl;
                    QuadMatrix[i] = numQuad;
                }
                // 记录结束时间并计算执行时间
                auto end_time3 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> exec_time3 = end_time3 - start_time3;
                time_file3 << exec_time3.count() << std::endl;


                // 对 QuadMatrix 进行排序，获取对应的索引
                std::vector<int> id(QuadMatrix.size());
                std::iota(id.begin(), id.end(), 0);
                std::sort(id.begin(), id.end(), [&](int a, int b)
                          { return QuadMatrix[a] > QuadMatrix[b]; });
                /*
                                std::vector<int> id(TraMatrix.size());
                                std::iota(id.begin(), id.end(), 0);
                                std::sort(id.begin(), id.end(), [&](int a, int b)
                                          { return TraMatrix[a] > TraMatrix[b]; });
                */
                // 声明 Candidate 向量
                std::vector<int> Candidate;

                // 如果 id 大小小于 10，则将所有索引加入 Candidate，否则只加入前 10 个
                if (id.size() < 10)
                {
                    for (size_t i = 0; i < id.size(); ++i)
                    {
                        Candidate.push_back(hash_index[id[i]]);
                    }
                }
                else
                {
                    for (size_t i = 0; i < 10; ++i)
                    {
                        Candidate.push_back(hash_index[id[i]]);
                    }
                }
                // 打印 Candidate
                // 获取当前图片数据
                MatrixXf src_img = AllIMG[cloud_i];
                float maxSim = -1;
                int maxInd = -1;
                // 记录当前时间
                auto start_time1 = std::chrono::high_resolution_clock::now();
                // 遍历 Candidate，计算相似度，并获取最大相似度及其索引
                for (const int index : Candidate)
                {

                    MatrixXf tgt_img = AllIMG[index];
                    
                    float result = corr_sim_P(src_img, tgt_img);
                    
                    if (result > maxSim)
                    {
                        maxSim = result;
                        maxInd = index;
                    }
                }
                // 记录结束时间并计算执行时间
                auto end_time1 = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> exec_time2 = end_time1- start_time1;
                time_file2 << exec_time2.count() << std::endl;
                // 将结果存入 xx 向量并输出
                xx.emplace_back(cloud_i, maxInd, maxSim);
                std::cout << "No." << cloud_i << ", loopFrameId: " << maxInd << ", dist: " << maxSim << std::endl;
                loop_file << cloud_i << " " << maxInd << " " << 1 - (float)maxSim << std::endl;
            }
            else
            {
                // 如果当前索引小于 skip，存入默认结果并输出
                xx.emplace_back(cloud_i, -1, 0.0f);
                std::cout << "No." << cloud_i << ", loopFrameId: " << -1 << ", dist: " << 0 << std::endl;
                loop_file << cloud_i << " " << -1 << " " << 0 << std::endl;
            }
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}

/*
void LCDManager::JORD_IDD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/051/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/052/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/061/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0621/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0622/";
    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_051.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_052.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_061.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0621.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0622.txt", std::ios::out);
    std::string loop_file_name = "/home/zw/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/IDD_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        myIDD::SCManager IDD_demo;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            IDD_demo.makeAndSaveScancontextAndKeys(laserCloudIn);
            std::pair<int, float> result = IDD_demo.detectLoopClosureID(pcd_number);
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();

    return;
}
*/
void LCDManager::JORD_OFD(std::string dataset_sequence)
{
    using PointType = pcl::PointXYZI;

    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/051/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/052/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/061/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0621/";
    // std::string data_root = "/mnt/1t/dataset/JORD/jord_pcd/0622/";
    std::string data_root = "/home/zw/桌面/JORD2/JORD2_pcd/" + dataset_sequence + "/";

    vector<DataFile> file_list;
    DataLoader data_loader;
    if (data_loader.jord_pcd_loader(data_root, file_list) == -1)
    {
        printf("No file loaded.\n");
        // return -1;
        return;
    }

    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_051.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_052.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_061.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0621.txt", std::ios::out);
    // std::ofstream loop_file("/home/zw/桌面/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/SC_JORD_0622.txt", std::ios::out);
    std::string loop_file_name = "/home/zw/JM_LCD(0808)/catkin_ws/src/JM_LCD/results/OFD_JORD_" + dataset_sequence + ".txt";
    std::ofstream loop_file(loop_file_name, std::ios::out);

    if (loop_file.is_open())
    {
        myIDD::SCManager OFD_demo;
        for (size_t cloud_i = 0; cloud_i < file_list.size(); cloud_i++)
        {
            string pcd_path = file_list[cloud_i].file_name;
            int pcd_number = file_list[cloud_i].order_number;
            pcl::PointCloud<PointType>::Ptr pc_to_read(new pcl::PointCloud<PointType>); // 这里不能用 ConstPtr

            // 加载PCD文件
            if (pcl::io::loadPCDFile<PointType>(pcd_path, *pc_to_read) == -1)
            {
                PCL_ERROR("Couldn't read PCD file.\n");
                // return -1;
                return;
            }

            pcl::PointCloud<PointType> laserCloudIn = *pc_to_read;
            OFD_demo.makeAndSaveScancontextAndKeys(laserCloudIn);
            if (pcd_number < 50)
            {
                loop_file << pcd_number << " " << -1 << " " << 0 << std::endl;
                continue;
            }
            std::pair<int, float> result = OFD_demo.detectLoopClosureID(pcd_number);
            // cout << bin_number << "    " << result.first << "    " << result.second << '\n';
            loop_file << pcd_number << " " << result.first << " " << result.second << std::endl;
        }
    }
    else
    {
        printf("Outputfile can't open.\n");
        // return -1;
        return;
    }
    loop_file.close();
}