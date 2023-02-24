// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filterCloud(new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes, filterRes, filterRes);
    sor.filter(*filterCloud);

    // create cropBox object
    typename pcl::PointCloud<PointT>::Ptr roiCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roiBox(true);
    roiBox.setInputCloud(filterCloud);
    roiBox.setMax(maxPoint);
    roiBox.setMin(minPoint);
    roiBox.filter(*roiCloud);

    // remove roof points
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roof.setInputCloud(roiCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (const int &id : indices)
        inliers->indices.push_back(id);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(roiCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*roiCloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return roiCloud;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>);

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;
    // Extract the inliers
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    extract.setNegative(true);
    extract.filter(*obstCloud);

    extract.setNegative(false);
    extract.filter(*planeCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);

    return segResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::pclSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    // Create the filtering object

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    return segResult;
}

template <typename PointT>
std::vector<int> ProcessPointClouds<PointT>::RansacPlane(const typename pcl::PointCloud<PointT>::Ptr &cloud, int maxIterations, const float &distanceTol)
{
    int &&len = cloud->points.size();
    std::vector<int> inliersResult;
    srand((unsigned)time(NULL));

    while (maxIterations-- > 0)
    {
        // choose three random points
        std::unordered_set<int> initialSet;
        while (initialSet.size() < 3)
            initialSet.insert(rand() % len);

        std::vector<int> tmpInliers;
        for (const int &i : initialSet)
            tmpInliers.push_back(i);

        const PointT &p1 = cloud->points[tmpInliers[0]];
        const PointT &p2 = cloud->points[tmpInliers[1]];
        const PointT &p3 = cloud->points[tmpInliers[2]];

        // Plane function
        double &&a = ((p2.y - p1.y) * (p3.z - p1.z) - (p2.z - p1.z) * (p3.y - p1.y));
        double &&b = ((p2.z - p1.z) * (p3.x - p1.x) - (p2.x - p1.x) * (p3.z - p1.z));
        double &&c = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));
        double &&d = (0 - (a * p1.x + b * p1.y + c * p1.z));
        float &&dem = sqrt(a * a + b * b + c * c);

        for (int i = 0; i < len; ++i)
        {
            if (i == tmpInliers[0] || i == tmpInliers[1] || i == tmpInliers[2])
                continue;

            const PointT &point = cloud->points[i];
            float &&dist = fabs(a * point.x + b * point.y + c * point.z + d) / dem;
            if (dist - distanceTol < 1E-6)
                tmpInliers.push_back(i);
        }

        if (tmpInliers.size() > inliersResult.size())
            inliersResult = move(tmpInliers);
    }
    return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::projectSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Implement RANSAC Plane
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    std::vector<int> &&idSet = RansacPlane(cloud, maxIterations, distanceThreshold);

    if (idSet.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    for (const int &id : idSet)
        inliers->indices.push_back(id);

    // Create the filtering object

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "[+] project RANSAC plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);

    return segResult;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::pclClustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float &clusterTolerance, const int &minSize, const int &maxSize)
{
    std::cout << "[+] pcl clustering library" << std::endl;
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    std::vector<typename pcl::PointCloud<PointT>::Ptr> retClusters;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance); // 2cm
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (const auto &cluster : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr tmpCloud(new pcl::PointCloud<PointT>);

        for (const int &idx : cluster.indices)
            tmpCloud->points.push_back(cloud->points[idx]);

        tmpCloud->width = tmpCloud->points.size();
        tmpCloud->height = 1;
        tmpCloud->is_dense = true;

        std::cout << "[+] pcl library cluster size: " << tmpCloud->points.size() << " data points." << std::endl;
        retClusters.push_back(move(tmpCloud));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "[+] pcl clustering took " << elapsedTime.count() << " milliseconds and found " << retClusters.size() << " clusters" << std::endl;

    return retClusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &visted, const int &id, KdTree *&tree, const float &distTol)
{
    visted[id] = true;
    cluster.push_back(id);
    const PointT &point = cloud->points[id];
    std::vector<int> nearby = tree->search(std::vector<float>{point.x, point.y, point.z}, distTol);
    for (const int &next : nearby)
    {
        if (visted[next] == false)
            proximity(cloud, cluster, visted, next, tree, distTol);
    }
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *&tree, const float &distTol, const int &minSize, const int &maxSize)
{
    // TODO: Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    int &&len = cloud->points.size();
    std::vector<bool> visted(len, false);

    for (int i = 0; i < len; ++i)
    {
        if (visted[i] != false)
            continue;

        std::vector<int> tmpCluster;
        proximity(cloud, tmpCluster, visted, i, tree, distTol);
        if (tmpCluster.size() < minSize || tmpCluster.size() > maxSize)
            continue;
        clusters.push_back(move(tmpCluster));
    }

    std::cout << "[+] project euclidean cluster find " << clusters.size() << " clusters" << std::endl;
    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::projectClustering(typename pcl::PointCloud<PointT>::Ptr cloud, const float &clusterTolerance, const int &minSize, const int &maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> retClusters;

    // Creating the KdTree object for the search method of the extraction
    KdTree *tree = new KdTree;

    for (int i = 0; i < cloud->points.size(); i++)
    {
        const PointT &point = cloud->points[i];
        tree->insert(std::vector<float>{point.x, point.y, point.z}, i);
    }

    std::cout << "[+] project KDtree built" << std::endl;

    std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

    delete tree;

    for (std::vector<int> cluster : clusters)
    {
        typename pcl::PointCloud<PointT>::Ptr tmpCloud(new pcl::PointCloud<PointT>());
        for (int id : cluster)
            tmpCloud->points.push_back(cloud->points[id]);

        tmpCloud->width = tmpCloud->points.size();
        tmpCloud->height = 1;
        tmpCloud->is_dense = true;

        retClusters.push_back(move(tmpCloud));
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "[+] project clustering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return retClusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}