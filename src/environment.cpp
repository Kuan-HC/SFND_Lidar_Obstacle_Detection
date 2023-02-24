/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::cout << "[+] Simple high way" << std::endl;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *roofLidar = new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcd = roofLidar->scan();

    /* render lidar */
    // renderRays(viewer, roofLidar->position, pcd);
    // renderPointCloud(viewer, pcd, "pcd_file");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pcdProcesser;
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pcdProcesser.projectSegmentPlane(pcd, 25, 0.3);
    // renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "plane", Color(0, 1, 0));

    /**
     * Clustering
     */
    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pcdProcesser.projectClustering(segmentCloud.first, 1.0, 3, 30);
    int colorId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (const pcl::PointCloud<pcl::PointXYZ>::Ptr &cluster : clusters)
    {
        renderPointCloud(viewer, cluster, "Object" + std::to_string(colorId), colors[colorId % colors.size()]);

        Box box = pcdProcesser.BoundingBox(cluster);
        renderBox(viewer, box, ++colorId, Color(1, 0, 0), 0.4f);
    }

    // setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> &pointProcessor, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    inputCloud = pointProcessor.FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-10, -5, -2, 1), Eigen::Vector4f(30, 8, 1, 1));
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.projectSegmentPlane(inputCloud, 25, 0.3);

    /* Clustering */
    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor.projectClustering(segmentCloud.first, 0.6, 10, 500);
    int colorId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

    for (const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster : clusters)
    {
        renderPointCloud(viewer, cluster, "Object" + std::to_string(colorId), colors[colorId % colors.size()]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, colorId, Color(1, 0, 0), 0.4f);
        ++colorId;
    }

    // rendering ground
    renderPointCloud(viewer, segmentCloud.second, "ground", Color(0, 1, 0));
}

void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");

    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessor.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessor, inputCloudI);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce();
    }
}