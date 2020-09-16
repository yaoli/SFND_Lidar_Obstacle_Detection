/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <chrono>
#include <thread>
std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}
void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> &pp, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudPoints) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pp.FilterCloud(cloudPoints, 0.2, Eigen::Vector4f(-15, -8., -5., 1.), Eigen::Vector4f(50., 8., 5., 1.));
  //renderPointCloud(viewer, cloud, "city block");
  
  // segment road and obstacle
  int max_iter = 200;
  float distance_tol = 0.1; // within this distance considered as belonging to inliers
  std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr,
            typename pcl::PointCloud<pcl::PointXYZI>::Ptr>
    segmentCloud = pp.SegmentPlane(cloud, max_iter, distance_tol);
  //renderPointCloud(viewer, segmentCloud.first, "obstacle", Color(1, 0, 0));
  renderPointCloud(viewer, segmentCloud.second, "road", Color(0, 1, 0));
  
  // clustering
  float radius = 0.6;
  int min_n_points = 10;
  int max_n_points = 1000;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
      pp.Clustering(segmentCloud.first, radius, min_n_points, max_n_points);
  int clusterId = 0;
  std::vector<Color> colors = {
      Color(1, 0, 0), Color(0, 1, 0),
      Color(0, 0, 1)}; // this is called "list initialization", specific to STL
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    pp.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstacle" + std::to_string(clusterId),
                     colors[clusterId % colors.size()]);
    Box box = pp.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // Create lidar sensor on the heap, not on the stack (way too small)
  Lidar *lidar = new Lidar(cars, 0.);
  pcl::PointCloud<pcl::PointXYZ>::Ptr points = lidar->scan();
  // renderRays(viewer, lidar->position, points);
  renderPointCloud(viewer, points, "point cloud", Color(255., 255., 255.));

  // Create point processor
  ProcessPointClouds<pcl::PointXYZ> pp;

  // segmentation
  int max_iter = 200;
  float distance_tol =
      0.2; // within this distance considered as belonging to inliers
  std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr,
            typename pcl::PointCloud<pcl::PointXYZ>::Ptr>
      segmentCloud = pp.Segment(points, max_iter, distance_tol);
  // renderPointCloud(viewer, segmentCloud.first, "road", Color(1, 0, 0));
  // renderPointCloud(viewer, segmentCloud.second, "obstacle", Color(0, 1, 0));

  // clustering
  float radius = 1.0;
  int min_n_points = 3;
  int max_n_points = 30;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
      pp.Clustering(segmentCloud.second, radius, min_n_points, max_n_points);
  int clusterId = 0;
  std::vector<Color> colors = {
      Color(1, 0, 0), Color(0, 1, 0),
      Color(0, 0, 1)}; // this is called "list initialization", specific to STL
  for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters) {
    std::cout << "cluster size ";
    pp.numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstacle" + std::to_string(clusterId),
                     colors[clusterId]);
    Box box = pp.BoundingBox(cluster);
    renderBox(viewer, box, clusterId);
    ++clusterId;
  }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
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

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  
  //simpleHighway(viewer);
  ProcessPointClouds<pcl::PointXYZI> pp;
  std::vector<boost::filesystem::path> stream = pp.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  
  while (!viewer->wasStopped()) {
    // Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    // Load pcd and run obstacle detection
    cloud = pp.loadPcd((*streamIterator).string());
    cityBlock(viewer, pp, cloud);
    streamIterator++;

    // run in loop
    if (streamIterator == stream.end()) {
      streamIterator = stream.begin();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    viewer->spinOnce();
  }
}
