#include "pch.h"
//#include <pcl/point_cloud.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/console/print.h>
//#include <pcl/console/parse.h>
//#include <pcl/console/time.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/common/centroid.h>
//#include <pcl/common/common.h>
//#include "shape.h"
//#include <iostream>
//#include <vector>
//#include <ctime>
//
//using VisualizerPtr = boost::shared_ptr<pcl::visualization::PCLVisualizer>;
//void SetCameraFocus(VisualizerPtr viewer, double x, double y, double z) {
//    viewer->setCameraPosition(x, y - 50., z, 0, 0, 1);
//
//    std::vector<pcl::visualization::Camera> cams;
//    viewer->getCameras(cams);
//
//    for (auto&& camera : cams)
//    {
//        camera.focal[0] = x;
//        camera.focal[1] = y;
//        camera.focal[2] = z;
//    }
//
//    viewer->setCameraParameters(cams[0]);
//}
//const size_t kMininumPointsNumber = 4000;
//const double kEpsAngle = 8.;
//const float kLeafSize = 0.0005f;
//shape::PointCloud3D::Ptr
//MoveInliersToNewCloud(shape::PointCloud3D::Ptr origin, pcl::PointIndices::Ptr inliers) {
//    pcl::ExtractIndices<pcl::PointXYZ> extract;
//    extract.setInputCloud(origin);
//    extract.setIndices(inliers);
//    extract.setNegative(false);
//    shape::PointCloud3D::Ptr inliers_cloud(new shape::PointCloud3D());
//    extract.filter(*inliers_cloud);
//    extract.setNegative(true);
//    extract.filter(*origin);
//    return inliers_cloud;
//}
//
//void AddPointCloud(
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
//    shape::PointCloud3D::Ptr p,
//    std::string name,
//    size_t point_size,
//    double r, double g, double b, bool as_center = false) {
//    if (viewer->wasStopped()) {
//        pcl::console::print_warn("Viewer was stopped ");
//        return;
//    }
//    pcl::visualization::PointCloudColorHandlerCustom<shape::Point3D> single_color(p, r, g, b);
//    viewer->addPointCloud<pcl::PointXYZ>(p, single_color, name);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, name);
//    if (as_center) {
//        shape::Point3D centroid;
//        pcl::computeCentroid(*p, centroid);
//        SetCameraFocus(viewer, centroid.x, centroid.y, centroid.z);
//    }
//};
//
////void SpinUntilClose(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer) {
////    while (!viewer->wasStopped())
////    {
////        viewer->spinOnce(100);
////        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
////    }
////}
//
//
//shape::PointCloud3D::Ptr
//FilterBackgroundFromTarget(shape::PointCloud3D::Ptr target, shape::PointCloud3D::Ptr background, int numKNN, float disKNN) {
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud(background);
//    pcl::PointIndices::Ptr maybeWorkspace(new pcl::PointIndices);
//    std::vector<int> pointIdxNKNSearch(numKNN);
//    std::vector<float> pointNKNSquaredDistance(disKNN);
//    for (int i = 0; i < target->points.size(); ++i) {
//        if (kdtree.nearestKSearch(target->points[i], numKNN, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//        {
//            if (pointNKNSquaredDistance[numKNN - 1] < disKNN) {
//                maybeWorkspace->indices.push_back(i);
//            }
//
//        }
//    }
//    return MoveInliersToNewCloud(target, maybeWorkspace);
//}
//
//int main(int argc, char** argv)
//{
//    srand(time(NULL));
//    if (argc != 3) {
//        PCL_ERROR("Usage: kdtree.exe workspace_point_cloud point_cloud to search");
//    }
//
//    pcl::console::TicToc profiler_timer;
//
//    pcl::PointCloud<pcl::PointXYZ>::Ptr workspace(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr to_search(new pcl::PointCloud<pcl::PointXYZ>);
//    if (pcl::io::loadPCDFile(argv[1], *workspace) != 0) {
//        std::cerr << "Read " << argv[1] << " as workspace Failed. exit.";
//        exit(2);
//    }
//    if (pcl::io::loadPCDFile(argv[2], *to_search) != 0) {
//        std::cerr << "Read " << argv[2] << "as to_search Failed. exit.";
//        exit(2);
//    }
//
//    profiler_timer.tic();
//    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//    kdtree.setInputCloud(workspace);
//    profiler_timer.toc_print();
//
//    // K nearest neighbor search
//
//    const int K = 3;
//    const float DistanceThresholdForKNN = 5.0;
//    pcl::PointIndices::Ptr maybeWorkspace(new pcl::PointIndices);
//    std::vector<int>;
//    std::vector<int> pointIdxNKNSearch(K);
//    std::vector<float> pointNKNSquaredDistance(K);
//    profiler_timer.tic();
//    for (int i = 0; i < to_search->points.size(); ++i) {
//        if (kdtree.nearestKSearch(to_search->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
//        {
//            if (pointNKNSquaredDistance[K - 1] < DistanceThresholdForKNN) {
//                maybeWorkspace->indices.push_back(i);
//            }
//
//        }
//    }
//    profiler_timer.toc_print();
//    pcl::console::print_info("Size of points to_search:%d\n", to_search->points.size());
//    pcl::console::print_info("Size of points belong to workspace:%d\n", maybeWorkspace->indices.size());
//
//    auto workspace_in_to_search = MoveInliersToNewCloud(to_search, maybeWorkspace);
//
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> kdtreeViewer(new pcl::visualization::PCLVisualizer("KDTree Viewer"));
//
//    kdtreeViewer->setBackgroundColor(1., 1.0, 1.0);
//    kdtreeViewer->initCameraParameters();
//
//
//    AddPointCloud(kdtreeViewer, workspace_in_to_search, "workspace", 2, 255., 0., 255., true);
//    AddPointCloud(kdtreeViewer, to_search, "other", 2, 0., 0., 0.);
//    //SpinUntilClose(kdtreeViewer);
//    return 0;
//}