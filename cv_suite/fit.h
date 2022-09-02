#ifndef CVSUITE_SOURCE_DRIVER_IMAGE_FIT_H
#define CVSUITE_SOURCE_DRIVER_IMAGE_FIT_H

#include "shape.h"
#include <vector>
#include <pcl/filters/extract_indices.h>
namespace shape {
    template<typename ProfileModel>
    std::vector<Eigen::VectorXf> FitProfileFromPts(const common::VecLinePts& line_pts, const ProfileModel& profile_model) {
        //TODO
        std::vector<Eigen::VectorXf> coefs_of_lines;

        return coefs_of_lines;
    }

    std::vector<VecXf> FitProfileModel(const common::VecLinePts& line_pts, const ProfileModelPtr& profile_model, bool do_refine);


    shape::PointCloud3D::Ptr
        MoveInliersToNewCloud(shape::PointCloud3D::Ptr origin, pcl::PointIndices::Ptr inliers);
    // Utilities function designed for model coeffients got from above function.
    //Return a empty vector means no intersect was found.
    VecXf IntersectOfTwoLines(const VecXf& coef1, const VecXf& coef2);
    // return vector of size 0 if no intersections, otherwise size of 6 for point3d, direction3d.
    VecXf IntersectLineOfTwoPlanes(const Vec4f& coef1, const Vec4f& coef2);

    shape::PointCloud3D::Ptr StatFilter(shape::PointCloud3D::Ptr pc, const int kNN, const float StddevMulThresh);
    pcl::PointCloud<pcl::Boundary>::Ptr
        NormalEstimationBasedBoundary(pcl::PointCloud<shape::Point3D>::Ptr cloud, const char* normalPath);

    shape::PointCloud3D::Ptr
        FilterBackgroundFromTarget(shape::PointCloud3D::Ptr target, shape::PointCloud3D::Ptr background, const int numKNN, const float disKNN);
}
#endif //CVSUITE_SOURCE_DRIVER_IMAGE_FIT_H