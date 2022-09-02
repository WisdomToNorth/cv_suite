#include "pch.h"
#include "fit.h"

#include <opencv2/opencv.hpp>
//This solution is only for MSVC, not compatible enough
//#pragma warning(push)
//// sac_model.h(518): warning C4389: '!=': signed/unsigned mismatch
//#pragma warning(disable: 4389)
#include <pcl/sample_consensus/sac_model_line.h>
//#pragma warning(pop)
#include <pcl/sample_consensus/msac.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#pragma warning(disable : 4996)

namespace shape {
    template<typename VecPoints>
    PointCloud3D VecPt2PointCloud(const VecPoints& vec) {
        PointCloud3D pts;
        for (const auto& pt : vec) {
            pts.push_back({ pt.x, pt.y, 0.f });
        }
        return pts;
    }

    template<typename VecPoints>
    PointCloud3D::Ptr VecPt2PointCloudPtr(const VecPoints& vec) {
        PointCloud3D::Ptr pts(new PointCloud3D());
        for (const auto& pt : vec) {
            pts->push_back({ pt.x, pt.y, 0.f });
        }
        return pts;
    }
    std::vector<VecXf> FitProfileModel(const common::VecLinePts& line_pts, const ProfileModelPtr& profile_model, bool do_refine) {
        using SACModel = pcl::SampleConsensusModelLine<shape::Point3D>;
        auto groups_linept = profile_model->SelectPoints(line_pts);
        auto groups_linept_idx = profile_model->SelectPointsIndex(line_pts);
        //auto cloud3d = VecPt2PointCloud(line_pts);
        auto cloud3d_ptr = VecPt2PointCloudPtr(line_pts);
        std::vector<VecXf> coefs_of_lines;
        for (auto i = 0; i < groups_linept_idx.size(); ++i) {
            const auto& pt_idxes = groups_linept_idx[i];
            if (pt_idxes.size() == 0) {
                coefs_of_lines.push_back(VecXf{});
                continue;
            }
            SACModel::Ptr ptr_line_model(new SACModel(cloud3d_ptr, pt_idxes));
            pcl::MEstimatorSampleConsensus<Point3D> sac_model(ptr_line_model, 10.);
            sac_model.computeModel(1);
            std::vector<int> computed_inliers;
            sac_model.getInliers(computed_inliers);
            Eigen::VectorXf coef;
            sac_model.getModelCoefficients(coef);
            if (coef.size() == 0) {
                coefs_of_lines.push_back(VecXf{});
                continue;
            }
            else if (do_refine) {
                //auto b = std::chrono::system_clock::now();
                LOG_IF(INFO, !sac_model.refineModel(3.0, 1000)) << "[FitProfileModel] refine failed";
                //auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - b).count();
                /*
                std::stringstream ss1, ss2;
                ss1 << coef;
                sac_model.getModelCoefficients(coef);
                ss2 << coef;
                LOG(INFO) << "[FitProfileModel] refine cost: " << ms << "ms. Coef from :" << ss1.str() << " to: " << ss2.str();
                */
            }
            VecXf v;
            v.resize(coef.size());
            Eigen::VectorXf::Map(&v[0], coef.size()) = coef;
            coefs_of_lines.push_back(v);
        }
        return coefs_of_lines;
    }
    VecXf IntersectOfTwoLines(const VecXf& coef1, const VecXf& coef2) {
        constexpr float factor = 1000.f;
        // Two points on each line.
        //CV_Assert(coef1.size() == 6 && coef2.size() == 6);
        if (coef1.size() != 6 || coef2.size() != 6) return VecXf{};
        float x1 = coef1[0], y1 = coef1[1], x3 = coef2[0], y3 = coef2[1];
        //difference of two points on one line.
        auto diff_x21 = factor * coef1[3], diff_y21 = factor * coef1[4],
            diff_x43 = factor * coef2[3], diff_y43 = factor * coef2[4];
        auto denominator = diff_x21 * diff_y43 - diff_y21 * diff_x43;
        if (cv::abs(denominator) < FLT_EPSILON) {
            return VecXf{};
        }
        auto xy12 = (x1 * (y1 + diff_y21) - y1 * (x1 + diff_x21)),
            xy34 = (x3 * (y3 + diff_y43) - y3 * (x3 + diff_x43));
        auto res = VecXf(2);
        res[0] = (diff_x21 * xy34 - diff_x43 * xy12) / denominator;
        res[1] = (diff_y21 * xy34 - diff_y43 * xy12) / denominator;
        return res;
    }
    VecXf IntersectLineOfTwoPlanes(const Vec4f& coef1, const Vec4f& coef2) {
        const Eigen::Vector3f normal1 = Eigen::Map<decltype(normal1)>(&coef1[0]);
        const Eigen::Vector3f normal2 = Eigen::Map<decltype(normal2)>(&coef2[0]);
        // norm of Hypothetic plane 3, also the direction of intersection
        const Eigen::Vector3f normal3 = normal2.cross(normal1);
        Eigen::Matrix3f three_normals;
        three_normals << normal1, normal2, normal3;
        const float det = three_normals.determinant();
        if (std::abs(det) <= std::numeric_limits<float>::epsilon())
            return VecXf();

        Eigen::Vector3f line_p = (normal2.cross(normal3) * (-coef1[3]) +
            normal3.cross(normal1) * (-coef2[3])) / det;
        VecXf res;
        res.push_back(line_p(0)); res.push_back(line_p(1)); res.push_back(line_p(2));
        res.push_back(normal3(0)); res.push_back(normal3(1)); res.push_back(normal3(2));
        return res;
    }

    shape::PointCloud3D::Ptr
        MoveInliersToNewCloud(shape::PointCloud3D::Ptr origin, pcl::PointIndices::Ptr inliers) {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(origin);
        extract.setIndices(inliers);
        extract.setNegative(false);
        shape::PointCloud3D::Ptr inliers_cloud(new shape::PointCloud3D());
        extract.filter(*inliers_cloud);
        extract.setNegative(true);
        extract.filter(*origin);
        return inliers_cloud;
    }

    shape::PointCloud3D::Ptr
        StatFilter(shape::PointCloud3D::Ptr pc, const int kNN, const float StddevMulThresh) {
        pcl::StatisticalOutlierRemoval<shape::Point3D> sor;
        sor.setInputCloud(pc);
        sor.setMeanK(kNN);
        sor.setStddevMulThresh(StddevMulThresh);
        auto filtered = shape::PointCloud3D::Ptr(new shape::PointCloud3D);
        sor.filter(*filtered);
        return filtered;
    }


    pcl::PointCloud<pcl::Boundary>::Ptr NormalEstimationBasedBoundary(pcl::PointCloud<shape::Point3D>::Ptr cloud, const char* normalPath) {
        // fill in the cloud data here
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        // Output datasets
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch(10.0);

        // Compute the features
        ne.compute(*normals);
        if (normalPath && normals->size() > 0) {
            if (pcl::io::savePCDFile(std::string(normalPath), *normals) != 0) {
                pcl::console::print_warn("Failed to save normal: %s\n", normalPath);
            }
            else {
                pcl::console::print_info("Saved normal: %s\n", normalPath);
            }
        }
        // estimate normals and fill in \a normals
        pcl::PointCloud<pcl::Boundary>::Ptr boundaries(new pcl::PointCloud<pcl::Boundary>);
        pcl::BoundaryEstimation<shape::Point3D, pcl::Normal, pcl::Boundary> est;
        est.setInputCloud(cloud);
        est.setInputNormals(normals);
        est.setRadiusSearch(3.0);   // 3mm radius
        est.setSearchMethod(typename pcl::search::KdTree<shape::Point3D>::Ptr(new pcl::search::KdTree<shape::Point3D>));
        est.compute(*boundaries);
        return boundaries;
    }

    shape::PointCloud3D::Ptr
        FilterBackgroundFromTarget(shape::PointCloud3D::Ptr target, shape::PointCloud3D::Ptr background, const int numKNN, const float disKNN) {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(background);
        pcl::PointIndices::Ptr maybeWorkspace(new pcl::PointIndices);
        std::vector<int> pointIdxNKNSearch(numKNN);
        std::vector<float> pointNKNSquaredDistance(disKNN);
        for (int i = 0; i < target->points.size(); ++i) {
            if (kdtree.nearestKSearch(target->points[i], numKNN, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                if (pointNKNSquaredDistance[numKNN - 1] < disKNN) {
                    maybeWorkspace->indices.push_back(i);
                }

            }
        }
        return MoveInliersToNewCloud(target, maybeWorkspace);
    }
}