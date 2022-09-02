#include "pch.h"
#include "image_algorithm.h"
#include "test.h"
#include "core.h"

#include <opencv2/video.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <future>
#include <functional>

void image::Eigen22(const cv::Mat& mat22, cv::Mat& eigen_value, cv::Mat& eigen_vector) {
    eigen_value = cv::Mat::zeros(1, 2, CV_64FC1);
    eigen_vector = cv::Mat::zeros(2, 2, CV_64FC1);
    CV_Assert(mat22.rows == mat22.cols && mat22.cols == 2 && mat22.at<double>(0, 1) == mat22.at<double>(1, 0));
    auto ev_p = eigen_value.ptr<double>();
    auto evv_p = eigen_vector.ptr<double>();
    Eigen22(mat22.at<double>(0, 0), mat22.at<double>(0, 1), mat22.at<double>(1, 1), ev_p, evv_p);
}

double image::Correlation1D(const cv::Mat& input1, const cv::Mat& input2) {
    CV_Assert(!input1.empty() &&
        !input2.empty() &&
        input1.rows == 1 &&
        input2.rows == 1 &&
        input1.cols == input2.cols &&
        input1.channels() == input2.channels() &&
        input1.type() == input2.type() && input2.depth() <= CV_32S);

    int cn = input1.channels();
    cv::Mat row_mean1, row_mean2;

    cv::reduce(input1, row_mean1, 1, cv::REDUCE_AVG, CV_MAKE_TYPE(CV_64F, cn));
    cv::reduce(input2, row_mean2, 1, cv::REDUCE_AVG, CV_MAKE_TYPE(CV_64F, cn));
    int mean1 = static_cast<int>(row_mean1.at<double>(0, 0));
    int mean2 = static_cast<int>(row_mean2.at<double>(0, 0));

    auto input1_mean = input1 - mean1;
    auto input2_mean = input2 - mean2;

    double top_sum = input1_mean.dot(input2_mean);
    double bottom_sum = input1_mean.dot(input1_mean) * input2_mean.dot(input2_mean);

    double bottom_root = std::sqrt(bottom_sum);
    if ((bottom_root - 0.) < FLT_EPSILON) return 0.;
    return top_sum / bottom_root;
}

int image::ImgLineHeight(const cv::Mat& laser_mat, const uchar gray_threshold) {
    if (laser_mat.empty() || laser_mat.type() != CV_8UC1) return 0;
    //cv::threshold(m_MatROI, m_MatROI,0, 255, CV_THRESH_OTSU);
    int ImgHeight = laser_mat.rows;
    int ImgWidth = laser_mat.cols;
    int ptSUM = 0, ptCount = 0, xIndex = 0;

    for (int i = 0; i < ImgHeight; ++i) {
        ptCount = 0;
        for (int j = 0; j < ImgWidth; ++j) {
            if (laser_mat.at<uchar>(i, j) >= gray_threshold) {
                ++ptCount;
            }
        }
        if (ptCount > ptSUM) {
            ptSUM = ptCount;
            xIndex = i;
        }
    }
    return ImgHeight - xIndex;
    //return ImgHeight -xIndex - gROI_Y;
}

void image::PlotReducedHist(const cv::Mat& in, cv::Mat& out) {
    CV_Assert(in.depth() == CV_8U && !in.empty());

    cv::Mat reducedRows;
    // Reduce to one-column mat
    cv::reduce(in, reducedRows, 1, cv::REDUCE_SUM, CV_32SC1);
    cv::Mat normalized;
    // Normalize :
    // normType = INF (max(src) = 1)
    // range(0 - in.cols)
    cv::normalize(reducedRows, normalized, in.cols, 0, cv::NORM_INF);

    auto color_black = cv::Scalar(0);
    cv::Mat plotMat(in.size(), CV_8UC3, color_black);
    for (int i = 0; i < normalized.rows; ++i) {
        auto barSize = normalized.at<int32_t>(i, 0);
        //Point(col, row);
        //Draw red line.
        cv::line(plotMat, cv::Point(0, i), cv::Point(barSize, i), cv::Scalar(0, 0, 255));
    }
    out = plotMat;
}

void image::SetCenterLineY(cv::Mat src) {
    if (src.empty()) return;

    int ImgHeight = src.rows;
    int ImgWidth = src.cols;

    int ptCount = 0, yCroodsSum = 0, yIndex = 0;

    for (int i = 0; i < ImgWidth; ++i) {
        ptCount = 0;
        yCroodsSum = 0;
        for (int j = 0; j < ImgHeight; ++j) {
            if (src.at<uchar>(j, i) == 255) {
                ++ptCount;
                yCroodsSum += j;
            }
        }
        if (ptCount > 0) yIndex = yCroodsSum / ptCount;
        for (int j = 0; j < ImgHeight; ++j) {
            if (src.at<uchar>(j, i) == 255 && j != yIndex) {
                src.at<uchar>(j, i) = 0;
            }
        }
    }
}

void image::PeakDetectionCenterMass(const cv::Mat& in, cv::Mat& out, int axis) {
    CV_Assert(!in.empty() && in.depth() == CV_8UC1);

    const int imax = axis == 0 ? in.rows : in.cols;
    const int jmax = axis == 0 ? in.cols : in.rows;
    const int NEIGHBOUR_PIXEL_NUM = 7;//6?

    for (int i = 0; i < imax; ++i) {
        auto p = in.ptr<uchar>(i);
        //int neighbour_sums = 0;
        //double max = 0.0; int max_index = 0;
        for (int j = NEIGHBOUR_PIXEL_NUM; j < jmax - NEIGHBOUR_PIXEL_NUM; ++j) {

            double bottom = 0;
            double top = 0;
            for (int r = 1; r < NEIGHBOUR_PIXEL_NUM; ++r)
            {
                bottom += p[j - r];
                bottom += p[j + r];

                top += r * p[j + r] - r * p[j - r];

            }
            bottom += p[j];
            // inner loop  the maximum of nearby 3 points.
            //p[j] =1;
        }
    }
    out = in.clone();
}


image::StergerLineDetector::StergerLineDetector() {
}

image::StergerLineDetector::~StergerLineDetector() {
}

void image::StergerLineDetector::Compute(const cv::Mat& source, Float sigma, Float salience_threshold) {
    _Covolution(source, sigma);
    _LinePoints(source, salience_threshold);
}

void image::StergerLineDetector::GetLines() const {}

cv::Mat image::StergerLineDetector::GetSalientMat() const {
    return ev_mat_;
}

cv::Mat image::StergerLineDetector::GetFlagMat() const {
    return flag_mat_;
}

cv::Mat image::StergerLineDetector::GetNormMat() const {
    return n_mat_;
}

cv::Mat image::StergerLineDetector::GetAngleMat() const {
    return angle_mat_;
}

cv::Mat image::StergerLineDetector::GetSubPixelMat() const {
    return p_mat_;
}
const common::VecLinePts&
image::StergerLineDetector::GetSubPixelVec() const {
    return subpixels_;
}
//[-255.0, 255.0]
cv::Mat image::StergerLineDetector::GetConvolv(int r, int c) {
    cv::Mat a = GetConvolv32F(r, c);
    double min, max;
    cv::minMaxLoc(a, &min, &max);
    double absmax = cv::max(cv::abs(min), cv::abs(max));
    double scale = 255 / absmax;
    a.convertTo(a, CV_32FC1, scale);
    return a;
}

cv::Mat image::StergerLineDetector::GetConvolv32F(int r, int c) {
    cv::Mat a;
    if (r == 1) {
        a = (c == 0) ? r1_c0_ : r1_c1_;
    }
    else if (r == 2) {
        a = r2_c0_;
    }
    else if (r == 0) {
        if (c == 1) a = r0_c1_;
        if (c == 2) a = r0_c2_;
    }
    return a;
}
cv::Mat image::StergerLineDetector::GetTestMat() const {
    return test_mat_;
}
std::vector<image::StergerLineDetector::Float>
image::StergerLineDetector::GetPositionInfo(int r, int c) {
    if (r > r1_c0_.rows || c > r1_c0_.cols) return std::vector<Float>();
    std::vector<Float> image_info;
    image_info.push_back(r1_c0_.at<Float>(r, c));
    image_info.push_back(r0_c1_.at<Float>(r, c));
    image_info.push_back(r1_c1_.at<Float>(r, c));
    image_info.push_back(r2_c0_.at<Float>(r, c));
    image_info.push_back(r0_c2_.at<Float>(r, c));
    return image_info;
}

void image::StergerLineDetector::_Covolution(const cv::Mat& in, const Float sigma) {
    constexpr int ddepth = CV_32F;
    //test::mytimer conv_bench{ "Convolution" };
    auto gaussian_0 = Get1DGaussian(-1, sigma, ddepth, 0);
    auto gaussian_1 = Get1DGaussian(-1, sigma, ddepth, 1);
    auto gaussian_2 = Get1DGaussian(-1, sigma, ddepth, 2);
    //Output of covolution: r1_c0 means 
    //convolved with first de rivative of gaussian kernel in row direction and gaussian kernel in column.
    // simple parallel of std::async:
    // Direct call to std::async(cv::sepFilter2d, Args....) leads to compiler errors('no overloading of std::async'alike).
    // so wrap them with lambda.
    // reduce computing time from 270 ms to 70ms (MSVC2015 Release, 500 * 800 images)
    // Next: TBB? GPU?
    auto colv10 = std::async(std::launch::async, [&]() {
        cv::sepFilter2D(in, r1_c0_, ddepth, gaussian_1, gaussian_0);
        });
    auto colv01 = std::async(std::launch::async, [&]() {
        cv::sepFilter2D(in, r0_c1_, ddepth, gaussian_0, gaussian_1);
        });
    auto colv11 = std::async(std::launch::async, [&]() {
        cv::sepFilter2D(in, r1_c1_, ddepth, gaussian_1, gaussian_1);
        });
    auto colv20 = std::async(std::launch::async, [&]() {
        cv::sepFilter2D(in, r2_c0_, ddepth, gaussian_2, gaussian_0);
        });
    auto colv02 = std::async(std::launch::async, [&]() {
        cv::sepFilter2D(in, r0_c2_, ddepth, gaussian_0, gaussian_2);
        });
    colv10.wait(); colv01.wait(); colv11.wait(); colv20.wait(); colv02.wait();
    //For debug ImageWatch Display
    auto r1c0 = r1_c0_, r0c1 = r0_c1_, r1c1 = r1_c1_, r2c0 = r2_c0_, r0c2 = r0_c2_;
}

void image::StergerLineDetector::_LinePoints(const cv::Mat& source, Float salience_threshold) {
    //Compute eigen values and eigen vectors.
    //Two eigen values(float or double) each pixel, stored as 2 channels.
    cv::Mat eigen(source.size(),
        CV_MAKE_TYPE(CV_32F, 2),
        cv::Scalar(0.0, 0.0));

    //Two eigen vectors, stored as 
    cv::Mat eigen_vec(source.size(),
        CV_MAKE_TYPE(CV_32F, 4),
        cv::Scalar(0.0, 0.0, 0.0, 0.0)
    );
    // maximum eigenvalue of each pixel.
    ev_mat_ = cv::Mat::zeros(source.size(), CV_32FC1);
    // is_max: 0, 1, or 2.
    flag_mat_ = cv::Mat::zeros(source.size(), CV_8UC1);
    //n_mat: norm direction of line.
    n_mat_ = cv::Mat::zeros(source.size(), CV_32FC2);
    //p_mat: point's subpixel position.
    p_mat_ = cv::Mat::zeros(source.size(), CV_32FC2);
    //angle_mat: angle from atan(norm direction)
    angle_mat_ = cv::Mat::zeros(source.size(), CV_32FC1);
    //test_mat: debug info.
    test_mat_ = cv::Mat::zeros(source.size(), CV_32FC2);
    //  Benchmark
    //test::mytimer eigen_bench{ "Eigen" };
    //e_salient_ = cv::Mat::zeros(source.size(), CV_64FC1);
    for (int i = 0; i < source.rows; ++i) {
        //const double* sourcep = source.ptr<double>(i);
        Float* eigen_p = eigen.ptr<Float>(i), * eigen_vec_p = eigen_vec.ptr<Float>(i),
            * rc = r1_c1_.ptr<Float>(i), * rr = r2_c0_.ptr<Float>(i),
            * cc = r0_c2_.ptr<Float>(i), * ev = ev_mat_.ptr<Float>(i),
            // For subpixel line location
            * r = r1_c0_.ptr<Float>(i), * c = r0_c1_.ptr<Float>(i),
            * p = p_mat_.ptr<Float>(i), * n = n_mat_.ptr<Float>(i);
        //* e_val = e_salient_.ptr<Float>(i),
    //auto * debug = test_mat_.ptr<Float>(i);
        uchar* ismax = flag_mat_.ptr<uchar>(i);
        float* angle = angle_mat_.ptr<float>(i);

        for (int j = 0; j < source.cols; ++j) {
            //2 eigen values and 2*2 eigen vectors.
            auto eigenval = eigen_p + j * 2;
            auto eigenvec = eigen_vec_p + j * 4;
            _CalcEigen(rr[j], rc[j], cc[j], eigenval, eigenvec);
            // Light Mode 
            auto val = -eigenval[0];
            if (val > salience_threshold) {
                ev[j] = val;
                auto n1 = eigenvec[0];
                auto n2 = eigenvec[1];
                // FIXED
                int x = 2 * j, y = 2 * j + 1;
                n[x] = n1; n[y] = n2;
                //TODO verify the meaning of this angle.
                angle[j] = image::atanxy(n2, n1);
                //For subpixel line location
                size_t j2 = j * 2;
                _SubPixel(i, j, r[j], c[j], rr[j], rc[j], cc[j], n1, n2, p + j2, ismax + j);
                if (*(ismax + j) == 255) {
                    subpixels_.emplace_back(p[j2], p[j2 + 1], angle[j]);
                }
            }
        }
    }
}

//Compute the eigen of 2*2 matrix. output store in eigval and eigvec.
void image::StergerLineDetector::_CalcEigen(Float rr, Float rc, Float cc,
    Float* eigval, Float* eigvec) {
    Float theta, t, c, s, e1, e2, n1, n2; /* , phi;
    /* Compute the eigenvalues and eigenvectors of the Hessian matrix. */
    if (rc != 0.0) {
        theta = static_cast<Float>(0.5) * (cc - rr) / rc;
        t = static_cast<Float>(1.0) / (fabs(theta) + sqrt(theta * theta + static_cast<Float>(1.0)));
        if (theta < static_cast<Float>(0.0)) t = -t;
        c = static_cast<Float>(1.0) / sqrt(t * t + static_cast<Float>(1.0));
        s = t * c;
        e1 = rr - t * rc;
        e2 = cc + t * rc;
    }
    else {
        c = static_cast<Float>(1.0);
        s = static_cast<Float>(0.0);
        e1 = rr;
        e2 = cc;
    }
    n1 = c;
    n2 = -s;

    /* If the absolute value of an eigenvalue is larger than the other, put that
    eigenvalue into first position.  If both are of equal absolute value, put
    the negative one first. */
    if (fabs(e1) > fabs(e2)) {
        eigval[0] = e1;
        eigval[1] = e2;

        eigvec[0] = n1;
        eigvec[1] = n2;
        eigvec[2] = -n2;
        eigvec[3] = n1;
    }
    else if (fabs(e1) < fabs(e2)) {
        eigval[0] = e2;
        eigval[1] = e1;

        eigvec[0] = -n2;
        eigvec[1] = n1;
        eigvec[2] = n1;
        eigvec[3] = n2;
    }
    else {
        if (e1 < e2) {
            eigval[0] = e1;
            eigval[1] = e2;

            eigvec[0] = n1;
            eigvec[1] = n2;
            eigvec[2] = -n2;
            eigvec[3] = n1;
        }
        else {
            eigval[0] = e2;
            eigval[1] = e1;

            eigvec[0] = -n2;
            eigvec[1] = n1;
            eigvec[2] = n1;
            eigvec[3] = n2;
        }
    }
}

void image::HistPlot(const cv::Mat& in) {
    const int hist_size = 30;
    const int channels = 0;
    std::vector<float> range{ 0, 255 };
    cv::Mat hist;
    std::vector<cv::Mat> input_array_of_arrays{ in };
    cv::calcHist(input_array_of_arrays,
        std::vector<int>{channels},
        cv::Mat(),
        hist,
        std::vector<int>{hist_size},
        range);

    double maxVal = 0;
    cv::minMaxLoc(hist, 0, &maxVal, 0, 0);

    const int scale = 10;
    // cv::Mat hist_img = cv::Mat::zeros(hist_size * scale, )

}

void image::Scale(const cv::Mat& in, cv::Mat& out, const double limit, const int dtype)
{
    double min, max;
    cv::minMaxLoc(in, &min, &max);
    double absmax = cv::max(cv::abs(min), cv::abs(max));
    double scale = limit / absmax;
    in.convertTo(out, dtype, scale);
}
