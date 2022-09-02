#include "pch.h"
#include "utility.h"

#include <algorithm>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>
#include <opencv2/highgui.hpp>



test::Benchmark::Benchmark(const char* path, const char* out) :
    input_folder_(path),
    output_folder_(fs::path(path) /= fs::path(out)),
    mat_(),
    isBroken_(true) {
    if (!input_folder_.is_absolute()) isBroken_ = true;
    //Create output folder
    if (!fs::exists(output_folder_)) {
        if (fs::create_directory(output_folder_)) isBroken_ = false;
    }
    else {
        isBroken_ = false;
    }
}

test::Benchmark::~Benchmark() {
}

void test::Benchmark::AppendOperator(std::function<void(cv::Mat&, fs::path)> new_op) {
    operator_vec.push_back(new_op);
}

void test::Benchmark::operator()() {
    _DirLoop();
}

void test::Benchmark::_DirLoop() {
    if (fs::is_directory(input_folder_)) {
        for (auto& entry : fs::directory_iterator(input_folder_)) {
            auto entry_p = entry.path();
            auto filename = entry_p.filename();
            fs::path out_filename = output_folder_;
            out_filename /= filename;
            std::cout << "[Read]:" << entry_p.generic_string() << std::endl;
            mat_ = cv::imread(entry_p.generic_string(), cv::IMREAD_GRAYSCALE);
            if (mat_.empty()) continue;
            for (auto& op : operator_vec) {
                op(mat_, out_filename);
            }

            cv::imwrite(out_filename.generic_string(), mat_);
        }
    }


}

bool test::Benchmark::_CheckExtension(const fs::path& p, const char* suffix) {
    auto ex = p.extension();
    if (ex.empty()) return false;
    return (ex.generic_string() == std::string(suffix));
}

void test::Scale(const cv::Mat& in, cv::Mat& out, const double limit, const int dtype) {
    double min_value, max_value;
    cv::minMaxLoc(in, &min_value, &max_value);
    double absmax = std::max(cv::abs(min_value), cv::abs(max_value));
    double scale = limit / absmax;
    in.convertTo(out, dtype, scale);
}

void test::ShiftToU8(const cv::Mat& in, cv::Mat& out) {
    in.convertTo(out, CV_8UC1, 0.5, 255 / 2);
}

cv::Mat test::SimplePlot(const cv::Mat& in, const int y_axis, const int x_thickness, bool show) {
    cv::Mat normed;
    //将in 按比例缩放。最大不超过y_axis
    if (y_axis <= 0) return cv::Mat();
    Scale(in, normed, y_axis);

    cv::Mat plot_mat(y_axis * 2, in.cols * x_thickness, CV_8UC3, cv::Scalar(0));
    for (int i = 0; i < in.cols; ++i) {
        const int x_ = i * x_thickness;
        cv::Point a;
        int value_y = value_y = static_cast<int>(normed.at<double>(0, i));
        int axis_y = y_axis - value_y;
        a = cv::Point(x_, axis_y);
        cv::Point x_axis(x_, y_axis);

        cv::line(plot_mat, a, x_axis, cv::Scalar(0, 255, 0), x_thickness);
    }
    if (show) {
        cv::namedWindow("simple-plot");
        cv::imshow("simple-plot", plot_mat);
        cv::waitKey(3000);
    }
    return plot_mat;
}

cv::Mat test::MatABS(const cv::Mat& a)
{
    cv::Mat normed;
    if (a.type() == CV_8UC1)
    {
        a.convertTo(normed, CV_32SC1, 1, -1 * (255 / 2));
    }
    else
    {
        normed = a;
    }
    normed = cv::abs(normed);
    normed.convertTo(normed, CV_8UC1);
    return normed;
}

void thinningIteration(cv::Mat& img, int iter) {
    CV_Assert(img.channels() == 1);
    CV_Assert(img.depth() != sizeof(uchar));
    CV_Assert(img.rows > 3 && img.cols > 3);

    cv::Mat marker = cv::Mat::zeros(img.size(), CV_8UC1);

    int nRows = img.rows;
    int nCols = img.cols;

    if (img.isContinuous()) {
        nCols *= nRows;
        nRows = 1;
    }

    int x, y;
    uchar* pAbove;
    uchar* pCurr;
    uchar* pBelow;
    uchar* nw, * no, * ne; // north (pAbove)
    uchar* we, * me, * ea;
    uchar* sw, * so, * se; // south (pBelow)

    uchar* pDst;

    // initialize row pointers
    pAbove = NULL;
    pCurr = img.ptr<uchar>(0);
    pBelow = img.ptr<uchar>(1);

    for (y = 1; y < img.rows - 1; ++y) {
        // shift the rows up by one
        pAbove = pCurr;
        pCurr = pBelow;
        pBelow = img.ptr<uchar>(y + 1);

        pDst = marker.ptr<uchar>(y);

        // initialize col pointers
        no = &(pAbove[0]);
        ne = &(pAbove[1]);
        me = &(pCurr[0]);
        ea = &(pCurr[1]);
        so = &(pBelow[0]);
        se = &(pBelow[1]);

        for (x = 1; x < img.cols - 1; ++x) {
            // shift col pointers left by one (scan left to right)
            nw = no;
            no = ne;
            ne = &(pAbove[x + 1]);
            we = me;
            me = ea;
            ea = &(pCurr[x + 1]);
            sw = so;
            so = se;
            se = &(pBelow[x + 1]);

            int A = (*no == 0 && *ne == 1) + (*ne == 0 && *ea == 1) +
                (*ea == 0 && *se == 1) + (*se == 0 && *so == 1) +
                (*so == 0 && *sw == 1) + (*sw == 0 && *we == 1) +
                (*we == 0 && *nw == 1) + (*nw == 0 && *no == 1);
            int B = *no + *ne + *ea + *se + *so + *sw + *we + *nw;
            int m1 = iter == 0 ? (*no * *ea * *so) : (*no * *ea * *we);
            int m2 = iter == 0 ? (*ea * *so * *we) : (*no * *so * *we);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                pDst[x] = 1;
        }
    }

    img &= ~marker;
}

void thinning(const cv::Mat& src, cv::Mat& dst) {
    dst = src.clone();
    dst /= 255; // convert to binary image

    cv::Mat prev = cv::Mat::zeros(dst.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(dst, 0);
        thinningIteration(dst, 1);
        cv::absdiff(dst, prev, diff);
        dst.copyTo(prev);
    } while (cv::countNonZero(diff) > 0);

    dst *= 255;
}
