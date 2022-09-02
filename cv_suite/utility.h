#ifndef CVSUITE_SOURCE_UTILITY_H
#define CVSUITE_SOURCE_UTILITY_H

#include "image.h"

#include <functional>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
namespace test {
    // Benchmark: 对一批图像进行处理评估效果。
    // 通过AppendOperator增加图像处理函数，对构造传入的文件夹下的所有图片进行批处理，结果保存在其下output文件夹中。
    class Benchmark {
    public:
        //Absolute path;
        Benchmark(const char* path, const char* output = "output");
        ~Benchmark();

        void AppendOperator(std::function<void(cv::Mat&, fs::path)>);
        void operator()();

        bool Bad() { return isBroken_; }
    private:
        fs::path input_folder_;
        fs::path output_folder_;
        std::vector<image::ImageProcessBase*> algo_vec_;
        std::vector<std::function<void(cv::Mat&, fs::path)>> operator_vec;
        cv::Mat mat_;
        bool isBroken_;

        void _DirLoop();
        inline bool _CheckExtension(const fs::path&, const char* suffix);
    };

    // [in]: 任意矩阵
    // [out]: 缩放后的矩阵，缩放比例scale = limit/矩阵的最大绝对值。
    void Scale(const cv::Mat& in, cv::Mat& out, const double limit, const int dtype = CV_64FC1);
    // [in]: 数值在[-255， 255]的矩阵
    // [out]: 一个将0点平移至uchar中值(255/2=127）的u8矩阵。 
    void ShiftToU8(const cv::Mat& in, cv::Mat& out);
    // [in]: 输入一组一维（row 为0）的mat，绘制一维曲线。
    //		输入 mat 的类型可为 64F, 32S, 8U
    //		y_axis表示绘图的y轴长为[-y_axis， y_axis]，x_thickness 表示绘制图形的粗细
    //		
    cv::Mat SimplePlot(const cv::Mat& in, const int y_axis = 500, const int x_thickness = 1, bool show = true);

    //返回绝对值矩阵，如输入为U8C1，先平移127再做绝对值
    cv::Mat MatABS(const cv::Mat& a);

    //根据传入的reduce将多通道图片转换成单通道。
    template<typename Ele, int CH>
    cv::Mat MultiCH2One(const cv::Mat& in, std::function<uchar(const Ele*, const int, const int)> reduce)
    {
        CV_Assert(in.isContinuous());
        const Ele* data = in.ptr<Ele>(0);
        int cols = in.cols * CH;
        cv::Mat ch1 = cv::Mat::zeros(in.size(), CV_8UC1);
        for (int i = 0; i < in.rows; ++i)
        {
            for (int j = 0; j < in.cols; ++j)
            {
                ch1.at<uchar>(i, j) = reduce(data + cols * i + j * CH, i, j);
            }
        }
        return ch1;
    }
}

void thinningIteration(cv::Mat& img, int iter);

/**
* Function for thinning the given binary image
*
* Parameters:
* 		src  The source image, binary with range = [0,255]
* 		dst  The destination image
*/
void thinning(const cv::Mat& src, cv::Mat& dst);

#endif