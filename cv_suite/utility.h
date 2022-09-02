#ifndef CVSUITE_SOURCE_UTILITY_H
#define CVSUITE_SOURCE_UTILITY_H

#include "image.h"

#include <functional>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
namespace test {
    // Benchmark: ��һ��ͼ����д�������Ч����
    // ͨ��AppendOperator����ͼ���������Թ��촫����ļ����µ�����ͼƬ�����������������������output�ļ����С�
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

    // [in]: �������
    // [out]: ���ź�ľ������ű���scale = limit/�����������ֵ��
    void Scale(const cv::Mat& in, cv::Mat& out, const double limit, const int dtype = CV_64FC1);
    // [in]: ��ֵ��[-255�� 255]�ľ���
    // [out]: һ����0��ƽ����uchar��ֵ(255/2=127����u8���� 
    void ShiftToU8(const cv::Mat& in, cv::Mat& out);
    // [in]: ����һ��һά��row Ϊ0����mat������һά���ߡ�
    //		���� mat �����Ϳ�Ϊ 64F, 32S, 8U
    //		y_axis��ʾ��ͼ��y�᳤Ϊ[-y_axis�� y_axis]��x_thickness ��ʾ����ͼ�εĴ�ϸ
    //		
    cv::Mat SimplePlot(const cv::Mat& in, const int y_axis = 500, const int x_thickness = 1, bool show = true);

    //���ؾ���ֵ����������ΪU8C1����ƽ��127��������ֵ
    cv::Mat MatABS(const cv::Mat& a);

    //���ݴ����reduce����ͨ��ͼƬת���ɵ�ͨ����
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