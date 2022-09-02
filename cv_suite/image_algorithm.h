#ifndef CVSUITE_SOURCE_DRIVER_IMAGE_ALGORITHM_H
#define CVSUITE_SOURCE_DRIVER_IMAGE_ALGORITHM_H
#include "core.h"

// Use math const defined in cmath header.
/**
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES 1
#endif
#include <cmath>
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
*/

#include <numeric>
#include <functional>
#include <opencv2/core.hpp>

namespace image {
    ///------------------
    constexpr double PI = CV_PI;
    constexpr double SQRT_PI = 1.772453850905516027;
    constexpr double SQRT_2_PI_INV = 0.398942280401432677939946059935;
    constexpr double SQRT_2 = 1.41421356237309504880;
    ///------------------

    // �ⷽ��
    //[in] a,b,y: 0 = a x +b
    //[out]:x
    //�ⷽ�� 0 = ax + b
    // double result;
    // if(SolveLinearX(a,b,result) {
    // result....
    template<typename Float>
    inline bool SolveLinearX(Float a, Float b, Float& x) {
        if (abs(a - static_cast<Float>(0.0)) < std::numeric_limits<Float>::epsilon()) {
            return false;
        }
        else {
            x = -b / a;
            return true;
        }
    }
    // Throw expecption when a = 0.0
    template<typename Float>
    inline Float SolveLinearUnsafe(Float a, Float b) {
        return -b / a;
    }

    // 2��2 ������ֵ������
    // �������� 2��2 �� F ����
    template<typename Float>
    void Eigen22(Float rr, Float rc, Float cc, Float* eigval, Float* eigvec) {
        Float theta, t, c, s, e1, e2, n1, n2; /* , phi;
                                               /* Compute the eigenvalues and eigenvectors of the Hessian matrix. */
        if (rc != static_cast<Float>(0.0)) {
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
    void Eigen22(const cv::Mat& mat22, cv::Mat& eigen_value, cv::Mat& eigen_vector);
    // ����1D����Ƚ������
    // [in]: two equal-size, one-row mat,depth <= U32
    // [out]: correlation.
    double Correlation1D(const cv::Mat& input1, const cv::Mat& input2);
    //
    // [in] : gray scale threshold.
    // [out]:  
    int ImgLineHeight(const cv::Mat& laser_mat, const uchar gray_threshold = 180);
    // ����ÿ�е����������ܺ͡�
    // [in]: CV_8U mat
    // [out]: a histgraph-like mat displaying the column-reduced mat.
    void PlotReducedHist(const cv::Mat& in, cv::Mat& out);
    // ���ķ���
    // Peak Detection: Centor of Mass
    void PeakDetectionCenterMass(const cv::Mat& in, cv::Mat& out, int);

    // ÿһ�е����ĵ���ɵ������ߣ�Ҫ�����ֵ��ͼ
    void SetCenterLineY(cv::Mat src);
    // ��˹������
    /* Functions to compute the integral, and the 0th and 1st derivative of the
    Gaussian function 1/(sqrt(2*PI)*sigma)*exp(-0.5*x^2/sigma^2) */
    namespace gaussian {
        template<typename Float>
        inline Float normal(const Float x) {
            return static_cast<Float>(0.5) * std::erfc(-x / static_cast<Float>(SQRT_2));
        }
        /* Integral of the Gaussian function */
        template<typename Float>
        inline Float phi0(const Float x, const Float sigma) {
            return normal(x / sigma);
        }
        /* The Gaussian function */
        template<typename Float>
        inline Float phi1(const Float x, const Float sigma) {
            Float t;
            t = x / sigma;
            return static_cast<Float>(SQRT_2_PI_INV) / sigma * std::exp(static_cast<Float>(-0.5) * t * t);
        }

        /* First derivative of the Gaussian function */
        template<typename Float>
        inline Float phi2(const Float x, const Float sigma) {
            const Float t = x / sigma;
            return -x * static_cast<Float>(SQRT_2_PI_INV) / std::pow(sigma, static_cast<Float>(3.0)) * std::exp(static_cast<Float>(-0.5) * t * t);
        }
    }
    //�õ�����ģ��/�˲��ĸ�˹��
    // Get 1-D Gaussian kernel or its derivative
    // [out] mat with row=1 and col = n*2+1, type = CV_F64U1.
    // [note] you can also pass n=-1 to automatically choose n depending on sigma.
    template<typename Float>
    cv::Mat Get1DGaussianKernel(int n, Float sigma, int ktype, unsigned derivative) {
        CV_Assert(derivative < 3);
        constexpr int SMALL_GAUSSIAN_SIZE = 7;
        const int DERIV_ORDER = 3;
        static const float small_gaussian_tab[][SMALL_GAUSSIAN_SIZE / 2 + 1][SMALL_GAUSSIAN_SIZE] = {
            //derivative = 0, small gaussian kernel
            {
                { 1.f },
                { 0.25f, 0.5f, 0.25f },
                { 0.0625f, 0.25f, 0.375f, 0.25f, 0.0625f },
                { 0.03125f, 0.109375f, 0.21875f, 0.28125f, 0.21875f, 0.109375f, 0.03125f }
            },
            //derivative = 1, derivative gaussian kernel
            {
                {}

            },
            //derivative = 2
            {
                {}
            }
        };

        const float* fixed_kernel = n % 2 == 1 && n <= SMALL_GAUSSIAN_SIZE && sigma <= 0 ?
            small_gaussian_tab[derivative][n >> 1] : 0;

        CV_Assert(ktype == CV_32F || ktype == CV_64F);
        cv::Mat kernel(n, 1, ktype);
        float* cf = kernel.ptr<float>();
        Float* cd = kernel.ptr<Float>();

        Float sigmaX = sigma > 0 ? sigma : ((n - 1) * static_cast<Float>(0.5) - 1) * static_cast<Float>(0.3) + static_cast<Float>(0.8);
        Float scale2X = -static_cast<Float>(0.5) / (sigmaX * sigmaX);
        Float sum = 0;

        int i;
        for (i = 0; i < n; i++) {
            Float x = i - (n - 1) * static_cast<Float>(0.5);
            Float t = fixed_kernel ? (Float)fixed_kernel[i] : std::exp(scale2X * x * x);
            if (ktype == CV_32F) {
                cf[i] = (float)t;
                sum += cf[i];
            }
            else {
                cd[i] = t;
                sum += cd[i];
            }
        }

        sum = 1. / sum;
        for (i = 0; i < n; i++) {
            if (ktype == CV_32F)
                cf[i] = (float)(cf[i] * sum);
            else
                cd[i] *= sum;
        }

        return kernel;
    }

    //kernel size = 2 * n + 1 or depend on sigma if n is too small
    template<typename Float>
    cv::Mat Get1DGaussian(int n, const Float sigma, const int ktype, const unsigned derivative) {
        CV_Assert(sigma > 0);
        const int kernel_size = (n < sigma * 3) ?
            static_cast<int>(sigma * 4) * 2 + 1 : 2 * n + 1;
        // kernel_size = 2n + 1
        n = kernel_size / 2;
        cv::Mat a(1, kernel_size, CV_MAKE_TYPE(ktype, 1));
        Float* d_p = a.ptr<Float>() + n;

        switch (derivative) {
        case 0: {
            // calculate gaussian from its integer to get a sum of 1.
            for (int i = -n + 1; i <= n - 1; ++i) {
                d_p[i] = gaussian::phi0(-i + static_cast<Float>(0.5), sigma) - gaussian::phi0(-i - static_cast<Float>(0.5), sigma);
            }
            d_p[-n] = static_cast<Float>(1.0) - gaussian::phi0(n - static_cast<Float>(0.5), sigma);
            d_p[n] = gaussian::phi0(-n + static_cast<Float>(0.5), sigma);
            break;
        }
        case 1: {
            for (int i = -n + 1; i <= n - 1; ++i) {
                d_p[i] = gaussian::phi1(-i - static_cast<Float>(0.5), sigma) - gaussian::phi1(-i + static_cast<Float>(0.5), sigma);
            }
            d_p[-n] = -gaussian::phi1(n - static_cast<Float>(0.5), sigma);
            d_p[n] = gaussian::phi1(-n + static_cast<Float>(0.5), sigma);
            break;
        }
        case 2: {
            for (int i = -n + 1; i <= n - 1; ++i) {
                d_p[i] = gaussian::phi2(-i + static_cast<Float>(0.5), sigma) - gaussian::phi2(-i - static_cast<Float>(0.5), sigma);
            }
            d_p[-n] = -gaussian::phi2(n - static_cast<Float>(0.5), sigma);
            d_p[n] = gaussian::phi2(-n + static_cast<Float>(0.5), sigma);
            break;
        }
        default:
            break;
        }

        return a;
    }

    //Steger�������߼�⡣
    // Paper: 
    // Steger, Carsten. "An Unbiased Detector of Curvilinear Structures."  IEEE Transactions on parttern analysis and machine intelligence 20.2(1998).
    using AngleLen = std::pair<float, size_t>;
    struct Line {
        AngleLen angle;
        common::Point2D<float> start;
        common::Point2D<float> end;
    };
    struct StegerParam {
        float sigma;
        std::vector<AngleLen> template_lines;
    };

    struct StegerResult {
        using Points = std::vector<common::Point2D<float>>;
        Points p;
        std::vector<Line> fitted_lines;
    };

    class StergerLineDetector :public cv::Algorithm {
    public:
        using Float = float;
        ;
        StergerLineDetector();
        ~StergerLineDetector();

        void Compute(const cv::Mat& source, Float sigma, Float salience_threshold = 0.1);
        void GetLines() const;
        cv::Mat GetSalientMat() const;
        cv::Mat GetFlagMat() const;
        cv::Mat GetNormMat() const;
        cv::Mat GetAngleMat() const;
        cv::Mat GetSubPixelMat() const; //CV_32FC2
        const common::VecLinePts& GetSubPixelVec() const;
        cv::Mat GetConvolv(int r, int c);
        cv::Mat GetConvolv32F(int r, int c);
        cv::Mat GetTestMat() const;
        std::vector<Float> GetPositionInfo(int r, int c);
        static constexpr Float PIXEL_BOUNDARY = static_cast<Float>(0.6);
    private:
        // 
        void _Covolution(const cv::Mat& source, const Float sigma);
        // Compute for information per pixel.
        void _LinePoints(const cv::Mat& source, Float salience_threshold);
        // Compute for sub pixels after _LinePoints
        void inline _SubPixel(int i, int j, Float r, Float c, Float rr, Float rc, Float cc, Float n1, Float n2, Float* p, uchar* ismax) {
            auto a = rr * n1 * n1 + static_cast<Float>(2.0) * rc * n1 * n2 + cc * n2 * n2;
            auto b = r * n1 + c * n2;
            if (std::abs(a) > std::numeric_limits<decltype(a)>::epsilon()) {
                auto t = -b / a;
                auto p1 = t * n1;
                auto p2 = t * n2;
                if (fabs(p1) <= PIXEL_BOUNDARY && fabs(p2) <= PIXEL_BOUNDARY) {
                    *ismax = 255;
                    //if (val >= low) {
                    //	if (val >= high)
                    //		ismax[j] = 2;
                    //	else
                    //		ismax[j] = 1;
                    //}
                    p[0] = static_cast<Float>(i) + p1;
                    p[1] = static_cast<Float>(j) + p2;
                }
            }
        }
        // eigen vec and eigen value of 2*2 matrix 
        inline void  _CalcEigen(Float dfdrr, Float dfdrc, Float dfdcc, Float* eigval, Float* eigvec);

        // conv result
        cv::Mat r1_c0_, r0_c1_, r1_c1_, r2_c0_, r0_c2_;
        // 
        cv::Mat gaussian_0_, gaussian_1_, gaussian_2_;
        // sigma of gaussian
        Float sigma_, salience_threshold_;

        ///result of linepoint, same size with the input image.
        // ev_mat_: the maximum eigenvalue of each pixel
        cv::Mat ev_mat_;
        // 0, 1 or 2, depend on elements of ev_mat compared with low and high threshod.
        // flag
        cv::Mat flag_mat_;
        // Norm direction
        cv::Mat n_mat_;
        // angle result
        cv::Mat angle_mat_;
        // Subpixel position.
        cv::Mat p_mat_;
        // Debug��
        cv::Mat test_mat_;
        //bool isParallel_;
        common::VecLinePts subpixels_;
    };


    // Histgram
    void HistPlot(const cv::Mat& in);

    void Scale(const cv::Mat& in, cv::Mat& out, const double limit, const int dtype = CV_64FC1);


    template<typename Ele>
    std::vector<int> ColScan(const cv::Mat& in)
    {
        std::vector<int>indexes;
        indexes.reserve(in.rows);
        for (int i = 0; i < in.rows; ++i)
        {
            auto row_p = in.ptr<Ele>(i);
            int max_index = 0;
            Ele max = 0;
            for (int j = 0; j < in.cols; ++j)
            {
                if (row_p[j] > max)
                {
                    max = row_p[j]; max_index = j;
                }
            }
            indexes.emplace_back(max_index);
        }
        return indexes;
    }
    template<typename Ele>
    std::vector<int> ColScanF(const cv::Mat& in, std::function<int(const Ele*, int)> scan_function)
    {
        std::vector<int>indexes;
        indexes.reserve(in.rows);
        for (int i = 0; i < in.rows; ++i)
        {
            const Ele* row_p = in.ptr<Ele>(i);
            indexes.emplace_back(scan_function(row_p, in.cols));
        }
        return indexes;
    }
    //scan_function: [in]cols array, [in]cols len, [in]rows number, [out]index.
    template<typename Ele>
    std::vector<int> ColScanF(const cv::Mat& in, std::function<int(const Ele*, int, int)> scan_function)
    {
        std::vector<int>indexes;
        indexes.reserve(in.rows);
        for (int i = 0; i < in.rows; ++i)
        {
            const Ele* row_p = in.ptr<Ele>(i);
            indexes.emplace_back(scan_function(row_p, in.cols, i));
        }
        return indexes;
    }

    //atan: return [0.0, 90.0]
    template<typename Float>
    Float atanxy(Float x, Float y) {
        Float degree_raw = cv::fastAtan2(y, x);
        Float degree = (degree_raw - static_cast<Float>(180.0) > 0.3) ? degree_raw - static_cast<Float>(180.0) : degree_raw;
        return degree;
    }
}
#endif