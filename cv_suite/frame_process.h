#pragma once
#include "image.h"
#include "imageprocessor/image_process.h"
#include "camera.h"
#include <memory>
#include <set>
#include <mutex>

namespace image {
    class FrameProcess;
    using ImageProcessPtr = std::shared_ptr<image::ImageProcessBase>;
    using driver::CapturedFramePtr;
    using FramePtr = std::shared_ptr<FrameProcess>;
    using FrameWeakPtr = std::weak_ptr<FrameProcess>;

    class FrameProcess {
        std::set<ImageProcessPtr> receivers_imgproc_;
        mutable std::mutex receivers_lock_;
        //void afterImageProcess();
    public:
        FrameProcess();
        ~FrameProcess();

        FrameProcess(const FrameProcess&) = delete;
        FrameProcess& operator=(const FrameProcess&) = delete;

        // Add a image processer for grabbed frame.
        // return the interator pointing to where the ptr stored in internal vector.
        void addImageProcess(ImageProcessPtr);
        void removeImageProcess(ImageProcessPtr index);
        void process(CapturedFramePtr);
    };
}