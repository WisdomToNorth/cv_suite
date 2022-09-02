#include "pch.h"

#include "frame_process.h"
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include  <glog/logging.h>
#include <chrono>
namespace image {
    using driver::CapturedFrame;

    FrameProcess::FrameProcess() :
        receivers_imgproc_(),
        receivers_lock_() {
    }

    FrameProcess::~FrameProcess() {
        LOG(INFO) << "[FrameProcess::~FrameProcess] deleting... wait for lock";
        std::lock_guard<std::mutex> lock(receivers_lock_);
        LOG(INFO) << "[FrameProcess::~FrameProcess] lock acuqired";
    }

    void FrameProcess::addImageProcess(ImageProcessPtr processer) {
        std::lock_guard<std::mutex> lock(receivers_lock_);
        receivers_imgproc_.insert(processer);
    }
    void FrameProcess::removeImageProcess(ImageProcessPtr ptr) {
        std::lock_guard<std::mutex> lock(receivers_lock_);
        auto found = receivers_imgproc_.find(ptr);
        if (found != receivers_imgproc_.end()) {
            receivers_imgproc_.erase(found);
        }
        else {
            LOG(WARNING) << "[FrameProcess::removeImageProcess] pass a ImageProcessPtr not in set: " << ptr->getName();
        }

    }
    void FrameProcess::process(CapturedFramePtr this_frame) {
        // This is called in camera driver's thread as a callback!
        std::lock_guard<std::mutex> lock(receivers_lock_);
        for (auto& processor : receivers_imgproc_) {
            if (!processor->isDisabled()) {
                processor->setOrigin(this_frame->image,
                    std::chrono::duration_cast<std::chrono::milliseconds>(this_frame->timepoint_sys.time_since_epoch()).count());
                processor->store();
                processor->process();
                processor->output();
            }
        }
    }
}