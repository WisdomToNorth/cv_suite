#pragma once
#ifndef CVSUITE_SOURCE_DRIVER_CAMERA_H
#define CVSUITE_SOURCE_DRIVER_CAMERA_H

#include "core.h"
#include <string>
#include <vector>
#include <mutex>
#include <functional>
#include <boost/noncopyable.hpp>
#include <opencv2/core.hpp>

namespace driver {
    enum class CallbackType {
        CAPTURE,
        FEATURE,
        OFFLINE
    };

    struct CapturedFrame :public common::ImageID {
        cv::Mat				image;
        cv::Mat				raw_image;
        int					viewID;
        int32_t				imgSize;
        const char* imgSuffix_;
        CapturedFrame() = default;
        CapturedFrame(const CapturedFrame&) = default;
        CapturedFrame& operator=(const CapturedFrame&) = default;
        ~CapturedFrame() = default;
    };
    using CapturedFramePtr = std::shared_ptr<CapturedFrame>;
    // Copy data from camera buffer, for GIGE cameras from DaHeng.
    // raw: wheter the image will be transpose and flip
    //void fromDHCamera(const GX_FRAME_CALLBACK_PARAM &frame, CapturedFrame &output, bool raw = false);
    //<code>
    //fromDHCamera(pframeData, worker->mat);
    //worker->process();
    //worker->output();
    //</code>
    // see image/frame_process.h for new version.
    // void GX_STDC callbackGxIAPI_deprecated(GX_FRAME_CALLBACK_PARAM *frame);

    using GX_STATUS = int32_t;
    class LibController : boost::noncopyable {
    public:
        LibController();
        ~LibController();
        GX_STATUS getLibStatus() const;
        std::string getLibStatusString() const;
        bool closeLib();
        bool enumCameraList();

    private:
        GX_STATUS libStatus_;
    };

    enum class CameraState {
        Unknown = 0,
        Closed = 1,
        Open = 2,
        Snapping = 3,
    };
    extern const char* state_cstr[];

    using CameraCallback = std::function<void(CapturedFramePtr)>;
    struct CameraImpl;

    // Usage: register your callback by RegisterCameraCallback.
    class Camera {
        std::unique_ptr<CameraImpl> d_;

    public:
        static LibController libraryController;

        explicit Camera(int index);
        ~Camera();
        Camera(const Camera&) = delete;
        Camera& operator=(const Camera&) = delete;

        //isTrigger: if true, you can call SnapCommand to mannually trigger camera.
        //Implemented Trigger mode: software and auto
        //heartbeat_ms: heartbeat time in milliseconds.
        bool Open(const bool isTrigger, const int heartbeat_ms);
        bool Close();
        //bool snapStart(GXCaptureCallBack captureCallbackFunction, void* callbackParam);
        bool SnapStart();
        bool SnapStop();
        bool SnapCommand();
        int64_t  m_nPixelColorFilter;
        void RegisterCameraCallback(CameraCallback);
        bool HasRegisteredCallback() const;
        CameraCallback& GetCamCallback() const;

        CameraState GetState() const;
        const char* GetStateString() const;
        std::string GetLastErrorString() const;
        //Set and get
        bool SetShutter(double shutter);
        bool SetGain(double gain);
        bool SetCameraROI(int64_t offsetX, int64_t offsetY, int64_t width, int64_t height);
        std::tuple<bool, int64_t, int64_t, int64_t, int64_t> GetCameraROI() const;
        std::tuple<bool, double> GetShutter();
        //-----
        // [out]:
        //GX_DEV_HANDLE getHandle() const;
        // [out]:
        int GetIndex() const;
        bool IsTriggerMode() const;
        std::tuple<int64_t, int64_t> GetWidthHeight() const;
    };


}
#endif
