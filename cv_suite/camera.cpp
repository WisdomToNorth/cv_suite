#include "pch.h"
#include "camera.h"

#include <memory>
#include <functional>
#include <boost/lexical_cast.hpp>
#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>

#include <GxIAPI.h>
#ifdef WIN32
#include <windows.h>
#endif
#pragma warning(disable : 4996)
namespace driver {
    using lock_guard = std::lock_guard<std::mutex>;
    //definition of static member in class Camera.
    LibController Camera::libraryController;
#ifdef WIN32
    //[Input]: wstring with utf16 encoding
    //[Output]: string can be used in fopen
    //[Exception]: throw when failed.
    //[Note]: Using vector as buffer (although reserved) and coping from vector buffer to the string 
    //make this function not efficient enough, compared to direct return a heap allocated char*.
    std::string wchar2ansi(std::wstring wstrcode) {
        int aSize = ::WideCharToMultiByte(CP_ACP, 0, wstrcode.c_str(), -1, NULL, 0, NULL, NULL);
        if (aSize == ERROR_NO_UNICODE_TRANSLATION) {
            throw std::exception("Invalid UTF-8 sequence.");
        }
        if (aSize == 0) {
            throw std::exception("Error in conversion.");
        }
        std::vector<char> bufVec(aSize + 1);
        int convResult = ::WideCharToMultiByte(CP_OEMCP, 0, wstrcode.c_str(), -1, &bufVec[0], aSize, NULL, NULL);
        if (convResult != aSize) {
            throw std::exception("La falla!");
        }
        return std::string(bufVec.data());
    }
#endif
    //Log
    //std::vector<spdlog::sink_ptr> sinks;
    //std::shared_ptr<spdlog::logger> driver_logger;
    const char* state_cstr[]{
        "UnKnown",
        "Closed",
        "Open",
        "Snapping"
    };

    enum CAM_STATE {
        CAM_STATUS_REGISTER_CALLBACK = -30,
        CAM_STATUS_SNAPSTART_WRONGSTATE = -31,
        CAM_STATUS_SNAPSTOP_WRONGSTATE = -32,
        CAM_STATUS_OPEN_WRONGSTATE = -33,
        CAM_STATUS_CLOSE_WRONGSTATE = -34,
        CAM_STATUS_NO_DEVICE_DETECTED = -35,
        CAM_STATUS_TRIGGER_MODE_WRONG = -36,
        CAM_STATUS_NO_CALLBACK_REGISTERED = -37
    };

    //Chinese error string.
    std::string GetErrorString(GX_STATUS errorStatus) {
        std::string strShow;

        switch (errorStatus) {
        case GX_STATUS_SUCCESS:
            strShow = "SUCCESS";
            break;
        case GX_STATUS_ERROR:
            strShow = "GX_STATUS_ERROR";
            break;
        case GX_STATUS_NOT_FOUND_TL:
            strShow = "GX_STATUS_NOT_FOUND_TL";
            break;
        case GX_STATUS_NOT_FOUND_DEVICE:
            strShow = "GX_STATUS_NOT_FOUND_DEVICE";
            break;
        case GX_STATUS_OFFLINE:
            strShow = "GX_STATUS_OFFLINE";
            break;
        case GX_STATUS_INVALID_PARAMETER:
            strShow = "GX_STATUS_INVALID_PARAMETER";
            break;
        case GX_STATUS_INVALID_HANDLE:
            strShow = "GX_STATUS_INVALID_HANDLE";
            break;
        case GX_STATUS_INVALID_CALL:
            strShow = "GX_STATUS_INVALID_CALL";
            break;
        case GX_STATUS_INVALID_ACCESS:
            strShow = "GX_STATUS_INVALID_ACCESS";
            break;
        case GX_STATUS_NEED_MORE_BUFFER:
            strShow = "GX_STATUS_NEED_MORE_BUFFER";
            break;
        case GX_STATUS_ERROR_TYPE:
            strShow = "GX_STATUS_ERROR_TYPE";
            break;
        case GX_STATUS_OUT_OF_RANGE:
            strShow = "GX_STATUS_OUT_OF_RANGE";
            break;
        case GX_STATUS_NOT_IMPLEMENTED:
            strShow = "GX_STATUS_NOT_IMPLEMENTED";
            break;
        case GX_STATUS_NOT_INIT_API:
            strShow = "GX_STATUS_NOT_INIT_API";
            break;
        case GX_STATUS_TIMEOUT:
            strShow = "GX_STATUS_TIMEOUT";
            break;
            // Custom Error Code:
        case CAM_STATUS_REGISTER_CALLBACK:
            strShow = "";
            break;
        case CAM_STATUS_SNAPSTART_WRONGSTATE:
            strShow = "CAM_STATUS_SNAPSTART_WRONGSTATE";
            break;
        case CAM_STATUS_SNAPSTOP_WRONGSTATE:
            strShow = "CAM_STATUS_SNAPSTOP_WRONGSTATE";
            break;
        case CAM_STATUS_OPEN_WRONGSTATE:
            strShow = "CAM_STATUS_OPEN_WRONGSTATE";
            break;
        case CAM_STATUS_CLOSE_WRONGSTATE:
            strShow = "CAM_STATUS_CLOSE_WRONGSTATE";
            break;
        case CAM_STATUS_NO_DEVICE_DETECTED:
            strShow = "CAM_STATUS_NO_DEVICE_DETECTED";
            break;
        case CAM_STATUS_TRIGGER_MODE_WRONG:
            strShow = "CAM_STATUS_TRIGGER_MODE_WRONG";
            break;
        case CAM_STATUS_NO_CALLBACK_REGISTERED:
            strShow = "CAM_STATUS_NO_CALLBACK_REGISTERED";
        default:
            strShow = "Unknwon Code:" + std::to_string(errorStatus);
            break;
        }

        return strShow;
    }

    LibController::LibController() : libStatus_(GX_STATUS_NOT_INIT_API) {
        libStatus_ = GXInitLib();
        //sinks.push_back(std::make_shared<spdlog::sinks::stderr_sink_mt>());
        //sinks.push_back(std::make_shared<spdlog::sinks::daily_file_sink_st>("driver", "txt", 23, 59));
        //driver_logger = std::make_shared<spdlog::logger>("driver", begin(sinks), end(sinks));;
        //driver_logger->info("Test");
    }

    LibController::~LibController() {
        GXCloseLib();
    }

    GX_STATUS LibController::getLibStatus() const {
        return libStatus_;
    }

    std::string LibController::getLibStatusString() const {
        return GetErrorString(getLibStatus());
    }

    bool LibController::closeLib() {
        libStatus_ = GXCloseLib();
        return libStatus_ == GX_STATUS_SUCCESS;
    }

    bool LibController::enumCameraList() {
        return true;
    }
    namespace {
        void FromDHCamera(const GX_FRAME_CALLBACK_PARAM& frame, CapturedFrame& output, bool raw) {
            if (frame.status == GX_FRAME_STATUS_SUCCESS) {
                output.timepoint = std::chrono::high_resolution_clock::now();
                output.timepoint_sys = std::chrono::system_clock::now();
                //output.imgSize = frame.nImgSize;
                output.timestamp = frame.nTimestamp;
                output.frameID = frame.nFrameID;
                output.imgSuffix_ = ".bmp";
                output.image.create(frame.nHeight, frame.nWidth, CV_8UC3);
                if (!global::clock_point.IsSync()) {
                    global::clock_point.SyncImageTime(static_cast<common::ImageID>(output), output.timepoint);
                }
                else {
                    global::weld_data.frame_vector.push_back(
                        std::make_shared<common::ImageID>(static_cast<common::ImageID>(output)));
                }

                memcpy(output.image.data, frame.pImgBuf, frame.nHeight * frame.nWidth * 3);
                if (!raw) {
                    cv::transpose(output.image, output.image);
                    cv::flip(output.image, output.image, 1);
                }
            }
            else {
                //TODO: log
            }
        }

        void GX_STDC CallbackGxIAPI(GX_FRAME_CALLBACK_PARAM* frame) {
            //std::unique_lock<std::mutex> l(camera_lock);
            auto this_frame = std::make_shared<CapturedFrame>();
            // we should guarentee this is a alive FrameProcess object.
            auto cam = static_cast<Camera*>(frame->pUserParam);
            if (cam->HasRegisteredCallback()) {
                FromDHCamera(*frame, *this_frame, true);
                cam->GetCamCallback()(this_frame);
            }
        }
    }

    struct CameraImpl {
        int					hIndex_;
        GX_DEV_HANDLE		hDevice_;
        GX_STATUS			status_;
        CameraState			state_;

        int64_t				imageSize_;
        int64_t				imageHeight_;
        int64_t				imageWidth_;
        int64_t				triggerMode_;
        double				shutter_;
        double				gain_;
        bool				isTrigger_;
        //camera_lock control the callback and open, snap, close of camera.
        mutable std::mutex camera_lock_;
        CameraCallback cameraCallback_;
        CameraImpl(int index) :
            hIndex_(index), //Camera index
            hDevice_(nullptr), //handler
            status_(GX_STATUS_SUCCESS), //status code	
            state_(CameraState::Closed), //current state
            imageSize_(0), // image size
            imageHeight_(0), //
            imageWidth_(0), //
            triggerMode_(GX_TRIGGER_MODE_OFF), //
            shutter_(0), //
            gain_(0),//
            isTrigger_(false),
            camera_lock_(),
            cameraCallback_() {}


        bool SendCommand(enum GX_FEATURE_ID commandID) {
            lock_guard l(camera_lock_);
            status_ = GXSendCommand(hDevice_, commandID);
            return status_ == GX_STATUS_SUCCESS;
        }

        bool RegisterCaptureCallback(GXCaptureCallBack captureCallbackFunction, void* callbackParam) {
            lock_guard l(camera_lock_);
            status_ = GXRegisterCaptureCallback(hDevice_, callbackParam, captureCallbackFunction);
            return status_ == GX_STATUS_SUCCESS;
        }

        bool UnregisterCaptureCallback() {
            lock_guard l(camera_lock_);
            status_ = GXUnregisterCaptureCallback(hDevice_);
            return status_ == GX_STATUS_SUCCESS;
        }

        bool SetFloat(double f, GX_FEATURE_ID_CMD param_id) {
            lock_guard l(camera_lock_);
            status_ = GXSetFloat(hDevice_, param_id, f);
            return status_ == GX_STATUS_SUCCESS;
        }

        std::tuple<bool, double> GetFloat(GX_FEATURE_ID_CMD param_id) {
            lock_guard l(camera_lock_);
            double val;
            status_ = GXGetFloat(hDevice_, param_id, &val);
            return std::make_tuple(status_ == GX_STATUS_SUCCESS, val);
        }

        bool _SetPixelFormat8Bit() {
            int64_t nPixelSize = 0;
            uint32_t nEnmuEntry = 0;
            size_t nBufferSize = 0;
            //BOOL      bIs8bit = TRUE;

            GX_ENUM_DESCRIPTION* pEnumDescription = NULL;
            //GX_ENUM_DESCRIPTION  *pEnumTemp = NULL;
            status_ = GXGetEnum(hDevice_, GX_ENUM_PIXEL_SIZE, &nPixelSize);
            if (status_ != GX_STATUS_SUCCESS) return false;
            if (nPixelSize == GX_PIXEL_SIZE_BPP8) {
                return true;
            }
            else {
                status_ = GXGetEnumEntryNums(hDevice_, GX_ENUM_PIXEL_FORMAT, &nEnmuEntry);
                if (status_ != GX_STATUS_SUCCESS) return false;

                nBufferSize = nEnmuEntry * sizeof(GX_ENUM_DESCRIPTION);
                pEnumDescription = new GX_ENUM_DESCRIPTION[nEnmuEntry];

                status_ = GXGetEnumDescription(hDevice_, GX_ENUM_PIXEL_FORMAT, pEnumDescription, &nBufferSize);
                if (status_ != GX_STATUS_SUCCESS) {
                    if (pEnumDescription == NULL) {
                        delete[]pEnumDescription;
                        pEnumDescription = NULL;
                    }
                    return status_ == GX_STATUS_SUCCESS;
                }
                for (uint32_t i = 0; i < nEnmuEntry; i++) {
                    if ((pEnumDescription[i].nValue & GX_PIXEL_8BIT) == GX_PIXEL_8BIT) {
                        status_ = GXSetEnum(hDevice_, GX_ENUM_PIXEL_FORMAT, pEnumDescription[i].nValue);
                        break;
                    }
                }

                if (pEnumDescription != NULL) {
                    delete[]pEnumDescription;
                    pEnumDescription = NULL;
                }
            }
            return status_ == GX_STATUS_SUCCESS;
        }
    };

    Camera::Camera(int index) :
        d_(std::make_unique<CameraImpl>(index))//
    {}

    // open close snapStart snapStop...
    // return false and change  status_ to corresponding error code if errors occur
    // return true and except of open change state_

    // [note]
    // open() will not change state_
    // 
    // It is guarenteed that open state has not registered callback
    bool Camera::Open(const bool isTrigger, const int heartbeat_ms) {
        const GX_OPEN_MODE_CMD openMode = GX_OPEN_INDEX;
        const GX_ACCESS_MODE_CMD accessMode = GX_ACCESS_EXCLUSIVE;
        GX_TRIGGER_MODE_ENTRY triggerMode = isTrigger ? GX_TRIGGER_MODE_ON : GX_TRIGGER_MODE_OFF;
        d_->isTrigger_ = isTrigger;
        LOG(INFO) << "[Camera::Open] Opening... Trigger Mode: " << isTrigger ? "GX_TRIGGER_MODE_ON" : "GX_TRIGGER_MODE_OFF";
        if (d_->hIndex_ <= 0) return false;
        std::string tempNumStr = std::to_string(d_->hIndex_);
        GX_OPEN_PARAM openParam;
        openParam.pszContent = const_cast<char*>(tempNumStr.c_str());
        openParam.openMode = openMode;
        openParam.accessMode = accessMode;
        uint32_t nDevNum = 0;

        switch (d_->state_) {
        case CameraState::Closed: {
            lock_guard hold(d_->camera_lock_);
            d_->status_ = GXUpdateDeviceList(&nDevNum, 1000);
            // Suppose to check status?
            if (nDevNum <= 0) {
                d_->status_ = CAM_STATUS_NO_DEVICE_DETECTED;
                LOG(WARNING) << "[Camera::Open] Open Failed. Found no Camera.";
                return false;
            }
            d_->status_ = GXOpenDevice(&openParam, &d_->hDevice_);
            if (d_->status_ != GX_STATUS_SUCCESS) return false;
            if (d_->hDevice_ == nullptr) return false;
            d_->state_ = CameraState::Open;
            d_->status_ = GXSetEnum(d_->hDevice_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            if (d_->status_ != GX_STATUS_SUCCESS) return false;

            d_->status_ = GXSetEnum(d_->hDevice_, GX_ENUM_TRIGGER_MODE, triggerMode);
            if (d_->status_ != GX_STATUS_SUCCESS) return false;
            else d_->triggerMode_ = triggerMode;
            //Software trigger
            if (d_->triggerMode_ == GX_TRIGGER_MODE_ON) {
                d_->status_ = GXSetEnum(d_->hDevice_, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
                if (d_->status_ != GX_STATUS_SUCCESS) return false;
            }

            GXSetInt(d_->hDevice_, GX_INT_GEV_HEARTBEAT_TIMEOUT, heartbeat_ms);//Heartbeat: now 20s.
            return true;
        }

        case CameraState::Open:
        case CameraState::Unknown:
        case CameraState::Snapping:
        default:
            d_->status_ = CAM_STATUS_OPEN_WRONGSTATE;
            return false;
        }
    }

    // Snapping | Open -> Closed
    bool Camera::Close() {
        switch (d_->state_) {
        case CameraState::Snapping:
            if (!SnapStop())
                return false;
        case CameraState::Open: {
            lock_guard l(d_->camera_lock_);
            d_->status_ = GXCloseDevice(d_->hDevice_);
            if (d_->status_ != GX_STATUS_SUCCESS)
                return false;
            //TODO set status to error code��
            d_->hDevice_ = nullptr;
            d_->state_ = CameraState::Closed;
            return true;
        }
        case CameraState::Closed:
            return true;
        default:
            d_->status_ = CAM_STATUS_SNAPSTOP_WRONGSTATE;
            return false;
        }
    }

    //Open -> Snapping.
    bool Camera::SnapStart() {
        if (!d_->cameraCallback_) {
            LOG(WARNING) << "[Camera::SnapStart]:" << "No cameraCallback registered!";
            return false;
        }
        LOG(INFO) << "[Camera::snapStart]: ""Try to start snapping.";
        switch (d_->state_) {
        case CameraState::Open: {
            d_->RegisterCaptureCallback(CallbackGxIAPI, this);
            if (!d_->SendCommand(GX_COMMAND_ACQUISITION_START)) {
                LOG(WARNING) << "[Camera::snapStart]: Snapstarting failed " << GetLastErrorString();
            }
            else {
                d_->state_ = CameraState::Snapping;
                return true;
            }
        }
        case CameraState::Snapping:
            return true;
        case CameraState::Closed:
        case CameraState::Unknown:
        default:
            d_->status_ = CAM_STATUS_SNAPSTART_WRONGSTATE;
            return false;
        }
    }

    // Snapping -> Open.
    bool Camera::SnapStop() {
        LOG(INFO) << "[Camera::SnapStop]: ""Try to stop snapping.";
        switch (d_->state_) {
        case CameraState::Snapping: {
            if (!d_->SendCommand(GX_COMMAND_ACQUISITION_STOP)) {
                return false;
            }
            d_->state_ = CameraState::Open;
            if (!d_->UnregisterCaptureCallback()) {
                LOG(WARNING) << "[Camera::SnapStop]: Unregister callback failed!.";
                return false;
            }
            return true;
        }
        case CameraState::Open:
            return true;
        case CameraState::Closed:
        case CameraState::Unknown:
        default:
            d_->status_ = CAM_STATUS_SNAPSTOP_WRONGSTATE;
            return false;
        }

    }
    bool Camera::SnapCommand() {
        if (d_->triggerMode_ != GX_TRIGGER_MODE_ON) {
            d_->status_ = CAM_STATUS_TRIGGER_MODE_WRONG;
            return false;
        }
        return d_->SendCommand(GX_COMMAND_TRIGGER_SOFTWARE);
    }
    CameraState Camera::GetState() const {
        lock_guard l(d_->camera_lock_);
        return d_->state_;
    }

    const char* Camera::GetStateString() const {
        return state_cstr[static_cast<size_t>(GetState())];
    }

    std::string Camera::GetLastErrorString() const {
        lock_guard l(d_->camera_lock_);
        return GetErrorString(d_->status_);
    }

    void Camera::RegisterCameraCallback(CameraCallback callbacK_cam) {
        lock_guard l(d_->camera_lock_);
        d_->cameraCallback_ = callbacK_cam;
    }

    bool Camera::HasRegisteredCallback() const {
        lock_guard l(d_->camera_lock_);
        return d_->cameraCallback_ != nullptr;
    }

    CameraCallback& Camera::GetCamCallback() const {
        lock_guard l(d_->camera_lock_);
        return d_->cameraCallback_;
    }

    bool Camera::SetShutter(double shutter) {
        LOG(INFO) << "[Camera::setShutter] set to" << shutter;
        return d_->SetFloat(shutter * 1000, GX_FLOAT_EXPOSURE_TIME);
    }

    bool Camera::SetGain(double gain) {
        return d_->SetFloat(gain, GX_FLOAT_GAIN);
    }
    bool Camera::SetCameraROI(int64_t offsetX, int64_t offsetY, int64_t width, int64_t height) {
        lock_guard hold(d_->camera_lock_);
        int64_t max_width = 99999, max_height = 99999;
        d_->status_ = GXGetInt(d_->hDevice_, GX_INT_WIDTH_MAX, &max_width);
        d_->status_ = GXGetInt(d_->hDevice_, GX_INT_HEIGHT_MAX, &max_height);
        if (d_->status_ != GX_STATUS_SUCCESS || offsetX + width > max_width || offsetY + height > max_height) {
            return false;
        }
        d_->status_ = GXSetInt(d_->hDevice_, GX_INT_WIDTH, width);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Set width faild! " << GetLastErrorString();
        }
        d_->status_ = GXSetInt(d_->hDevice_, GX_INT_HEIGHT, height);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Set height faild! " << GetLastErrorString();
        }
        d_->status_ = GXSetInt(d_->hDevice_, GX_INT_OFFSET_X, offsetX);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Set offsetX faild! " << GetLastErrorString();
        }
        d_->status_ = GXSetInt(d_->hDevice_, GX_INT_OFFSET_Y, offsetY);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Set offsetY faild! " << GetLastErrorString();
        }
        return d_->status_ == GX_STATUS_SUCCESS;
    }
    std::tuple<bool, int64_t, int64_t, int64_t, int64_t> Camera::GetCameraROI() const {
        lock_guard hold(d_->camera_lock_);
        int64_t width, height, offsetX, offsetY;
        d_->status_ = GXGetInt(d_->hDevice_, GX_INT_WIDTH, &width);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Get  width! " << GetLastErrorString();
        }
        d_->status_ = GXGetInt(d_->hDevice_, GX_INT_HEIGHT, &height);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Get  height! " << GetLastErrorString();
        }
        d_->status_ = GXGetInt(d_->hDevice_, GX_INT_OFFSET_X, &offsetX);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Get  offsetX! " << GetLastErrorString();
        }
        d_->status_ = GXGetInt(d_->hDevice_, GX_INT_OFFSET_Y, &offsetY);
        if (d_->status_ != GX_STATUS_SUCCESS) {
            LOG(WARNING) << "[Camera::SetCameraROI]: Get  offsetY! " << GetLastErrorString();
        }
        return std::make_tuple(d_->status_ == GX_STATUS_SUCCESS,
            offsetX, offsetY, width, height);
    }
    std::tuple<bool, double> Camera::GetShutter() {
        return d_->GetFloat(GX_FLOAT_EXPOSURE_TIME);
    }


    int Camera::GetIndex() const {
        return d_->hIndex_;
    }
    bool Camera::IsTriggerMode() const {
        return d_->isTrigger_;
    }
    std::tuple<int64_t, int64_t> Camera::GetWidthHeight() const {
        return std::make_tuple(d_->imageWidth_, d_->imageHeight_);
    }

    // Destructor
    Camera::~Camera() {
        Close();
    }
    /*
    void GX_STDC callbackGxIAPI_deprecated(GX_FRAME_CALLBACK_PARAM* framePtr)
    {
        //std::unique_lock<std::mutex> l(camera_lock_);
        CapturedFrame this_frame;
        image::ImageProcessBase *worker = static_cast<image::ImageProcessBase*>(framePtr->pUserParam);
        worker->setTimePoint(std::chrono::system_clock::now());
        fromDHCamera(*framePtr, this_frame, true);
        worker->setOrigin(this_frame.image);
        worker->store();
        worker->process();
        worker->output();
    }
    */
}
