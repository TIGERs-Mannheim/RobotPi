/*
 * camera.h
 *
 *  Created on: 21.08.2017
 *      Author: AndreR
 */

#pragma once

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_connection.h"

#include "interface/FrameYUV420.h"

#include <functional>
#include <thread>

namespace rpi
{

enum CameraIso
{
    CAMERA_ISO_100,
    CAMERA_ISO_200,
    CAMERA_ISO_400,
    CAMERA_ISO_800,
};

enum CameraRotation
{
    CAMERA_ROTATE_0,
    CAMERA_ROTATE_90,
    CAMERA_ROTATE_180,
    CAMERA_ROTATE_270,
};

enum CameraMirror
{
    CAMERA_MIRROR_NONE,
    CAMERA_MIRROR_HORIZONTAL,
    CAMERA_MIRROR_VERTICAL,
    CAMERA_MIRROR_BOTH,
};

enum class ECameraRes
{
    RES_2560x1920_FPS_15,
    RES_1280x960_FPS_42,
    RES_640x480_FPS_90
};

struct Resolution
{
    int32_t width;
    int32_t height;
};

struct ExposureSettings
{
    bool autoMode;
    float analogGain; // 1.0 - 8.0
    float digitalGain; // 1.0 - 2.0
    uint32_t exposureTimeUs; // maximum depends on framerate

    bool operator==(const ExposureSettings& rhs) const;
};

struct WhiteBalanceSettings
{
    bool autoMode;
    float redGain; // 0.0 - 8.0, typical 0.9 - 1.9
    float blueGain; // 0.0 - 8.0, typical 0.9 - 1.9

    bool operator==(const WhiteBalanceSettings& rhs) const;
};

class RPiCamera
{
public:
    using FrameCallback = std::function<void(FrameYUV420*)>;

    RPiCamera();
    virtual ~RPiCamera();

    void start(ECameraRes res);
    void stop();

    void triggerCapture();
    void setRecording(bool enable);

    void setFrameCallback(FrameCallback cb) { previewFrameCallback_ = cb; }

    void setExposureSettings(ExposureSettings settings);
    void setWhiteBalanceSettings(WhiteBalanceSettings settings);
    void setEncoderQuantisation(uint32_t quantisation); // approx. 10-40, low value means better quality

    bool isRecording() const { return recording_; }
    uint32_t getRecordSize() const { return recordSize_; }
    float getRecordDuration() const;
    const char* getRecordFilename() const { return videoFilename_; }

    uint32_t getImagesTaken() const { return imagesTaken_; }
    const char* getImageFilename() const { return imageFilename_; }

    int64_t getStcTimestampUs();
    const Resolution& getResolution() const { return camResolution_; }

private:
    friend void previewOutputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf);
    friend void rendererInputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf);
    friend void imageEncoderOutputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf);
    friend void videoEncoderOutputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf);

    void releaseFrame(MMAL_BUFFER_HEADER_T* pBuf);
    void handlePreviewFrame(MMAL_BUFFER_HEADER_T* pBuf);
    void handleImageEncoderFrame(MMAL_BUFFER_HEADER_T* pBuf);
    void handleVideoEncoderFrame(MMAL_BUFFER_HEADER_T* pBuf);

    void setSaturation(int saturation); // -100 - 100
    void setSharpness(int sharpness); // -100 - 100
    void setContrast(int contrast); // -100 - 100
    void setBrightness(int brightness); // 0 - 100
    void setIso(CameraIso iso);
    void setMeteringMode(MMAL_PARAM_EXPOSUREMETERINGMODE_T mode);
    void setExposureCompensation(int expComp); // -10 - 10
    void setExposureMode(MMAL_PARAM_EXPOSUREMODE_T mode);
    void setAwbMode(MMAL_PARAM_AWBMODE_T awb_mode);
    void setAwbGains(float r_gain, float b_gain);
    void setRotation(CameraRotation rotation);
    void setMirror(CameraMirror flip);
    void setShutterSpeed(int speed_us);
    void setAlgorithmControl(MMAL_PARAMETER_ALGORITHM_CONTROL_ALGORITHMS_T algo, bool enable);
    void setUseCase(MMAL_PARAM_CAMERA_USE_CASE_T useCase);
    void setZeroShutterLag(bool enable);
    void setAnalogGain(float analog);
    void setDigitalGain(float digital);

    void setupCamera();
    void setupStillPort();
    void setupVideoPort();
    void configurePreviewPort();
    void setDefaultParameters();

    void setupRenderer();
    void configureRenderer();

    void setupImageEncoder();

    void setupVideoEncoder();

    void run();

    void printPortEncodings(MMAL_PORT_T* pPort);

    FrameCallback previewFrameCallback_;

    ECameraRes eResolution_;
    Resolution camResolution_;
    uint16_t framerate_;

    ExposureSettings exposureSettings_;
    WhiteBalanceSettings whiteBalanceSettings_;

    bool runThread;
    std::thread processingThread_;

    MMAL_COMPONENT_T* pCameraComponent;
    MMAL_PORT_T* pPreviewPort;
    MMAL_POOL_T* pPreviewPool;
    MMAL_PORT_T* pStillPort;
    MMAL_PORT_T* pVideoPort;

    MMAL_QUEUE_T* pFrameQueue;

    MMAL_COMPONENT_T* pRenderer;
    MMAL_PORT_T* pRendererPort;

    MMAL_COMPONENT_T* pImageEncoder_;
    MMAL_CONNECTION_T* pImageEncoderConnection_;
    MMAL_POOL_T* pImageEncoderPool_;
    FILE* pImageEncoderFile_;
    uint32_t imagesTaken_;
    char imageFilename_[80];

    MMAL_COMPONENT_T* pVideoEncoder_;
    MMAL_CONNECTION_T* pVideoEncoderConnection_;
    MMAL_POOL_T* pVideoEncoderPool_;
    bool recording_;
    FILE* pVideoEncoderFile_;
    char videoFilename_[80];
    uint32_t recordSize_;
    uint64_t recordStartTime_;
};

} // namespace rpi
