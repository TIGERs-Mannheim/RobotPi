/*
 * camera.c
 *
 *  Created on: 21.08.2017
 *      Author: AndreR
 */

#include "RPiCamera.h"
#include <cstdio>
#include <stdexcept>
#include <map>

#include "util/log.h"

#include "interface/mmal/mmal_logging.h"
#include "interface/mmal/mmal_buffer.h"
#include "interface/mmal/util/mmal_util.h"
#include "interface/mmal/util/mmal_util_params.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"
#include "interface/mmal/mmal_parameters_camera.h"

// Standard port setting for the camera component
#define MMAL_CAMERA_PREVIEW_PORT  0
#define MMAL_CAMERA_VIDEO_PORT    1
#define MMAL_CAMERA_CAPTURE_PORT  2

// Video render needs at least 2 buffers to get to 60fps. 3 buffers for 90fps.
#define VIDEO_OUTPUT_BUFFERS_NUM 3

namespace rpi
{

bool ExposureSettings::operator==(const ExposureSettings& rhs) const
{
    return autoMode == rhs.autoMode && analogGain == rhs.analogGain && digitalGain == rhs.digitalGain && exposureTimeUs == rhs.exposureTimeUs;
}

bool WhiteBalanceSettings::operator==(const WhiteBalanceSettings& rhs) const
{
    return autoMode == rhs.autoMode && redGain == rhs.redGain && blueGain == rhs.blueGain;
}

struct CameraRes
{
    int32_t width;
    int32_t height;
    int32_t fps;
};

const std::map<ECameraRes, CameraRes> eCameraResData {
    { ECameraRes::RES_2560x1920_FPS_15, { 2560, 1920, 15 } },
    { ECameraRes::RES_1280x960_FPS_42, { 1280, 960, 42 } },
    { ECameraRes::RES_640x480_FPS_90, { 640, 480, 90 } },
};

/**************************************
 * MMAL Callbacks
 **************************************/
static void controlCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuffer)
{
    (void)pPort;
    mmal_buffer_header_release(pBuffer);
}

// MMAL Callback from camera preview output port.
void previewOutputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf)
{
    RPiCamera* pCamera = (RPiCamera*)pPort->userdata;

    if(pBuf->length == 0)
    {
        fprintf(stderr, "%s: zero-length buffer => EOS\n", pPort->name);
        mmal_buffer_header_release(pBuf);
    }
    else if(pBuf->data == NULL)
    {
        fprintf(stderr, "%s: zero buffer handle\n", pPort->name);
        mmal_buffer_header_release(pBuf);
    }
    else
    {
        pCamera->handlePreviewFrame(pBuf);
    }
}

// called when renderer is done with a buffer
void rendererInputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf)
{
    RPiCamera* pCamera = (RPiCamera*)pPort->userdata;
    pCamera->releaseFrame(pBuf);
}

void imageEncoderOutputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf)
{
    RPiCamera* pCamera = (RPiCamera*)pPort->userdata;
    pCamera->handleImageEncoderFrame(pBuf);
}

void videoEncoderOutputCallback(MMAL_PORT_T* pPort, MMAL_BUFFER_HEADER_T* pBuf)
{
    RPiCamera* pCamera = (RPiCamera*)pPort->userdata;
    pCamera->handleVideoEncoderFrame(pBuf);
}

/**************************************
 * Camera setup and configuration functions
 **************************************/
RPiCamera::RPiCamera()
{
    pCameraComponent = 0;
    pPreviewPool = 0;
    pPreviewPort = 0;
    pFrameQueue = 0;
    eResolution_ = ECameraRes::RES_1280x960_FPS_42;
    camResolution_.width = 1280;
    camResolution_.height = 960;
    framerate_ = 42;
    recording_ = false;
    previewFrameCallback_ = 0;
    pImageEncoderFile_ = 0;
    imagesTaken_ = 0;
    pVideoEncoderFile_ = 0;
    memset(videoFilename_, 0, sizeof(videoFilename_));
    recordSize_ = 0;

    pFrameQueue = mmal_queue_create();

    runThread = true;
    processingThread_ = std::thread(&RPiCamera::run, this);

    // fill the pCameraComponent
    setupCamera();
    setDefaultParameters();

    setupStillPort();
    configurePreviewPort();
    setupVideoPort();

    MMAL_STATUS_T status;

    // Pool + queue to hold preview frames
    pPreviewPool = mmal_port_pool_create(pPreviewPort, pPreviewPort->buffer_num, pPreviewPort->buffer_size);
    if(!pPreviewPool)
    {
        LogError("Error allocating preview pool\n");
        status = MMAL_ENOMEM;
        throw mmal_error(status);
    }

    setupImageEncoder();
    setupVideoEncoder();

    setupRenderer();
    configureRenderer();

    // Enable render port callback
    status = mmal_port_enable(pRendererPort, rendererInputCallback);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to enable renderer input port\n");
        throw mmal_error(status);
    }
}

RPiCamera::~RPiCamera()
{
    // shut down processing thread first
    runThread = false;
    if(processingThread_.joinable())
        processingThread_.join();

    // Renderer
    if(pRendererPort && pRendererPort->is_enabled)
        mmal_port_disable(pRendererPort);

    if(pRenderer)
    {
        mmal_component_disable(pRenderer);
        mmal_component_destroy(pRenderer);
    }

    // Camera - Preview
    if(pPreviewPort && pPreviewPort->is_enabled)
        mmal_port_disable(pPreviewPort);

    if(pPreviewPool)
        mmal_port_pool_destroy(pPreviewPort, pPreviewPool);

    // Camera => Image encoder connection
    if(pImageEncoderConnection_)
    {
        mmal_connection_disable(pImageEncoderConnection_);
        mmal_connection_destroy(pImageEncoderConnection_);
    }

    // Camera => Video encoder connection
    if(pVideoEncoderConnection_)
    {
        mmal_connection_disable(pVideoEncoderConnection_);
        mmal_connection_destroy(pVideoEncoderConnection_);
    }

    // Image encoder
    if(pImageEncoder_ && pImageEncoder_->output[0]->is_enabled)
        mmal_port_disable(pImageEncoder_->output[0]);

    if(pImageEncoderPool_)
        mmal_port_pool_destroy(pImageEncoder_->output[0], pImageEncoderPool_);

    if(pImageEncoder_)
    {
        mmal_component_disable(pImageEncoder_);
        mmal_component_destroy(pImageEncoder_);
    }

    // Video encoder
    if(pVideoEncoder_ && pVideoEncoder_->output[0]->is_enabled)
        mmal_port_disable(pVideoEncoder_->output[0]);

    if(pVideoEncoderPool_)
        mmal_port_pool_destroy(pVideoEncoder_->output[0], pVideoEncoderPool_);

    if(pVideoEncoder_)
    {
        mmal_component_disable(pVideoEncoder_);
        mmal_component_destroy(pVideoEncoder_);
    }

    // Disable all our ports that are not handled by connections
    if(pCameraComponent)
    {
        MMAL_PORT_T* pCameraVideoPort = pCameraComponent->output[MMAL_CAMERA_VIDEO_PORT];
        if(pCameraVideoPort && pCameraVideoPort->is_enabled)
            mmal_port_disable(pCameraVideoPort);

        mmal_component_disable(pCameraComponent);
        mmal_component_destroy(pCameraComponent);
    }

    // Camera preview => processing thread queue
    if(pFrameQueue)
        mmal_queue_destroy(pFrameQueue);
}

void RPiCamera::handlePreviewFrame(MMAL_BUFFER_HEADER_T* pBuf)
{
    if(mmal_queue_length(pFrameQueue) < 2)
    {
        // only enqueue if there is space available
        mmal_queue_put(pFrameQueue, pBuf);
    }
    else
    {
        // release directly
        releaseFrame(pBuf);
    }
}

void RPiCamera::releaseFrame(MMAL_BUFFER_HEADER_T* pBuf)
{
    // release buffer back to the pool
    mmal_buffer_header_release(pBuf);

    // and send one back to the port (if still open)
    if(pPreviewPool && pPreviewPort->is_enabled)
    {
        MMAL_BUFFER_HEADER_T* pNewBuf = mmal_queue_get(pPreviewPool->queue);

        if(pNewBuf)
        {
            MMAL_STATUS_T status = mmal_port_send_buffer(pPreviewPort, pNewBuf);
            if(status != MMAL_SUCCESS)
                fprintf(stderr, "Could not send buffer to port %d\n", status);
        }
        else
        {
            fprintf(stderr, "Could not get buffer from port queue\n");
        }
    }
}

void RPiCamera::handleImageEncoderFrame(MMAL_BUFFER_HEADER_T* pBuf)
{
    if(pBuf->length)
    {
        if(!pImageEncoderFile_)
        {
            time_t rawtime;
            struct tm* timeinfo;

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            strftime(imageFilename_, sizeof(imageFilename_), "img_%Y-%m-%d_%H-%M-%S.jpg", timeinfo);

            pImageEncoderFile_ = fopen(imageFilename_, "w+");
            if(!pImageEncoderFile_)
            {
                LogError("Cannot open JPG output file: %s\n", imageFilename_);
            }
            else
            {
                LogInfo("Saving still capture to: %s\n", imageFilename_);
            }
        }

        mmal_buffer_header_mem_lock(pBuf);

        int bytesWritten = fwrite(pBuf->data, 1, pBuf->length, pImageEncoderFile_);

        mmal_buffer_header_mem_unlock(pBuf);

        if(bytesWritten != (int)pBuf->length)
        {
            LogError("Write failed\n");
        }
    }

    if(pBuf->flags & MMAL_BUFFER_HEADER_FLAG_FRAME_END)
    {
        fclose(pImageEncoderFile_);
        pImageEncoderFile_ = 0;
        imagesTaken_++;
    }

    if(pBuf->flags & MMAL_BUFFER_HEADER_FLAG_TRANSMISSION_FAILED)
    {
        LogInfo("transmission failed flag set\n");
        fclose(pImageEncoderFile_);
        pImageEncoderFile_ = 0;
    }

    // release buffer back to the pool
    mmal_buffer_header_release(pBuf);

    // and send one back to the port (if still open)
    if(pImageEncoderPool_ && pImageEncoder_->output[0]->is_enabled)
    {
        MMAL_BUFFER_HEADER_T* pNewBuf = mmal_queue_get(pImageEncoderPool_->queue);

        if(pNewBuf)
        {
            MMAL_STATUS_T status = mmal_port_send_buffer(pImageEncoder_->output[0], pNewBuf);
            if(status != MMAL_SUCCESS)
                LogError("Could not send buffer to port %d\n", status);
        }
        else
        {
            LogError("Could not get buffer from port queue\n");
        }
    }
}

void RPiCamera::handleVideoEncoderFrame(MMAL_BUFFER_HEADER_T* pBuf)
{
//  LogInfo("Video frame data. len: %u, alloc: %u, flags: 0x%08X\n", pBuf->length, pBuf->alloc_size, pBuf->flags);

    if(pBuf->length && recording_)
    {
        if(!pVideoEncoderFile_)
        {
            time_t rawtime;
            struct tm* timeinfo;

            time(&rawtime);
            timeinfo = localtime(&rawtime);

            strftime(videoFilename_, sizeof(videoFilename_), "vid_%Y-%m-%d_%H-%M-%S.h264", timeinfo);

            pVideoEncoderFile_ = fopen(videoFilename_, "w+");
            if(!pVideoEncoderFile_)
            {
                LogError("Cannot open H264 output file: %s\n", videoFilename_);
            }
            else
            {
                LogInfo("Saving video capture to: %s\n", videoFilename_);
            }

            recordSize_ = 0;
            recordStartTime_ = vcos_getmicrosecs64();
        }

        mmal_buffer_header_mem_lock(pBuf);

        int bytesWritten = fwrite(pBuf->data, 1, pBuf->length, pVideoEncoderFile_);

        mmal_buffer_header_mem_unlock(pBuf);

        recordSize_ += bytesWritten;

        if(bytesWritten != (int)pBuf->length)
        {
            LogError("Write failed\n");
        }
    }

    if(pVideoEncoderFile_ && !recording_)
    {
        fclose(pVideoEncoderFile_);
        pVideoEncoderFile_ = 0;
    }

    // release buffer back to the pool
    mmal_buffer_header_release(pBuf);

    // and send one back to the port (if still open)
    if(pVideoEncoderPool_ && pVideoEncoder_->output[0]->is_enabled)
    {
        MMAL_BUFFER_HEADER_T* pNewBuf = mmal_queue_get(pVideoEncoderPool_->queue);

        if(pNewBuf)
        {
            MMAL_STATUS_T status = mmal_port_send_buffer(pVideoEncoder_->output[0], pNewBuf);
            if(status != MMAL_SUCCESS)
                LogError("Could not send buffer to port %d\n", status);
        }
        else
        {
            LogError("Could not get buffer from port queue\n");
        }
    }
}

float RPiCamera::getRecordDuration() const
{
    if(!recording_)
        return 0.0f;

    uint64_t now = vcos_getmicrosecs64();

    return (now - recordStartTime_) * 1e-6f;
}

int64_t RPiCamera::getStcTimestampUs()
{
    MMAL_PARAMETER_UINT64_T time = { { MMAL_PARAMETER_SYSTEM_TIME, sizeof(time) }, 0 };

    if(mmal_port_parameter_get(pPreviewPort, &time.hdr) == MMAL_SUCCESS)
        return (int64_t)time.value;

    return 0;
}

void RPiCamera::triggerCapture()
{
    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(pStillPort, MMAL_PARAMETER_CAPTURE, 1);
    if(status != MMAL_SUCCESS)
        LogError("Could not trigger still capture %d\n", status);
}

void RPiCamera::setRecording(bool enable)
{
    if(enable == recording_)
        return;

    if(enable)
        start(ECameraRes::RES_1280x960_FPS_42);

    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(pVideoPort, MMAL_PARAMETER_CAPTURE, enable ? 1 : 0);
    if(status != MMAL_SUCCESS)
        LogError("Could not set video recording state %d\n", status);

    recording_ = enable;
}

void RPiCamera::setupCamera()
{
    // Create the camera component
    MMAL_STATUS_T status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &pCameraComponent);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "Failed to create camera component\n");
        throw mmal_error(status);
    }

    // select camera
    MMAL_PARAMETER_INT32_T cameraNum = { { MMAL_PARAMETER_CAMERA_NUM, sizeof(cameraNum) }, 0 };
    status = mmal_port_parameter_set(pCameraComponent->control, &cameraNum.hdr);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "Could not select camera : error %d\n", status);
        throw mmal_error(status);
    }
    if(!pCameraComponent->output_num)
    {
        status = MMAL_ENOSYS;
        fprintf(stderr, "Camera doesn't have output ports\n");
        throw mmal_error(status);
    }

    // Enable the camera, and tell it its control callback function
    status = mmal_port_enable(pCameraComponent->control, controlCallback);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "Unable to enable control port : error %d\n", status);
        throw mmal_error(status);
    }

    //  set up the camera configuration
    MMAL_PARAMETER_CAMERA_CONFIG_T camConfig = {
        { MMAL_PARAMETER_CAMERA_CONFIG, sizeof(camConfig) },
        .max_stills_w = VCOS_ALIGN_UP(2592, 32),
        .max_stills_h = VCOS_ALIGN_UP(1944, 16),
        .stills_yuv422 = 0,
        .one_shot_stills = 1,
        .max_preview_video_w = VCOS_ALIGN_UP(2592, 32),
        .max_preview_video_h = VCOS_ALIGN_UP(1944, 16),
        .num_preview_video_frames = VIDEO_OUTPUT_BUFFERS_NUM,
        .stills_capture_circular_buffer_height = 0,
        .fast_preview_resume = 0,
        .use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RAW_STC
    };

    status = mmal_port_parameter_set(pCameraComponent->control, &camConfig.hdr);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "Unable to set camera config %d\n", status);
        throw mmal_error(status);
    }

    // enable camera component
    status = mmal_component_enable(pCameraComponent);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "camera component couldn't be enabled\n");
        throw mmal_error(status);
    }

    pVideoPort = pCameraComponent->output[MMAL_CAMERA_VIDEO_PORT];

    pPreviewPort = pCameraComponent->output[MMAL_CAMERA_PREVIEW_PORT];
    pPreviewPort->buffer_num = VIDEO_OUTPUT_BUFFERS_NUM;
    pPreviewPort->buffer_size = (2592 * 1952 * 3) / 2; // maximum possible size at full resolution and YUV420 encoding
    pPreviewPort->userdata = (struct MMAL_PORT_USERDATA_T*)this;
}

void RPiCamera::setupStillPort()
{
    MMAL_STATUS_T status;

    pStillPort = pCameraComponent->output[MMAL_CAMERA_CAPTURE_PORT];
    MMAL_ES_FORMAT_T* pESFormat = pStillPort->format;

    pESFormat->encoding = MMAL_ENCODING_OPAQUE;
    pESFormat->encoding_variant = MMAL_ENCODING_VARIANT_DEFAULT;
    pESFormat->es->video.width = VCOS_ALIGN_UP(2592, 32);
    pESFormat->es->video.height = VCOS_ALIGN_UP(1944, 16);
    pESFormat->es->video.crop.x = 0;
    pESFormat->es->video.crop.y = 0;
    pESFormat->es->video.crop.width = 2592;
    pESFormat->es->video.crop.height = 1944;
    pESFormat->es->video.frame_rate.num = 0;
    pESFormat->es->video.frame_rate.den = 1;

    status = mmal_port_format_commit(pStillPort);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "camera still format couldn't be set\n");
        throw mmal_error(status);
    }

    // Ensure there are enough buffers to avoid dropping frames
    pStillPort->buffer_num = pStillPort->buffer_num_recommended;
    pStillPort->buffer_size = pStillPort->buffer_size_recommended;

    pStillPort->userdata = (struct MMAL_PORT_USERDATA_T*)this;
}

void RPiCamera::setupVideoPort()
{
    MMAL_ES_FORMAT_T* pESFormat = pVideoPort->format;

    pESFormat->encoding = MMAL_ENCODING_OPAQUE;
    pESFormat->encoding_variant = MMAL_ENCODING_VARIANT_DEFAULT;

    pESFormat->es->video.width = VCOS_ALIGN_UP(camResolution_.width, 32);
    pESFormat->es->video.height = VCOS_ALIGN_UP(camResolution_.height, 16);
    pESFormat->es->video.crop.x = 0;
    pESFormat->es->video.crop.y = 0;
    pESFormat->es->video.crop.width = camResolution_.width;
    pESFormat->es->video.crop.height = camResolution_.height;
    pESFormat->es->video.frame_rate.num = framerate_;
    pESFormat->es->video.frame_rate.den = 1;

    MMAL_STATUS_T status = mmal_port_format_commit(pVideoPort);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "camera video port format couldn't be set\n");
        throw mmal_error(status);
    }

    pVideoPort->buffer_num = pVideoPort->buffer_num_recommended;
    pVideoPort->buffer_size = pVideoPort->buffer_size_recommended;
}

void RPiCamera::configurePreviewPort()
{
    MMAL_ES_FORMAT_T* pESFormat = pPreviewPort->format;

    pESFormat->encoding = MMAL_ENCODING_I420;
    pESFormat->encoding_variant = MMAL_ENCODING_VARIANT_DEFAULT;

    pESFormat->es->video.width = VCOS_ALIGN_UP(camResolution_.width, 32);
    pESFormat->es->video.height = VCOS_ALIGN_UP(camResolution_.height, 16);
    pESFormat->es->video.crop.x = 0;
    pESFormat->es->video.crop.y = 0;
    pESFormat->es->video.crop.width = camResolution_.width;
    pESFormat->es->video.crop.height = camResolution_.height;
    pESFormat->es->video.frame_rate.num = framerate_;
    pESFormat->es->video.frame_rate.den = 1;

    MMAL_STATUS_T status = mmal_port_parameter_set_boolean(pPreviewPort, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "Failed to enable zero copy on camera preview port\n");
        throw mmal_error(status);
    }

    LogInfo("Preview port caps: 0x%08X\n", pPreviewPort->capabilities);

    status = mmal_port_format_commit(pPreviewPort);
    if(status != MMAL_SUCCESS)
    {
        fprintf(stderr, "camera preview format couldn't be set\n");
        throw mmal_error(status);
    }
}

void RPiCamera::setupRenderer()
{
    // create render component
    MMAL_STATUS_T status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_RENDERER, &pRenderer);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to create camera render component\n");
        throw mmal_error(status);
    }
    if(!pRenderer->input_num)
    {
        status = MMAL_ENOSYS;
        LogError("No input ports found on component");
        throw mmal_error(status);
    }

    // enable renderer
    status = mmal_component_enable(pRenderer);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to enable preview/null sink component (%u)", status);
        throw mmal_error(status);
    }

    pRendererPort = pRenderer->input[0];
    MMAL_DISPLAYREGION_T param;
    param.hdr.id = MMAL_PARAMETER_DISPLAYREGION;
    param.hdr.size = sizeof(MMAL_DISPLAYREGION_T);
    param.set = MMAL_DISPLAY_SET_LAYER;
    param.layer = 2;
    param.set |= MMAL_DISPLAY_SET_ALPHA;
    param.alpha = 255;
    param.set |= MMAL_DISPLAY_SET_DEST_RECT;
    param.fullscreen = 0;
    param.dest_rect.x = 0;
    param.dest_rect.y = 0;
    param.dest_rect.width = 1280;
    param.dest_rect.height = 960;

    status = mmal_port_parameter_set(pRendererPort, &param.hdr);
    if(status != MMAL_SUCCESS && status != MMAL_ENOSYS)
    {
        LogError("unable to set renderer port parameters (%u)", status);
        throw mmal_error(status);
    }

    status = mmal_port_parameter_set_boolean(pRendererPort, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to enable zero copy on renderer port\n");
        throw mmal_error(status);
    }

    pRendererPort->buffer_num = pPreviewPort->buffer_num;
    pRendererPort->buffer_size = pPreviewPort->buffer_size;
    pRendererPort->userdata = (struct MMAL_PORT_USERDATA_T*)this;

    LogDebug("Render buffers: %d x %dB\n", pRendererPort->buffer_num, pRendererPort->buffer_size);

    LogInfo("Render port caps: 0x%08X\n", pRendererPort->capabilities);
}

void RPiCamera::configureRenderer()
{
    // Set the format of the input port to match the output one
    mmal_format_copy(pRendererPort->format, pPreviewPort->format);

    MMAL_STATUS_T status = mmal_port_format_commit(pRendererPort);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to commit format\n");
        throw mmal_error(status);
    }
}

void RPiCamera::setupImageEncoder()
{
    MMAL_STATUS_T status;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_IMAGE_ENCODER, &pImageEncoder_);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to create image encoder component\n");
        throw mmal_error(status);
    }

    MMAL_PORT_T* pEncoderInPort = pImageEncoder_->input[0];
    MMAL_PORT_T* pEncoderOutPort = pImageEncoder_->output[0];

    pEncoderOutPort->userdata = (struct MMAL_PORT_USERDATA_T*)this;

    // We want same format on input and output
    mmal_format_copy(pEncoderOutPort->format, pEncoderInPort->format);

    // Specify out output format
    pEncoderOutPort->format->encoding = MMAL_ENCODING_JPEG;

    pEncoderOutPort->buffer_size = pEncoderOutPort->buffer_size_recommended;
    if(pEncoderOutPort->buffer_size < pEncoderOutPort->buffer_size_min)
        pEncoderOutPort->buffer_size = pEncoderOutPort->buffer_size_min;

    pEncoderOutPort->buffer_num = pEncoderOutPort->buffer_num_recommended;
    if(pEncoderOutPort->buffer_num < pEncoderOutPort->buffer_num_min)
        pEncoderOutPort->buffer_num = pEncoderOutPort->buffer_num_min;

    // Commit the port changes to the output port
    status = mmal_port_format_commit(pEncoderOutPort);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set format on video encoder output port");
        throw mmal_error(status);
    }

    // Set the JPEG quality level
    status = mmal_port_parameter_set_uint32(pEncoderOutPort, MMAL_PARAMETER_JPEG_Q_FACTOR, 95);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set JPEG quality");
        throw mmal_error(status);
    }

    // Set the JPEG restart interval
    status = mmal_port_parameter_set_uint32(pEncoderOutPort, MMAL_PARAMETER_JPEG_RESTART_INTERVAL, 0);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set JPEG restart interval");
        throw mmal_error(status);
    }

    // Set up any required thumbnail
    MMAL_PARAMETER_THUMBNAIL_CONFIG_T param_thumb = { { MMAL_PARAMETER_THUMBNAIL_CONFIGURATION, sizeof(MMAL_PARAMETER_THUMBNAIL_CONFIG_T) },
                    0, 0, 0, 0 };
    status = mmal_port_parameter_set(pEncoderOutPort, &param_thumb.hdr);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set JPEG thumbnail config");
        throw mmal_error(status);
    }

    //  Enable component
    status = mmal_component_enable(pImageEncoder_);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to enable image encoder component");
        throw mmal_error(status);
    }

    /* Create pool of buffer headers for the output port to consume */
    pImageEncoderPool_ = mmal_port_pool_create(pEncoderOutPort, pEncoderOutPort->buffer_num, pEncoderOutPort->buffer_size);
    if(!pImageEncoderPool_)
    {
        LogError("Failed to create buffer header pool for encoder output port %s", pEncoderOutPort->name);
        throw mmal_error(0);
    }

    status = mmal_connection_create(&pImageEncoderConnection_, pStillPort, pEncoderInPort,
    MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to create encoder connection\n");
        throw mmal_error(status);
    }

    status = mmal_connection_enable(pImageEncoderConnection_);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to enable encoder connection\n");
        throw mmal_error(status);
    }

    // Enable the encoder output port and tell it its callback function
    status = mmal_port_enable(pEncoderOutPort, &imageEncoderOutputCallback);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to enable encoder out port\n");
        throw mmal_error(status);
    }

    // fill encoder output port with buffers
    uint32_t i = 0;
    while(mmal_queue_length(pImageEncoderPool_->queue) > 0)
    {
        MMAL_BUFFER_HEADER_T* pBuf = mmal_queue_get(pImageEncoderPool_->queue);
        if(!pBuf)
            LogError("Unable to get a required buffer %d from pool queue\n", i);

        status = mmal_port_send_buffer(pEncoderOutPort, pBuf);
        if(status != MMAL_SUCCESS)
        {
            LogError("Error sending buffer to port %d\n", status);
            throw mmal_error(status);
        }

        i++;
    }
}

void RPiCamera::setupVideoEncoder()
{
    MMAL_STATUS_T status;

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_ENCODER, &pVideoEncoder_);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to create video encoder component\n");
        throw mmal_error(status);
    }

    MMAL_PORT_T* pEncoderInPort = pVideoEncoder_->input[0];
    MMAL_PORT_T* pEncoderOutPort = pVideoEncoder_->output[0];

    pEncoderOutPort->userdata = (struct MMAL_PORT_USERDATA_T*)this;

    // We want same format on input and output
    mmal_format_copy(pEncoderOutPort->format, pEncoderInPort->format);

    // Specify out output format
    pEncoderOutPort->format->encoding = MMAL_ENCODING_H264;
    pEncoderOutPort->format->bitrate = 50000000; // 50MBit/s
    pEncoderOutPort->format->es->video.frame_rate.den = 0;
    pEncoderOutPort->format->es->video.frame_rate.den = 1;

    pEncoderOutPort->buffer_size = pEncoderOutPort->buffer_size_recommended;
    if(pEncoderOutPort->buffer_size < pEncoderOutPort->buffer_size_min)
        pEncoderOutPort->buffer_size = pEncoderOutPort->buffer_size_min;

    pEncoderOutPort->buffer_num = pEncoderOutPort->buffer_num_recommended;
    if(pEncoderOutPort->buffer_num < pEncoderOutPort->buffer_num_min)
        pEncoderOutPort->buffer_num = pEncoderOutPort->buffer_num_min;

    // Commit the port changes to the output port
    status = mmal_port_format_commit(pEncoderOutPort);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set format on video encoder output port");
        throw mmal_error(status);
    }

    MMAL_PARAMETER_VIDEO_PROFILE_T videoProfile;
    videoProfile.hdr.id = MMAL_PARAMETER_PROFILE;
    videoProfile.hdr.size = sizeof(videoProfile);
    videoProfile.profile[0].profile = MMAL_VIDEO_PROFILE_H264_HIGH;
    videoProfile.profile[0].level = MMAL_VIDEO_LEVEL_H264_42;

    status = mmal_port_parameter_set(pEncoderOutPort, &videoProfile.hdr);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set H264 profile");
        throw mmal_error(status);
    }

    status = mmal_port_parameter_set_boolean(pEncoderInPort, MMAL_PARAMETER_VIDEO_IMMUTABLE_INPUT, 1);
    if(status != MMAL_SUCCESS)
    {
        LogWarn("Unable to set immutable input flag");
        // not a critical error
    }

    status = mmal_port_parameter_set_boolean(pEncoderOutPort, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_HEADER, 1);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set inline header flag");
        throw mmal_error(status);
    }

    status = mmal_port_parameter_set_boolean(pEncoderOutPort, MMAL_PARAMETER_VIDEO_ENCODE_SPS_TIMING, 1);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set SPS timing flag");
        throw mmal_error(status);
    }

    status = mmal_port_parameter_set_boolean(pEncoderOutPort, MMAL_PARAMETER_VIDEO_ENCODE_INLINE_VECTORS, 0);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set inline vectors flag");
        throw mmal_error(status);
    }

    //  Enable component
    status = mmal_component_enable(pVideoEncoder_);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to enable video encoder component");
        throw mmal_error(status);
    }

    LogDebug("Video encoder out buffers: %d x %dB\n", pEncoderOutPort->buffer_num, pEncoderOutPort->buffer_size);

    /* Create pool of buffer headers for the output port to consume */
    pVideoEncoderPool_ = mmal_port_pool_create(pEncoderOutPort, pEncoderOutPort->buffer_num, pEncoderOutPort->buffer_size);
    if(!pVideoEncoderPool_)
    {
        LogError("Failed to create buffer header pool for video encoder output port %s", pEncoderOutPort->name);
        throw mmal_error(0);
    }

    status = mmal_connection_create(&pVideoEncoderConnection_, pVideoPort, pEncoderInPort,
    MMAL_CONNECTION_FLAG_TUNNELLING | MMAL_CONNECTION_FLAG_ALLOCATION_ON_INPUT);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to create video encoder connection\n");
        throw mmal_error(status);
    }

    status = mmal_connection_enable(pVideoEncoderConnection_);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to enable video encoder connection\n");
        throw mmal_error(status);
    }

    // Enable the encoder output port and tell it its callback function
    status = mmal_port_enable(pEncoderOutPort, &videoEncoderOutputCallback);
    if(status != MMAL_SUCCESS)
    {
        LogError("Failed to enable video encoder out port\n");
        throw mmal_error(status);
    }

    // fill encoder output port with buffers
    uint32_t i = 0;
    while(mmal_queue_length(pVideoEncoderPool_->queue) > 0)
    {
        MMAL_BUFFER_HEADER_T* pBuf = mmal_queue_get(pVideoEncoderPool_->queue);
        if(!pBuf)
            LogError("Unable to get a required buffer %d from pool queue\n", i);

        status = mmal_port_send_buffer(pEncoderOutPort, pBuf);
        if(status != MMAL_SUCCESS)
        {
            LogError("Error sending buffer to port %d\n", status);
            throw mmal_error(status);
        }

        i++;
    }

    setEncoderQuantisation(20);
}

void RPiCamera::start(ECameraRes res)
{
    MMAL_STATUS_T status;

    if(pPreviewPort && pPreviewPort->is_enabled && recording_)
        return;

    if(pPreviewPort && pPreviewPort->is_enabled && eResolution_ == res)
        return;

    const CameraRes resData = eCameraResData.find(res)->second;

    camResolution_.width = resData.width;
    camResolution_.height = resData.height;
    framerate_ = resData.fps;

    eResolution_ = res;

    try
    {
        if(pPreviewPort && pPreviewPort->is_enabled)
            mmal_port_disable(pPreviewPort);

        if(pVideoEncoderConnection_ && pVideoEncoderConnection_->is_enabled)
            mmal_connection_disable(pVideoEncoderConnection_);

        // configure super-important preview port - the only one we really use
        configurePreviewPort();

        // configure video port (only used by H264 recording)
        setupVideoPort();

        // Enable preview port callback
        status = mmal_port_enable(pPreviewPort, previewOutputCallback);
        if(status != MMAL_SUCCESS)
        {
            fprintf(stderr, "Failed to enable camera preview port\n");
            throw mmal_error(status);
        }

        mmal_connection_enable(pVideoEncoderConnection_);

        configureRenderer();

        // fill preview port with buffers
        uint32_t i = 0;
        while(mmal_queue_length(pPreviewPool->queue) > 0)
        {
            MMAL_BUFFER_HEADER_T* pBuf = mmal_queue_get(pPreviewPool->queue);
            if(!pBuf)
                fprintf(stderr, "Unable to get a required buffer %d from pool queue\n", i);

            status = mmal_port_send_buffer(pPreviewPort, pBuf);
            if(status != MMAL_SUCCESS)
            {
                fprintf(stderr, "Error sending buffer to port %d\n", status);
                throw mmal_error(status);
            }

            i++;
        }
    }
    catch(const mmal_error& err)
    {
        if(pCameraComponent)
            mmal_component_destroy(pCameraComponent);

        throw;
    }
}

void RPiCamera::stop()
{
    setRecording(false);

    if(pPreviewPort && pPreviewPort->is_enabled)
        mmal_port_disable(pPreviewPort);

    if(pVideoEncoderConnection_ && pVideoEncoderConnection_->is_enabled)
        mmal_connection_disable(pVideoEncoderConnection_);
}

void RPiCamera::run()
{
    LogInfo("Starting render thread\n");

    while(runThread)
    {
        MMAL_BUFFER_HEADER_T* pBuf = mmal_queue_timedwait(pFrameQueue, 50);
        if(pBuf)
        {
            mmal_buffer_header_mem_lock(pBuf);

            if(pBuf->type)
            {
                uint8_t* pYData = pBuf->data + pBuf->type->video.offset[0];
                uint8_t* pUData = pBuf->data + pBuf->type->video.offset[1];
                uint8_t* pVData = pBuf->data + pBuf->type->video.offset[2];

                uint32_t width = pBuf->type->video.pitch[0];
                uint32_t height = pBuf->type->video.offset[1] / width;

                FrameMetadata meta;
                meta.timestampUs = pBuf->pts;
                meta.width = width;
                meta.height = height;

                MMAL_PARAMETER_CAMERA_SETTINGS_T set = { { MMAL_PARAMETER_CAMERA_SETTINGS, sizeof(set) }, 0 };
                if(mmal_port_parameter_get(pCameraComponent->control, &set.hdr) == MMAL_SUCCESS)
                {
                    meta.exposureUs = set.exposure;
                }

                FrameYUV420 frame(pYData, pUData, pVData, meta);

                if(previewFrameCallback_)
                {
                    previewFrameCallback_(&frame);
                }
            }

            mmal_buffer_header_mem_unlock(pBuf);

            MMAL_STATUS_T status = mmal_port_send_buffer(pRendererPort, pBuf);
            if(status != MMAL_SUCCESS)
            {
                LogError("Send error: %d\n", status);
            }
        }
    }
}

/**************************************
 * SETTER
 **************************************/
void RPiCamera::setDefaultParameters()
{
//  setIso(CAMERA_ISO_800);
//  setExposureMode(MMAL_PARAM_EXPOSUREMODE_OFF);
//  setAwbMode(MMAL_PARAM_AWBMODE_OFF);

    setExposureMode(MMAL_PARAM_EXPOSUREMODE_SPORTS);
    setAwbMode(MMAL_PARAM_AWBMODE_AUTO);

//  setShutterSpeed(10000);
//  setAwbGains(1.4f, 1.5f);
//  setAnalogGain(8.0f);
//  setDigitalGain(2.0f);
//  setExposureCompensation(0);
//  setAwbGains(1.8f, 1.0f);
//  setSharpness(0);
//  setContrast(0);
//  setBrightness(50);
//  setSaturation(0);
    setMirror(CAMERA_MIRROR_BOTH);

    setAlgorithmControl(MMAL_PARAMETER_ALGORITHM_CONTROL_ALGORITHMS_VIDEO_DENOISE, true);
    setUseCase(MMAL_PARAM_CAMERA_USE_CASE_VIDEO_CAPTURE);
    setZeroShutterLag(false);
}

void RPiCamera::setExposureSettings(ExposureSettings settings)
{
    if(exposureSettings_ == settings)
        return;

    if(settings.autoMode)
    {
        setExposureMode(MMAL_PARAM_EXPOSUREMODE_SPORTS);
        setAnalogGain(0.0f);
        setDigitalGain(0.0f);
        setShutterSpeed(0);
    }
    else
    {
        setExposureMode(MMAL_PARAM_EXPOSUREMODE_OFF);
        setAnalogGain(settings.analogGain);
        setDigitalGain(settings.digitalGain);
        setShutterSpeed(settings.exposureTimeUs);
    }

    LogInfo("Exposure updated. Auto: %d, analog: %f, digital: %f, exp: %u\n",
                    (int32_t)settings.autoMode, settings.analogGain, settings.digitalGain, settings.exposureTimeUs);

    exposureSettings_ = settings;
}

void RPiCamera::setWhiteBalanceSettings(WhiteBalanceSettings settings)
{
    if(whiteBalanceSettings_ == settings)
        return;

    if(settings.autoMode)
    {
        setAwbMode(MMAL_PARAM_AWBMODE_AUTO);
    }
    else
    {
        setAwbMode(MMAL_PARAM_AWBMODE_OFF);
        setAwbGains(settings.redGain, settings.blueGain);
    }

    LogInfo("WB updated. Auto: %d, red: %f, blue: %f\n", (int32_t)settings.autoMode, settings.redGain, settings.blueGain);

    whiteBalanceSettings_ = settings;
}

void RPiCamera::setEncoderQuantisation(uint32_t quantisation)
{
    MMAL_STATUS_T status;

    if(!pVideoEncoder_)
        return;

    MMAL_PORT_T* pEncoderOutPort = pVideoEncoder_->output[0];

    MMAL_PARAMETER_UINT32_T param = { { MMAL_PARAMETER_VIDEO_ENCODE_INITIAL_QUANT, sizeof(param) }, quantisation };
    status = mmal_port_parameter_set(pEncoderOutPort, &param.hdr);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set initial QP");
        throw mmal_error(status);
    }

    MMAL_PARAMETER_UINT32_T param2 = { { MMAL_PARAMETER_VIDEO_ENCODE_MIN_QUANT, sizeof(param) }, quantisation };
    status = mmal_port_parameter_set(pEncoderOutPort, &param2.hdr);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set min QP");
        throw mmal_error(status);
    }

    MMAL_PARAMETER_UINT32_T param3 = { { MMAL_PARAMETER_VIDEO_ENCODE_MAX_QUANT, sizeof(param) }, quantisation };
    status = mmal_port_parameter_set(pEncoderOutPort, &param3.hdr);
    if(status != MMAL_SUCCESS)
    {
        LogError("Unable to set max QP");
        throw mmal_error(status);
    }
}

// -100 - 100
void RPiCamera::setSaturation(int saturation)
{
    if(!pCameraComponent || saturation < -100 || saturation > 100)
        return;

    MMAL_RATIONAL_T value = { saturation, 100 };
    mmal_port_parameter_set_rational(pCameraComponent->control, MMAL_PARAMETER_SATURATION, value);
}

// -100 - 100
void RPiCamera::setSharpness(int sharpness)
{
    if(!pCameraComponent || sharpness < -100 || sharpness > 100)
        return;

    MMAL_RATIONAL_T value = { sharpness, 100 };
    mmal_port_parameter_set_rational(pCameraComponent->control, MMAL_PARAMETER_SHARPNESS, value);
}

// -100 - 100
void RPiCamera::setContrast(int contrast)
{
    if(!pCameraComponent || contrast < -100 || contrast > 100)
        return;

    MMAL_RATIONAL_T value = { contrast, 100 };
    mmal_port_parameter_set_rational(pCameraComponent->control, MMAL_PARAMETER_CONTRAST, value);
}

// 0 - 100
void RPiCamera::setBrightness(int brightness)
{
    if(!pCameraComponent || brightness < 0 || brightness > 100)
        return;

    MMAL_RATIONAL_T value = { brightness, 100 };
    mmal_port_parameter_set_rational(pCameraComponent->control, MMAL_PARAMETER_BRIGHTNESS, value);
}

// 100, 200, 400, 800
void RPiCamera::setIso(CameraIso iso)
{
    if(!pCameraComponent)
        return;

    uint32_t val;

    switch(iso)
    {
        case CAMERA_ISO_200:
            val = 200;
            break;
        case CAMERA_ISO_400:
            val = 400;
            break;
        case CAMERA_ISO_800:
            val = 800;
            break;
        default:
            val = 100;
            break;
    }

    mmal_port_parameter_set_uint32(pCameraComponent->control, MMAL_PARAMETER_ISO, val);
}

void RPiCamera::setMeteringMode(MMAL_PARAM_EXPOSUREMETERINGMODE_T mode)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_EXPOSUREMETERINGMODE_T meter_mode = { { MMAL_PARAMETER_EXP_METERING_MODE, sizeof(meter_mode) }, mode };
    mmal_port_parameter_set(pCameraComponent->control, &meter_mode.hdr);
}

// -10 - 10
void RPiCamera::setExposureCompensation(int expComp)
{
    if(!pCameraComponent)
        return;

    mmal_port_parameter_set_int32(pCameraComponent->control, MMAL_PARAMETER_EXPOSURE_COMP, expComp);
}

void RPiCamera::setExposureMode(MMAL_PARAM_EXPOSUREMODE_T mode)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_EXPOSUREMODE_T exp_mode = { { MMAL_PARAMETER_EXPOSURE_MODE, sizeof(exp_mode) }, mode };
    mmal_port_parameter_set(pCameraComponent->control, &exp_mode.hdr);
}

void RPiCamera::setAwbMode(MMAL_PARAM_AWBMODE_T awb_mode)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_AWBMODE_T param = { { MMAL_PARAMETER_AWB_MODE, sizeof(param) }, awb_mode };
    mmal_port_parameter_set(pCameraComponent->control, &param.hdr);
}

void RPiCamera::setAwbGains(float r_gain, float b_gain)
{
    if(!pCameraComponent || r_gain < 0.0f || b_gain < 0.0f)
        return;

    MMAL_PARAMETER_AWB_GAINS_T param = { { MMAL_PARAMETER_CUSTOM_AWB_GAINS, sizeof(param) }, { 0, 0 }, { 0, 0 } };

    param.r_gain.num = (unsigned int)(r_gain * 65536);
    param.b_gain.num = (unsigned int)(b_gain * 65536);
    param.r_gain.den = param.b_gain.den = 65536;
    mmal_port_parameter_set(pCameraComponent->control, &param.hdr);
}

// 0, 90, 180, 270
void RPiCamera::setRotation(CameraRotation rotation)
{
    if(!pCameraComponent)
        return;

    int32_t val;
    switch(rotation)
    {
        case CAMERA_ROTATE_90:
            val = 90;
            break;
        case CAMERA_ROTATE_180:
            val = 180;
            break;
        case CAMERA_ROTATE_270:
            val = 270;
            break;
        default:
            val = 0;
            break;
    }

    mmal_port_parameter_set_int32(pCameraComponent->output[0], MMAL_PARAMETER_ROTATION, val);
    mmal_port_parameter_set_int32(pCameraComponent->output[1], MMAL_PARAMETER_ROTATION, val);
    mmal_port_parameter_set_int32(pCameraComponent->output[2], MMAL_PARAMETER_ROTATION, val);
}

// H, V, both
void RPiCamera::setMirror(CameraMirror flip)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_MIRROR_T mirror = { { MMAL_PARAMETER_MIRROR, sizeof(MMAL_PARAMETER_MIRROR_T) }, MMAL_PARAM_MIRROR_NONE };

    switch(flip)
    {
        case CAMERA_MIRROR_HORIZONTAL:
            mirror.value = MMAL_PARAM_MIRROR_HORIZONTAL;
            break;
        case CAMERA_MIRROR_VERTICAL:
            mirror.value = MMAL_PARAM_MIRROR_VERTICAL;
            break;
        case CAMERA_MIRROR_BOTH:
            mirror.value = MMAL_PARAM_MIRROR_BOTH;
            break;
        default:
            mirror.value = MMAL_PARAM_MIRROR_NONE;
            break;
    }

    mmal_port_parameter_set(pCameraComponent->output[0], &mirror.hdr);
    mmal_port_parameter_set(pCameraComponent->output[1], &mirror.hdr);
    mmal_port_parameter_set(pCameraComponent->output[2], &mirror.hdr);
}

void RPiCamera::setShutterSpeed(int speed_us)
{
    if(!pCameraComponent)
        return;

    mmal_port_parameter_set_uint32(pCameraComponent->control, MMAL_PARAMETER_SHUTTER_SPEED, speed_us);
}

void RPiCamera::setAlgorithmControl(MMAL_PARAMETER_ALGORITHM_CONTROL_ALGORITHMS_T algo, bool enable)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_ALGORITHM_CONTROL_T ctrlConfig = {
        .hdr = { MMAL_PARAMETER_ALGORITHM_CONTROL, sizeof(ctrlConfig) },
        .algorithm = algo,
        .enabled = enable ? 1 : 0
    };

    mmal_port_parameter_set(pCameraComponent->control, &ctrlConfig.hdr);
}

void RPiCamera::setUseCase(MMAL_PARAM_CAMERA_USE_CASE_T useCase)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_CAMERA_USE_CASE_T useCaseCfg = { { MMAL_PARAMETER_CAMERA_USE_CASE, sizeof(useCaseCfg) }, useCase };
    mmal_port_parameter_set(pCameraComponent->control, &useCaseCfg.hdr);
}

void RPiCamera::setZeroShutterLag(bool enable)
{
    if(!pCameraComponent)
        return;

    MMAL_PARAMETER_ZEROSHUTTERLAG_T zeroLagCfg = {
        .hdr = { MMAL_PARAMETER_ZERO_SHUTTER_LAG, sizeof(zeroLagCfg) },
        .zero_shutter_lag_mode = enable ? 1 : 0,
        .concurrent_capture = 0
    };

    mmal_port_parameter_set(pCameraComponent->control, &zeroLagCfg.hdr);
}

void RPiCamera::setAnalogGain(float analog)
{
    if(!pCameraComponent)
        return;

    MMAL_RATIONAL_T rational = { 0, 65536 };
    rational.num = (unsigned int)(analog * 65536);
    mmal_port_parameter_set_rational(pCameraComponent->control, MMAL_PARAMETER_ANALOG_GAIN, rational);
}

void RPiCamera::setDigitalGain(float digital)
{
    if(!pCameraComponent)
        return;

    MMAL_RATIONAL_T rational = { 0, 65536 };
    rational.num = (unsigned int)(digital * 65536);
    mmal_port_parameter_set_rational(pCameraComponent->control, MMAL_PARAMETER_DIGITAL_GAIN, rational);
}

}
