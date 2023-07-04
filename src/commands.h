/*
 * commands.h
 *
 *  Created on: 31.10.2020
 *      Author: AndreR
 */

#pragma once

#include <stdint.h>

#pragma pack(push,1) // <== make sure all declarations are within this and "#pragma pack(pop)" below !!!

// this is copied from commands.h of TIGERs Firmware
#define CMD_EXT_BALL_DETECTIONS         0x0E00
#define CMD_EXT_SHUTDOWN                0x0E01
#define CMD_EXT_ROBOT_PI_VERSION        0x0E02
#define CMD_EXT_CAMERA_CONFIG           0x0E04
#define CMD_EXT_CAMERA_PREVIEW_LINE_160 0x0E05
#define CMD_EXT_CAMERA_CONTROL          0x0E07
#define CMD_EXT_CAMERA_TRIGGER_CAPTURE  0x0E08
#define CMD_EXT_CAMERA_STATS            0x0E09
#define CMD_EXT_CAMERA_CALIBRATION      0x0E0A
#define CMD_EXT_REMOTE_TIME             0x0E0C
#define CMD_EXT_ROBOT_STATE             0x0E0D
#define CMD_EXT_BALL_LOC_CONFIG         0x0E0E
#define CMD_EXT_COLOR_THRESHOLDS        0x0E10
#define CMD_EXT_STEP_CONFIG             0x0E15
#define CMD_EXT_POINT_DIST_SENSOR       0x0E16
#define CMD_EXT_POINT_DIST_SENSOR_CFG   0x0E17
#define CMD_EXT_UPDATE_PROGRESS         0x0E18

#define EXT_BALL_DETECTIONS_MAX_BALLS 10
typedef struct _ExtBallDetections
{
    uint32_t timestampUs;
    float robotPos[3]; // X, Y, W at given timestamp

    struct _balls
    {
        // 3D position in global frame (map frame)
        // sorted by: distance to robot, unit: [m]
        float pos[3];
        float vel[3];
        float linePos[2];
        float lineDir[2];
        uint32_t trackerId;
    } balls[EXT_BALL_DETECTIONS_MAX_BALLS];

    uint8_t numBalls;
} ExtBallDetections;

#define EXT_SHUTDOWN_KEY 0xA5B6C7D8
typedef struct _ExtShutdown
{
    uint32_t key; // must be equal to EXT_SHUTDOWN_KEY in order to initiate shutdown of RPi
} ExtShutdown;

typedef struct _ExtRobotPiVersion
{
    uint32_t version;   // 4 [major.minor.patch.dirty]
    uint32_t gitRef;    // 4 [SHA1]

    // compile/build time
    char date[32];
} ExtRobotPiVersion;

typedef struct _ExtCameraConfig
{
    uint8_t expAutoMode;
    float expAnalogGain; // 1.0 - 8.0
    float expDigitalGain; // 1.0 - 2.0
    uint32_t expTimeUs; // maximum depends on framerate, if auto exposure is on this is the upper limit
    uint8_t wbAutoMode;
    float wbRedGain; // 0.0 - 8.0, typical 0.9 - 1.9
    float wbBlueGain; // 0.0 - 8.0, typical 0.9 - 1.9
    int8_t forcedResolution; // 0-2 => low, mid, high. Other values => No forced mode (may be controlled by bot skill)
} ExtCameraConfig;

typedef struct _ExtCameraPreviewLine160
{
    uint16_t row;
    uint16_t data[160];
} ExtCameraPreviewLine160;

typedef struct _ExtCameraControl
{
    uint8_t resolution; // 0 - low, 1 - medium, 2 - high
    uint8_t recording;
} ExtCameraControl;

typedef struct _ExtCameraStats
{
    uint16_t width;
    uint16_t height;

    float dtMin;
    float dtMax;
    float dtAvg;
    float dtDev;

    float rtMin;
    float rtMax;
    float rtAvg;
    float rtDev;

    uint8_t recording;
    char recordFilename[32];
    uint32_t recordSize;
    float recordDuration;

    uint32_t imagesTaken;
    char imageFilename[32];
} ExtCameraStats;

typedef struct _ExtCameraCalibration
{
    char status[32];

    uint32_t resolutionX;
    uint32_t resolutionY;
    float focalLength;
    float principalPointX;
    float principalPointY;
    float distortionCoeff0;
    float distortionCoeff1;
    float height;
    float rotationY;
    float rotationP;
    float rotationR;
    float score;
} ExtCameraCalibration;

typedef struct _ExtRemoteTime
{
    uint32_t timestampUs;
} ExtRemoteTime;

typedef struct _ExtRobotState
{
    uint32_t timestampUs;

    float posGlobal[3];
    float velGlobal[3];
    float accGlobal[3];
} ExtRobotState;

typedef struct _ExtBallLocalisationConfig
{
    float topImageSkipFactor;
    float greedyCircleFactor;
    uint16_t minBlobArea;
    uint16_t maxTrackers;
    uint16_t trackerTimeoutMs;

    uint16_t historySize;
    float modelError;
    float measError;
    float maxVelocity;
    uint8_t usePlaneIntersection;
} ExtBallLocalisationConfig;

typedef struct _ExtPointDistanceSensorConfig
{
    uint8_t blackThreshold;
    uint8_t whiteThreshold;
    uint8_t tooWhiteThreshold;
    float bottomSkipPercent;
    float topSkipPercent;
    float centerCoverPercent;
} ExtPointDistanceSensorConfig;

#define EXT_COLOR_THRESHOLDS_ID_ORANGE 0
#define EXT_COLOR_THRESHOLDS_ID_WHITE 1
#define EXT_COLOR_THRESHOLDS_ID_BLACK 2

typedef struct _ExtColorThresholds
{
    uint8_t colorId;
    uint8_t y[2];
    uint8_t u[2];
    uint8_t v[2];
} ExtColorThresholds;

#define EXT_STEP_MASK_YPRESUMMER          0x01
#define EXT_STEP_MASK_COLOR_CLASSIFIER    0x02
#define EXT_STEP_MASK_REGION_EXTRACTOR    0x04
#define EXT_STEP_MASK_BALL_LOC            0x08
#define EXT_STEP_MASK_DIST_SENSOR         0x10
#define EXT_STEP_MASK_MINI_PREVIEW        0x20
#define EXT_STEP_MASK_RLE_RECORDER        0x40
#define EXT_STEP_MASK_CALIBRATION         0x80

typedef struct _ExtStepConfig
{
    uint32_t enabledSteps;
    uint32_t debugSteps;
    uint8_t debugLevel;
} ExtStepConfig;

typedef struct _ExtPointDistSensor
{
    uint32_t timestampUs;

    uint32_t validColumns;
    float avgHeight;
    float avgYBottom;
    uint8_t isMostlyWhite;
} ExtPointDistSensor;

typedef struct _ExtUpdateProgress
{
    uint8_t updateInProgress;
    char status[64];
} ExtUpdateProgress;

#pragma pack(pop) // end of message definitions
