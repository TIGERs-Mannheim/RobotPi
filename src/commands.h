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
#define CMD_SYSTEM_MATCH_FEEDBACK       14

#define CMD_EXT_BALL_DETECTIONS         0x0E00
#define CMD_EXT_SHUTDOWN                0x0E01
#define CMD_EXT_ROBOT_PI_VERSION        0x0E02
#define CMD_EXT_CAMERA_CONFIG           0x0E04
#define CMD_EXT_CAMERA_PREVIEW_LINE_160 0x0E05
#define CMD_EXT_CAMERA_PREVIEW_CONFIG   0x0E06
#define CMD_EXT_CAMERA_CONTROL          0x0E07
#define CMD_EXT_CAMERA_TRIGGER_CAPTURE  0x0E08
#define CMD_EXT_CAMERA_STATS            0x0E09
#define CMD_EXT_CAMERA_INTRINSICS       0x0E0A
#define CMD_EXT_CAMERA_EXTRINSICS       0x0E0B
#define CMD_EXT_REMOTE_TIME             0x0E0C
#define CMD_EXT_ROBOT_STATE             0x0E0D
#define CMD_EXT_BALL_DETECTION_CONFIG   0x0E0E
#define CMD_EXT_BALL_TRACKER_CONFIG     0x0E0F
#define CMD_EXT_COLOR_THRESHOLDS        0x0E10
#define CMD_EXT_FIELD_INFO              0x0E11
#define CMD_EXT_POSITION_DETECTION      0x0E12
#define CMD_EXT_GOAL_DETECTIONS         0x0E13

typedef struct _SystemMatchFeedback  // 22
{
    int16_t curPosition[3];     // [mm]
    int16_t curVelocity[3];     // [mm/s]
    uint8_t kickerLevel;        // [V]
    int16_t dribblerSpeed;      // [rpm]
    uint16_t batteryLevel;      // [mV]
    uint8_t barrierKickCounter;
    uint16_t features;          // movement, dribbler, barrier, straight, chip
    uint8_t hardwareId;
    uint8_t dribblerTemp;
} SystemMatchFeedback;

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
    // version number
    uint8_t major;
    uint8_t minor;

    // compile/build time
    char date[24];
} ExtRobotPiVersion;

typedef struct _ExtCameraConfig
{
    uint8_t expAutoMode;
    float expAnalogGain; // 1.0 - 8.0
    float expDigitalGain; // 1.0 - 2.0
    uint32_t expTimeUs; // maximum depends on framerate, not used during auto exposure
    uint8_t wbAutoMode;
    float wbRedGain; // 0.0 - 8.0, typical 0.9 - 1.9
    float wbBlueGain; // 0.0 - 8.0, typical 0.9 - 1.9
} ExtCameraConfig;

typedef struct _ExtCameraPreviewLine160
{
    uint16_t row;
    uint16_t data[160];
} ExtCameraPreviewLine160;

typedef struct _ExtCameraPreviewConfig
{
    uint8_t enable;
} ExtCameraPreviewConfig;

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

typedef struct _ExtCameraIntrinsics
{
    uint16_t resolution[2]; // resolution used during calibration (preferably native resolution)
    float focalLength; // in pixels, at calibration resolution. Assumed to be equal for X and Y.
    float principalPoint[2]; // in pixels, at calibration resolution.
    float radialDistortion[2];
} ExtCameraIntrinsics;

typedef struct _ExtCameraExtrinsics
{
    float cameraTiltDeg; // degree downward from horizontal plane (e.g. +20 is slightly downward)
    float cameraPos[3]; // translation from robot base to camera
    float groundClearance; // distance from floor to robot base (bottom side of robot's base plate)
} ExtCameraExtrinsics;

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

typedef struct _ExtBallDetectionConfig
{
    float topImageSkipFactor;
    uint32_t minBlobArea;
    float ballDiameter;
    float greedyCircleFactor;
    uint32_t maxTrackers;
    float trackerTimeout;
} ExtBallDetectionConfig;

typedef struct _ExtBallTrackerConfig
{
    float modelError;
    float measError;
    float maxVelocity;
    uint32_t historySize;
} ExtBallTrackerConfig;

#define EXT_COLOR_THRESHOLDS_ID_ORANGE 0
#define EXT_COLOR_THRESHOLDS_ID_WHITE 1
#define EXT_COLOR_THRESHOLDS_ID_GREEN 2
typedef struct _ExtColorThresholds
{
    uint8_t colorId;
    uint8_t y[2];
    uint8_t u[2];
    uint8_t v[2];
} ExtColorThresholds;

typedef struct _ExtFieldInfo
{
    uint32_t timestampUs;

    float fieldSize[2]; // x,y in m inclusive field margin
    float botOrientation; // in rad relative to field
} ExtFieldInfo;

typedef struct _ExtPositionDetection
{
    uint32_t timestampUs;

    float pos[2]; // x,y in m in field coordinates
} ExtPositionDetection;

#define EXT_GOAL_DETECTIONS_MAX_GOALS 1
typedef struct _ExtGoalDetections
{
    uint32_t timestampUs;

    struct _goals
    {
        float leftAngle; // in rad (towards negative)
        float rightAngle; // in rad (towards positive)
    } goals[EXT_GOAL_DETECTIONS_MAX_GOALS];

    uint8_t numGoals;
} ExtGoalDetections;

#pragma pack(pop) // end of message definitions
