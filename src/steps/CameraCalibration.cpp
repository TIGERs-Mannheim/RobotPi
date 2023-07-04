#include <iostream>
#include "CameraCalibration.h"


#define PATTERN_COLUMNS 6
#define PATTERN_ROWS    6
#define PATTERN_SIZE (PATTERN_COLUMNS*PATTERN_ROWS)

#define PATTERN_BLOB_DISTANCE        0.03f      // in m
#define PATTERN_START_BASE_DISTANCE  0.192929f  // in m


#define MODEL_SIZE 9


static Eigen::VectorXf cameraToParams(Camera& camera)
{
    ExtCameraCalibration& calibration = camera.getCalibration();

    Eigen::VectorXf params(MODEL_SIZE);
    params(0) = calibration.focalLength;
    params(1) = calibration.principalPointX;
    params(2) = calibration.principalPointY;
    params(3) = calibration.distortionCoeff0;
    params(4) = calibration.distortionCoeff1;
    params(5) = calibration.height;
    params(6) = calibration.rotationY;
    params(7) = calibration.rotationP;
    params(8) = calibration.rotationR;
    return params;
}

static void paramsToCamera(Camera& camera, const Eigen::VectorXf& params)
{
    ExtCameraCalibration& calibration = camera.getCalibration();

    calibration.focalLength = params(0);
    calibration.principalPointX = params(1);
    calibration.principalPointY = params(2);
    calibration.distortionCoeff0 = params(3);
    calibration.distortionCoeff1 = params(4);
    calibration.height = params(5);
    calibration.rotationY = params(6);
    calibration.rotationP = params(7);
    calibration.rotationR = params(8);

    camera.applyCalibration();
}

static Camera getScaledCamera(const State& state)
{
    Camera camera;
    camera.setCurrentResolutionWidth(state.uvWidth);
    ExtCameraCalibration& calibration = camera.getCalibration();
    float calibrationFactor = (float)state.uvWidth / (float)calibration.resolutionX;
    calibration.resolutionX = state.uvWidth;
    calibration.resolutionY = state.uvHeight;
    calibration.focalLength *= calibrationFactor;
    calibration.principalPointX *= calibrationFactor;
    calibration.principalPointY *= calibrationFactor;
    return camera;
}

static std::vector<Eigen::Vector2f> generatePattern()
{
    std::vector<Eigen::Vector2f> pattern;

    const float halfColumns = (PATTERN_COLUMNS-1) / 2.0f;
    for(int y = 0; y < PATTERN_ROWS; y++)
    {
        for(int x = 0; x < PATTERN_COLUMNS; x++)
        {
            pattern.emplace_back(((float)x - halfColumns) * PATTERN_BLOB_DISTANCE, ((float)y) * PATTERN_BLOB_DISTANCE + PATTERN_START_BASE_DISTANCE);
        }
    }

    return pattern;
}

static float mapDistance(const Camera& camera, const Eigen::Vector2f& imagePoint, const Eigen::Vector2f& patternPoint)
{
    return (camera.intersectRayWithGround(camera.getNormalizedRay(camera.undistort(imagePoint)), 0) - patternPoint).norm();
}

static void scoreCamera(Camera& camera, const std::vector<Eigen::Vector2f>& imagePoints, const std::vector<Eigen::Vector2f>& patternPoints)
{
    float score = 0.0;
    for(int i = 0; i < PATTERN_SIZE; i++)
    {
        float distance = mapDistance(camera, imagePoints[i], patternPoints[i]);
        score += distance*distance;
    }

    camera.getCalibration().score = score;
}


CameraCalibration::CameraCalibration()
{
    addCommandHandler<ExtStepConfig>(CMD_EXT_STEP_CONFIG, [&] (ExtStepConfig* pCfg) {
        bool enable = pCfg->enabledSteps & EXT_STEP_MASK_CALIBRATION;

        if(isEnabled() != enable)
            hasCalibrated_ = false;

        handleStepConfig(*pCfg, EXT_STEP_MASK_CALIBRATION);
    });
}

void CameraCalibration::execute(State& state)
{
    const uint32_t minCalibPointSize = 5;
    const float maximalScore = 0.001;
    Eigen::VectorXf minima(MODEL_SIZE);
    minima(0) = minima(1) = minima(2) = 0;
    minima(3) = minima(4) = -10.0;
    minima(5) = 0.05;
    minima(6) = minima(7) = minima(8) = -M_PI;
    Eigen::VectorXf maxima(MODEL_SIZE);
    maxima(0) = 2000;
    maxima(1) = state.uvWidth;
    maxima(2) = state.uvHeight;
    maxima(3) = maxima(4) = 10.0;
    maxima(5) = 0.10;
    maxima(6) = maxima(7) = maxima(8) = M_PI;

    if(hasCalibrated_)
        return;

    strcpy(state.camera.getCalibration().status, "Searching pattern");
    state.pComm->write({CMD_EXT_CAMERA_CALIBRATION, state.camera.getCalibration()});

    // Find the "paper" (largest white region at the bottom of the image)
    ColoredRegion* paper = nullptr;
    uint32_t bottom = state.uvHeight - 1;
    for(ColoredRegion& region : state.frameColoredRegions[Color::White])
    {
        if(region.y2 == bottom && (paper == nullptr || paper->area < region.area))
            paper = &region;
    }

    if(paper == nullptr)
        return;

    // Find the calibration points
    std::vector<Eigen::Vector2f> imagePoints;
    for(ColoredRegion& region : state.frameColoredRegions[Color::Orange])
    {
        if(region.x1 > paper->x1 && region.x2 < paper->x2 && region.y1 > paper->y1 && region.area > minCalibPointSize)
            imagePoints.emplace_back(region.cen_x, region.cen_y);
    }
    if(imagePoints.size() != PATTERN_SIZE)
        return;

    auto startTime = std::chrono::high_resolution_clock::now();

    // Order points in pattern order
    // Sort all points along y-Axis
    std::sort(imagePoints.begin(), imagePoints.end(), [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) { return a(1) > b(1); });
    // Sort each row along x-Axis
    auto itr = imagePoints.begin();
    for(int y = 0; y < PATTERN_ROWS; y++)
        std::sort(itr + y*PATTERN_COLUMNS, itr + (y+1) * PATTERN_COLUMNS, [](const Eigen::Vector2f& a, const Eigen::Vector2f& b) { return a(0) < b(0); });

    std::vector<Eigen::Vector2f> patternPoints = generatePattern();
    Eigen::Vector2i resolution(state.uvWidth, state.uvHeight);
    Eigen::VectorXf delta(MODEL_SIZE);
    delta(0) = delta(1) = delta(2) = 1.0;   // Focal length, principalX, principalY
    delta(3) = delta(4) =            0.01;  // distortion0, distortion1
    delta(5) =                       0.001; // height
    delta(6) = delta(7) = delta(8) = 0.01;  // camRotY, camRotP, camRotR

    Camera camera = getScaledCamera(state);
    if(getDebugLevel())
    {
        std::cout << "Start CamParams: " << cameraToParams(camera).transpose() << std::endl << std::endl;
    }

    // Gradient descent
    scoreCamera(camera, imagePoints, patternPoints);
    Camera testCam = camera;
    while(delta.cwiseAbs().sum() > 1e-8)
    {
        bool change = false;

        for(int i = 0; i < MODEL_SIZE; i++)
        {
            Eigen::VectorXf params = cameraToParams(camera);

            params(i) -= delta(i);
            paramsToCamera(testCam, params);
            scoreCamera(testCam, imagePoints, patternPoints);
            if(testCam.getCalibration().score < camera.getCalibration().score)
            {
                camera = testCam;
                change = true;
            }

            params(i) += 2*delta(i);
            paramsToCamera(testCam, params);
            scoreCamera(testCam, imagePoints, patternPoints);
            if(testCam.getCalibration().score < camera.getCalibration().score)
            {
                camera = testCam;
                change = true;
            }
        }

        if(!change)
        {
            if(getDebugLevel())
            {
                std::cout << "Score: " << sqrtf(camera.getCalibration().score) / PATTERN_SIZE << " m" << std::endl;
                std::cout << "Delta: " << delta.transpose() << std::endl;
                std::cout << "CamParams: " << cameraToParams(camera).transpose() << std::endl << std::endl;
            }
            delta /= 10;
        }
    }

    // Sanity checks
    camera.getCalibration().score = sqrtf(camera.getCalibration().score)/PATTERN_SIZE; // Metrify score
    if(camera.getCalibration().score > maximalScore)
    {
        std::cout << "Calibration result discarded: Bad score" << std::endl;
        return;
    }

    Eigen::VectorXf params = cameraToParams(camera);
    for(int i = 0; i < MODEL_SIZE; i++)
    {
        if(params(i) < minima(i) || params(i) > maxima(i))
        {
            std::cout << "Calibration result discarded: parameter " << i << " out of bounds (" << params(i) << ")" << std::endl;
            return;
        }
    }

    if(getDebugLevel())
    {
        std::cout << "Calibration " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - startTime).count() / 1000.0 << " ms" << std::endl;
        for(int i = 0; i < PATTERN_SIZE; i++)
        {
            Eigen::Vector2f imagePoint = imagePoints[i] * 2;
            Eigen::Vector2f patternPoint = camera.distort(camera.getImageCoordinates(camera.getBaseToCam() * Eigen::Vector3f(patternPoints[i](0), patternPoints[i](1), 0))) * 2;

            state.pFrame->drawLine(imagePoint(0), imagePoint(1), patternPoint(0), patternPoint(1), {127, 0, 255});
        }
    }

    strcpy(camera.getCalibration().status, "Calibrated");

    state.camera = camera;
    state.camera.saveCalibration(CAMERA_CALIBRATION_FILE_PATH);
    state.pComm->write({CMD_EXT_CAMERA_CALIBRATION, state.camera.getCalibration()});
    hasCalibrated_ = true;
}
