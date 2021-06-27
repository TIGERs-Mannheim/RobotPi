/*
 * cvutils.h
 *
 *  Created on: 01.02.2021
 *      Author: FelixW
 */

#pragma once

#include <opencv2/opencv.hpp>
#include <interface/FrameYUV420.h>

/*
 * Loads an BGR (opencv default) image from the given file path into an FrameYUV420
 */
FrameYUV420 imageByPath(const char *path);

/*
 * Converts an YUVFrame420 to an opencv BGR Mat
 */
cv::Mat frameToMat(FrameYUV420& yuvFrame);

/*
 * Displays the YUVFrame using cv::imshow (blocking call)
 */
void showFrame(FrameYUV420& yuvFrame);
