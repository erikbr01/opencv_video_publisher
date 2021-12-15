#pragma once

#include <iostream>
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class ArucoFunctions {
private:
public:
  void createArucoMarker(cv::Ptr<cv::aruco::Dictionary> dictionary);
  cv::Mat DetectArucoMarker(cv::Mat frame,
                            cv::Ptr<cv::aruco::Dictionary> dictionary);
};