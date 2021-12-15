#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

class Calibrator {
private:
public:
  int camera_calib(cv::Mat &cameraMatrix, cv::Mat &distanceCoefficients);
  bool loadCamerCalibration(std::string name, cv::Mat &cameraMatrix,
                            cv::Mat &distanceCoefficients);
};