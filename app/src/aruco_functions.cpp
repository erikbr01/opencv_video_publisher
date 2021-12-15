#include <aruco_functions.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

void ArucoFunctions::createArucoMarker(
    cv::Ptr<cv::aruco::Dictionary> dictionary) {
  Mat outputMarker;

  for (int i = 0; i < 50; i++) {
    aruco::drawMarker(dictionary, i, 500, outputMarker, 1);
    ostringstream convert;
    string imageName = "4x4Marker_";
    convert << imageName << i << ".png";
    imwrite(convert.str(), outputMarker);
  }
  // cv::aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
  // cv::imwrite("/Users/erik/Documents/Raptor/perception/marker23.png",
  //             markerImage);
}

cv::Mat
ArucoFunctions::DetectArucoMarker(cv::Mat frame,
                                  cv::Ptr<cv::aruco::Dictionary> dictionary) {
  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters =
      cv::aruco::DetectorParameters::create();
  cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds,
                           parameters, rejectedCandidates);

  // Draw box on detected markers
  cv::Mat detectedImage = frame.clone();
  cv::aruco::drawDetectedMarkers(detectedImage, markerCorners, markerIds);

  return detectedImage;
}
