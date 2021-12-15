
// FastDDS
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "MocapPubSubTypes.h"
#include "default_participant.h"
#include "default_publisher.h"
#include "paths.h"
#include "sensor_msgs/msgs/Mocap.h"
#include "set_parameters.h"

#include "include_helper.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// OpenCV parameters ---------------------

// Physical dimension of the markers used in meters
const float arucoSquareDimension = 0.162f;

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R) {
  Mat Rt;
  transpose(R, Rt);
  Mat shouldBeIdentity = Rt * R;
  Mat I = Mat::eye(3, 3, shouldBeIdentity.type());

  return norm(I, shouldBeIdentity) < 1e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R) {

  assert(isRotationMatrix(R));

  float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) +
                  R.at<double>(1, 0) * R.at<double>(1, 0));

  bool singular = sy < 1e-6; // If

  float x, y, z;
  if (!singular) {
    x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
    y = atan2(-R.at<double>(2, 0), sy);
    z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
  } else {
    x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
    y = atan2(-R.at<double>(2, 0), sy);
    z = 0;
  }
  return Vec3f(x, y, z);
}

// Monitors the camera and estimates the pose of any detected markers
int startWebcamMonitoring(DDSPublisher *mocap_pub, const Mat &cameraMatrix,
                          const Mat &distanceCoefficients,
                          float arucoSquareDimension) {
  cpp_msg::Mocap mocap_msg;

  mocap_msg.header.id = "AruCo detection camera";

  Mat frame;

  vector<int> markerIds;
  vector<vector<Point2f>> markerCorners, rejectedCandidates;
  aruco::DetectorParameters parameters;

  Ptr<aruco::Dictionary> markerDictionary = aruco::getPredefinedDictionary(
      aruco::PREDEFINED_DICTIONARY_NAME::DICT_6X6_250);

  VideoCapture vid(1);

  if (!vid.isOpened()) {
    return -1;
  }
  namedWindow("Webcam", WINDOW_AUTOSIZE);

  vector<Vec3d> rotationVectors, translationVectors;

  while (true) {
    if (!vid.read(frame))
      break;

    aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
    aruco::estimatePoseSingleMarkers(markerCorners, arucoSquareDimension,
                                     cameraMatrix, distanceCoefficients,
                                     rotationVectors, translationVectors);

    for (int i = 0; i < markerIds.size(); i++) {
      aruco::drawAxis(frame, cameraMatrix, distanceCoefficients,
                      rotationVectors[i], translationVectors[i], 0.1f);
    }

    if (markerIds.size() > 0) {
      Vec3d rvec = rotationVectors[0];
      Vec3d tvec = translationVectors[0];

      Mat rotationMatrix;
      Rodrigues(rvec, rotationMatrix);

      Vec3f rotation_euler_zyx =
          180 / M_PI * rotationMatrixToEulerAngles(rotationMatrix);

      mocap_msg.pose.position.x = tvec[0];
      mocap_msg.pose.position.y = tvec[1];
      mocap_msg.pose.position.z = tvec[2];

      mocap_msg.pose.orientation_euler.roll = rotation_euler_zyx[0];
      mocap_msg.pose.orientation_euler.pitch = rotation_euler_zyx[1];
      mocap_msg.pose.orientation_euler.yaw = rotation_euler_zyx[2];

      mocap_pub->publish(mocap_msg);
    }

    imshow("Webcam", frame);

    if (waitKey(30) >= 0)
      break;
  }
  return 1;
}

int main(int argc, char *argv[]) {
  Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
  Mat distanceCoefficients;

  Calibrator calib;

  calib.loadCamerCalibration("camCalibration", cameraMatrix,
                             distanceCoefficients);
  cout << "loaded coefficients" << endl;
  // load from YAML File
  set_parameters(paths::parameters_path);
  const int N = parameters::objects.size(); // Number of Objects

  ////////////////////////////////////////////////////////////
  // FastDDS objects

  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "mocap_publisher_test");

  std::string topic = parameters::topic_prefix + parameters::objects.at(0);

  // Create DDS publisher vector
  DDSPublisher mocap_pub =
      DDSPublisher(idl_msg::MocapPubSubType(), topic, dp.participant());
  mocap_pub.init();

  startWebcamMonitoring(&mocap_pub, cameraMatrix, distanceCoefficients,
                        arucoSquareDimension);

  // ////////////////////////////////////////////////////////
  // for (int i = 0; i < N; i++) {
  //   mocap_msg.at(i).header.timestamp = _Output_GetFrameNumber.FrameNumber;
  // }
  // ////////////////////////////////////////////////////////

  // ///////////////////////////////////////////////////////////////
  // for (int i = 0; i < N; i++) {
  //   mocap_msg.at(i).latency = MyClient.GetLatencyTotal().Total;
  // }

  // ////////////////////////////////////////////
  // for (int i = 0; i < N; i++) {
  //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
  //     mocap_msg.at(i).header.id = SubjectName;
  //   }
  // }
  // ////////////////////////////////////////////

  // // Capture position in FastDDS message
  // ////////////////////////////////////////////
  // for (int i = 0; i < N; i++) {
  //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
  //     mocap_msg.at(i).pose.position.x =
  //         _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
  //     mocap_msg.at(i).pose.position.y =
  //         _Output_GetSegmentGlobalTranslation.Translation[1] / 1000.0;
  //     mocap_msg.at(i).pose.position.z =
  //         _Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;
  //   }
  // }

  // ////////////////////////////////////////////
  // for (int i = 0; i < N; i++) {
  //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
  //     mocap_msg.at(i).pose.orientation_quat.x =
  //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[0];
  //     mocap_msg.at(i).pose.orientation_quat.y =
  //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[1];
  //     mocap_msg.at(i).pose.orientation_quat.z =
  //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[2];
  //     mocap_msg.at(i).pose.orientation_quat.w =
  //         _Output_GetSegmentGlobalRotationQuaternion.Rotation[3];
  //   }
  // }

  // // Publish data
  // for (int i = 0; i < N; i++) {
  //   if (SubjectName.compare(parameters::objects.at(i)) == 0) {
  //     mocap_pub.at(i).publish(mocap_msg.at(i));
  //   }
  // }
  // ////////////////////////////////////
}
