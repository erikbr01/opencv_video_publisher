
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
#include "large_data_participant.h"
#include "paths.h"
#include "sensor_msgs/msgs/Mocap.h"


#include "Image720pPubSubTypes.h"
#include "sensor_msgs/msgs/Image720p.h"

#include "include_helper.h"
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Monitors the camera and estimates the pose of any detected markers
int startWebcamMonitoring(DDSPublisher *img_pub) {
  cout << "entering monitor" << endl;
  idl_msg::Image720p img_msg;

  img_msg.header().id() = "Webcam video feed";

  Mat frame;

  cout << "vidcap" << endl;

  VideoCapture vid(1);

  if (!vid.isOpened()) {
    return -1;
  }
  namedWindow("Webcam", WINDOW_AUTOSIZE);

  while (true) {
    if (!vid.read(frame))
      break;

    imshow("Webcam", frame);

    if (waitKey(30) >= 0)
      break;

    // Publish image

    // This should correspond to 1280*720*3 -> check for current attached webcam
    uint length = frame.total() * frame.channels();

    // copy frame to array
    uchar *image_arr = frame.isContinuous() ? frame.data : frame.clone().data;
    cout << "transfer" << endl;

    // transfer to image message
    for (int i = 0; i < length; i++) {
      img_msg.frame()[i] = image_arr[i];
    }

    // this stops the webcam preview
    img_pub->publish(img_msg);
    cout << "test" << endl;
  }
  return 1;
}

int main(int argc, char *argv[]) {

  // Create participant. Arguments-> Domain id, QOS name
  DefaultParticipant dp(0, "video_publisher_test");

  DDSPublisher img_pub(idl_msg::Image720pPubSubType(), "img_topic",
                       dp.participant());

  img_pub.init();

  cout << "success init" << endl;

  cout << "entering monitor" << endl;

  idl_msg::Image720p img_msg;

  img_msg.header().id() = "Webcam video feed";

  Mat frame;

  cout << "vidcap" << endl;

  VideoCapture vid(1);

  if (!vid.isOpened()) {
    return -1;
  }
  namedWindow("Webcam", WINDOW_AUTOSIZE);

  while (true) {
    if (!vid.read(frame))
      break;

    imshow("Webcam", frame);

    if (waitKey(30) >= 0)
      break;

    // Publish image

    // This should correspond to 1280*720*3 -> check for current attached webcam
    uint length = frame.total() * frame.channels();

    // copy frame to array
    uchar *image_arr = frame.isContinuous() ? frame.data : frame.clone().data;

    // transfer to image message
    for (int i = 0; i < length; i++) {
      img_msg.frame()[i] = image_arr[i];
    }

    // this stops the webcam preview
    img_pub.publish(img_msg);
  }
}
