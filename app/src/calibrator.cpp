#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "calibrator.h"

using namespace cv;
using namespace std;

// Program to calibrate a camera

// edge length of printed checkerboard squares in metres
const float calibrationSquareDimension = 0.02f;
// size in terms of numbers of corners inside the chessboard
const Size chessboardDimensions = Size(6, 9);

// creates the known positions on the board using the given spacing and board
// size
void createKnownBoardPositions(Size boardSize, float squareEdgeLength,
                               vector<Point3f> &corners) {

  for (int i = 0; i < boardSize.height; i++) {
    for (int j = 0; j < boardSize.width; j++) {
      corners.push_back(
          Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
    }
  }
}

// finds the corner coordinates of the chessboard using the OpenCV function
void getChessboardCorners(vector<Mat> images,
                          vector<vector<Point2f>> &allFoundCorners,
                          bool showResults) {
  for (vector<Mat>::iterator iter = images.begin(); iter != images.end();
       iter++) {
    vector<Point2f> pointBuf;
    bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf,
                                       CALIB_CB_ADAPTIVE_THRESH |
                                           CALIB_CB_NORMALIZE_IMAGE);

    if (found) {
      allFoundCorners.push_back(pointBuf);
    }

    if (showResults) {
      drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
      imshow("Looking for corners", *iter);
      waitKey(0);
    }
  }
}

// match actual coordinates to coordinates where they should be with the OpenCV
// function
void cameraCalibration(vector<Mat> calibrationImages, Size boardSize,
                       float squareEdgeLength, Mat &cameraMatrix,
                       Mat &distanceCoefficients) {
  vector<vector<Point2f>> checkerboardImageSpacePoints;
  getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

  vector<vector<Point3f>> worldSpaceCornerPoints(1);

  createKnownBoardPositions(boardSize, squareEdgeLength,
                            worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(),
                                worldSpaceCornerPoints[0]);

  vector<Mat> rVectors, tVectors;
  distanceCoefficients = Mat::zeros(8, 1, CV_64F);
  calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints,
                  boardSize, cameraMatrix, distanceCoefficients, rVectors,
                  tVectors);
}

// match actual coordinates to coordinates where they should be with the OpenCV
// function
void cameraCalibrationWithSavedImages(vector<Mat> calibrationImages,
                                      Size boardSize, float squareEdgeLength,
                                      Mat &cameraMatrix,
                                      Mat &distanceCoefficients) {
  vector<vector<Point2f>> checkerboardImageSpacePoints;
  getChessboardCorners(calibrationImages, checkerboardImageSpacePoints, false);

  vector<vector<Point3f>> worldSpaceCornerPoints(1);

  createKnownBoardPositions(boardSize, squareEdgeLength,
                            worldSpaceCornerPoints[0]);
  worldSpaceCornerPoints.resize(checkerboardImageSpacePoints.size(),
                                worldSpaceCornerPoints[0]);

  vector<Mat> rVectors, tVectors;
  distanceCoefficients = Mat::zeros(8, 1, CV_64F);
  calibrateCamera(worldSpaceCornerPoints, checkerboardImageSpacePoints,
                  boardSize, cameraMatrix, distanceCoefficients, rVectors,
                  tVectors);
}

// save the calibration values to a file
bool saveCameraCalibration(string name, Mat cameraMatrix,
                           Mat distanceCoefficients) {

  ofstream outStream(name);
  if (outStream) {

    // Save the camera matrix

    uint16_t rows = cameraMatrix.rows;
    uint16_t columns = cameraMatrix.cols;

    outStream << rows << endl;
    outStream << columns << endl;

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < columns; c++) {
        double value = cameraMatrix.at<double>(r, c);
        outStream << value << endl;
      }
    }

    // Save the distance coefficients

    rows = distanceCoefficients.rows;
    columns = distanceCoefficients.cols;

    outStream << rows << endl;
    outStream << columns << endl;

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < columns; c++) {
        double value = distanceCoefficients.at<double>(r, c);
        outStream << value << endl;
      }
    }
    outStream.close();
    return true;
  }
  return false;
}

// Reads camera matrix and distance coefficients from file
bool Calibrator::loadCamerCalibration(string name, Mat &cameraMatrix,
                                      Mat &distanceCoefficients) {
  ifstream inStream(name);
  if (inStream) {
    uint16_t rows;
    uint16_t columns;

    // Read camera matrix

    inStream >> rows;
    inStream >> columns;

    cameraMatrix = Mat(Size(columns, rows), CV_64F);

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < columns; c++) {
        double temp = 0;
        inStream >> temp;
        cameraMatrix.at<double>(r, c) = temp;
      }
    }

    // Read distance coefficients
    inStream >> rows;
    inStream >> columns;

    distanceCoefficients = Mat::zeros(rows, columns, CV_64F);

    for (int r = 0; r < rows; r++) {
      for (int c = 0; c < columns; c++) {
        double temp = 0;
        inStream >> temp;
        distanceCoefficients.at<double>(r, c) = temp;
      }
    }
    inStream.close();
    return true;
  }
  return false;
}

// Working principe: take snapshots by pressing the spacebar, when you have over
// 15 snapshots, press enter to get the calibration values
int Calibrator::camera_calib(Mat &cameraMatrix, Mat &distanceCoefficients) {

  Mat frame;
  Mat gray;
  Mat drawToFrame;

  vector<Mat> savedImages;

  vector<vector<Point2f>> markerCorners, rejectedCandidates;

  VideoCapture vid(1);

  if (!vid.isOpened()) {
    return 0;
  }

  int framesPerSecond = 20;

  int imageCount = 0;

  namedWindow("Webcam", WINDOW_AUTOSIZE);

  while (true) {
    if (!vid.read(frame))
      break;

    vector<Vec2f> foundPoints;
    bool found = false;

    cvtColor(frame, gray, COLOR_BGR2GRAY);
    found = findChessboardCorners(gray, chessboardDimensions, foundPoints,
                                  CALIB_CB_ADAPTIVE_THRESH |
                                      CALIB_CB_NORMALIZE_IMAGE);
    frame.copyTo(drawToFrame);
    drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints,
                          found);
    if (found) {
      imshow("Webcam", drawToFrame);
    } else {
      imshow("Webcam", frame);
    }
    char input = waitKey(1000 / framesPerSecond);

    // explicit casting is necessary for some reason
    int character = (int)input;

    switch (character) {
    case 32:
      // saving image on spacebar
      if (found) {
        imageCount++;
        Mat temp;
        frame.copyTo(temp);
        savedImages.push_back(temp);

        ostringstream convert;
        string imageName = "calibration_image_";
        convert << imageName << imageCount << ".png";
        imwrite(convert.str(), temp);
        cout << "Image " << imageCount << "saved successfully" << endl;
      }
      break;
    case 27:
      // exit on esc
      return 0;
      break;
    case 13:
      // start calibration on enter
      if (savedImages.size() > 14) {
        cout << "trying to calibrate" << endl;

        cameraCalibration(savedImages, chessboardDimensions,
                          calibrationSquareDimension, cameraMatrix,
                          distanceCoefficients);
        cout << "trying to save" << endl;
        saveCameraCalibration("camCalibration", cameraMatrix,
                              distanceCoefficients);
        cout << "Calibration done, file saved" << endl;
      }

    default:
      break;
    }
  }
  return 0;
}
