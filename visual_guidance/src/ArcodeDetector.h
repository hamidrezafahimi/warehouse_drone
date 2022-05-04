#ifndef ARCODEDETECTION_H
#define ARCODEDETECTION_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/aruco.hpp>
#include <algorithm>
#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>
#include "time.h"
using namespace cv;
using namespace std;
#define PI 3.14159265


class ArcodeDetector {

    Mat image, imageCopy, camMatrix, distCoeffs;
    // static vector<vector<float>> selectedIds;
    vector<vector<Point2f>> rejected;
    int theId = 0;
    float markerLength = 0.1;
    bool imDimsSet = false;
    cv::Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(2));
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    int it = 0;

    float calculateDistance(Vec3d);
    bool readCameraParameters(string , Mat &, Mat &);
    void get6dofInFrameBodyCoordinates(Vec3d, Vec3d, Vec3d&, Vec3d&);
    void validateArcodes();
    vector<int> findContourCenter(vector<Point2f>);
    void getBoxList();
    void getNavigationData(Mat &);
    void setImDims(int, int);

public:

    bool success = false;
    // static bool areSelectedIdsSet;
    static vector<vector<Point2f>> corners;
    static vector< Vec3d > rvecs, tvecs;
    static int imageWidth, imageHeight;
    static vector<vector<int>> boxList;
    static vector< int > ids;
    static Rect roi;
    static bool roiSet;


    ArcodeDetector(string);
    void imageCallback(Mat&);
    void setSelectedIds(vector<vector<int>>);

};


#endif
