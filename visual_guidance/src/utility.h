#ifndef UTILITY_H
#define UTILITY_H


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"
#include "opencv2/opencv.hpp"
#include <math.h>
#include <vector>
#include <iostream>

using namespace cv;
using namespace std;
// #define PI 3.14159265


namespace utility
{

  float calculate3dDistance(Vec3d);
  float calculate2dDistance(Point2f, Point2f);
  vector<int> findContourCenter(vector<Point2f>);
  // float cellCostFunc(int, int);

}


#endif
