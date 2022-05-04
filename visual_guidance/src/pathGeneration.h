#ifndef PATHGENERATION_H
#define PATHGENERATION_H


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
#include "visual_guidance/sixDof.h"
#include <iostream>
#include <math.h>
#include "ArcodeDetector.h"
using namespace cv;
using namespace std;


class WarehouseStatist
{

  float calculateDistance(Point2f, Point2f);

public:

  // WarehouseStatist();
  vector<vector<int>> extractShelfArrangement(vector<vector<int>>,
                                              vector<vector<int>>);

};



class PathGenerator
{

  float II, JJ, x_vel, verticalImageCenterRatio = 0.25;
  int id, allCellsNum;
  bool guideVectorSet = false, arrangementSet = false, isBestSet = false,
       setPointSet = false;
  vector <vector <int>> allCellNames = {}, arrangement, bestIds;
  float cellWidth, cellHeight;
  vector <vector <float>> guideVectors;
  vector <int> bestCell;
  ArcodeDetector adLink;
  Point2f setPoint;
  vector <Vec3d> bestTvecs;
  vector <vector <int>> bestBoxList;
  int mix = 0, miy = 0;
  int roiOffset = 30;


  void setId(int);
  vector<int> findBestCell();
  vector<vector<int>> findBestIds(vector<int>);
  vector<vector<float>> findGuideVectors(vector<int>);
  float cellCostFunc(int, int, vector<int>);
  vector<int> getCellCentroid();
  vector<float> fuseTvecsData();
  bool checkScatteredTvec(Vec3d, int, char);
  void setSetPoint(int, int);
  void updateBestData();
  void updateROI();


public:


  PathGenerator(int, int, float, float, float, float, ArcodeDetector&);
  // PathGenerator(int, int, float, float, float, float, float, ArcodeDetector);
  vector <int> generate(vector<vector<int>>);
  vector<float> guide();

};


#endif
