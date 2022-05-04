#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <math.h>

#include "MotionDetection.h"

class ObstacleDetection{

public:

	ObstacleDetection(int, int, int, int, int, int, int, int, int, float);
	~ObstacleDetection();
	void detectObs(cv::Mat *, cv::Mat *, cv::Mat *, cv::Mat *, int);

private:

	int threshold;
	int foeWinSize;
	int gap;
	int margin;
	int erosionElementSize;
	int dilationElementSize;
	int maximumTTC;
	int avoidanceTTC;
	float cosineThreshold;
	MotionDetection *md;
	std::vector<cv::Point2f> foeWin;

	void getFlowVectors(std::vector<cv::Point2f> *, std::vector<cv::Point2f> *, std::vector<cv::Point2f> *, std::vector<cv::Point2f> *, int);
	void VecToMat(std::vector<cv::Point2f> *, cv::Mat *);
	void findFOE(std::vector<cv::Point2f> *, cv::Mat *);
	cv::Point2f findFOESortBased(std::vector<cv::Point2f> *, cv::Mat *, int, int);
	void timeToContact(std::vector<cv::Point2f> *, cv::Mat *, cv::Point2f, cv::Mat *, cv::Mat **);
	void formDepthImg(cv::Mat ***, std::vector<cv::Point2f> *, std::vector<float> *, float, float, int, int);
	bool isPositive(float);
	void erosion( cv::Mat **, int, int );
	void dilation( cv::Mat **, int, int );
	float distance(cv::Point2f, cv::Point2f);
	float absolute(float, float);
	cv::Point2f MAUpdate(cv::Point2f);
	
};

#endif // OBSTACLE_DETECTION_H