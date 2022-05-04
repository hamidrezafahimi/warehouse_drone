#ifndef MOTION_DETECTION_H
#define MOTION_DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

class MotionDetection{

public: 

	std::vector<cv::Point2f> *pts;

	MotionDetection(int);
	~MotionDetection();
	void detectMotion(cv::Mat *, cv::Mat **, std::vector<cv::Point2f> *);
	std::vector<cv::Point2f> calcFlow(cv::Mat *, std::vector<cv::Point2f> *);

private:

	bool prevCaptured;
	bool prev1Captured; 
	bool prevGrayCaptured; 
	int threshold;
	cv::Mat *prev;
	cv::Mat *prev1;
	cv::Mat *prevGray;

	uint8_t rgbDiff(uint8_t, uint8_t, uint8_t, uint16_t, uint16_t);
	uint8_t intensityDiff(uint8_t, uint16_t, uint16_t);
	void featuresToBeTracked(cv::Mat *, cv::Mat *, int);

};

#endif // MOTION_DETECTION_H