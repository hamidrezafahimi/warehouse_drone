#include <opencv2/opencv.hpp>

#include <MotionDetection.h>
#include <iostream>
#include <stdlib.h> 
#include <time.h>

MotionDetection::MotionDetection(int thresh){
	pts = new std::vector<cv::Point2f>();
	prev = new cv::Mat();
	prev1 = new cv::Mat();
	prevGray = new cv::Mat();
	prevCaptured = false;
	prev1Captured = false;
	prevGrayCaptured = false;
	threshold = thresh;

	srand (time(NULL));
}
	
MotionDetection::~MotionDetection(){
	prev->release();
	prev1->release();
	prevGray->release();
	pts->clear();
}

void MotionDetection::detectMotion(cv::Mat *newRGB, cv::Mat **motionBin, std::vector<cv::Point2f> * movingPts){

	if (!prevCaptured)
	{
		*prev = newRGB->clone();
		prevCaptured = true;
	}
	else if(prevCaptured && !prev1Captured)
	{
		*prev1 = newRGB->clone();
		prev1Captured = true;
	}
	else{

		int movingCount = 0;

		for (int row=0; row < newRGB->rows; row++)
		{
			for (int col=0; col < newRGB->cols; col++)
			{
				(*motionBin)->at<uchar>(row, col) = rgbDiff(newRGB->at<cv::Vec3b>(row,col)[0],
														   newRGB->at<cv::Vec3b>(row,col)[1],
														   newRGB->at<cv::Vec3b>(row,col)[2],
														   row, col);

				if ((*motionBin)->at<uchar>(row, col) == 255) movingCount++;
			}
		}

		*prev1 = prev->clone();
		*prev = newRGB->clone();

		// featuresToBeTracked(motionBin, movingCount);

		// *movingPts = *pts; 
	}

}

void MotionDetection::featuresToBeTracked(cv::Mat *bin, int count){

	pts->clear();

	for (int row=0; row < bin->rows; row++)
	{
		for (int col=0; col < bin->cols; col++)
		{
			if (bin->at<uchar>(row, col) == 255)
			{
				if (count < 50)
				{
					
					pts->push_back(cv::Point2f(col, row));
				}
				else if (50 <= count < 500)
				{
					if (rand()%20 == 0) pts->push_back(cv::Point2f(col, row));
				}
				else if (500 <= count < 1000)
				{
					if (rand()%200 == 0) pts->push_back(cv::Point2f(col, row));
				}
				else if (1000 <= count < 5000)
				{
					if (rand()%500 == 0) pts->push_back(cv::Point2f(col, row));
				}
				else if (5000 <= count < 15000)
				{
					if (rand()%2000 == 0) pts->push_back(cv::Point2f(col, row));
				}
				else
				{
					if (rand()%10000 == 0) pts->push_back(cv::Point2f(col, row));
				}
			}	
		}
	}
}

uint8_t MotionDetection::rgbDiff(uint8_t r, uint8_t g, uint8_t b, uint16_t row, uint16_t col)
{
	float diff1 = ( r - prev->at<cv::Vec3b>(row,col)[0] )*( r - prev->at<cv::Vec3b>(row,col)[0] ) +
				  ( g - prev->at<cv::Vec3b>(row,col)[1] )*( g - prev->at<cv::Vec3b>(row,col)[1] ) + 
				  ( b - prev->at<cv::Vec3b>(row,col)[2] )*( b - prev->at<cv::Vec3b>(row,col)[2] );

	float diff2 = ( r - prev1->at<cv::Vec3b>(row,col)[0] )*( r - prev1->at<cv::Vec3b>(row,col)[0] ) +
				  ( g - prev1->at<cv::Vec3b>(row,col)[1] )*( g - prev1->at<cv::Vec3b>(row,col)[1] ) + 
				  ( b - prev1->at<cv::Vec3b>(row,col)[2] )*( b - prev1->at<cv::Vec3b>(row,col)[2] );

	return (diff1 > threshold*threshold && diff2 > threshold*threshold) ? 255 : 0;

}

std::vector<cv::Point2f> MotionDetection::calcFlow(cv::Mat *newGray, std::vector<cv::Point2f> *featurePts)
{
	std::vector<cv::Point2f> nextPts;

	if (!prevGrayCaptured)
	{
		*prevGray = newGray->clone();
		prevGrayCaptured = true;
	}
	else
	{
	
		std::vector<uchar> status;
        std::vector<float> err;
        cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

        if (featurePts->size() == 0) return nextPts;

		cv::calcOpticalFlowPyrLK(*prevGray, *newGray, *featurePts, nextPts, status, err, cv::Size(30,30),
                                 3, termcrit, 0, 0.001);

		// for (int n=status.size()-1; n >= 0 ; n--)
		// {
		// 	if (status.at(n) == 0)
		// 	{
		// 		nextPts.erase(nextPts.begin()+n);
		// 		featurePts->erase(pts->begin()+n);
		// 	}
		// }
		*prevGray = newGray->clone();
	}

	return nextPts;
}
