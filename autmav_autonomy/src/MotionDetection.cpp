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

	cv::Mat gray, diff1, diff2, thresh1, thresh2;
	cv::cvtColor(*newRGB, gray, cv::COLOR_BGR2GRAY);

	if (!prevCaptured)
	{
		*prev = gray.clone();
		prevCaptured = true;
	}
	else if(prevCaptured && !prev1Captured)
	{
		*prev1 = gray.clone();
		prev1Captured = true;
	}
	else{

		// for (int row=0; row < gray.rows; row++)
		// {
		// 	for (int col=0; col < gray.cols; col++)
		// 	{
		// 		// cv::Vec3b pix = newRGB->at<cv::Vec3b>(row,col);
		// 		// (*motionBin)->at<uchar>(row, col) = rgbDiff(pix[0],
		// 		// 										   pix[1],
		// 		// 										   pix[2],
		// 		// 										   row, col);

		// 		uint8_t pix = gray.at<uint8_t>(row,col);
		// 		(*motionBin)->at<uchar>(row, col) = intensityDiff(pix, row, col);
		// 	}
		// }

		cv::absdiff(*prev, gray, diff1);
		cv::absdiff(*prev1, gray, diff2);
        cv::threshold(diff1, thresh1, threshold, 255, cv::THRESH_BINARY);
        cv::threshold(diff2, thresh2, threshold, 255, cv::THRESH_BINARY);

        **motionBin = thresh1 | thresh2;

		*prev1 = prev->clone();
		*prev = gray.clone();

		
		featuresToBeTracked(*motionBin, &gray, 0);

		*movingPts = *pts; 
	}

}

void MotionDetection::featuresToBeTracked(cv::Mat *bin, cv::Mat *gray, int count){

	pts->clear();

	// cv::Mat motionImage;
	// cv::bitwise_and(*bin,*gray,motionImage);

	goodFeaturesToTrack(*gray, *pts, 100, 0.03, 7, *bin, 7, false, 0.04);

	// cv::imshow("motionImage",motionImage);

	// for (int row=0; row < bin->rows; row++)
	// {
	// 	for (int col=0; col < bin->cols; col++)
	// 	{
	// 		if (bin->at<uchar>(row, col) == 255)
	// 		{
	// 			if (count < 50)
	// 			{
					
	// 				pts->push_back(cv::Point2f(col, row));
	// 			}
	// 			else if (50 <= count < 500)
	// 			{
	// 				if (rand()%20 == 0) pts->push_back(cv::Point2f(col, row));
	// 			}
	// 			else if (500 <= count < 1000)
	// 			{
	// 				if (rand()%200 == 0) pts->push_back(cv::Point2f(col, row));
	// 			}
	// 			else if (1000 <= count < 5000)
	// 			{
	// 				if (rand()%500 == 0) pts->push_back(cv::Point2f(col, row));
	// 			}
	// 			else if (5000 <= count < 15000)
	// 			{
	// 				if (rand()%2000 == 0) pts->push_back(cv::Point2f(col, row));
	// 			}
	// 			else
	// 			{
	// 				if (rand()%10000 == 0) pts->push_back(cv::Point2f(col, row));
	// 			}
	// 		}	
	// 	}
	// }
}

uint8_t MotionDetection::rgbDiff(uint8_t r, uint8_t g, uint8_t b, uint16_t row, uint16_t col)
{
	cv::Vec3b prev_pix = prev->at<cv::Vec3b>(row,col);
	cv::Vec3b prev1_pix = prev1->at<cv::Vec3b>(row,col);
	float diff1 = ( r - prev_pix[0] )*( r - prev_pix[0] ) +
				  ( g - prev_pix[1] )*( g - prev_pix[1] ) + 
				  ( b - prev_pix[2] )*( b - prev_pix[2] );

	float diff2 = ( r - prev1_pix[0] )*( r - prev1_pix[0] ) +
				  ( g - prev1_pix[1] )*( g - prev1_pix[1] ) + 
				  ( b - prev1_pix[2] )*( b - prev1_pix[2] );

	return (diff1 > threshold*threshold && diff2 > threshold*threshold) ? 255 : 0;

}

uint8_t MotionDetection::intensityDiff(uint8_t i, uint16_t row, uint16_t col)
{
	uint8_t prev_pix = prev->at<uchar>(row,col);
	uint8_t prev1_pix = prev1->at<uchar>(row,col);
	float diff1 = ( i - prev_pix )*( i - prev_pix );
	float diff2 = ( i - prev1_pix )*( i - prev1_pix );

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
