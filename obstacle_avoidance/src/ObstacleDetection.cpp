#include <opencv2/opencv.hpp>

#include <ObstacleDetection.h>
#include <iostream>


ObstacleDetection::ObstacleDetection(int mar, int thresh, int winSize, int g, int motionDetectionThreshold, 
									 int erosionElemSize, int dilationElemSize, int max_ttc, int avoidance_ttc,
									 float cosineThresh)
{
	md = new MotionDetection(motionDetectionThreshold);
	threshold = thresh;
	foeWinSize = winSize;
	gap = g;
	margin = mar;
	erosionElementSize = erosionElemSize;
	dilationElementSize = dilationElemSize;
	cosineThreshold = cosineThresh;
	maximumTTC = max_ttc;
	avoidanceTTC = avoidance_ttc;
}

void ObstacleDetection::detectObs(cv::Mat *image, cv::Mat *destination, cv::Mat *flowImg, cv::Mat *bin, cv::Mat *depth_unfiltered, cv::Mat *depth, double dt, bool active)
{
  *destination = image->clone();
  cv::Mat gray, flowMat, flowUnitMat;
  std::vector<cv::Point2f> movingPts, prevPts, nextPts, flow, flowUnit;

  cv::cvtColor(*image, gray, cv::COLOR_BGR2GRAY);

  md->detectMotion(image, &bin, &movingPts);
  for (int i=0; i<image->cols; i=i+gap)
  {
  	for (int j=0; j<image->rows; j=j+gap)
  	{
  		prevPts.push_back(cv::Point2f(i,j));
  	}
  }

  nextPts = md->calcFlow(&gray, &prevPts);
  getFlowVectors(&prevPts, &nextPts, &flow, &flowUnit, (dt != 0) ? dt:1e5);
  VecToMat(&flowUnit, &flowUnitMat);
  VecToMat(&flow, &flowMat);

  // findFOE(&prevPts, &flowUnitMat);
  cv::Point2f FOE = MAUpdate(findFOESortBased(&prevPts, &flowUnitMat, image->cols, image->rows));

  // cv::Point2f FOE(image->cols/2, image->rows/2);

  if(FOE.x > margin && FOE.x < image->cols - margin && FOE.y > margin && FOE.y < image->rows - margin)
  {
  	circle(*destination, FOE, 20, cv::Scalar(0,255,0), 5, 8, 0);
  	putText(*destination, "FOE", cv::Point2f(FOE.x, FOE.y-30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 2);
  	putText(*destination, "moving forward", cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 2);
  }
  else
  {
  	// circle(*destination, FOE, 20, cv::Scalar(0,0,255), 5, 8, 0);
  	putText(*destination, "impure translation", cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,0,255), 2);
  	return;
  }
  
  timeToContact(&prevPts, &flowMat, FOE, destination, &depth_unfiltered);

  *depth = *depth_unfiltered;

  erosion(&depth, 0, erosionElementSize);
  dilation(&depth, 0, dilationElementSize);

  if (prevPts.size() == 0 || nextPts.size() == 0) return; 

  // for (int i=0; i<prevPts.size(); i++){
  //   cv::arrowedLine(*destination, prevPts.at(i), nextPts.at(i), cv::Scalar(255,0,0), 2, 8, 0, 0.1);
  // }

}

void ObstacleDetection::getFlowVectors(std::vector<cv::Point2f> *prevPts, std::vector<cv::Point2f> *nextPts, std::vector<cv::Point2f> *flow, std::vector<cv::Point2f> *flowUnit, int dt)
{
	flow->clear();
	flowUnit->clear();

	if (prevPts->size() == 0 || nextPts->size() == 0) return;
	std::vector<int> idxsToBeRemoved; 

	for (int i=0; i<prevPts->size(); i++){
		float u = (nextPts->at(i).x - prevPts->at(i).x) / dt;
		float v = (nextPts->at(i).y - prevPts->at(i).y) / dt;
		float vel = sqrt( pow(u,2) + pow(v,2));

		if (vel > threshold)
		{
			flow->push_back(cv::Point2f(u, v));
			flowUnit->push_back(cv::Point2f(u/vel, v/vel));
		}
		else
		{
			idxsToBeRemoved.push_back(i);
		}
	}

	for (int i=idxsToBeRemoved.size()-1; i>=0; i--)
	{
		prevPts->erase(prevPts->begin() + idxsToBeRemoved.at(i));
		nextPts->erase(nextPts->begin() + idxsToBeRemoved.at(i));
	}
}

void ObstacleDetection::VecToMat(std::vector<cv::Point2f> *vec, cv::Mat *out)
{
	if (vec->size() == 0) return;

	*out = cv::Mat::zeros(cv::Size(2,vec->size()), CV_32FC1);

	for (int i=0; i<vec->size(); i++){
		out->at<float>(i,0) = vec->at(i).y;
		out->at<float>(i,1) = vec->at(i).x;
	}
}

void ObstacleDetection::findFOE(std::vector<cv::Point2f> *pts, cv::Mat *A)
{
	if (pts->size() == 0 || A->rows == 0) return;

	cv::Mat B = cv::Mat::zeros(cv::Size(1,pts->size()), CV_32FC1);

	for (int i=0; i<pts->size(); i++){
		B.at<float>(i,0) = (pts->at(i).x * A->at<float>(i,1) - pts->at(i).y * A->at<float>(i,0));
	}

	cv::Mat result = (( A->t() * (*A) ) * A->t() ) * B;
	std::cout << result << std::endl;

}

cv::Point2f ObstacleDetection::findFOESortBased(std::vector<cv::Point2f> *pts, cv::Mat *flow, int width, int height)
{
	if (flow->rows == 0) return cv::Point2f(0, 0);

	int minErrX = -1;
	int minErr = 1e9;
	for (int i=0; i<width; i=i+gap)
	{
		int err = 0;
		for (int j=0; j<pts->size(); j++)
		{
			if (pts->at(j).x <= i)
			{
				if( isPositive(flow->at<float>(j,1)) )
				{
					err++;
				}
			}
			else
			{
				if( !isPositive(flow->at<float>(j,1)) )
				{
					err++;
				}
			}
		}

		if (err < minErr)
		{
			minErr = err;
			minErrX = i;
		}

	}

	int minErrY = -1;
	minErr = 1e9;
	for (int i=0; i<height; i=i+gap)
	{
		int err = 0;
		for (int j=0; j<pts->size(); j++)
		{
			if (pts->at(j).y <= i)
			{
				if( isPositive(flow->at<float>(j,0)) )
				{
					err++;
				}
			}
			else
			{
				if( !isPositive(flow->at<float>(j,0)) )
				{
					err++;
				}
			}
		}

		if (err < minErr)
		{
			minErr = err;
			minErrY = i;
		}

	}

	// std::cout << "[ " << minErrX << ", " << minErrY << " ]" << std::endl;

	return cv::Point2f(minErrX, minErrY);
}

bool ObstacleDetection::isPositive(float val)
{
	return (val >= 0);
}

cv::Point2f ObstacleDetection::MAUpdate(cv::Point2f newVal)
{
	if (newVal.x == 0 || newVal.y == 0) goto ave;

	if (foeWin.size() < foeWinSize)
	{
		foeWin.push_back(newVal);
	}
	else
	{
		foeWin.erase(foeWin.begin());
		foeWin.push_back(newVal);
	}

	ave:

	cv::Point2f sum(0, 0);
	for (int i=0; i<foeWin.size(); i++)
	{
		sum += foeWin.at(i);
	}

	if (foeWin.size() != 0)
	{
		return sum / int(foeWin.size());
	}
	else
	{
		return cv::Point2f(0, 0);
	}
}

void ObstacleDetection::timeToContact(std::vector<cv::Point2f> *pts, cv::Mat *flow, cv::Point2f foe, cv::Mat *dst, cv::Mat **depth)
{
	if (flow->rows == 0) return;

	float minTTC = 1e9;
	float maxTTC = 0;
	std::vector<float> TTCs;
	for (int i=0; i<pts->size(); i++)
	{

		cv::Point2f ptRelativeToFOE = pts->at(i)-foe;
		cv::Point2f flowVec(flow->at<float>(i,1), flow->at<float>(i,0));
		float dotProduct = ptRelativeToFOE.x * flowVec.x + ptRelativeToFOE.y * flowVec.y;

		float cosine = 0;
		float ttc = 0;
		
		if (dotProduct!=0) cosine = dotProduct / (absolute(ptRelativeToFOE.x, ptRelativeToFOE.y)*absolute(flowVec.x, flowVec.y));

		if (cosine < cosineThreshold){
			ttc = maximumTTC;
		}
		else
		{
			ttc = distance(pts->at(i), foe) / absolute(flow->at<float>(i,0), flow->at<float>(i,1));
		}

		ttc = (ttc < maximumTTC) ? ttc:maximumTTC;

		if (ttc > maxTTC) maxTTC = ttc;
		if (ttc < minTTC) minTTC = ttc;

		TTCs.push_back(ttc);
		// if (ttc < avoidanceTTC) circle(*dst, pts->at(i), 5, cv::Scalar(0,0,255), 3, 8, 0);

	}

	formDepthImg(&depth, pts, &TTCs, minTTC, maxTTC, dst->cols, dst->rows);
}

void ObstacleDetection::formDepthImg(cv::Mat ***depth, std::vector<cv::Point2f> *pts, std::vector<float> *TTCs, float minTTC, float maxTTC, int width, int height)
{

	if (TTCs->size() == 0) return;

	int depthHeight = height / gap;
	int depthWidth = width / gap;

	for (int i=0; i<pts->size(); i++)
	{
		int x = pts->at(i).x / gap;
		int y = pts->at(i).y / gap;
		if (y > depthHeight || x > depthWidth )	return;
		

		// float d = fabs(TTCs->at(i) - maxTTC) / fabs(maxTTC - minTTC);
		// (**depth)->at<uchar>(y,x) = int(255*d);
		(**depth)->at<uchar>(y,x) = (TTCs->at(i) < avoidanceTTC) ? 255: 0;
	}

	// for (int i=0; i<pts->size(); i++)
	// {
	// 	int minXIdx = pts->at(i).x - gap/2; minXIdx = (minXIdx > 0) ? minXIdx : 0;
	// 	int minYIdx = pts->at(i).y - gap/2; minYIdx = (minYIdx > 0) ? minYIdx : 0;
	// 	int maxXIdx = pts->at(i).x + gap/2; maxXIdx = (maxXIdx < width) ? maxXIdx : width - 1;
	// 	int maxYIdx = pts->at(i).y + gap/2; maxYIdx = (maxYIdx < height) ? maxYIdx : height - 1;

	// 	float d = fabs(TTCs->at(i) - maxTTC) / fabs(maxTTC - minTTC);
	// 	for(int j=minXIdx; j<=maxXIdx; j++)
	// 	{
	// 		for(int k=minYIdx; k<=maxYIdx; k++)
	// 		{
	// 			(**depth)->at<uchar>(k,j) = int(255*d);
	// 		}
	// 	}

	// }

}

void ObstacleDetection::erosion( cv::Mat ** img, int erosion_elem, int erosion_size)
{
  int erosion_type;
  if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

  cv::Mat element = cv::getStructuringElement( erosion_type,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );

  cv::erode( **img, **img, element );
}

void ObstacleDetection::dilation( cv::Mat ** img, int dilation_elem, int dilation_size)
{
  int dilation_type;
  if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }

  cv::Mat element = getStructuringElement( dilation_type,
                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       cv::Point( dilation_size, dilation_size ) );

  cv::dilate( **img, **img, element );
}

float ObstacleDetection::distance(cv::Point2f p1, cv::Point2f p2)
{
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

float ObstacleDetection::absolute(float u, float v)
{
	return sqrt(pow(u, 2) + pow(v, 2));
}