#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;

class arucoBoardDetector{

public:

	arucoBoardDetector(int, char **, int);

	~arucoBoardDetector();

	Vec3f findTargetInCamFrame(Mat *, Mat *, int);
	Vec3f lowpass_update(Vec3f);

private:

	int dictionaryId;
	bool showRejected;
	bool estimatePose;
	float markerLength;
	Ptr<aruco::DetectorParameters> detectorParams;
	Ptr<aruco::Dictionary> dictionary;
	Ptr<aruco::Board> board;
	Mat camMatrix, distCoeffs;
	double totalTime;
	int totalIterations;
	int markersX;
	int markersY;
	float markerSeparation;
	vector<Vec3f> lowpass_win;
	int lowpass_win_size;

	static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	    FileStorage fs(filename, FileStorage::READ);
	    if(!fs.isOpened())
	        return false;
	    fs["camera_matrix"] >> camMatrix;
	    fs["distortion_coefficients"] >> distCoeffs;
	    return true;
	}

	static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
	    FileStorage fs(filename, FileStorage::READ);
	    if(!fs.isOpened())
	        return false;
	    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	    // fs["doCornerRefinement"] >> params->doCornerRefinement;
	    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	    fs["markerBorderBits"] >> params->markerBorderBits;
	    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	    return true;
	}

	bool isRotationMatrix(Mat &);
	Vec3f rotationMatrixToEulerAngles(Mat &);
};