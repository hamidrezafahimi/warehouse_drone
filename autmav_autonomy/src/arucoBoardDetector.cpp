#include "arucoBoardDetector.h"

const char* about = "Basic marker detection";
const char* keys  =
		"{w        |       | Number of markers in X direction }"
        "{h        |       | Number of markers in Y direction }"
        "{s        |       | Separation between two consecutive markers in the grid (in meters)}"
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{v        |       | Input from video file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side lenght (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{nn       |       | Node name }"
        "{it       |       | Image topic name }"
        "{otn      |       | Output topic name }"
        "{r        |       | show rejected candidates too }";

arucoBoardDetector::arucoBoardDetector(int argc, char *argv[], int _lowpass_win_size){
	CommandLineParser parser(argc, argv, keys);
  	parser.about(about);

  	if(argc < 2) {
        parser.printMessage();
        throw "Bad parameters";
    }

    lowpass_win_size = _lowpass_win_size;

    markersX = parser.get<int>("w");
    markersY = parser.get<int>("h");
    dictionaryId = parser.get<int>("d");
    showRejected = parser.has("r");
    estimatePose = parser.has("c");
    markerLength = parser.get<float>("l");
    markerSeparation = parser.get<float>("s");

    detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            throw "Invalid detector parameters file";
        }
    }

    if(!parser.check()) {
        parser.printErrors();
        throw "Unable to parse";
    }

    dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    board = aruco::GridBoard::create(markersX, markersY, float(markerLength),
                                                      float(markerSeparation), dictionary);

    if(estimatePose) {
        bool readOk = readCameraParameters(parser.get<string>("c"), camMatrix, distCoeffs);
        if(!readOk) {
            throw "Invalid camera file";
        }
    }
}

bool arucoBoardDetector::isRotationMatrix(Mat &R){
	Mat Rt;
	transpose(R, Rt);
	Mat shouldBeIdentity = Rt * R;
	Mat I = Mat::eye(3,3, shouldBeIdentity.type());

	return  norm(I, shouldBeIdentity) < 1e-6;
}

Vec3f arucoBoardDetector::rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));

    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);

}

Vec3f arucoBoardDetector::findTargetInCamFrame(Mat *image, Mat *image_result, int factor){
	Vec3f target;
	vector< int > ids;
    vector< vector< Point2f > > corners, rejected;
    Mat rvecs, tvecs;

    // detect markers and estimate pose
    aruco::detectMarkers(*image, dictionary, corners, ids, detectorParams, rejected);

    int valid;
    if(estimatePose && ids.size() > 0)
        valid = aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvecs,
                                         tvecs);

    // draw results
    image->copyTo(*image_result);

    if(ids.size() > 0) {
        aruco::drawDetectedMarkers(*image_result, corners, ids);

        if(estimatePose) {

            // aruco::drawAxis(*image_result, camMatrix, distCoeffs, rvecs, tvecs,
            //                 markerLength * 1.5f);
            tvecs.at<double> (0 , 0) = tvecs.at<double> (0 , 0) + factor * 0.5;
            tvecs.at<double> (1 , 0) = tvecs.at<double> (1 , 0) - factor * 0.5;
            aruco::drawAxis(*image_result, camMatrix, distCoeffs, rvecs, tvecs,
                            markerLength * 1.5f);

            cv::imshow("bottom_image", *image_result);
    		cv::waitKey(1);
            return Vec3f(tvecs.at<double> (0 , 0), tvecs.at<double> (1 , 0), tvecs.at<double> (2 , 0));
        }
    }

    cv::imshow("bottom_image", *image_result);
    cv::waitKey(1);
    return Vec3f(0, 0, 0);
}

Vec3f arucoBoardDetector::lowpass_update(Vec3f new_val){
	if (lowpass_win.size() < lowpass_win_size) {
		lowpass_win.push_back(new_val);
	}
	else{
		lowpass_win.erase(lowpass_win.begin());
		lowpass_win.push_back(new_val);
	}

	Vec3f sum;
	for(int i=0; i<lowpass_win.size(); i++){
		sum[0] += lowpass_win.at(i)[0];
		sum[1] += lowpass_win.at(i)[1];
		sum[2] += lowpass_win.at(i)[2];
	}

	sum[0] = sum[0] / lowpass_win.size();
	sum[1] = sum[1] / lowpass_win.size();
	sum[2] = sum[2] / lowpass_win.size();

	return sum;
}