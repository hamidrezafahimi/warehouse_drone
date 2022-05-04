#include "ArcodeDetector.h"
#include "utility.h"



// vector<int> ArcodeDetector::ids;
// vector<vector<float>> ArcodeDetector::selectedIds;
vector<vector<Point2f>> ArcodeDetector::corners;
vector< Vec3d > ArcodeDetector::rvecs, ArcodeDetector::tvecs;
int ArcodeDetector::imageWidth, ArcodeDetector::imageHeight;
vector<vector<int>> ArcodeDetector::boxList = {};
vector< int > ArcodeDetector::ids;
Rect ArcodeDetector::roi;
bool ArcodeDetector::roiSet = false;

// bool ArcodeDetector::areSelectedIdsSet = false;


ArcodeDetector::ArcodeDetector(string camMatAddress)
{
  readCameraParameters(camMatAddress, camMatrix, distCoeffs);
}



bool ArcodeDetector::readCameraParameters(string filename, Mat &camMatrix,
                                          Mat &distCoeffs)
{
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}



void ArcodeDetector::get6dofInFrameBodyCoordinates(Vec3d rvec, Vec3d tvec,
                                                   Vec3d& r, Vec3d& t)
{
  Mat rot_mat = Mat::zeros(3,3,CV_64F);
  float roll, pitch, yaw, x, y, z;

  Rodrigues(rvec, rot_mat);

  roll = atan2 (-rot_mat.at<double>(2,1),rot_mat.at<double>(2,2)) * 180 / PI;
  pitch = asin(rot_mat.at<double>(2,0)) * 180 / PI;
  yaw = atan2(-rot_mat.at<double>(1,0), rot_mat.at<double>(0,0))*180/PI;
  //
  r = {roll, pitch, yaw};

  x = ((-rot_mat.at<double>(0,0))*tvec[0]) + ((-rot_mat.at<double>(1,0))*tvec[1])\
   + ((-rot_mat.at<double>(2,0))*tvec[2]);
  y = ((-rot_mat.at<double>(0,1))*tvec[0]) + ((-rot_mat.at<double>(1,1))*tvec[1])\
   + ((-rot_mat.at<double>(2,1))*tvec[2]);
  z = ((-rot_mat.at<double>(0,2))*tvec[0]) + ((-rot_mat.at<double>(1,2))*tvec[1])\
   + ((-rot_mat.at<double>(2,2))*tvec[2]);

  t = {x, y, z};
}



void ArcodeDetector::imageCallback(Mat& image)
{
  // clock_t start, end;
  if (!imDimsSet)
    setImDims(image.cols, image.rows);

  cout<<"-------------------"<<endl;

  success = false;
  tvecs.clear();
  rvecs.clear();
  boxList.clear();
  Mat cim;

  if (roiSet){
    std::cout << "e1  "/* << roi.x */<< '\n';
    cim = image(roi);
    // std::ostringstream name;
    // name << "/home/hamidreza/thesis/workSpace/src/visual_guidance/src/df_" << it++ << ".png";
    // cv::imwrite(name.str(), cim);
    // std::cout << "e2" << '\n';
    aruco::detectMarkers(cim, dictionary, corners, ids, detectorParams, rejected);
    // cout<<ids.size()<<endl;
    }
  else {
    std::cout << "e3" << '\n';
    aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
    cim = image;//.clone();
    }

  if (corners.size() == 0)
    return;
  else
    success = true;

  validateArcodes();

  getBoxList();

  // start = clock();
  getNavigationData(cim);

  cv::imwrite("/home/hamidreza/thesis/workSpace/cim.png", image);
  // end = clock();
  // double time_taken = double(end - start)/ double(CLOCKS_PER_SEC);
  // cout << "Time taken by program is : " << fixed
  //      << time_taken << setprecision(5);
  // cout << " sec " << endl;

  // imshow("out", image);
  // imshow("target", cim);

  // waitKey(1);
}



void ArcodeDetector::getNavigationData(Mat &image)
{
  float minDist = 1e7, dist;
  vector<Vec3d> rs, ts;
  // clock_t start, end;
  // start = clock();

  aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix,
                                   distCoeffs, rs, ts);
  // end = clock();


  if(ids.size() > 0) {
    aruco::drawDetectedMarkers(image, corners, ids);
    for(unsigned int i = 0; i < ids.size(); i++)
    {
      Vec3d r, t;
      aruco::drawAxis(image, camMatrix, distCoeffs, rs[i], ts[i],
                      markerLength * 0.5f);
      get6dofInFrameBodyCoordinates(rs[i], ts[i], r, t);
      rvecs.push_back(r);
      tvecs.push_back(t);
      // selectedIds[i].push_back(r[0]);
      // selectedIds[i].push_back(r[1]);
      // selectedIds[i].push_back(r[2]);
      // selectedIds[i].push_back(t[0]);
      // selectedIds[i].push_back(t[1]);
      // selectedIds[i].push_back(t[2]);
    }
  }

  // double time_taken = double(end - start)/ double(CLOCKS_PER_SEC);
  // cout << "Time taken by program is : " << fixed
  //      << time_taken << setprecision(5);
  // cout << " sec " << endl;
}



void ArcodeDetector::validateArcodes()
{
  float maxArea = -1;
  vector<int> areas;

  for(unsigned int i = 0; i < corners.size(); i++)
  {
    areas.push_back(contourArea(corners[i]));
    if (maxArea<areas[i])
      maxArea = areas[i];
  }

  for(unsigned int i = 0; i < corners.size(); i++)
  {
    if (areas[i]<(0.25*maxArea))
    {
      areas.erase(areas.begin() + i);
      corners.erase(corners.begin() + i);
      ids.erase(ids.begin() + i);
      i--;
    }
  }
}



vector<int> ArcodeDetector::findContourCenter(vector<Point2f> contour)
{
  int length = contour.size(), x = 0, y = 0;

  for (unsigned int k=0; k<length; k++)
  {
    x += contour[k].x;
    y += contour[k].y;
  }
  x /= length;
  y /= length;
  return {x, y};
}



void ArcodeDetector::getBoxList()
{
  // std::cout << "getBoxList" << '\n';
  vector<int> data = {};
  int num = corners.size();
  for (int k=0; k<num; ++k)
  {
    data = findContourCenter(corners[k]);
    data.push_back(ids[k]);
    boxList.push_back(data);
    // cout<<boxList[k][0]<<'\t'<<boxList[k][1]<<'\t'<<boxList[k][2]<<endl;
  }
}



void ArcodeDetector::setImDims(int w, int h)
{
  imageHeight = h;
  imageWidth = w;
  imDimsSet = true;
}
// void ArcodeDetector::setSelectedIds(vector<vector<int>> sids)
// {
//   selectedIds.clear();
//   selectedCorners.clear();
//
//   for (int i = 0; i < ids.size(); i++)
//   {
//     for (int j = 0; j < sids.size(); j++)
//     {
//       if (ids[i] == sids[j][0])
//       {
//         selectedIds.push_back({float(sids[j][0])});
//         selectedCorners.push_back(corners[i]);
//       }
//     }
//   }
//
//   areSelectedIdsSet = true;
// }
