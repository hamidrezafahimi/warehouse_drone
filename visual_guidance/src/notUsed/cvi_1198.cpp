#include "visual_guidance/shelfGrid.h"
#include "ArcodeDetector.h"
#include "pathGeneration.h"
#include "time.h"


ArcodeDetector ad("/home/hamidreza/thesis/arcode/calib2.txt");
WarehouseStatist was;
PathGenerator pgn(10, 6, 0.572, 0.3, 8.01, 3.5, ad);
// PathGenerator pgn(10, 6, 0.572, 0.3, 8, 3.5, 0.5, &ad);
bool imageReceived = false, gridImReceived = false;
vector<vector<int>> shelfData, nodeData, shelfArrangement;
vector<float> cmnds;
Mat gridIm;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  // clock_t start, end;
  // start = clock();
  Mat image, gimg;
  image = cv_bridge::toCvShare(msg, "bgr8")->image;
  // std::cout << "--------------"<<endl;
  // cvtColor(image, gimg, cv::COLOR_RGB2GRAY);
  // cout<<gimg.at<int>(100, 100)<<endl;
  ad.imageCallback(image);
  // end = clock();
  imageReceived = true;
  if (ad.tvecs.size())
    cmnds = pgn.guide();

  // end = clock();
  // double time_taken = double(end - start)/ double(CLOCKS_PER_SEC);
  // cout << "Time taken by program is : " << fixed
  //      << time_taken << setprecision(5);
  // cout << " sec " << endl;
}



void gridImageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  gridIm = cv_bridge::toCvShare(msg, "bgr8")->image;
  gridImReceived = true;
}




void shelfGridCallback(const visual_guidance::shelfGridConstPtr& grid)
{
  vector<vector<int>> empties = {};
  shelfData.clear();
  nodeData.clear();
  std::cout << "ind" << '\n';

  for(int k=0; k<((grid->gridInfo).size()/4); ++k)
  {
    // pointInfo.clear()
    shelfData.push_back({(grid->gridInfo)[4*k], (grid->gridInfo)[4*k+1], (grid->gridInfo)[4*k+2], (grid->gridInfo)[4*k+3]});
    // std::cout << shelfData[k][0] << '\t';
    // std::cout << shelfData[k][1] << '\t';
    // std::cout << shelfData[k][2] << '\t';
    // std::cout << shelfData[k][3] << '\t';
    // std::cout << '\n';
  }
  // std::cout << "--------------"<<endl;
  for(int k=0; k<((grid->nodeGridInfo).size()/4); ++k)
  {
    // pointInfo.clear()
    nodeData.push_back({(grid->nodeGridInfo)[4*k], (grid->nodeGridInfo)[4*k+1], (grid->nodeGridInfo)[4*k+2], (grid->nodeGridInfo)[4*k+3]});
    // std::cout << shelfData[k][0] << '\t';
    // std::cout << shelfData[k][1] << '\t';
    // std::cout << shelfData[k][2] << '\t';
    // std::cout << shelfData[k][3] << '\t';
    // std::cout << '\n';
  }

  cout<<(imageReceived && ad.success)<<endl;

  if (imageReceived && ad.success)
  {
    shelfArrangement = was.extractShelfArrangement(shelfData, ad.boxList);
    // std::cout << "1" << '\n';
    // std::cout << "in" << '\n';
    pgn.generate(shelfArrangement);
    // std::cout << "2" << '\n';

    // for(int k=0; k<shelfArrangement.size(); ++k)
    // {
    //   std::cout << shelfArrangement[k][0] << '\t';
    //   std::cout << shelfArrangement[k][1] << '\t';
    //   std::cout << shelfArrangement[k][2] << '\t';
    //   std::cout << '\n';
    // }
    // std::cout << "\n-----------------------------------\n";
  }

  imageReceived = false;
  return;
}



int main(int argc, char *argv[]){

  ros::init(argc, argv, "cvi");
  ros::NodeHandle nh;

  // ros::Publisher pub = nh.advertise<visual_guidance::sixDof>("visual_guide", 1);
  // visual_guidance::sixDof smsg;

  // cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber imSub = it.subscribe("/quadrotor_1/front/image_raw", 1,\
   imageCallback);
  ros::Subscriber gridSub = nh.subscribe("/shelfInfo", 1, shelfGridCallback);
  ros::Subscriber gridImSub = nh.subscribe("/gridImage", 1, gridImageCallback);
  ros::Rate loop_rate(10); // Loop at 10Hz


  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
