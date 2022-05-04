#include "visual_guidance/shelfGrid.h"
#include "visual_guidance/sixDof.h"
#include "visual_guidance/cell.h"
#include "ArcodeDetector.h"
#include "pathGeneration.h"
#include "time.h"


ArcodeDetector ad("/home/hamidreza/thesis/arcode/calib2.txt");
WarehouseStatist was;

// PathGenerator pgn(10, 6, 0.572, 0.3, 8.01, 3.5, ad);
// PathGenerator pgn(10, 6, 0.572, 0.3, 5.01, 3.5, ad);
PathGenerator pgn(10, 6, 0.572, 0.3, 3, 2.5, ad);

bool imageReceived = false, gridReceived = false, cmdGenerated = false, sendCell = false;
vector<vector<int>> shelfData, nodeData, shelfArrangement;
vector<float> cmnds;
int cellRow, cellCol;
Mat gridIm;



void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!gridReceived)
    return;

  // clock_t start, end;
  // start = clock();

  std::cout << "imageCallback" << '\n';
  Mat image;
  image = cv_bridge::toCvShare(msg, "bgr8")->image;
  ad.imageCallback(image);
  if (ad.tvecs.size() && !cmdGenerated)
  {
    // std::cout << "ed" << '\n';
    cmnds = pgn.guide();
    cmdGenerated = true;
  }

  // end = clock();
  // double time_taken = double(end - start)/ double(CLOCKS_PER_SEC);
  // cout << "Time taken by program is : " << fixed
  //      << time_taken << setprecision(5);
  // cout << " sec " << endl;

}



void gridImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout << "gridImageCallback" << '\n';

  gridIm = cv_bridge::toCvShare(msg)->image;
  ad.imageCallback(gridIm);

}




void shelfGridCallback(const visual_guidance::shelfGridConstPtr& grid)
{
  // vector<vector<int>> empties = {};
  vector <int> cell;
  shelfData.clear();
  nodeData.clear();
  std::cout << "shelfGridCallback" << '\n';

  for(int k=0; k<((grid->gridInfo).size()/4); ++k)
  {
    shelfData.push_back({(grid->gridInfo)[4*k], (grid->gridInfo)[4*k+1], (grid->gridInfo)[4*k+2], (grid->gridInfo)[4*k+3]});
  }

  // for(int k=0; k<((grid->nodeGridInfo).size()/4); ++k)
  // {
  //   nodeData.push_back({(grid->nodeGridInfo)[4*k], (grid->nodeGridInfo)[4*k+1], (grid->nodeGridInfo)[4*k+2], (grid->nodeGridInfo)[4*k+3]});
  // }
  if (ad.success)
  {
    shelfArrangement = was.extractShelfArrangement(shelfData, ad.boxList);
    std::cout << "af 1" << '\n';
    cell = pgn.generate(shelfArrangement);
    std::cout << "af 2" << '\n';
    cellRow = cell[0];
    cellCol = cell[1];
    std::cout << "af 3" << '\n';

    sendCell = true;
    gridReceived = true;
  }

  // imageReceived = false;
  return;
}



int main(int argc, char *argv[]){

  // ros::NodeHandle nh;
  ros::init(argc, argv, "cvi");
  ros::NodeHandle nh;
  ros::Publisher cellCmdPub = nh.advertise<visual_guidance::cell>("cellTopic", 1);
  visual_guidance::cell cellMsg;
  // ros::Publisher pub = nh.advertise<visual_guidance::sixDof>("visual_guide", 1);
  // visual_guidance::sixDof smsg;

  // cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  ros::Subscriber gridImSub = nh.subscribe("/gridImage", 1, gridImageCallback);
  ros::Subscriber gridSub = nh.subscribe("/shelfInfo", 1, shelfGridCallback);
  // image_transport::Subscriber imSub = it.subscribe("/quadrotor_1/front/image_raw", 1,\
   imageCallback);
  ros::Publisher cmdPub = nh.advertise<visual_guidance::sixDof>("moveCommand", 1);
  visual_guidance::sixDof msg;

  ros::Rate loop_rate(10); // Loop at 10Hz

  // std::cout << "ros ok:  "<< ros::ok() << '\n';
  while (ros::ok())
  {
    if (cmdGenerated)
    {
      msg.x = cmnds[0];
      msg.y = cmnds[1];
      msg.z = cmnds[2];
      cmdPub.publish(msg);
      cmdGenerated = false;
    }

    if (sendCell)
    {
      cellMsg.i = cellRow;
      cellMsg.j = cellCol;
      std::cout << "/* message */" << '\n';
      cellCmdPub.publish(cellMsg);
      std::cout << "published cell" << '\n';

      sendCell = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
