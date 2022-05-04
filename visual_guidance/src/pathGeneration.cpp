#include "pathGeneration.h"
#include "ArcodeDetector.h"
#include "utility.h"



// WarehouseStatist::WarehouseStatist()
// {
//   return;
// }


vector<vector<int>> WarehouseStatist::extractShelfArrangement(vector<vector<int>> shelf,
                                                              vector<vector<int>> boxes)//,
                                                            //  vector<vector<int>>& emptyCells)
{
  vector<vector<int>> table;
  vector<int> vec;//, fullCells = {};
  float minDist, dist;
  int idx;
  // emptyCells = shelf;

  for (int i=0; i<boxes.size(); i++)
  {
    minDist = 1e6;
    // cout<<"-----------contour:"<<endl;
    for (int j=0; j<shelf.size(); j++)
    {
      dist = utility::calculate2dDistance(Point2f(boxes[i][0], boxes[i][1]), Point2f(shelf[j][2], shelf[j][3]));
      if (minDist>dist)
      {
        minDist = dist;
        vec = {shelf[j][0], shelf[j][1], boxes[i][2]};
        idx = j;
      }
    }
  // emptyCells.erase(emptyCells.begin() + idx);
  // cout<<vec[0]<<'\t'<<vec[1]<<'\t'<<vec[2]<<endl;
  table.push_back(vec);
  }

  return table;
}



PathGenerator::PathGenerator(int I, int J, float width, float height,
                             float setPointI, float setPointJ, ArcodeDetector& ad)
                             : adLink(ad),
                               II(setPointI),
                               JJ(setPointJ),
                               cellWidth(width),
                               cellHeight(height)
{
  allCellsNum = I * J;

  for (int i=1; i<=I; i++)
  {
    for (int j=1; j<=J; j++)
    {
      allCellNames.push_back({i , j});
    }
  }
}



vector<int> PathGenerator::generate(vector<vector<int>> sArrangement)
{
  std::cout << "gen 1" << '\n';
  if (!arrangementSet)
  {
    arrangement = sArrangement;
    arrangementSet = true;
  }
  // std::cout << "eed" << '\n';
  std::cout << "gen 2" << '\n';

  if (!isBestSet)
  {
    std::cout << "gen 3" << '\n';

    bestCell = findBestCell();
    std::cout << "gen 4" << '\n';
    bestIds = findBestIds(bestCell);
    std::cout << "gen 5" << '\n';
    guideVectors = findGuideVectors(bestCell);
    // adLink.setSelectedIds(bestIds);
  }
  std::cout << "gen 6" << '\n';

  return bestCell;
}



vector<int> PathGenerator::findBestCell()
{
  float minCost = 1e6;
  float cost;
  vector<int> bCell;
  bool flag = false;
  int d;

//  std::cout << "qw1" << '\n';
  for (int k=0; k<allCellsNum; k++)
  {
    // cout<<allCellsNum[k][0]<<'\t'<<allCellsNum[k][1]<<endl;
    flag = false;
//    std::cout << "qw2" << '\n';

    for (int p=0; p<arrangement.size(); p++)
    {
//      std::cout << "qw3" << '\n';
      // cout<<arrangement[p][0]<<'\t'<<arrangement[p][1]<<endl;
      if (allCellNames[k][0]==arrangement[p][0] && allCellNames[k][1]==arrangement[p][1])
      {
//        std::cout << "qw31" << '\n';
        flag = true;
        d = p;
        break;
      }
    }
    if (flag)
    {
//      std::cout << "qw4" << '\n';
      // cout<<arrangement[d][0]<<'\t'<<arrangement[d][1]<<'\t'<<arrangement[d][2]<<endl;
      continue;
    }
//    std::cout << "qw" << '\n';
    cost = cellCostFunc(allCellNames[k][0], allCellNames[k][1], {});
//    std::cout << "wq" << '\n';
    // cout<<allCellNames[k][0]<<'\t'<<allCellNames[k][1]<<'\t'<<cost<<endl;
    if (minCost>cost)
    {
      minCost = cost;
      bCell = {allCellNames[k][0], allCellNames[k][1]};
    }
  }

  cout<<"best:       "<<bCell[0]<<'\t'<<bCell[1]<<endl;
  std::cout << "---------------------" << '\n';
  return bCell;
}



vector<vector<float>> PathGenerator::findGuideVectors(vector<int> cell)
{
  vector<vector<float>> gvs;
  int iDist, jDist;
  // cout<<arrangement.size()<<endl;
  // cout<<cell[0]<<'\t'<<cell[1]<<endl;
  for (int k = 0; k<bestIds.size(); k++)
  {
    // jDist = cell[1] - arrangement[k][1];
    // iDist = cell[0] - arrangement[k][0];
    // arrangement[k].push_back(jDist);
    // arrangement[k].push_back(iDist);
    gvs.push_back({bestIds[k][1]*cellWidth, (bestIds[k][2]*cellHeight)+0.0525});
    std::cout << bestIds[k][0]<<'\t'<< gvs[k][0]<<'\t'<< gvs[k][1]<<'\n';
  }

  return gvs;
}



vector<vector<int>> PathGenerator::findBestIds(vector<int> cell)
{
  vector<vector<float>> costList = {};
  vector<float> info;
  vector<vector<int>> theList = {};
  float/* minCost = 1e6,*/ cost;
  for (int k=0; k<arrangement.size(); k++)
  {
    cost = cellCostFunc(arrangement[k][0], arrangement[k][1], cell);
    // if (minCost>=cost)
      // minCost = cost;
      info = {float(arrangement[k][2]), cost};
      costList.push_back(info);
  }

  for (int k=0; k<costList.size(); k++)
  {
    if (costList[k][1] < 3)
    {
      // cout<</*costList[k][0]<<*/'\t'<<costList[k][1]<<endl;
      theList.push_back({int(costList[k][0]), cell[1] - arrangement[k][1], cell[0] - arrangement[k][0]});
    }
  }

  return theList;
}



float PathGenerator::cellCostFunc(int i, int j, vector<int> criterion)
{
  float m, n;
  m = float(i);
  n = float(j);
  if (criterion.size()==0)
    return sqrt(((m-II)*(m-II)) + ((n-JJ)*(n-JJ)));
  else
    return sqrt(((m-criterion[0])*(m-criterion[0])) + ((n-criterion[1])*(n-criterion[1])));
}



vector<float> PathGenerator::guide()
{
  float x_lin = x_vel, y_lin, z_lin;//, z_ang;
  vector<int> centroid;
  vector<float> lin;

  updateBestData();

  // std::cout << bestIds.size() << '\n';
  // for (int k = 0; k<bestIds.size(); k++)
  // {
  //   cout<<bestIds[k][0]<<'\t'<<bestIds[k][1]<<'\t'<<bestIds[k][2]<<endl;
  // }
  updateROI();

  lin = fuseTvecsData();

  y_lin = lin[0];
  z_lin = lin[1];
  // cout<<lin[0]<<'\t'<<lin[1]<<endl;
  // centroid = getCellCentroid();

  return {x_lin, y_lin, z_lin};
}



void PathGenerator::updateBestData()
{
  bestTvecs.clear();
  bestBoxList.clear();
  // std::cout << "/* updateBestData */" << '\n';
  // cout<<bestIds.size()<<'\n';

  for (int p = 0; p < bestIds.size(); p++)
  {
    for(int k = 0; k < adLink.boxList.size(); k++)
    {
      // if (p==0)
      if (adLink.boxList[k][2] == bestIds[p][0])
      {
        bestTvecs.push_back({adLink.tvecs[k][0], adLink.tvecs[k][1], guideVectors[p][0]\
           - adLink.tvecs[k][0], guideVectors[p][1] - adLink.tvecs[k][1]} );
        // bestTvecs.push_back({adLink.tvecs[k][0], adLink.tvecs[k][1]} );
        bestBoxList.push_back(adLink.boxList[k]);
        // cout<<adLink.boxList[k][0]<<'\t'<<adLink.boxList[k][1]<<'\t'<<adLink.boxList[k][2]<<endl;

        break;
      }
    }
    // std::cout << "u" << '\n';
  }

  // for (size_t i = 0; i < bestBoxList.size(); i++) {
  //   for (size_t j = 0; j < bestBoxList[i].size(); j++) {
  //     std::cout << bestBoxList[i][j] << '\t';
  //   }
  //   std::cout << '\n';
  // }

  // std::cout << "/* bestTvecs */" << '\n';
  // for (int p = 0; p < bestTvecs.size(); p++)
  // {
  //   for(int k = 0; k < 3; k++)
  //   {
  //     cout<<bestTvecs[k][p]<<'\t';
  //   }
  //   cout<<endl;
  // }
}



vector<float> PathGenerator::fuseTvecsData()
{
  float yDisplacement = 0, zDisplacement = 0;
  int numy = 0, numz = 0;

  // std::cout << guideVectors.size() << '\n';
  for (int k = 0; k<bestTvecs.size(); k++)
  {
    if (checkScatteredTvec(bestTvecs[k], k, 'y')){
      // std::cout << "k  " << k << '\n';
      continue;}

    yDisplacement += (bestTvecs[k][2]);

    numy++;
  }

  for (int k = 0; k<bestTvecs.size(); k++)
  {
    if (checkScatteredTvec(bestTvecs[k], k, 'z')){
      // std::cout << "k  " << k << '\n';
      continue;}

    zDisplacement += (bestTvecs[k][3]);
    numz++;
  }
  // std::cout << "--------------------" << '\n';
  yDisplacement /= numy;
  zDisplacement /= numz;
  cout<<yDisplacement<<'\t'<<zDisplacement/*<<'\t'<<num*/<<endl;

  return {yDisplacement, zDisplacement};
}



bool PathGenerator::checkScatteredTvec(Vec3d vec, int k, char checkVar)
{
  if (!setPointSet)
    setSetPoint(adLink.imageWidth/2, verticalImageCenterRatio*(adLink.imageHeight));

  // cout << k <<'\t' << mix <<'\t' <<miy <<'\t' <<mix + bestBoxList[k][0] <<'\t'<< \
  // miy + bestBoxList[k][1] <<'\t' << setPoint.x <<'\t' <<setPoint.y<<'\t' \
  // << vec[0]<<'\t' << vec[1]<<'\t' << vec[2] <<'\t' << vec[3] <<'\n';

  // cout<<adLink.boxList[k][0]<<'\t'<<setPoint.x <<'\t'<< vec[0] <<'\t'<< adLink.boxList[k][1]<<'\t'<< setPoint.y<<'\t'<< vec[1]<<endl;
  if (checkVar == 'y')
    return !(((mix + bestBoxList[k][0]) < setPoint.x) == (vec[0] > 0));
  else if (checkVar == 'z')
    return !(((miy + bestBoxList[k][1]) < setPoint.y) == (vec[1] < 0));
  // return false;
}



void PathGenerator::setSetPoint(int x, int y)
{
  setPoint = Point2f(x, y);
  setPointSet = true;
}



void PathGenerator::updateROI()
{
  // cout<<bestBoxList.size()<<'\n';
  vector<Point2f> roiCorners;
  int maxx = -1, minx = 1e6, maxy = -1, miny = 1e6;
  // cout << "updateROI" << '\n';

  for (int k = 0; k < bestBoxList.size(); k++)
  {
    // cout<<bestBoxList[k].size()<<'\n';

    for (int p = 0; p < bestBoxList[k].size(); p++)
    {
      // cout<<bestBoxList[k][p]<<'\t';
      if (maxx < bestBoxList[k][0])
          maxx = bestBoxList[k][0];
      if (maxy < bestBoxList[k][1])
          maxy = bestBoxList[k][1];
      if (minx > bestBoxList[k][0])
          minx = bestBoxList[k][0];
      if (miny > bestBoxList[k][1])
          miny = bestBoxList[k][1];
    }
    // std::cout << '\n';
  }
  // int mix, miy;
  if (mix==0 && miy==0){
    mix = ((minx-roiOffset)>0?(minx-roiOffset):0);
    miy = ((miny-roiOffset)>0?(miny-roiOffset):0);
  }
  else{
    mix = ((mix+minx)<roiOffset?0:mix+minx-roiOffset);
    miy = ((miy+miny)<roiOffset?0:miy+miny-roiOffset);
  }
  int w = maxx - minx + (2*roiOffset), h = maxy - miny + (2*roiOffset);
  // cout<<mix<<'\t'<<miy<<'\t'<<w<<'\t'<<h<<endl;
  adLink.roi = Rect(mix, miy, w, h);
  adLink.roiSet = true;
}
// vector<int> PathGenerator::getCellCentroid()
// {
//   vector<int> output;
//
//   // cout<<bestIds.size()<<endl;
//   // cout<<adLink.selectedCorners.size()<<endl;
//   for (int k = 0; k<bestIds.size(); k++)
//   {
//     for (int p = 0; p<bestIds[k].size(); p++)
//     {
//       cout<<bestIds[k][p]<<'\t';
//     }
//     cout<<endl;
//   }
//   cout<<"-------------------\n";
//   for (int k = 0; k<bestIds.size(); k++)
//   {
//     for (int p = 0; p<bestIds[k].size(); p++)
//     {
//       cout<<adLink.selectedCorners[k][p]<<'\t';
//     }
//     cout<<endl;
//   }
//   return output;
// }
