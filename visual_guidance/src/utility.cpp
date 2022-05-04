#include "utility.h"



float utility::calculate3dDistance(Vec3d tvec)
{
  return sqrt((tvec[0]*tvec[0])+(tvec[1]*tvec[1])+(tvec[2]*tvec[2]));
}



float utility::calculate2dDistance(Point2f p1, Point2f p2)
{
  return sqrt(((p1.x-p2.x)*(p1.x-p2.x))+((p1.y-p2.y)*(p1.y-p2.y)));
}
