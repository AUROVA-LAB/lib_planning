#include "../includes/util.h"

Util::~Util()
{
}

Util::Util()
{
}

vector<double> Util::newVector()
{
  vector<double> newVector;
  newVector.push_back(0);
  newVector.push_back(0);
  newVector.push_back(0);
  newVector.push_back(0);
  return newVector;
}

vector<double> Util::newVector(double x, double y, double z, double yaw)
{
  vector<double> newVector;
  newVector.push_back(x);
  newVector.push_back(y);
  newVector.push_back(z);
  newVector.push_back(yaw);
  return newVector;
}

vector<vector<double> > Util::newMatrix()
{
  vector<vector<double> > newVector(4);
  for (int i = 0; i < 4; i++)
  {
    if (i == 0)
    {
      newVector[i].push_back(1);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
    } else if (i == 1)
    {
      newVector[i].push_back(0);
      newVector[i].push_back(1);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
    } else if (i == 2)
    {
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(100000000000);
      newVector[i].push_back(0);
    } else if (i == 3)
    {
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(30);
    }
  }

  return newVector;
}

vector<double> Util::LLToUTM(double lat, double lon)
{
  Ellipsoid a;
  double UtmY;
  double UtmX;
  char UTMZone[30];
  vector<double> coordinates;
  int RefEllipsoid = 23; //WGS-84. See list with file "LatLong- UTM conversion.cpp" for id numbers
  a.LLtoUTM(RefEllipsoid, lat, lon, UtmY, UtmX, UTMZone);
  coordinates.push_back(UtmX);
  coordinates.push_back(UtmY);
  coordinates.push_back(0);
  coordinates.push_back(0);
  return coordinates;
}

