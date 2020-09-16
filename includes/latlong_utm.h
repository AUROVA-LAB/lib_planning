//LatLong- UTM conversion..h
//definitions for lat/long to UTM and UTM to lat/lng conversions
#include <string.h>

#ifndef LATLONGCONV
#define LATLONGCONV

class Ellipsoid
{
public:
  Ellipsoid()
  {
    char array[1];
    this->id = -1;
    this->ellipsoidName = array;
    this->EquatorialRadius = -1;
    this->eccentricitySquared = -1;
  }
  ;
  Ellipsoid(int Id, char *name, double radius, double ecc)
  {
    this->id = Id;
    this->ellipsoidName = name;
    this->EquatorialRadius = radius;
    this->eccentricitySquared = ecc;
  }

  int id;
  char *ellipsoidName;
  double EquatorialRadius;
  double eccentricitySquared;

  void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long,
      double &UTMNorthing, double &UTMEasting, char *UTMZone);
  void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing,
      const double UTMEasting, const char *UTMZone, double &Lat, double &Long);
  char UTMLetterDesignator(double Lat);
  void LLtoSwissGrid(const double Lat, const double Long, double &SwissNorthing,
      double &SwissEasting);
  void SwissGridtoLL(const double SwissNorthing, const double SwissEasting,
      double &Lat, double &Long);

};

#endif
