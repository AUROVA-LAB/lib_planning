#ifndef INCLUDES_UTIL_H_
#define INCLUDES_UTIL_H_

#include "../includes/latlong_utm.h"
#include <vector>
#include <iostream>

using namespace std;

class Util {
private:
public:
  virtual ~Util();
  Util();

  /**
   * Possible algorithms
   */
  enum Algorithm
  {
    AStar,
    Dijkstra
  };

  static Algorithm algorithm; // this is legal

  /**
   * Create a 4x4 matrix
   */
  vector<vector<double> > static newMatrix();

  /**
   * Create a vector of length 4
   */
  vector<double> static newVector();

  /**
   * Create a 4x4 matrix
   */
  vector<double> static newVector(double x, double y, double z, double yaw);

  /**
   * Transform from long lat to UTM
   */
  vector<double> static LLToUTM(double lat, double lon);

};

#endif
