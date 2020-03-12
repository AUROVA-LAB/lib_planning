#ifndef INCLUDES_UTIL_H_
#define INCLUDES_UTIL_H_

#include <vector>
#include <iostream>

#include "./LatLongToUTM.h"
using namespace std;

class Util {
private:
public:
	virtual ~Util();
	Util();
	vector<vector<double> > static newMatrix();
	vector<double> static newVector();
	vector<double> static newVector(double x, double y, double z, double yaw);
	vector<double> static LLToUTM(double lat, double lon);

};

#endif
