
#ifndef INCLUDES_GRAPH_H_
#define INCLUDES_GRAPH_H_

#include <iostream>
#include <vector>
#include "Planning_Graph.h"
#include "Util.h"
#include <unistd.h>
#include "./rapidxml/rapidxml.hpp"
#include "./rapidxml/rapidxml_utils.hpp"
using namespace std;
using namespace rapidxml;

struct Pose{
	vector<double> coordinates;
	vector<vector<double> > matrix;
};

class Graph {
protected:
	void xmlRead(string url, vector<vector<double> > matrix);
	Planning_Graph graph;
public:
	Graph(string url, vector<vector<double> > matrix,string typeDistance, double radiusDistance);
	virtual ~Graph();
	Pose getNextPose(Pose myPosition, Pose endGoal);
	vector<vector<double> > static newMatrix(double x, double y, double z, double yaw);

};

#endif
