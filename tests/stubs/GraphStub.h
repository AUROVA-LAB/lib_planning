#ifndef GRAPHSTUB_H_
#define GRAPHSTUB_H_

#include "../../includes/Graph.h"

#include <vector>
#include <iostream>

using namespace std;

class GraphStub : public Graph {
public:

	GraphStub(string url, vector<vector<double> > matrix,string typeDistance, double radiusDistance);

	Planning_Graph getPlanningGraph();

};

#endif
