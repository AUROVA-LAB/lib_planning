#include "./GraphStub.h"


GraphStub::GraphStub(string url, vector<vector<double> > matrix,string typeDistance, double radiusDistance) : Graph(url,matrix,typeDistance,radiusDistance){}

Planning_Graph GraphStub::getPlanningGraph(){ return graph; }

