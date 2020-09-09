#ifndef INCLUDES_GRAPH_H_
#define INCLUDES_GRAPH_H_

#include "../includes/planning_graph.h"
#include "../includes/util.h"
#include <iostream>
#include <vector>
#include <unistd.h>
#include "./rapidxml/rapidxml.hpp"
#include "./rapidxml/rapidxml_utils.hpp"
using namespace std;
using namespace rapidxml;

/**
 * Position from coordinates and the covariance matrix
 */
struct Pose
{
  vector<double> coordinates;
  vector<vector<double> > matrix;
};

struct StNodes
{
  long id;
  vector<double> coordinates;
  vector<vector<double> > matrix;
  vector<long> nodesConnected;
};

class Graph
{
public:
  /**
   * Graph to calculate the shortest path
   */
  PlanningGraph planning_graph_;


  Graph(string typeDistance, double radiusDistance);
  Graph(string url, vector<vector<double> > matrix, string typeDistance,
      double radiusDistance);
  virtual ~Graph();

  /**
   * Loads the information from a file. The file uses Latitude Longitude coordinates.
   */
  void xmlReadLatLong(string url, vector<vector<double> > matrix);

  /**
   * Returns the next planned position
   */
  Pose getNextPose(Pose myPosition, Pose endGoal);

  /**
   * Returns the path of planned positions
   */
  vector<Pose> getPathPoses(Pose myPosition, Pose endGoal);

  /**
   * Create a covariance matrix
   */
  vector<vector<double> > static newMatrix(double x, double y, double z,
      double yaw);

  /**
   * Write a graph in a Xml file
   */
  void xmlWrite(string fileName, int precision);
  /**
   * Loads the information from a file. The file uses a UTM coordinates.
   */
  void xmlReadUTM(string url);

  /**
   * Loads the information from a struct.
   */
  void loadStructGraph(vector<StNodes> st_nodes);

  /**
   * Returns the graph as a struct
   */
  vector<StNodes> getStructGraph();

  /**
   * Sets algorithm A* to be used
   */
  void setAStarAlgorithm();

  /**
   * Sets algorithm Dijkstra to be used
   */
  void setDijkstraAlgorithm();

  PlanningGraph getPlanningGraph();

};

#endif
