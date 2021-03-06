#ifndef INCLUDES_PLANNING_GRAPH_H_
#define INCLUDES_PLANNING_GRAPH_H_

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <bits/stdc++.h>
#include "link.h"
#include "node.h"
#include "util.h"
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

class PlanningGraph
{
public:

  /**
   * The algorithm used
   */
  Util::Algorithm typeAlgortihm_;

  /**
   * Nodes saved on the map
   */
  vector<Node> nodes_;

  /**
   * Links between nodes
   */
  vector<Link> links_;

  /**
   * Final goal
   */
  Position finalGoal_;

  /**
   * Distance of proximity of the vehicle to the node
   */
  double radiusVehicle_;

  /**
   * Minimum distance to insert a node
   */
  double minimumRadiusNodes_;

  /**
   * Minimum orientation to declare a position as node
   */
  double minimumDegreesNodes_;

  /**
   * Set the type of distance calculated (Euclidean "E",Mahalanobis "M")
   */
  Util::Distances typeDistance_;

  /**
   * Ìf it has been in the closest node
   */
  bool lastNodePassed_;

  /**
   * The last node visited
   */
  Node lastNodeVisited_;

  /**
   * First node of the graph to visit
   */
  Node firstNodeGraph_;

  /**
   * Last node of the graph to visit before going to the final goal
   */
  Node lastNodeGraph_;

  /**
   * The best path among all options
   */
  vector<Node> bestPath_;

  /**
   * To initialize a default vector
   */
  vector<double> newVector();

  /**
   * Get the closest node from a position
   */
  Node getCloserNode(Position pos);

  /**
   * Get the XY sense using three nodes
   */
  double calculateSense(Node pos1, Node pos2, Node pos3);

  /**
   * Calculate a distance between a node and a position. This distance
   * can be Mahalanabis or Euclidean
   */
  double getDistanceNodePosition(Position pos, Node node);

  /**
   * Get the two closest nodes from a position
   */
  vector<Node> getCloserNodes(Position pos);

  /**
   * Calculate which will be the next best pose
   */
  Node evaluateNextNode(Position initPos);

  /**
   * Check if exist link between nodes
   */
  bool existLinkBetweenNodes(Node node1, Node node2);

public:

  /**
   * Calculate the distance to the goal using Dijkstra
   */
  double calculateDijkstra(Node initNode, Node endNode, vector<Node> &bestpath);
  /**
   * Calculate the distance to the goal using A*
   */
  double calculateAStar(Node initNode, Node endNode, vector<Node> &bestpath);

  /**
   * Returns the heuristic used in the A * algorithm
   */
  double calculateAStarHDistance(Node initNode, Node endNode);

//public:

  PlanningGraph();
  ~PlanningGraph();

  /**
   * Add a new node
   */
  void addNode(Node n);

  /**
   * Add a new node
   */
  void addNode(vector<double> coordinates, vector<vector<double> > matrix);

  /**
   * Add a new node
   */
  void addNode(double x, double y, double z, double cost);

  /**
   * Add a new node
   */
  void addNode(long id, vector<double> coordinates,
      vector<vector<double> > matrix);

  /**
   * Create a link to join nodes
   */
  void addLink();

  /**
   * Set link between nodes using ids
   */
  void addLinkBetweenNodesById(long id1, long id2);
  /**
   * Set link between nodes using poses
   */
  void addLinkBetweenNodes(double x1, double y1, double z1, double x2,
      double y2, double z2);
  /**
   * Set link between nodes using poses
   */
  void addLinkBetweenNodes(vector<double> coordinates1,
      vector<double> coordinates2);

  /**
   * Set the final goal
   */
  void setfinalGoal(Position finalGoal);
  /**
   * Set if is used Mahalanobis or Euclidean distance
   */
  void setDistances(Util::Distances type);

  double getRadiusVehicle();
  void setRadiusVehicle(double radius);
  double getMinimumRadiusNodes();
  void setMinimumRadiusNodes(double radius);
  double getMinimumDegreesNodes();
  void setMinimumDegreesNodes(double degrees);

  /**
   * Returns all nodes
   */
  vector<Node> getNodes();

  /**
   * Returns all nodes pointers
   */
  vector<Node*> getNodesPointers();

  /**
   * Returns all links
   */
  vector<Link> getLinks();

  /**
   * Returns the next calculated node
   */
  Node getNextNode(Position initPos);

  /**
   * Returns the calculated path
   */
  vector<Node> getPathNodes(Position initPos);

  /**
   * Returns the number of links
   */
  int getNumberLinks();

  /**
   * Returns the number of nodes
   */
  int getNumberNodes();

  /**
   * Set algorithm
   */
  void setAlgorithm(Util::Algorithm algorithm);


  /**
   * Returns only the path nodes
   */
  vector<Node> bestPathNodes(Position initPos,vector<Node> allNodes);

};

#endif
