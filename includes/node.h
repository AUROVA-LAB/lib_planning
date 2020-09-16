#ifndef INCLUDES_NODE_H_
#define INCLUDES_NODE_H_

#include <eigen3/Eigen/Cholesky>
#include <eigen3/Eigen/Dense>
#include <bits/stdc++.h>
#include "../includes/link.h"
#include "../includes/position.h"
#include <string>
#include <vector>
using namespace std;
using namespace Eigen;
class Node: public Position
{

public:

  vector<Link*> links_;
  /**
   * Used in the dijsktra and A* algorithm to determine the distance to the node
   */
  double distance_;
  /**
   * Used in the dijsktra and A* algorithm to determine if it has been evaluated
   */
  bool seen_;

  /**
   * Used in the A* algorithm to determine the distance to the final node
   */
  double h_;

  /**
   * Node identifier
   */
  long id_;

  /**
   * If the node is near an obstacle, add an additional cost
   */
  double cost_;

  /**
   * Coordinates expressed in the vector of the eigen library
   */
  Vector4d coordinatesEigen_;
  /**
   * Coordinates expressed in the matrix of the eigen library
   */
  Matrix4d covarianceMatrixEigen_;

  Node();
  Node(Position pos);
  Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix);
  Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix,
      double cost);
  Node(long id, vector<double> coordinates,
      vector<vector<double> > covarianceMatrix);

  ~Node();

  void initEigenValues(vector<double> v, vector<vector<double> > m);

  /**
   * Set a link to the node
   */
  void addLink(Link &link);

  vector<Link*> getLinks();
  bool operator==(Node &node) const;
  bool equals(Node node);
  bool equalCoordinatesXYZ(vector<double> coordinates);
  long getId();
  void setId(long id);
  Vector4d getEigenCoordinates();
  Matrix4d getEigenMatrix();

  string toString();

  /**
   * Calculate the euclidean distance
   */
  double calculateEuclideanDistance(Node node);

  /**
   * Calculate the mahalanobis distance
   */
  double calculateMahalanobisDistance(Node node);
};

#endif
