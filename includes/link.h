#ifndef INCLUDES_LINK_H_
#define INCLUDES_LINK_H_
#include <iostream>
#include <vector>
#include <bits/stdc++.h>

using namespace std;

class Node;

class Link {
private:

public:

  /**
   * Linking nodes. There are two nodes.
   */
  vector<Node*> nodes_;

  /**
   * Distance between nodes
   */
  float distance_;

  /**
   * Mahalanobis distance between nodes
   */
  float mahalanobisDistance_;

  /**
   * Euclidean distance between nodes
   */
  float euclideanDistance_;

  /**
   * Link identifier
   */
  long id_;

  Link();
  Link(long id);
  ~Link();

  /**
   * Connect two nodes
   */
  void addNodes(Node *n1, Node *n2);
  float getDistance();
  long getId();
  void setDistance(string type);
  float getMahalanobisDistance();
  void setMahalanobisDistance(float distance);
  float getEuclideanDistance();
  void setEuclideanDistance(float distance);
  vector<Node*> getNodes();

};

#endif
