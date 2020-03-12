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

  vector<Node*> nodes;
  //vector<Point> points;
  float distance;
  float mahalanobisDistance;
  float euclideanDistance;
  long id;

  Link();
  Link(long id);
  ~Link();

  //vector<Point> getPoints();
  vector<Node*> getNodes();
  //void addPoint(Point p);
  void addNodes(Node *n1, Node *n2);
  float getDistance();
  long getId();
  void setDistance(string type);
  float getMahalanobisDistance();
  void setMahalanobisDistance(float distance);
  float getEuclideanDistance();
  void setEuclideanDistance(float distance);

};

#endif
