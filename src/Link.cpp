#include "../includes/Link.h"

Link::Link() {
}

Link::Link(long id) {
  this->id = id;
}

Link::~Link() {
}

vector<Node*> Link::getNodes() {
  return nodes;
}

void Link::addNodes(Node *n1, Node *n2) {
  nodes.push_back(n1);
  nodes.push_back(n2);
}

float Link::getDistance() {
  return this->distance;
}

long Link::getId() {
  return this->id;
}

float Link::getMahalanobisDistance() {
  return this->mahalanobisDistance;
}
float Link::getEuclideanDistance() {
  return this->euclideanDistance;
}
void Link::setDistance(string type) {
  if (type == "E") {
    this->distance = this->euclideanDistance;
  } else if (type == "M") {
    this->distance = this->mahalanobisDistance;
  }
}

void Link::setMahalanobisDistance(float distance) {
  this->distance = distance;
  this->mahalanobisDistance = distance;
}

void Link::setEuclideanDistance(float distance) {
  this->distance = distance;
  this->euclideanDistance = distance;
}
