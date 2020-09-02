#include "../includes/link.h"

Link::Link() {
}

Link::Link(long id) {
  this->id_ = id;
}

Link::~Link() {
}

vector<Node*> Link::getNodes() {
  return this->nodes_;
}

void Link::addNodes(Node *n1, Node *n2) {
  this->nodes_.push_back(n1);
  this->nodes_.push_back(n2);
}

float Link::getDistance() {
  return this->distance_;
}

long Link::getId() {
  return this->id_;
}

float Link::getMahalanobisDistance() {
  return this->mahalanobisDistance_;
}
float Link::getEuclideanDistance() {
  return this->euclideanDistance_;
}
void Link::setDistance(string type) {
  if (type == "E") {
    this->distance_ = this->euclideanDistance_;
  } else if (type == "M") {
    this->distance_ = this->mahalanobisDistance_;
  }
}

void Link::setMahalanobisDistance(float distance) {
  this->distance_ = distance;
  this->mahalanobisDistance_ = distance;
}

void Link::setEuclideanDistance(float distance) {
  this->distance_ = distance;
  this->euclideanDistance_ = distance;
}
