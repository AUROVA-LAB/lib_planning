#include "../includes/position.h"

Position::Position() {

}

Position::Position(const Position &p) {
  this->coordinates_ = p.coordinates_;
  this->covarianceMatrix_ = p.covarianceMatrix_;
}

Position::Position(vector<double> coordinates,
    vector<vector<double> > covarianceMatrix) {
  this->coordinates_ = coordinates;
  this->covarianceMatrix_ = covarianceMatrix;
}

Position::~Position() {
}

double Position::getX() const {
  return coordinates_[0];
}

double Position::getY() const {
  return coordinates_[1];
}

double Position::getZ() const {
  return coordinates_[2];
}

double Position::getYaw() const {
  return coordinates_[3];
}

void Position::setX(double x) {
  this->coordinates_[0] = x;
}
void Position::setY(double y) {
  this->coordinates_[1] = y;
}
void Position::setZ(double z) {
  this->coordinates_[2] = z;
}
void Position::setYaw(double yaw) {
  this->coordinates_[3] = yaw;
}

vector<double> Position::getCoordinates() {
  return this->coordinates_;
}
vector<vector<double> > Position::getMatrix() {
  return this->covarianceMatrix_;
}

bool Position::operator==(Position &pos) const {
  if (this->getX() == pos.getX() && this->getY() == pos.getY()
      && this->getZ() == pos.getZ())
    return true;
  else
    return false;
}

bool Position::equals(Position pos) {
  if (this->getX() == pos.getX() && this->getY() == pos.getY()
      && this->getZ() == pos.getZ())
    return true;
  else
    return false;
}

bool Position::isValid() {
  return this->coordinates_.size() == 4;
}

