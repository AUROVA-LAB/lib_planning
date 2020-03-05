#include "../includes/Position.h"

Position::Position() {

}

Position::Position(const Position &p){
	this->coordinates = p.coordinates;
	this->covarianceMatrix = p.covarianceMatrix;
}

Position::Position(vector<double> coordinates, vector<vector<double> > covarianceMatrix) {
	this->coordinates = coordinates;
	this->covarianceMatrix = covarianceMatrix;
}

Position::~Position() {
}

double Position::getX() const{
	return coordinates[0];
}

double Position::getY() const{
	return coordinates[1];
}

double Position::getZ() const{
	return coordinates[2];
}

double Position::getYaw() const{
	return coordinates[3];
}

void Position::setX(double x){
	this->coordinates[0] = x;
}
void Position::setY(double y){
	this->coordinates[1] = y;
}
void Position::setZ(double z){
	this->coordinates[2] = z;
}
void Position::setYaw(double yaw){
	this->coordinates[3] = yaw;
}

vector<double> Position::getCoordinates(){
	return coordinates;
}
vector<vector<double> > Position::getMatrix(){
	return covarianceMatrix;
}

bool Position::operator==(Position& pos) const
{
	if(this->getX() == pos.getX() && this->getY() == pos.getY() && this->getZ() == pos.getZ())
		return true;
	else return false;
}

bool Position::equals(Position pos)
{
	if(this->getX() == pos.getX() && this->getY() == pos.getY() && this->getZ() == pos.getZ())
		return true;
	else return false;
}

bool Position::isValid() {
	return coordinates.size()==4;
}

