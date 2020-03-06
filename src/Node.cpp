#include "../includes/Node.h"

Node::Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix) : Position(coordinates,covarianceMatrix) {}

Node::Node(Position pos) : Position(pos){
}

Node::Node(long id,vector<double> coordinates, vector<vector<double> > covarianceMatrix) : Position(coordinates,covarianceMatrix) {
	this->id=id;

}


Node::Node() : Position(){

}

Node::~Node() {
}

vector<Link*> Node::getLinks(){
	return this->links;
}

long Node::getId(){
	return this->id;
}

void Node::addLink(Link &link) {
	this->links.push_back(&link);
}

bool Node::operator==(Node& node) const
{
	if(this->getX() == node.getX() && this->getY() == node.getY() && this->getZ() == node.getZ())
		return true;
	else return false;
}

bool Node::equals(Node node)
{
	if(this->getX() == node.getX() && this->getY() == node.getY() && this->getZ() == node.getZ())
		return true;
	else return false;
}

bool Node::equalCoordinatesXYZ(vector<double> coordinates){
	if(this->getX() == coordinates[0] && this->getY() == coordinates[1] && this->getZ() == coordinates[2])
		return true;
	else return false;
}

double Node::calculateEuclideanDistance(Node node){
	double x = this->getX() - node.getX();
	double y = this->getY() - node.getY();
	double z = this->getZ() - node.getZ();
	return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)* 1.0);
}

double Node::calculateMahalanobisDistance(Node node){

	Vector4d vectorX;
	Vector4d vectorG;
	Vector4d vectorZ;
	Matrix4d matrixP;
	Matrix4d matrixQ;
	Matrix4d matrixZ;

	for (int i = 0; i < node.getCoordinates().size()-1; i++)
	{
		vectorX(i)= node.getCoordinates()[i];
	}
	for (int i = 0; i < this->getCoordinates().size()-1; i++)
	{
		vectorG(i)= this->getCoordinates()[i];

	}
	for (int i = 0; i < node.getMatrix().size(); i++)
	{
		for (int j = 0; j < node.getMatrix()[i].size(); j++){
			matrixP(i,j)= node.getMatrix()[i][j];
			matrixQ(i,j)= this->getMatrix()[i][j];
		}
	}

	vectorZ = vectorG-vectorX;
	matrixZ = matrixP*matrixQ*matrixP.transpose();
	double aux = (vectorZ.transpose()*matrixZ.inverse()*vectorZ).value();
	double mahalanobisDistance = sqrt(aux);

	return mahalanobisDistance;
}


string Node::toString(){
	std::ostringstream x,y,z,yaw,degree;
	x <<this->getX();
	y <<this->getY();
	z <<this->getZ();
	yaw <<this->getYaw();
	string text = "("+x.str()+","+y.str()+","+z.str()+","+yaw.str()+")  ";
	return text;
}
