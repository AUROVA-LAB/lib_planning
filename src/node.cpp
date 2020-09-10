#include "../includes/node.h"

Node::Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix) : Position(
    coordinates, covarianceMatrix)
{
  this->id_ = -1;
  this->cost_ = 0;
  this->distance_ = 0;
  this->seen_ = false;
  this->h_ = 0;
}

Node::Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix,
    double cost) : Position(coordinates, covarianceMatrix)
{
  this->id_ = -1;
  this->cost_ = cost;
  this->distance_ = 0;
  this->seen_ = false;
  this->h_ = 0;
}

Node::Node(Position pos) : Position(pos)
{
  this->id_ = -1;
  this->cost_ = 0;
  this->distance_ = 0;
  this->seen_ = false;
  this->h_ = 0;
}

Node::Node(long id, vector<double> coordinates,
    vector<vector<double> > covarianceMatrix) : Position(coordinates,
    covarianceMatrix)
{
  this->id_ = id;
  this->cost_ = 0;
  this->distance_ = 0;
  this->seen_ = false;
  this->h_ = 0;
}

Node::Node() : Position()
{
  this->id_ = -1;
  this->cost_ = 0;
  this->distance_ = 0;
  this->seen_ = false;
  this->h_ = 0;
}

Node::~Node()
{
}

vector<Link*> Node::getLinks()
{
  return this->links_;
}

long Node::getId()
{
  return this->id_;
}

void Node::setId(long id)
{
  this->id_ = id;
}

void Node::addLink(Link &link)
{
  this->links_.push_back(&link);
}

bool Node::operator==(Node &node) const
{
  if (this->getX() == node.getX() && this->getY() == node.getY()
      && this->getZ() == node.getZ())
    return true;
  else
    return false;
}

bool Node::equals(Node node)
{
  if (this->getX() == node.getX() && this->getY() == node.getY()
      && this->getZ() == node.getZ())
    return true;
  else
    return false;
}

bool Node::equalCoordinatesXYZ(vector<double> coordinates)
{
  if (this->getX() == coordinates[0] && this->getY() == coordinates[1]
      && this->getZ() == coordinates[2])
    return true;
  else
    return false;
}

double Node::calculateEuclideanDistance(Node node)
{
  double x = this->getX() - node.getX();
  double y = this->getY() - node.getY();
  double z = this->getZ() - node.getZ();
  return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2) * 1.0);
}

double Node::calculateMahalanobisDistance(Node node)
{

  Vector4d vectorX;
  Vector4d vectorG;
  Vector4d vectorZ;
  Matrix4d matrixP;
  Matrix4d matrixQ;
  Matrix4d matrixZ;
  Vector4d vectorAux;


  for (unsigned int i = 0; i < node.getCoordinates().size(); i++)
  {
    vectorX(i) = node.getCoordinates()[i];
  }
  for (unsigned int i = 0; i < this->getCoordinates().size(); i++)
  {
    vectorG(i) = this->getCoordinates()[i];

  }
  for (unsigned int i = 0; i < node.getMatrix().size(); i++)
  {
    for (unsigned int j = 0; j < node.getMatrix()[i].size(); j++)
    {
      matrixP(i, j) = node.getMatrix()[i][j];
      matrixQ(i, j) = this->getMatrix()[i][j];
    }
  }

  vectorZ = vectorG - vectorX;
  matrixZ.noalias() = matrixP * matrixQ * matrixP.transpose();
  vectorAux.noalias() = matrixZ.inverse() * vectorZ;
  double aux = (vectorZ.transpose() * vectorAux).value();
  double mahalanobisDistance = sqrt(aux);

  return mahalanobisDistance;
}

string Node::toString()
{
  std::ostringstream x, y, z, yaw, degree;
  x << this->getX();
  y << this->getY();
  z << this->getZ();
  yaw << this->getYaw();
  string text = "(" + x.str() + "," + y.str() + "," + z.str() + "," + yaw.str()
      + ")  ";
  return text;
}
