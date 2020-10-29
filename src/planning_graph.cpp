#include "../includes/planning_graph.h"

PlanningGraph::PlanningGraph()
{
  this->nodes_.reserve(100000);
  this->links_.reserve(100000);
  this->finalGoal_ = *(new Position(Util::newVector(), Util::newMatrix()));
  this->radiusVehicle_ = 0.5;
  this->minimumRadiusNodes_ = 1;
  this->minimumDegreesNodes_ = 45;
  this->typeDistance_ = Util::Euclidean;
  this->lastNodePassed_ = false;
  this->typeAlgortihm_ = Util::Dijkstra;
}

PlanningGraph::~PlanningGraph()
{
}

Node PlanningGraph::evaluateNextNode(Position initPos)
{
  Node nextNode;
  vector<Node> initNodes;
  vector<Node> bestpath;
  Node lastNodePath;
  double distance = DBL_MAX, minDistance = DBL_MAX;
  Node finalGoalNode(this->finalGoal_);
  lastNodePath = getCloserNode(this->finalGoal_);
  initNodes = getCloserNodes(initPos);

  nextNode = initNodes[0];

  // Calculate the distance from diferents initial positions
  for (unsigned int i = 0; i < initNodes.size(); i++)
  {
    distance = getDistanceNodePosition(initPos, initNodes[i]);
    if (distance > this->radiusVehicle_)
    {
      distance += getDistanceNodePosition(this->finalGoal_, lastNodePath);
      bestpath.clear();

      if (this->typeAlgortihm_ == Util::AStar)
      {
        distance += calculateAStar(initNodes[i], lastNodePath, bestpath);

      } else if (this->typeAlgortihm_ == Util::Dijkstra)
      {
        distance += calculateDijkstra(initNodes[i], lastNodePath, bestpath);
      }
      if (distance < minDistance)
      {
        nextNode = initNodes[i];
        minDistance = distance;
        this->bestPath_ = bestpath;

        // Set first and last node from the graph to visit
        this->firstNodeGraph_ = nextNode;
        this->lastNodeGraph_ = lastNodePath;
      }
    }
  }

  // Set the values with the best path
  if (this->typeAlgortihm_ == Util::AStar)
  {
    calculateAStar(this->firstNodeGraph_, this->lastNodeGraph_, bestpath);

  } else if (this->typeAlgortihm_ == Util::Dijkstra)
  {
    calculateDijkstra(this->firstNodeGraph_, this->lastNodeGraph_, bestpath);
  }

  // Check if the closest position is the goal or a node from the graph
  Node endNode(this->finalGoal_);
  double distanceToEndPos = getDistanceNodePosition(initPos, endNode);
  double endNodeDistance = getDistanceNodePosition(initPos, lastNodePath);

  bool aux = nextNode.equals(lastNodePath);
  bool aux2 = endNodeDistance >= distanceToEndPos;

  // If the lastNode has been reached it moves on to the final goal.
  if (endNodeDistance < this->radiusVehicle_ || (aux2 && aux))
  {
    this->lastNodePassed_ = true;
  }

  // If the last node has already been reached it will go to the goal
  if (this->lastNodePassed_)
  {
    nextNode = endNode;
  }
  return nextNode;
}

bool PlanningGraph::existLinkBetweenNodes(Node node1, Node node2)
{
  bool exist = false;
  Node n1, n2;
  // Find the two nodes of the graph
  for (unsigned int i = 0; i < this->nodes_.size(); i++)
  {
    if (node1.equals(this->nodes_[i]))
    {
      n1 = this->nodes_[i];
    } else if (node2.equals(this->nodes_[i]))
    {
      n2 = this->nodes_[i];
    }
  }
  // Check if exists a link
  for (unsigned int i = 0; i < n1.getLinks().size(); i++)
  {
    if (n1.getLinks()[i]->nodes_[0]->equals(n2)
        || n1.getLinks()[i]->nodes_[1]->equals(n2))
    {
      exist = true;
    }
  }
  return exist;
}

double PlanningGraph::calculateSense(Node pos1, Node pos2, Node pos3)
{
  vector<double> degrees;

  vector<double> vector1;
  vector<double> vector2;

  vector1.push_back(pos2.getX() - pos1.getX());
  vector1.push_back(pos2.getY() - pos1.getY());
  vector2.push_back(pos3.getX() - pos2.getX());
  vector2.push_back(pos3.getY() - pos2.getY());

  double degreeXY = atan2(vector2[1], vector2[0])
      - atan2(vector1[1], vector1[0]);

  degreeXY = degreeXY * (180.0 / 3.141592653589793238463);

  if (degreeXY < -180)
  {
    degreeXY += 360;
  } else if (degreeXY > 180)
  {
    degreeXY -= 360;
  }

  return degreeXY;
}

double PlanningGraph::calculateDijkstra(Node initNode, Node endNode,
    vector<Node> &bestpath)
{
  bool allNodesSeen = false;
  int selectedNode = 0;
  Node nextNodeToEvaluate;
  vector<Node*> path;
  double distanceBetweenNodes = 0;
  double newDistance = 0;
  Node *childNode;

  //Initialize values
  for (unsigned int i = 0; i < this->nodes_.size(); i++)
  {
    path.push_back(&this->nodes_[i]);
    path[i]->seen_ = false;
    if (this->nodes_[i] == initNode)
    {
      path[i]->distance_ = 0;
      selectedNode = i;
      nextNodeToEvaluate = this->nodes_[i];
    } else
    {
      path[i]->distance_ = DBL_MAX;
      path[i]->seen_ = false;
    }
  }

  // Dijsktra's algorithm
  while (!allNodesSeen)
  {
    path[selectedNode]->seen_ = true;

    // Assign distance to connected nodes
    for (unsigned int i = 0; i < nextNodeToEvaluate.getLinks().size(); i++)
    {
      // Calculate the distances to reach to the child node
      distanceBetweenNodes = nextNodeToEvaluate.getLinks()[i]->distance_;
      newDistance = distanceBetweenNodes + nextNodeToEvaluate.distance_;

      // Check which it is the node connected
      if (*nextNodeToEvaluate.getLinks()[i]->nodes_[0] == nextNodeToEvaluate)
      {
        childNode = nextNodeToEvaluate.getLinks()[i]->nodes_[1];
      } else
      {
        childNode = nextNodeToEvaluate.getLinks()[i]->nodes_[0];
      }
      // If the node is near an obstacle, add an additional cost
      double cost = childNode->cost_;
      newDistance += cost;

      if (newDistance < childNode->distance_)
      {
        childNode->distance_ = newDistance;
      }
    }

    allNodesSeen = true;
    double newDistance = DBL_MAX;
    // Get next Node to evaluate
    for (unsigned int i = 0; i < this->nodes_.size(); i++)
    {
      if (path[i]->seen_ == false && path[i]->distance_ < newDistance)
      {
        allNodesSeen = false;
        selectedNode = i;
        nextNodeToEvaluate = *path[i];
        newDistance = path[i]->distance_;
      }
    }

  }

  double totalDistance = -1;
  // Get the distance to the final Node
  for (unsigned int i = 0; i < path.size(); i++)
  {
    if (endNode == *path[i])
    {
      totalDistance = path[i]->distance_;
    }
    bestpath.push_back(*path[i]);
  }

  return totalDistance;
}

double PlanningGraph::calculateAStarHDistance(Node initNode, Node endNode)
{
  // Use the distance between two nodes to calculate H
  return getDistanceNodePosition(initNode, endNode);
}

double PlanningGraph::calculateAStar(Node initNode, Node endNode,
    vector<Node> &bestpath)
{
  Node *currentNode, *childNode;
  vector<Node*> openList;

  //Initialize values
  for (unsigned int i = 0; i < this->nodes_.size(); i++)
  {
    if (this->nodes_[i] == initNode)
    {
      this->nodes_[i].distance_ = 0;
      this->nodes_[i].seen_ = true;
      this->nodes_[i].distance_ = 0;
      Position childPos(this->nodes_[i].getCoordinates(),
          this->nodes_[i].getMatrix());
      this->nodes_[i].h_ = calculateAStarHDistance(childPos, endNode);
      openList.push_back(&this->nodes_[i]);
    } else
    {
      this->nodes_[i].distance_ = -1;
      this->nodes_[i].h_ = DBL_MAX;
      this->nodes_[i].seen_ = false;
    }
  }

  // A* algorithm
  while (!openList.empty())
  {

    int pos = 0;
    currentNode = openList[pos];
    // Get the next node to evaluate
    for (unsigned int i = 1; i < openList.size(); i++)
    {
      double f1 = openList[i]->distance_ + openList[i]->h_ + openList[i]->cost_;
      double f2 = currentNode->distance_ + currentNode->h_ + currentNode->cost_;
      if (f1 < f2)
      {
        currentNode = openList[i];
        pos = i;
      }
    }
    bestpath.push_back(*currentNode);
    openList.erase(openList.begin() + pos);

    // If the end node has been reached, the calculation ends
    if (currentNode->equals(endNode))
    {
      return currentNode->distance_;
    }

    // Select the childs of the current node
    for (unsigned int i = 0; i < currentNode->getLinks().size(); i++)
    {
      // Check which it is the node connected
      if (currentNode->getLinks()[i]->nodes_[0]->equals(*currentNode))
      {
        childNode = currentNode->getLinks()[i]->nodes_[1];
      } else
      {
        childNode = currentNode->getLinks()[i]->nodes_[0];
      }

      // If the node it has not been evaluated
      if (!childNode->seen_)
      {
        // Set the distances to use A* to each child node
        double distanceBetweenNodes = currentNode->getLinks()[i]->distance_;
        double newDistance = distanceBetweenNodes + currentNode->distance_;

        // Check if the distances has been set
        if (childNode->distance_ >= 0)
        {
          if (newDistance < childNode->distance_)
          {
            childNode->distance_ = newDistance;
          }
        } else
        {
          Position childPos(childNode->getCoordinates(),
              childNode->getMatrix());
          childNode->h_ = calculateAStarHDistance(childPos, endNode);
          childNode->distance_ = newDistance;
          openList.push_back(childNode);
        }
      }
    }
    currentNode->seen_ = true;
  }
  // The final node has not been reached
  return -1;
}

Node PlanningGraph::getCloserNode(Position pos)
{

  int n1;
  if (this->nodes_.size() > 0)
  {
    double distance = getDistanceNodePosition(pos, this->nodes_[0]);
    n1 = 0;

    for (unsigned int i = 1; i < this->nodes_.size(); i++)
    {
      double newDistance = getDistanceNodePosition(pos, this->nodes_[i]);
      if (newDistance < distance)
      {
        n1 = i;
        distance = newDistance;
      }
    }
  }
  return this->nodes_[n1];
}

vector<Node> PlanningGraph::getCloserNodes(Position pos)
{
  bool insideNode = false;
  Node n;
  vector<Node> connectedNodes;

  Node n1, n2;
  double distance1 = getDistanceNodePosition(pos, this->nodes_[0]);
  double distance2 = getDistanceNodePosition(pos, this->nodes_[1]);

  // Initialize the values
  if (distance1 < distance2)
  {
    n1 = this->nodes_[0];
    n2 = this->nodes_[1];
  } else
  {
    n1 = this->nodes_[1];
    n2 = this->nodes_[0];
    double aux = distance2;
    distance2 = distance1;
    distance1 = aux;
  }

  if (distance1 < this->radiusVehicle_)
  {
    insideNode = true;
    n = n1;
  }

  unsigned int i = 2;

  // Search all nodes and return the two closest
  while (i < this->nodes_.size() && !insideNode)
  {
    double newDistance = getDistanceNodePosition(pos, this->nodes_[i]);

    if (newDistance < distance1)
    {
      n2 = n1;
      distance2 = distance1;
      n1 = this->nodes_[i];
      distance1 = newDistance;
      // Check if the robot is inside a node
      if (newDistance < this->radiusVehicle_)
      {
        insideNode = true;
        n = this->nodes_[i];
      }
    } else if (newDistance < distance2)
    {
      n2 = this->nodes_[i];
      distance2 = newDistance;
    }
    i++;
  }

  // If the robot is inside the radius of the node, find the connected nodes
  if (insideNode)
  {
    for (unsigned int j = 0; j < n.links_.size(); j++)
    {
      if (*n.links_[j]->nodes_[0] == n)
      {
        connectedNodes.push_back(*n.links_[j]->nodes_[1]);
      } else
      {
        connectedNodes.push_back(*n.links_[j]->nodes_[0]);
      }
    }
  } else
  {
    connectedNodes.push_back(n1);
    connectedNodes.push_back(n2);
  }
  return connectedNodes;
}

Node PlanningGraph::getNextNode(Position initPos)
{
  Node nextNode;
  Node actualPosition(initPos);
  if (this->nodes_.size() > 0)
  {
    nextNode = evaluateNextNode(initPos);
    vector<Node> bestPathOfNodes = bestPathNodes(initPos, this->bestPath_);
    nextNode.setYaw(
        calculateSense(actualPosition, nextNode, bestPathOfNodes[1]));
  } else
  {
    nextNode = this->finalGoal_;
  }
  return nextNode;
}

vector<Node> PlanningGraph::getPathNodes(Position initPos)
{
  if (this->nodes_.size() > 0)
  {
    evaluateNextNode(initPos);
  } else
  {
    this->nodes_.push_back(this->finalGoal_);
  }

  vector<Node> bestPathOfNodes;

  // Returns only the nodes of the best path
  if (this->bestPath_.size() > 1)
  {
    bestPathOfNodes = bestPathNodes(initPos, this->bestPath_);
    int tamPath = bestPathOfNodes.size();

    //Calculate sense of the firts node
    if (tamPath > 1)
    {
      bestPathOfNodes[0].setYaw(
          calculateSense(initPos, bestPathOfNodes[0], bestPathOfNodes[1]));
    }

    //Calculate sense of the nodes of the path
    if (tamPath > 2)
    {
      for (int i = 1; i < tamPath - 1; i++)
      {
        bestPathOfNodes[i].setYaw(
            calculateSense(bestPathOfNodes[i - 1], bestPathOfNodes[i],
                bestPathOfNodes[i + 1]));
      }
    }
  } else
  {
    Node end(this->finalGoal_);
    bestPathOfNodes.push_back(end);
  }

  return bestPathOfNodes;
}

vector<Node> PlanningGraph::bestPathNodes(Position initPos,
    vector<Node> allNodes)
{
  vector<Node> minPath;
  Node nextNode;
  bool finish = false;

  // Initialize the values
  Node lastNodeGoal(this->finalGoal_);
  for (unsigned int i = 0; i < allNodes.size(); i++)
  {
    if (this->lastNodeGraph_.equals(allNodes[i]))
    {
      minPath.push_back(allNodes[i]);
      nextNode = allNodes[i];
    }
  }

// Get all the nodes it has to go through starting with the last one
  while (!minPath[0].equals(this->firstNodeGraph_))
  {
    Node auxNode;
    Node minNode;

    // Find the node that gets the best path
    for (unsigned int i = 0; i < nextNode.getLinks().size(); i++)
    {
      if (nextNode.equals(*nextNode.getLinks()[i]->getNodes()[0]))
      {
        auxNode = *nextNode.getLinks()[i]->getNodes()[1];
      } else
      {
        auxNode = *nextNode.getLinks()[i]->getNodes()[0];
      }
      if (i == 0 || minNode.distance_ == -1)
      {
        minNode = auxNode;
      }
      if (auxNode.distance_ < minNode.distance_ && auxNode.distance_ >= 0)
      {
        minNode = auxNode;
      }
    }
    minPath.insert(minPath.begin(), minNode);
    nextNode = minNode;
  }
  // Add last node to the path
  if (!this->lastNodeGraph_.equals(lastNodeGoal))
  {
    minPath.push_back(lastNodeGoal);
  }

  //Skip the first node if necessary
  if (minPath.size() > 1)
  {
    double distanceBetweenNodes = minPath[1].distance_ - minPath[0].distance_;
    double distanceInitialPosition = this->getDistanceNodePosition(initPos,
        minPath[1]);

    if (distanceInitialPosition < distanceBetweenNodes)
    {
      minPath.erase(minPath.begin());
    }
  }

  return minPath;
}

double PlanningGraph::getDistanceNodePosition(Position pos, Node node)
{
  Node newNode(pos);
  double distancia = 0;
  if (this->typeDistance_ == Util::Euclidean)
  {
    distancia = node.calculateEuclideanDistance(newNode);
  } else if (this->typeDistance_ == Util::Mahalanobis)
  {
    distancia = node.calculateMahalanobisDistance(newNode);
  }
  return distancia;
}

void PlanningGraph::addLinkBetweenNodes(vector<double> coordinates1,
    vector<double> coordinates2)
{

  int pos1 = -1, pos2 = -1;
  // Find the  nodes
  for (unsigned int i = 0; i < this->nodes_.size(); i++)
  {
    if (this->nodes_[i].equalCoordinatesXYZ(coordinates1))
    {
      pos1 = i;
    }
    if (this->nodes_[i].equalCoordinatesXYZ(coordinates2))
    {
      pos2 = i;
    }
  }

  // Connect the nodes if there is no link
  if (pos1 != -1 && pos2 != -1
      && !existLinkBetweenNodes(this->nodes_[pos1], this->nodes_[pos2]))
  {
    addLink();
    Link *l1 = &this->links_[this->links_.size() - 1];
    l1->nodes_.push_back(&this->nodes_[pos1]);
    l1->nodes_.push_back(&this->nodes_[pos2]);

    this->nodes_[pos1].links_.push_back(l1);
    this->nodes_[pos2].links_.push_back(l1);

    l1->setEuclideanDistance(
        this->nodes_[pos2].calculateEuclideanDistance(this->nodes_[pos1]));
    l1->setMahalanobisDistance(
        this->nodes_[pos2].calculateMahalanobisDistance(this->nodes_[pos1]));
    l1->setDistance(this->typeDistance_);
  }
}

void PlanningGraph::addLinkBetweenNodesById(long id1, long id2)
{

  int pos1 = -1, pos2 = -1;
  // Find the  nodes
  for (unsigned int i = 0; i < this->nodes_.size(); i++)
  {
    if (this->nodes_[i].getId() == id1)
    {
      pos1 = i;
    }
    if (this->nodes_[i].getId() == id2)
    {
      pos2 = i;
    }
  }

  // Connect the nodes if there is no link
  if (pos1 != -1 && pos2 != -1
      && !existLinkBetweenNodes(this->nodes_[pos1], this->nodes_[pos2]))
  {
    addLink();
    Link *l1 = &this->links_[this->links_.size() - 1];
    l1->nodes_.push_back(&this->nodes_[pos1]);
    l1->nodes_.push_back(&this->nodes_[pos2]);

    this->nodes_[pos1].links_.push_back(l1);
    this->nodes_[pos2].links_.push_back(l1);

    l1->setEuclideanDistance(
        this->nodes_[pos2].calculateEuclideanDistance(this->nodes_[pos1]));
    l1->setMahalanobisDistance(
        this->nodes_[pos2].calculateMahalanobisDistance(this->nodes_[pos1]));
    l1->setDistance(this->typeDistance_);
  }
}

void PlanningGraph::addNode(vector<double> coordinates,
    vector<vector<double> > matrix)
{
  Node n(coordinates, matrix);
  this->nodes_.push_back(n);
}

void PlanningGraph::addNode(long id, vector<double> coordinates,
    vector<vector<double> > matrix)
{
  Node n(id, coordinates, matrix);
  this->nodes_.push_back(n);
}

void PlanningGraph::addNode(Node n)
{
  this->nodes_.push_back(n);
}

void PlanningGraph::addNode(double x, double y, double z, double cost)
{
  Node n(Util::newVector(x, y, z, 0), Util::newMatrix());
  this->nodes_.push_back(n);

}

void PlanningGraph::addLink()
{
  Link l(this->links_.size());
  this->links_.push_back(l);
}

void PlanningGraph::addLinkBetweenNodes(double x1, double y1, double z1,
    double x2, double y2, double z2)
{
  this->addLinkBetweenNodes(Util::newVector(x1, y1, z1, 0),
      Util::newVector(x2, y2, z2, 0));
}

double PlanningGraph::getRadiusVehicle()
{
  return this->radiusVehicle_;
}

void PlanningGraph::setRadiusVehicle(double radius)
{
  this->radiusVehicle_ = radius;
}

double PlanningGraph::getMinimumRadiusNodes()
{
  return this->minimumRadiusNodes_;
}

void PlanningGraph::setMinimumRadiusNodes(double radius)
{
  this->minimumRadiusNodes_ = radius;
}

double PlanningGraph::getMinimumDegreesNodes()
{
  return this->minimumDegreesNodes_;
}

void PlanningGraph::setMinimumDegreesNodes(double degrees)
{
  this->minimumDegreesNodes_ = degrees;
}

void PlanningGraph::setfinalGoal(Position finalGoal)
{
  this->finalGoal_ = finalGoal;
  this->lastNodePassed_ = false;
}

void PlanningGraph::setDistances(Util::Distances type)
{
  this->typeDistance_ = type;
  for (unsigned int i = 0; i < this->links_.size(); i++)
  {
    this->links_[i].setDistance(type);
  }
}
vector<Node> PlanningGraph::getNodes()
{
  return this->nodes_;
}

vector<Node*> PlanningGraph::getNodesPointers()
{
  vector<Node*> nodesPointers;
  for (unsigned int i = 0; i < this->nodes_.size(); i++)
  {
    Node *node = &this->nodes_[i];
    nodesPointers.push_back(node);
  }
  return nodesPointers;
}

void PlanningGraph::setAlgorithm(Util::Algorithm algorithm)
{
  this->typeAlgortihm_ = algorithm;
}

vector<Link> PlanningGraph::getLinks()
{
  return this->links_;
}

int PlanningGraph::getNumberNodes()
{
  return this->nodes_.size();
}
int PlanningGraph::getNumberLinks()
{
  return this->links_.size();
}
