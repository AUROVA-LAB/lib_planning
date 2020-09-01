#include "../includes/planning_graph.h"

PlanningGraph::PlanningGraph() {
  this->nodes_.reserve(100000);
  this->links_.reserve(100000);
  this->dynamicPositions_.reserve(10000);
  this->finalGoal_ = *(new Position(Util::newVector(), Util::newMatrix()));
  this->radiusVehicle_ = 0.5;
  this->minimumRadiusNodes_ = 1;
  this->minimumDegreesNodes_ = 45;
  this->typeDistance_ = "E";
  this->lastNodePassed_ = false;
  this->typeAlgortihm_ = Util::Dijkstra;
}

PlanningGraph::~PlanningGraph() {
}

Node PlanningGraph::evaluateNextNode(Position initPos) {
  const clock_t begin_time = clock();

  Node nextNode, finalNode;
  double distance = INT_MAX, minDistance = INT_MAX;
  Node finalGoalNode(this->finalGoal_);
  vector<Node> endNodes;
  std::cout << "Tiempo dentro 0.1 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  Node lastNodePath = getCloserNode(this->finalGoal_);

  std::cout << "Tiempo dentro 0.2 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  if (finalGoalNode.equals(lastNodePath)) {
    endNodes.push_back(finalGoalNode);
  } else {
    endNodes.push_back(lastNodePath);
  }

  std::cout << "Tiempo dentro 0.3 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  vector<Node> initNodes = getCloserNodes(initPos);
  std::cout << "Tiempo dentro 0.4 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  for (unsigned int i = 0; i < initNodes.size(); i++) {
    for (unsigned int j = 0; j < endNodes.size(); j++) {
      distance = getDistanceNodePosition(initPos, initNodes[i]);
      if (distance > this->radiusVehicle_) {
        distance += getDistanceNodePosition(this->finalGoal_, endNodes[j]);
        if (this->typeAlgortihm_ == Util::AStar) {
          distance += calculateDijkstra(initNodes[i], endNodes[j]);
        } else if (this->typeAlgortihm_ == Util::Dijkstra) {
          distance += calculateDijkstra(initNodes[i], endNodes[j]);
        }
        if (distance < minDistance) {
          nextNode = initNodes[i];
          finalNode = endNodes[j];
          minDistance = distance;
        }
      }
    }
  }
  std::cout << "Tiempo dentro 1 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  // Update with the most favorable path
  if (this->typeAlgortihm_ == Util::AStar) {
    calculateAStar(nextNode, finalNode);
  } else if (this->typeAlgortihm_ == Util::Dijkstra) {
    calculateDijkstra(nextNode, finalNode);
  }
  this->lastNodeGraph_ = finalNode;
  std::cout << "Tiempo dentro 2 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  // Check if the closest position is the goal
  Node endNode(this->finalGoal_);
  double distanceToEndPos = getDistanceNodePosition(initPos, endNode);
  double endNodeDistance = getDistanceNodePosition(initPos,lastNodePath);


  bool aux = nextNode.equals(lastNodePath);
  bool aux2 = endNodeDistance >= distanceToEndPos;

  if (endNodeDistance < this->radiusVehicle_ || (aux2 && aux)) {
    this->lastNodePassed_ = true;
  }
  // If the last node has already been reached it will go to the goal
  if (this->lastNodePassed_) {
    nextNode = endNode;
  }
  std::cout << "Tiempo dentro 3 (s) "
      << float(clock() - begin_time) / CLOCKS_PER_SEC << endl;

  return nextNode;
}

bool PlanningGraph::existLinkBetweenNodes(Node node1, Node node2) {
  bool exist = false;
  Node n1, n2;
  for (unsigned int i = 0; i < this->nodes_.size(); i++) {
    if (node1.equals(this->nodes_[i])) {
      n1 = this->nodes_[i];
    } else if (node2.equals(this->nodes_[i])) {
      n2 = this->nodes_[i];
    }
  }
  for (unsigned int i = 0; i < n1.getLinks().size(); i++) {
    if (n1.getLinks()[i]->nodes_[0]->equals(n2)
        || n1.getLinks()[i]->nodes_[1]->equals(n2)) {
      exist = true;
    }
  }
  return exist;
}

void PlanningGraph::dynamicPositionsToNodes() {

  if (this->dynamicPositions_.size() > 2) {
    int i = this->dynamicPositions_.size() - 2;
    bool close = false;
    Node n1(this->dynamicPositions_[i - 1]);
    Node n2(this->dynamicPositions_[i]);
    Node n3(this->dynamicPositions_[i + 1]);
    if (this->nodes_.size() > 0) {
      Node closerNode = getCloserNode(this->dynamicPositions_[i]);
      double distanceNode = closerNode.calculateEuclideanDistance(n2);
      close = (distanceNode <= this->minimumRadiusNodes_);
      if (close) {
        if (this->lastNodeVisited_.isValid()
            && !closerNode.equals(this->lastNodeVisited_)) {
          if (!existLinkBetweenNodes(closerNode, this->lastNodeVisited_)) {
            addLinkBetweenNodes(this->lastNodeVisited_.getCoordinates(),
                closerNode.getCoordinates());
          }
        }
        this->lastNodeVisited_ = closerNode;
      }
    }

    double orientation = calculateSense(n1, n2, n3);
    if (!close
        && (orientation >= this->minimumDegreesNodes_
            || orientation <= (-this->minimumDegreesNodes_))) {
      Node newNode(this->dynamicPositions_[i].getCoordinates(),
          this->dynamicPositions_[i].getMatrix());
      addNode(newNode);
      if (this->lastNodeVisited_.isValid()) {
        addLinkBetweenNodes(this->lastNodeVisited_.getCoordinates(),
            newNode.getCoordinates());
      }
      this->lastNodeVisited_ = newNode;
    }
  }
}

double PlanningGraph::calculateSense(Node pos1, Node pos2, Node pos3) {
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

  if (degreeXY < -180) {
    degreeXY += 360;
  } else if (degreeXY > 180) {
    degreeXY -= 360;
  }
  //normalize [0, 2 π)
  //if (degreeXY < 0) { angle += 2 * M_PI; }

  //normalize (-π, π]
  /*if (degreeXY > M_PI)        { degreeXY -= 2 * M_PI; }
   else if (degreeXY <= -M_PI) { degreeXY += 2 * M_PI; }*/

  return degreeXY;
}

double PlanningGraph::calculateDijkstra(Node initNode, Node endNode) {

  bool allNodesSeen = false;
  int selectedNode = 0;
  Node nextNodeToEvaluate;
  vector<Node*> path;

  //Initialize values
  for (unsigned int i = 0; i < this->nodes_.size(); i++) {
    path.push_back(&this->nodes_[i]);
    path[i]->seen_ = false;
    if (this->nodes_[i] == initNode) {
      path[i]->distance_ = 0;
      selectedNode = i;
      nextNodeToEvaluate = this->nodes_[i];
    } else {
      path[i]->distance_ = DBL_MAX;
    }
  }

  // Dijsktra's algorithm
  while (!allNodesSeen) {
    path[selectedNode]->seen_ = true;

    // Assign distance to connected nodes
    for (unsigned int i = 0; i < nextNodeToEvaluate.getLinks().size(); i++) {
      double distanceBetweenNodes = nextNodeToEvaluate.getLinks()[i]->distance_;
      double newDistance = distanceBetweenNodes + nextNodeToEvaluate.distance_;

      Node *childNode;
      if (*nextNodeToEvaluate.getLinks()[i]->nodes_[0] == nextNodeToEvaluate) {
        childNode = nextNodeToEvaluate.getLinks()[i]->nodes_[1];
      } else {
        childNode = nextNodeToEvaluate.getLinks()[i]->nodes_[0];
      }
      // If the node is near an obstacle, add an additional cost
      double cost = childNode->cost_;
      newDistance += cost;

      if (newDistance < childNode->distance_) {
        childNode->distance_ = newDistance;
      }
    }

    // Get next Node to evaluate
    allNodesSeen = true;
    double newDistance = DBL_MAX;
    for (unsigned int i = 0; i < this->nodes_.size(); i++) {
      if (path[i]->seen_ == false && path[i]->distance_ < newDistance) {
        allNodesSeen = false;
        selectedNode = i;
        nextNodeToEvaluate = *path[i];
        newDistance = path[i]->distance_;
      }
    }
  }

  double totalDistance = -1;
  this->bestPath_.clear();
  // Get the distance to the final Node
  for (unsigned int i = 0; i < path.size(); i++) {
    if (endNode == *path[i]) {
      totalDistance = path[i]->distance_;
    }
    this->bestPath_.push_back(*path[i]);
  }

  return totalDistance;
}

double PlanningGraph::calculateAStarHDistance(Node initNode, Node endNode) {
  // Use the distance between two nodes to calculate H
  return getDistanceNodePosition(initNode, endNode);
}

double PlanningGraph::calculateAStar(Node initNode, Node endNode) {
  Node currentNode = initNode;
  Node previusNode = initNode;
  Node *childNode;
  vector<Node*> path;
  vector<Node*> deadEndPath;
  vector<Node*> neighbours;
  bool finalNode = false;
  double totalDistance = -1;
  this->bestPath_.clear();

  for (unsigned int i = 0; i < this->nodes_.size(); i++) {
    path.push_back(&this->nodes_[i]);
    path[i]->seen_ = false;
    if (this->nodes_[i] == initNode) {
      path[i]->distance_ = 0;
      path[i]->seen_ = true;
      currentNode = this->nodes_[i];
      this->bestPath_.push_back(*path[i]);
    } else {
      path[i]->distance_ = DBL_MAX;
    }
  }

  // A* algorithm
  while (!finalNode) {
    neighbours.clear();
    for (unsigned int i = 0; i < currentNode.getLinks().size(); i++) {
      double distanceG = currentNode.getLinks()[i]->distance_;
      double newDistance = currentNode.distance_;
      if (*currentNode.getLinks()[i]->nodes_[0] == currentNode) {
        childNode = currentNode.getLinks()[i]->nodes_[1];
      } else {
        childNode = currentNode.getLinks()[i]->nodes_[0];
      }
      // If the node is near an obstacle, add an additional cost
      double cost = childNode->cost_;
      Position childPos(childNode->getCoordinates(), childNode->getMatrix());
      double distanceH = calculateAStarHDistance(childPos, endNode);
      newDistance += cost + distanceG + distanceH;
      childNode->distance_ = newDistance;
      neighbours.push_back(childNode);

    }

    // Select the best next node
    double minDistance = DBL_MAX;
    for (unsigned int i = 0; i < neighbours.size(); i++) {
      neighbours[i]->seen_ = true;
      if (neighbours[i]->distance_ < minDistance) {
        currentNode = *neighbours[i];
        minDistance = neighbours[i]->distance_;
      }
    }
    this->bestPath_.push_back(currentNode);

    if (currentNode.equals(endNode)) {
      finalNode = true;
    }
  }

  this->bestPath_.clear();

  // Get the distance to the final Node
  for (unsigned int i = 0; i < path.size(); i++) {
    if (endNode == *path[i]) {
      totalDistance = path[i]->distance_;
    }
    this->bestPath_.push_back(*path[i]);
  }

  return totalDistance;

}

void PlanningGraph::addLinkBetweenNodes(vector<double> coordinates1,
    vector<double> coordinates2) {

  int pos1 = -1, pos2 = -1;
  // Find the  nodes
  for (unsigned int i = 0; i < this->nodes_.size(); i++) {
    if (this->nodes_[i].equalCoordinatesXYZ(coordinates1)) {
      pos1 = i;
    }
    if (this->nodes_[i].equalCoordinatesXYZ(coordinates2)) {
      pos2 = i;
    }
  }

  // Connect the nodes
  if (pos1 != -1 && pos2 != -1
      && !existLinkBetweenNodes(this->nodes_[pos1], this->nodes_[pos2])) {
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

void PlanningGraph::addLinkBetweenNodesById(long id1, long id2) {

  int pos1 = -1, pos2 = -1;
  // Find the nodes
  for (unsigned int i = 0; i < this->nodes_.size(); i++) {
    if (this->nodes_[i].getId() == id1) {
      pos1 = i;
    }
    if (this->nodes_[i].getId() == id2) {
      pos2 = i;
    }
  }

  // Connect the nodes
  if (pos1 != -1 && pos2 != -1
      && !existLinkBetweenNodes(this->nodes_[pos1], this->nodes_[pos2])) {
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
    vector<vector<double> > matrix) {
  Node n(coordinates, matrix);
  this->nodes_.push_back(n);
}

void PlanningGraph::addNode(long id, vector<double> coordinates,
    vector<vector<double> > matrix) {
  Node n(id, coordinates, matrix);
  this->nodes_.push_back(n);
}

void PlanningGraph::addNode(Node n) {
  this->nodes_.push_back(n);
}

void PlanningGraph::addNode(double x, double y, double z, double cost) {
  Node n(Util::newVector(x, y, z, 0), Util::newMatrix());
  this->nodes_.push_back(n);

}

void PlanningGraph::addLink() {
  Link l(this->links_.size());
  this->links_.push_back(l);
}

void PlanningGraph::addLinkBetweenNodes(double x1, double y1, double z1,
    double x2, double y2, double z2) {
  this->addLinkBetweenNodes(Util::newVector(x1, y1, z1, 0),
      Util::newVector(x2, y2, z2, 0));
}

Link* PlanningGraph::findLinkPointer(long id) {
  Link *l;
  for (unsigned int i = 0; this->links_.size() > i; i++) {
    if (id == this->links_[i].getId()) {
      l = &this->links_[i];
    }
  }
  return l;
}

Node PlanningGraph::getNextNode(Position initPos) {
  Node nextNode;
  Node actualPosition(initPos);
  if (this->nodes_.size() > 0) {
    nextNode = evaluateNextNode(initPos);
    nextNode.setYaw(
        calculateSense(actualPosition, nextNode, evaluateNextNode(nextNode)));
  } else {
    nextNode = this->finalGoal_;
  }
  return nextNode;
}

double PlanningGraph::getDistanceNodePosition(Position pos, Node node) {
  Node newNode(pos);
  double distancia;
  if (this->typeDistance_ == "E") {
    distancia = node.calculateEuclideanDistance(newNode);
  } else {
    distancia = node.calculateMahalanobisDistance(newNode);
  }
  return distancia;
}

Node PlanningGraph::getCloserNode(Position pos) {

  Node n1;
  if (this->nodes_.size() > 0) {
    double distance = getDistanceNodePosition(pos, this->nodes_[0]);
    n1 = this->nodes_[0];

    for (unsigned int i = 1; i < this->nodes_.size(); i++) {
      double newDistance = getDistanceNodePosition(pos, this->nodes_[i]);
      if (newDistance < distance) {
        n1 = this->nodes_[i];
        distance = newDistance;
      }
    }
  }
  return n1;
}

// Get the two closest nodes
vector<Node> PlanningGraph::getCloserNodes(Position pos) {
  bool insideNode = false;
  Node n;
  vector<Node> connectedNodes;

  Node n1, n2;
  double distance1 = getDistanceNodePosition(pos, this->nodes_[0]);
  double distance2 = getDistanceNodePosition(pos, this->nodes_[1]);

  if (distance1 < distance2) {
    n1 = this->nodes_[0];
    n2 = this->nodes_[1];
  } else {
    n2 = this->nodes_[0];
    n1 = this->nodes_[1];
  }
  if (distance1 < this->radiusVehicle_) {
    insideNode = true;
    n = this->nodes_[0];
  } else if(distance2 < this->radiusVehicle_) {
    insideNode = true;
    n = this->nodes_[1];
  }
  unsigned int i = 2;
  // Search all nodes and return the two closest
  while ( i < this->nodes_.size() && !insideNode ) {
    double newDistance = getDistanceNodePosition(pos, this->nodes_[i]);
    // Check if the robot is inside a node
    if (newDistance < this->radiusVehicle_) {
      insideNode = true;
      n = this->nodes_[i];
    }
    if (newDistance < distance1) {
      n2 = n1;
      distance2 = distance1;
      n1 = this->nodes_[i];
      distance1 = newDistance;
    } else if (newDistance < distance2) {
      n2 = this->nodes_[i];
      distance2 = newDistance;
    }
    i++;
  }
  connectedNodes.push_back(n1);
  connectedNodes.push_back(n2);

  // If the robot is inside the radius of the node, look for the connected nodes
  if (insideNode) {
    for (unsigned int j = 0; j < n.links_.size(); j++) {
      if (*n.links_[j]->nodes_[0] == n) {
        connectedNodes.push_back(*n.links_[j]->nodes_[1]);
      } else {
        connectedNodes.push_back(*n.links_[j]->nodes_[0]);
      }
    }
    // If the robot is not inside the radius of the node, look for the closest nodes
  }
  return connectedNodes;
}

/*vector<Node> PlanningGraph::getCloserNodes(Position pos) {
 bool insideNode = false;
 Node n;
 vector<Node> connectedNodes;
 // Check if the robot is inside a node
 for (unsigned int i = 0; i < this->nodes_.size(); i++) {
 double distance = getDistanceNodePosition(pos, this->nodes_[i]);
 if (distance < this->radiusVehicle_) {
 insideNode = true;
 n = this->nodes_[i];
 }
 }

 // If the robot is inside the radius of the node, look for the connected nodes
 if (insideNode) {
 for (unsigned int j = 0; j < n.links_.size(); j++) {
 if (*n.links_[j]->nodes_[0] == n) {
 connectedNodes.push_back(*n.links_[j]->nodes_[1]);
 } else {
 connectedNodes.push_back(*n.links_[j]->nodes_[0]);
 }
 }
 // If the robot is not inside the radius of the node, look for the closest nodes
 } else {
 Node n1, n2;
 double distance1 = getDistanceNodePosition(pos, this->nodes_[0]);
 double distance2 = getDistanceNodePosition(pos, this->nodes_[1]);

 if (distance1 < distance2) {
 n1 = this->nodes_[0];
 n2 = this->nodes_[1];
 } else {
 n2 = this->nodes_[0];
 n1 = this->nodes_[1];
 }

 // Search all nodes and return the two closest
 for (unsigned int i = 2; i < this->nodes_.size(); i++) {
 double newDistance = getDistanceNodePosition(pos, this->nodes_[i]);
 if (newDistance < distance1) {
 n2 = n1;
 distance2 = distance1;
 n1 = this->nodes_[i];
 distance1 = newDistance;
 } else if (newDistance < distance2) {
 n2 = this->nodes_[i];
 distance2 = newDistance;
 }
 }
 connectedNodes.push_back(n1);
 connectedNodes.push_back(n2);
 }
 return connectedNodes;
 }*/

vector<Node> PlanningGraph::bestPathNodes(vector<Node> allNodes) {
  vector<Node> minPath;

  // Copy all nodes
  for (unsigned int i = 0; i < allNodes.size(); i++) {
    if (this->lastNodeGraph_.equals(allNodes[i])) {
      minPath.push_back(allNodes[i]);
    }
  }
  // Get all the nodes it has to go through starting with the last one
  while (minPath[0].distance_ != 0) {
    Node nextNode = allNodes[0];
    Node auxNode;

    // Find the node that gets the best path
    for (unsigned int i = 0; i < allNodes[0].getLinks().size(); i++) {
      if (allNodes[0].equals(*allNodes[0].getLinks()[i]->getNodes()[0])) {
        auxNode = *allNodes[0].getLinks()[i]->getNodes()[1];
      } else {
        auxNode = *allNodes[0].getLinks()[i]->getNodes()[0];
      }

      if (auxNode.distance_ < nextNode.distance_) {
        nextNode = auxNode;
      }
    }
    minPath.insert(minPath.begin(), nextNode);
  }
  Node lastNodeGoal(this->finalGoal_);
  if (!this->lastNodeGraph_.equals(lastNodeGoal)) {
    minPath.push_back(lastNodeGoal);
  }
  return minPath;
}

vector<Node> PlanningGraph::getPathNodes(Position initPos) {
  Node actualPosition(initPos);
  if (this->nodes_.size() > 0) {
    evaluateNextNode(initPos);
  } else {
    this->nodes_.push_back(this->finalGoal_);
  }
  vector<Node> bestPathOfNodes;
  if (this->typeAlgortihm_ == Util::Dijkstra) {
    bestPathNodes(this->bestPath_);
  } else if (this->typeAlgortihm_ == Util::AStar) {
    bestPathOfNodes = this->bestPath_;
  }
  return bestPathOfNodes;
}

void PlanningGraph::addDynamicPosition(vector<double> coordinates,
    vector<vector<double> > matrix) {
  Position p(coordinates, matrix);
  this->dynamicPositions_.push_back(p);
  dynamicPositionsToNodes();
}

double PlanningGraph::getRadiusVehicle() {
  return this->radiusVehicle_;
}

void PlanningGraph::setRadiusVehicle(double radius) {
  this->radiusVehicle_ = radius;
}

double PlanningGraph::getMinimumRadiusNodes() {
  return this->minimumRadiusNodes_;
}

void PlanningGraph::setMinimumRadiusNodes(double radius) {
  this->minimumRadiusNodes_ = radius;
}

double PlanningGraph::getMinimumDegreesNodes() {
  return this->minimumDegreesNodes_;
}

void PlanningGraph::setMinimumDegreesNodes(double degrees) {
  this->minimumDegreesNodes_ = degrees;
}

void PlanningGraph::setfinalGoal(Position finalGoal) {
  this->finalGoal_ = finalGoal;
  this->lastNodePassed_ = false;
}

void PlanningGraph::setDistances(string type) {
  this->typeDistance_ = type;
  for (unsigned int i = 0; i < this->links_.size(); i++) {
    this->links_[i].setDistance(type);
  }
}
vector<Node> PlanningGraph::getNodes() {
  return this->nodes_;
}

vector<Node*> PlanningGraph::getNodesPointers() {
  vector<Node*> nodesPointers;
  for (unsigned int i = 0; i < this->nodes_.size(); i++) {
    Node *node = &this->nodes_[i];
    nodesPointers.push_back(node);
  }
  return nodesPointers;
}

void PlanningGraph::setAlgorithm(Util::Algorithm algorithm) {
  this->typeAlgortihm_ = algorithm;
}

vector<Link> PlanningGraph::getLinks() {
  return this->links_;
}

int PlanningGraph::getNumberNodes() {
  return this->nodes_.size();
}
int PlanningGraph::getNumberLinks() {
  return this->links_.size();
}
