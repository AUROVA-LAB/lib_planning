#include "../includes/Planning_Graph.h"

Planning_Graph::Planning_Graph() {
	this->nodes.reserve(100000);
	this->links.reserve(100000);
	this->dynamicPositions.reserve(10000);
	this->finalGoal = *(new Position(Util::newVector(), Util::newMatrix()));
	this->radiusVehicle = 0.5;
	this->minimumRadiusNodes = 1;
	this->minimumDegreesNodes = 45;
	this->typeDistance = "E";
	this->lastNodePassed = false;
}

Planning_Graph::~Planning_Graph() {
}

vector<Node> Planning_Graph::getNodes() {
	return nodes;
}

vector<Link> Planning_Graph::getLinks() {
	return links;
}

int Planning_Graph::getNumberNodes() {
	return nodes.size();
}
int Planning_Graph::getNumberLinks() {
	return links.size();
}

void Planning_Graph::addNode(vector<double> coordinates,
		vector<vector<double> > matrix) {
	Node n(coordinates, matrix);
	nodes.push_back(n);
}

void Planning_Graph::addNode(long id, vector<double> coordinates,
		vector<vector<double> > matrix) {
	Node n(id, coordinates, matrix);
	nodes.push_back(n);
}

void Planning_Graph::addNode(Node n) {
	nodes.push_back(n);
}

void Planning_Graph::addNode(double x, double y, double z, double cost) {
	Node n(Util::newVector(x, y, z, 0), Util::newMatrix());
	nodes.push_back(n);

}

void Planning_Graph::addLink() {
	Link l(links.size());
	links.push_back(l);
}

Link* Planning_Graph::findLinkPointer(long id) {
	Link *l;
	for (int i = 0; links.size() > i; i++) {
		if (id == links[i].getId()) {
			l = &links[i];
		}
	}
	return l;
}

void Planning_Graph::addDynamicPosition(vector<double> coordinates,
		vector<vector<double> > matrix) {
	Position p(coordinates, matrix);
	dynamicPositions.push_back(p);
	dynamicPositionsToNodes();
}

double Planning_Graph::getRadiusVehicle() {
	return radiusVehicle;
}

double Planning_Graph::setRadiusVehicle(double radius) {
	this->radiusVehicle = radius;
}

double Planning_Graph::getMinimumRadiusNodes() {
	return minimumRadiusNodes;
}

double Planning_Graph::setMinimumRadiusNodes(double radius) {
	this->minimumRadiusNodes = radius;
}

double Planning_Graph::getMinimumDegreesNodes() {
	return minimumDegreesNodes;
}

double Planning_Graph::setMinimumDegreesNodes(double degrees) {
	this->minimumDegreesNodes = degrees;
}

void Planning_Graph::setfinalGoal(Position finalGoal) {
	this->finalGoal = finalGoal;
	this->lastNodePassed = false;
}

void Planning_Graph::setDistances(string type) {
	this->typeDistance = type;
	for (int i = 0; i < links.size(); i++) {
		links[i].setDistance(type);
	}
}

void Planning_Graph::addLinkBetweenNodes(double x1, double y1, double z1,
		double x2, double y2, double z2) {
	this->addLinkBetweenNodes(Util::newVector(x1, y1, z1, 0),
			Util::newVector(x2, y2, z2, 0));
}

void Planning_Graph::addLinkBetweenNodes(vector<double> coordinates1,
		vector<double> coordinates2) {

	int pos1 = -1, pos2 = -1;
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i].equalCoordinatesXYZ(coordinates1)) {
			pos1 = i;
		}
		if (nodes[i].equalCoordinatesXYZ(coordinates2)) {
			pos2 = i;
		}
	}

	if (pos1 != -1 && pos2 != -1) {
		addLink();
		Link *l1 = &links[links.size() - 1];
		l1->nodes.push_back(&nodes[pos1]);
		l1->nodes.push_back(&nodes[pos2]);

		nodes[pos1].links.push_back(l1);
		nodes[pos2].links.push_back(l1);

		l1->setEuclideanDistance(
				nodes[pos2].calculateEuclideanDistance(nodes[pos1]));
		l1->setMahalanobisDistance(
				nodes[pos2].calculateMahalanobisDistance(nodes[pos1]));
		l1->setDistance(typeDistance);
	}
}

void Planning_Graph::addLinkBetweenNodesById(long id1, long id2) {

	int pos1 = -1, pos2 = -1;
	for (int i = 0; i < nodes.size(); i++) {
		if (nodes[i].getId() == id1) {
			pos1 = i;
		}
		if (nodes[i].getId() == id2) {
			pos2 = i;
		}
	}

	if (pos1 != -1 && pos2 != -1) {
		addLink();
		Link *l1 = &links[links.size() - 1];
		l1->nodes.push_back(&nodes[pos1]);
		l1->nodes.push_back(&nodes[pos2]);

		nodes[pos1].links.push_back(l1);
		nodes[pos2].links.push_back(l1);

		l1->setEuclideanDistance(
				nodes[pos2].calculateEuclideanDistance(nodes[pos1]));
		l1->setMahalanobisDistance(
				nodes[pos2].calculateMahalanobisDistance(nodes[pos1]));
		l1->setDistance(typeDistance);
	}
}

// Distance between positions
double Planning_Graph::distanceNodeAndPosition(Position pos, Node node) {
	Node newNode(pos);
	double distancia;
	if (typeDistance == "E") {
		distancia = node.calculateEuclideanDistance(newNode);
	} else {
		distancia = node.calculateMahalanobisDistance(newNode);
	}
	return distancia;
}

Node Planning_Graph::getCloserNode(Position pos) {

	Node n1;
	if (nodes.size() > 0) {
		double distance = distanceNodeAndPosition(pos, nodes[0]);
		n1 = nodes[0];

		for (unsigned int i = 1; i < nodes.size(); i++) {
			double newDistance = distanceNodeAndPosition(pos, nodes[i]);
			if (newDistance < distance) {
				n1 = nodes[i];
				distance = newDistance;
			}
		}
	}
	return n1;
}

// Get the two closest nodes
vector<Node> Planning_Graph::getCloserNodes(Position pos) {
	bool nodePosition = false;
	Node n;
	vector<Node> connectedNodes;
	// check if is inside a node
	for (int i = 0; i < nodes.size(); i++) {
		double distance = distanceNodeAndPosition(pos, nodes[i]);
		if (distance < radiusVehicle) {
			nodePosition = true;
			n = nodes[i];
		}
	}

	// Position is inside the radius of the node
	if (nodePosition) {
		for (int j = 0; j < n.links.size(); j++) {
			if (*n.links[j]->nodes[0] == n) {
				connectedNodes.push_back(*n.links[j]->nodes[1]);
			} else {
				connectedNodes.push_back(*n.links[j]->nodes[0]);
			}
		}
		// Robot is far the node
	} else {
		Node n1, n2;
		double distance1 = distanceNodeAndPosition(pos, nodes[0]);
		double distance2 = distanceNodeAndPosition(pos, nodes[1]);

		if (distance1 < distance2) {
			n1 = nodes[0];
			n2 = nodes[1];
		} else {
			n2 = nodes[0];
			n1 = nodes[1];
		}
		for (int i = 2; i < nodes.size(); i++) {
			double newDistance = distanceNodeAndPosition(pos, nodes[i]);
			if (newDistance < distance1) {
				n2 = n1;
				distance2 = distance1;
				n1 = nodes[i];
				distance1 = newDistance;
			} else if (newDistance < distance2) {
				n2 = nodes[i];
				distance2 = newDistance;
			}
		}
		connectedNodes.push_back(n1);
		connectedNodes.push_back(n2);
	}
	return connectedNodes;
}

Node Planning_Graph::getNextNode(Position initPos) {
	Node nextNode;
	Node actualPosition(initPos);
	if (nodes.size() > 0) {
		nextNode = evaluateNextNode(initPos);
		nextNode.setYaw(
				calculateSense(actualPosition, nextNode, evaluateNextNode(nextNode)));
	} else {
		nextNode = finalGoal;
	}
	return nextNode;
}

Node Planning_Graph::evaluateNextNode(Position initPos) {

	Node nextNode, finalNode;
	double distance = INT_MAX, minDistance = INT_MAX;
	Node finalGoalNode(finalGoal);
	vector<Node> endNodes;
	Node n2 = getCloserNode(finalGoal);
	if (finalGoalNode.equals(n2)) {
		endNodes.push_back(finalGoalNode);
	} else {
		endNodes.push_back(n2);
	}
	vector<Node> initNodes = getCloserNodes(initPos);

	for (int i = 0; i < initNodes.size(); i++) {
		for (int j = 0; j < endNodes.size(); j++) {
			distance = distanceNodeAndPosition(initPos, initNodes[i]);
			if (distance > radiusVehicle) {
				distance += distanceNodeAndPosition(finalGoal, endNodes[j]);
				distance += calculateDijkstra(initNodes[i], endNodes[j]);
				if (distance < minDistance) {
					nextNode = initNodes[i];
					finalNode = endNodes[j];
					minDistance = distance;
				}
			}
		}
	}

	calculateDijkstra(nextNode, finalNode);

	Node endNode(finalGoal);
	double distanceToEndPos = distanceNodeAndPosition(initPos, endNode);
	double endNodeDistance = distanceNodeAndPosition(initPos,
			getCloserNode(finalGoal));
	Node lastNodePath = getCloserNode(finalGoal);
	bool aux = nextNode.equals(lastNodePath);
	bool aux2 = endNodeDistance >= distanceToEndPos;
	if (endNodeDistance < radiusVehicle || (aux2 && aux)) {
		lastNodePassed = true;
	}
	if (lastNodePassed) {
		nextNode = endNode;
	}

	return nextNode;
}

bool Planning_Graph::existLinkBetweenNodes(Node node1, Node node2) {
	bool exist = false;
	Node n1, n2;
	for (int i = 0; i < nodes.size(); i++) {
		if (node1.equals(nodes[i])) {
			n1 = nodes[i];
		} else if (node2.equals(nodes[i])) {
			n2 = nodes[i];
		}
	}
	for (int j = 0; j < n1.getLinks().size(); j++) {
		if (n1.getLinks()[j]->nodes[0]->equals(n2)
				|| n1.getLinks()[j]->nodes[1]->equals(n2)) {
			exist = true;
		}
	}
	return exist;
}

void Planning_Graph::dynamicPositionsToNodes() {

	if (dynamicPositions.size() > 2) {
		int i = dynamicPositions.size() - 2;
		bool close = false;
		Node n1(dynamicPositions[i - 1]);
		Node n2(dynamicPositions[i]);
		Node n3(dynamicPositions[i + 1]);
		if (nodes.size() > 0) {
			Node closerNode = getCloserNode(dynamicPositions[i]);
			double distanceNode = closerNode.calculateEuclideanDistance(n2);
			close = (distanceNode <= minimumRadiusNodes);
			if (close) {
				if (lastNodeVisited.isValid() && !closerNode.equals(lastNodeVisited)) {
					if (!existLinkBetweenNodes(closerNode, lastNodeVisited)) {
						addLinkBetweenNodes(lastNodeVisited.getCoordinates(),
								closerNode.getCoordinates());
					}
				}
				lastNodeVisited = closerNode;
			}
		}

		double orientation = calculateSense(n1, n2, n3);
		if (!close
				&& (orientation >= minimumDegreesNodes
						|| orientation <= (-minimumDegreesNodes))) {
			Node newNode(dynamicPositions[i].getCoordinates(),
					dynamicPositions[i].getMatrix());
			addNode(newNode);
			if (lastNodeVisited.isValid()) {
				addLinkBetweenNodes(lastNodeVisited.getCoordinates(),
						newNode.getCoordinates());
			}
			lastNodeVisited = newNode;
		}
	}
}

double Planning_Graph::calculateSense(Node pos1, Node pos2, Node pos3) {
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

double Planning_Graph::calculateDijkstra(Node initNode, Node endNode) {

	bool allNodesSeen = false;
	int selectedNode = 0;
	Node nextNodeToEvaluate;
	vector<Node*> path;

	//Initialize values
	for (int i = 0; i < this->nodes.size(); i++) {
		path.push_back(&this->nodes[i]);
		path[i]->seen = false;
		if (nodes[i] == initNode) {
			path[i]->distance = 0;
			selectedNode = i;
			nextNodeToEvaluate = nodes[i];
		} else {
			path[i]->distance = DBL_MAX;
		}
	}

	// Dijsktra's algorithm
	while (!allNodesSeen) {
		path[selectedNode]->seen = true;

		// Assign distance to connected nodes
		for (int i = 0; i < nextNodeToEvaluate.getLinks().size(); i++) {
			double distanceBetweenNodes = nextNodeToEvaluate.getLinks()[i]->distance;
			double newDistance = distanceBetweenNodes + nextNodeToEvaluate.distance;

			Node *childNode;
			if (*nextNodeToEvaluate.getLinks()[i]->nodes[0] == nextNodeToEvaluate) {
				childNode = nextNodeToEvaluate.getLinks()[i]->nodes[1];
			} else {
				childNode = nextNodeToEvaluate.getLinks()[i]->nodes[0];
			}

			if (childNode->distance > newDistance) {
				// If the node is near an obstacle, add an additional cost
				double cost = childNode->cost;

				childNode->distance = newDistance + cost;
			}
		}

		// Get next Node to evaluate
		allNodesSeen = true;
		double newDistance = DBL_MAX;
		for (int i = 0; i < nodes.size(); i++) {
			if (path[i]->seen == false && path[i]->distance < newDistance) {
				allNodesSeen = false;
				selectedNode = i;
				nextNodeToEvaluate = *path[i];
				newDistance = path[i]->distance;
			}
		}
	}

	double totalDistance = -1;
	// Get the distance to the final Node
	for (int i = 0; i < path.size(); i++) {
		if (endNode == *path[i]) {
			totalDistance = path[i]->distance;
		}
	}

	this->newPath = path;

	return totalDistance;
}

