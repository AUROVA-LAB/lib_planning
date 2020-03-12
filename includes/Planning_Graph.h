#ifndef INCLUDES_PLANNING_GRAPH_H_
#define INCLUDES_PLANNING_GRAPH_H_

#include "Node.h"
#include "Link.h"
#include "Util.h"

#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <vector>
#include <bits/stdc++.h>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

class Planning_Graph {

private:

	/**
	 * Nodes saved on the map
	 */
	vector<Node> nodes;

	/**
	 * Links between nodes
	 */
	vector<Link> links;
	vector<Position> dynamicPositions;

	/**
	 * Final goal
	 */
	Position finalGoal;

	/**
	 * Distance of proximity of the vehicle to the node
	 */
	double radiusVehicle;

	/**
	 * Minimum distance to insert a node
	 */
	double minimumRadiusNodes;

	/**
	 * Minimum orientation to declare a position as node
	 */
	double minimumDegreesNodes;

	/**
	 * Set the type of distance calculated (Euclidean "E",Mahalanobis "M")
	 */
	string typeDistance;

	/**
	 * Ìf it has been in the closest node
	 */
	bool lastNodePassed;

	/**
	 * The last node visited
	 */
	Node lastNodeVisited;


	/**
	 * To check all the distances for each nodo
	 */
	vector<Node*> newPath;

	/**
	 * The best path among all options
	 */
	vector<Node*> bestPath;

	/**
	 * Create a link to join nodes
	 */
	void addLink();

	/**
	 * To initialize a default matrix
	 */
	vector<vector<double> > newMatrix();
	/**
	 * To initialize a default vector
	 */
	vector<double> newVector();


	/**
	 * Get the closest node from a position
	 */
	Node getCloserNode(Position pos);

	/**
	 * Get the XY sense using three nodes
	 */
	double calculateSense(Node pos1,Node pos2,Node pos3);

	/**
	 * Calculate a distance between a node and a position. This distance
	 * can be Mahalanabis or Euclidean
	 */
	double distanceNodeAndPosition(Position pos,Node node);

	/**
	 * Get the two closest nodes from a position
	 */
	vector<Node> getCloserNodes(Position pos);

	/**
	 * Calculate wich will be the next best pose
	 */
	Node evaluateNextNode(Position initPos);

	/**
	 * Check if exist link between nodes
	 */
	bool existLinkBetweenNodes(Node node1,Node node2);

	/**
	 * Convert saved positions to nodes
	 */
	void dynamicPositionsToNodes();

	/**
	 * Get the link pointer using an identifier
	 */
	Link* findLinkPointer(long id);

	/**
	 * Calculate the distance to the goal
	 */
	double calculateDijkstra(Node initNode, Node endNode);


public:

	Planning_Graph();
	~Planning_Graph();

	/**
	 * Add a new node
	 */
	void addNode(vector<double> coordinates,vector<vector<double> >  matrix);
	/**
	 * Add a new node
	 */
	void addNode(Node n);
	/**
	 * Add a new node
	 */
	void addNode(double x, double y, double z, double cost);
	/**
	 * Add a new node
	 */
	void addNode(long id,vector<double> coordinates,vector<vector<double> >  matrix);
	/**
	 * Set link between nodes using ids
	 */
	void addLinkBetweenNodesById(long id1,long id2);
	/**
	 * Set link between nodes using poses
	 */
	void addLinkBetweenNodes(double x1, double y1, double z1, double x2, double y2, double z2);
	/**
	 * Set link between nodes using poses
	 */
	void addLinkBetweenNodes(vector<double> coordinates1,vector<double> coordinates2);
	/**
	 * Saves a position
	 */
	void addDynamicPosition(vector<double> coordinates,vector<vector<double> >  matrix);

	/**
	 * Set the final goal
	 */
	void setfinalGoal(Position finalGoal);
	/**
	 * Set if is used Mahalanobis or Euclidean distance
	 */
	void setDistances(string type);

	double getRadiusVehicle();
	double setRadiusVehicle(double radius);
	double getMinimumRadiusNodes();
	double setMinimumRadiusNodes(double radius);
	double getMinimumDegreesNodes();
	double setMinimumDegreesNodes(double degrees);

	/**
	 * Returns all nodes
	 */
	vector<Node> getNodes();

	/**
	 * Returns all links
	 */
	vector<Link> getLinks();

	/**
	 * Returns the next calculated node
	 */
	Node getNextNode(Position initPos);

	/**
	 * Returns the number of links
	 */
	int getNumberLinks();

	/**
	 * Returns the number of nodes
	 */
	int getNumberNodes();



};

#endif
