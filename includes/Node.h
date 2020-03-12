#ifndef INCLUDES_NODE_H_
#define INCLUDES_NODE_H_

#include "Position.h"
#include "Link.h"
#include <Eigen/Cholesky>
#include <Eigen/Dense>
#include <bits/stdc++.h>
#include <string>
#include <vector>
using namespace std;
using namespace Eigen;
class Node: public Position {

public:
	Node();
	Node(Position pos);
	Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix);
	Node(vector<double> coordinates, vector<vector<double> > covarianceMatrix,
			double cost);
	Node(long id, vector<double> coordinates,
			vector<vector<double> > covarianceMatrix);

	~Node();

	vector<Link*> links;
	/**
	 * Used in the dijsktra algorithm to determine the distance to the node
	 */
	double distance;
	/**
	 * Used in the dijsktra algorithm to determine if it has been evaluated
	 */
	bool seen;

	long id;
	/**
	 * If the node is near an obstacle, add an additional cost
	 */
	double cost;

	void addLink(Link &link);
	vector<Link*> getLinks();
	bool operator==(Node &node) const;
	bool equals(Node node);
	bool equalCoordinatesXYZ(vector<double> coordinates);
	long getId();
	string toString();

	double calculateEuclideanDistance(Node node);
	double calculateMahalanobisDistance(Node node);

};

#endif
