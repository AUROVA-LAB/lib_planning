#include "../includes/Graph.h"

Graph::Graph(string url, vector<vector<double> > matrix,string typeDistance, double radiusDistance) {
	xmlRead(url, matrix);
	graph.setDistances(typeDistance);
	graph.setRadiusVehicle(radiusDistance);
}

Graph::~Graph() {
}

void Graph::xmlRead(string url, vector<vector<double> > matrix) {

	if(( access( url.c_str(), F_OK ) != -1 )){
		xml_document<> doc;
		xml_node<> * root_node;
		vector<double> coord;
		float lat,lon;
		long id,idNodo1=-1,idNodo2=-1;
		bool nextValue=true,nextValue2=true;


		// Read the xml file into a vector
		ifstream theFile (url.c_str());
		vector<char> buffer((istreambuf_iterator<char>(theFile)), istreambuf_iterator<char>());
		buffer.push_back('\0');

		// Parse the buffer using the xml file parsing library into doc
		doc.parse<0>(&buffer[0]);

		// Find our root node
		root_node = doc.first_node("osm");
		xml_node<> * node = root_node->first_node("node");

		while (nextValue)
		{
			while(node->next_sibling("node") && node->first_node("tag")){
				node = node->next_sibling();
			}
			if(!node->next_sibling("node")){
				nextValue=false;
				if(node){
					if(!node->first_node("tag")){
						lat = atof(node->first_attribute("lat")->value());
						lon = atof(node->first_attribute("lon")->value());
						id = atof(node->first_attribute("id")->value());
						coord=Util::LLToUTM(lat,lon);
						graph.addNode(id,coord,matrix);
					}
				}
			}else{
				lat = atof(node->first_attribute("lat")->value());
				lon = atof(node->first_attribute("lon")->value());
				id = atof(node->first_attribute("id")->value());
				coord=Util::LLToUTM(lat,lon);
				graph.addNode(id,coord,matrix);

				node = node->next_sibling();
			}
		}
		nextValue=true;
		xml_node<> * way = root_node->first_node("way");

		while (nextValue)
		{
			id = atof(way->first_attribute("id")->value());
			if(!way->next_sibling("way")){
				nextValue=false;
			}
			xml_node<> * nd = way->first_node("nd");
			nextValue2=true;
			idNodo1=-1;
			idNodo2=-1;
			while (nextValue2)
			{
				if(idNodo1 == -1){
					idNodo1 = atof(nd->first_attribute("ref")->value());
				}else{
					idNodo2 = atof(nd->first_attribute("ref")->value());
					graph.addLinkBetweenNodesById(idNodo1,idNodo2);
					idNodo1=idNodo2;
					idNodo2=-1;

				}
				if(!nd->next_sibling("nd")){
					nextValue2=false;
				}
				nd = nd->next_sibling();
			}
			way = way->next_sibling();

		}
		theFile.close();
	}else{
		cout << "File not found" << endl;
	}
}


vector<vector<double> > Graph::newMatrix(double x, double y, double z, double yaw){
	vector<vector<double> > newVector(4);
	for(int i=0; i<4;i++){
		if(i==0)
		{
			newVector[i].push_back(x);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
		}
		else if(i==1)
		{
			newVector[i].push_back(0);
			newVector[i].push_back(y);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
		}
		else if(i==2)
		{
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(z);
			newVector[i].push_back(0);
		}
		else if(i==3)
		{
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(0);
			newVector[i].push_back(yaw);
		}
	}

	return newVector;
}


Pose Graph::getNextPose(Pose myPose, Pose endGoal){
	Position myPosition(myPose.coordinates,myPose.matrix);
	Position finalGoal(endGoal.coordinates,endGoal.matrix);
	graph.setfinalGoal(finalGoal);
	Node n=graph.getNextNode(myPosition);
	if(n.getZ() > 10000 || n.getZ()<-10000){
		n.setZ(myPose.coordinates[3]);
	}
	Pose nextPose;
	nextPose.coordinates = n.getCoordinates();
	nextPose.matrix = n.getMatrix();
	return nextPose;
}



