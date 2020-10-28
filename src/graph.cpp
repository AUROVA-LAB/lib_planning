#include "../includes/graph.h"

Graph::Graph(string url, vector<vector<double> > matrix,
    Util::Distances typeDistance, double radiusDistance)
{
  xmlReadLatLong(url, matrix);
  this->planning_graph_.setDistances(typeDistance);
  this->planning_graph_.setRadiusVehicle(radiusDistance);
}

Graph::Graph(Util::Distances typeDistance, double radiusDistance)
{
  this->planning_graph_.setDistances(typeDistance);
  this->planning_graph_.setRadiusVehicle(radiusDistance);
}

Graph::~Graph()
{
}

void Graph::xmlReadLatLong(string url, vector<vector<double> > matrix)
{

  // Check if the file exists
  if ((access(url.c_str(), F_OK) != -1))
  {
    xml_document<> doc;
    xml_node<> *root_node;
    vector<double> coord;
    float lat, lon;
    long id, idNodo1 = -1, idNodo2 = -1;
    bool nextValue = true, nextValue2 = true;

    // Read the xml file into a vector
    ifstream theFile(url.c_str());
    vector<char> buffer((istreambuf_iterator<char>(theFile)),
        istreambuf_iterator<char>());
    buffer.push_back('\0');

    // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);

    // Find our root node
    root_node = doc.first_node("osm");
    xml_node<> *node = root_node->first_node("node");

    // Load the nodes using the keyword node
    while (nextValue)
    {
      // Ignore nodes with tag
      while (node->next_sibling("node") && node->first_node("tag"))
      {
        node = node->next_sibling();
      }
      if (!node->next_sibling("node"))
      {
        nextValue = false;
        if (node)
        {
          // Don't load nodes with tag
          if (!node->first_node("tag"))
          {
            lat = atof(node->first_attribute("lat")->value());
            lon = atof(node->first_attribute("lon")->value());
            id = atof(node->first_attribute("id")->value());
            coord = Util::LLToUTM(lat, lon);
            this->planning_graph_.addNode(id, coord, matrix);
          }
        }
      } else
      {
        lat = atof(node->first_attribute("lat")->value());
        lon = atof(node->first_attribute("lon")->value());
        id = atof(node->first_attribute("id")->value());
        coord = Util::LLToUTM(lat, lon);
        this->planning_graph_.addNode(id, coord, matrix);

        node = node->next_sibling();
      }
    }

    nextValue = true;
    xml_node<> *way = root_node->first_node("way");

    // Load the links. The keyword 'way' is for links and the keyword 'ref' is for nodes that link
    while (nextValue)
    {
      id = atof(way->first_attribute("id")->value());
      // Check if it is the last
      if (!way->next_sibling("way"))
      {
        nextValue = false;
      }
      xml_node<> *nd = way->first_node("nd");
      nextValue2 = true;
      idNodo1 = -1;
      idNodo2 = -1;

      // Connect all the nodes of a path
      while (nextValue2)
      {
        if (idNodo1 == -1)
        {
          idNodo1 = atof(nd->first_attribute("ref")->value());
        } else
        {
          idNodo2 = atof(nd->first_attribute("ref")->value());
          this->planning_graph_.addLinkBetweenNodesById(idNodo1, idNodo2);
          idNodo1 = idNodo2;
          idNodo2 = -1;

        }
        if (!nd->next_sibling("nd"))
        {
          nextValue2 = false;
        }
        nd = nd->next_sibling();
      }
      way = way->next_sibling();

    }
    theFile.close();
  } else
  {
    cout << "File not found" << endl;
  }
}

vector<vector<double> > Graph::newMatrix(double x, double y, double z,
    double yaw)
{
  vector<vector<double> > newVector(4);
  for (int i = 0; i < 4; i++)
  {
    if (i == 0)
    {
      newVector[i].push_back(x);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
    } else if (i == 1)
    {
      newVector[i].push_back(0);
      newVector[i].push_back(y);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
    } else if (i == 2)
    {
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(z);
      newVector[i].push_back(0);
    } else if (i == 3)
    {
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(0);
      newVector[i].push_back(yaw);
    }
  }

  return newVector;
}

Pose Graph::getNextPose(Pose myPose, Pose endGoal)
{
  Pose nextPose;
  // Check if the input values are valid
  if (!myPose.coordinates.empty() && !endGoal.coordinates.empty())
  {
    Position myPosition(myPose.coordinates, myPose.matrix);
    Position finalGoal(endGoal.coordinates, endGoal.matrix);
    this->planning_graph_.setfinalGoal(finalGoal);
    Node n = this->planning_graph_.getNextNode(myPosition);

    nextPose.coordinates = n.getCoordinates();
    nextPose.matrix = n.getMatrix();
  }
  return nextPose;
}

vector<Pose> Graph::getPathPoses(Pose myPose, Pose endGoal)
{
  vector<Pose> path;

  // Check if the input values are valid
  if (!myPose.coordinates.empty() && !endGoal.coordinates.empty())
  {
    Position myPosition(myPose.coordinates, myPose.matrix);
    Position finalGoal(endGoal.coordinates, endGoal.matrix);
    this->planning_graph_.setfinalGoal(finalGoal);

    vector<Node> nodes = this->planning_graph_.getPathNodes(myPosition);

    for (unsigned int i = 0; i < nodes.size(); i++)
    {
      Pose pose;
      pose.coordinates = nodes[i].getCoordinates();
      pose.matrix = nodes[i].getMatrix();
      path.push_back(pose);
    }
  }
  return path;
}

PlanningGraph Graph::getPlanningGraph()
{
  return planning_graph_;
}

void Graph::xmlWrite(string fileName, int precision)
{

  vector<Node*> nodes = this->planning_graph_.getNodesPointers();
  // Set identifiers to each node to identify it
  for (unsigned long i = 0; i < this->planning_graph_.getNodes().size(); i++)
  {
    nodes[i]->setId(i);
  }

  std::ofstream osmFile(fileName.c_str());
  osmFile << "<?xml version='1.0' encoding='UTF-8'?>" << endl;
  osmFile << "<osm version='1.0' >" << endl;
  osmFile << "<bounds />" << endl;
  osmFile << std::fixed << std::setprecision(precision);
  for (unsigned int i = 0; i < nodes.size(); i++)
  {
    osmFile << "<node id='" << nodes[i]->getId() << "' x='" << nodes[i]->getX()
        << "' y='" << nodes[i]->getY() << "' z='" << nodes[i]->getZ()
        << "' yaw='" << nodes[i]->getYaw() << "'>" << endl;
    osmFile << "<matrix xx='" << nodes[i]->getMatrix()[0][0] << "' xy='"
        << nodes[i]->getMatrix()[0][1] << "' xz='"
        << nodes[i]->getMatrix()[0][2] << "' xyaw='"
        << nodes[i]->getMatrix()[0][3] << "' yx='"
        << nodes[i]->getMatrix()[1][0] << "' yy='"
        << nodes[i]->getMatrix()[1][1] << "' yz='"
        << nodes[i]->getMatrix()[1][2] << "' yyaw='"
        << nodes[i]->getMatrix()[1][3] << "' zx='"
        << nodes[i]->getMatrix()[2][0] << "' zy='"
        << nodes[i]->getMatrix()[2][1] << "' zz='"
        << nodes[i]->getMatrix()[2][2] << "' zyaw='"
        << nodes[i]->getMatrix()[2][3] << "' yawx='"
        << nodes[i]->getMatrix()[3][0] << "' yawy='"
        << nodes[i]->getMatrix()[3][1] << "' yawz='"
        << nodes[i]->getMatrix()[3][2] << "' yawyaw='"
        << nodes[i]->getMatrix()[3][3] << "'/>" << endl;
    osmFile << "</node>" << endl;
  }
  vector<Link> links = this->planning_graph_.getLinks();

  for (unsigned int i = 0; i < links.size(); i++)
  {
    osmFile << "<way id='" << i << "'>" << endl;
    for (unsigned int j = 0; j < links[i].getNodes().size(); j++)
    {
      osmFile << "<nd ref='" << links[i].getNodes()[j]->getId() << "'/>"
          << endl;
    }
    osmFile << "</way>" << endl;
  }
  osmFile << "</osm>" << endl;
}

void Graph::xmlReadUTM(string url)
{

  // Check if the file exists
  if ((access(url.c_str(), F_OK) != -1))
  {
    xml_document<> doc;
    xml_node<> *root_node;
    long id, idNodo1 = -1, idNodo2 = -1;
    bool nextValue = true, nextValue2 = true;

    // Read the xml file into a vector
    ifstream theFile(url.c_str());
    vector<char> buffer((istreambuf_iterator<char>(theFile)),
        istreambuf_iterator<char>());
    buffer.push_back('\0');

    // Parse the buffer using the xml file parsing library into doc
    doc.parse<0>(&buffer[0]);

    // Find our root node
    root_node = doc.first_node("osm");
    xml_node<> *node = root_node->first_node("node");

    // Load the nodes using the keyword node
    while (nextValue)
    {
      vector<double> coord;
      vector<vector<double> > matrix(4);
      id = atof(node->first_attribute("id")->value());
      coord.push_back(atof(node->first_attribute("x")->value()));
      coord.push_back(atof(node->first_attribute("y")->value()));
      coord.push_back(atof(node->first_attribute("z")->value()));
      coord.push_back(atof(node->first_attribute("yaw")->value()));
      xml_node<> *mx = node->first_node("matrix");
      matrix[0].push_back(atof(mx->first_attribute("xx")->value()));
      matrix[0].push_back(atof(mx->first_attribute("xy")->value()));
      matrix[0].push_back(atof(mx->first_attribute("xz")->value()));
      matrix[0].push_back(atof(mx->first_attribute("xyaw")->value()));
      matrix[1].push_back(atof(mx->first_attribute("yx")->value()));
      matrix[1].push_back(atof(mx->first_attribute("yy")->value()));
      matrix[1].push_back(atof(mx->first_attribute("yz")->value()));
      matrix[1].push_back(atof(mx->first_attribute("yyaw")->value()));
      matrix[2].push_back(atof(mx->first_attribute("zx")->value()));
      matrix[2].push_back(atof(mx->first_attribute("zy")->value()));
      matrix[2].push_back(atof(mx->first_attribute("zz")->value()));
      matrix[2].push_back(atof(mx->first_attribute("zyaw")->value()));
      matrix[3].push_back(atof(mx->first_attribute("yawx")->value()));
      matrix[3].push_back(atof(mx->first_attribute("yawy")->value()));
      matrix[3].push_back(atof(mx->first_attribute("yawz")->value()));
      matrix[3].push_back(atof(mx->first_attribute("yawyaw")->value()));
      this->planning_graph_.addNode(id, coord, matrix);

      if (!node->next_sibling("node"))
        nextValue = false;
      else
        node = node->next_sibling();
    }

    nextValue = true;
    xml_node<> *way = root_node->first_node("way");

    // Load the links. The keyword 'way' is for links that connected nodes, the keyword 'nd' is for nodes that link
    // and  the keyword 'ref' is the identifier of the nodes
    while (nextValue)
    {
      id = atof(way->first_attribute("id")->value());
      if (!way->next_sibling("way"))
      {
        nextValue = false;
      }
      xml_node<> *nd = way->first_node("nd");
      nextValue2 = true;
      idNodo1 = -1;
      idNodo2 = -1;

      while (nextValue2)
      {
        if (idNodo1 == -1)
        {
          idNodo1 = atof(nd->first_attribute("ref")->value());
        } else
        {
          idNodo2 = atof(nd->first_attribute("ref")->value());
          this->planning_graph_.addLinkBetweenNodesById(idNodo1, idNodo2);
          idNodo1 = idNodo2;
          idNodo2 = -1;

        }
        if (!nd->next_sibling("nd"))
        {
          nextValue2 = false;
        }
        nd = nd->next_sibling();
      }
      way = way->next_sibling();

    }
    theFile.close();
  } else
  {
    cout << "File not found" << endl;
  }
}

vector<StNodes> Graph::getStructGraph()
{
  vector<StNodes> st_nodes;
  vector<Node*> allNodes = this->planning_graph_.getNodesPointers();
  // Set identifiers to each node to identify it
  for (unsigned long i = 0; i < this->planning_graph_.getNodes().size(); i++)
  {
    allNodes[i]->setId(i);
  }

  // Transform each node to Struct
  for (unsigned int i = 0; i < allNodes.size(); i++)
  {
    StNodes st_node;
    st_node.id = allNodes[i]->getId();
    st_node.coordinates = allNodes[i]->getCoordinates();
    st_node.matrix = allNodes[i]->getMatrix();
    for (unsigned int j = 0; j < allNodes[i]->getLinks().size(); j++)
    {
      long idNode;
      if (allNodes[i]->getLinks()[j]->getNodes()[0]->equals(*allNodes[i]))
      {
        idNode = allNodes[i]->getLinks()[j]->getNodes()[1]->getId();
      } else
      {
        idNode = allNodes[i]->getLinks()[j]->getNodes()[0]->getId();
      }
      st_node.nodesConnected.push_back(idNode);
    }
    st_nodes.push_back(st_node);
  }
  return st_nodes;
}

void Graph::loadStructGraph(vector<StNodes> st_nodes)
{
  // Load the nodes
  for (unsigned int i = 0; i < st_nodes.size(); i++)
  {
    vector<double> coord = st_nodes[i].coordinates;
    vector<vector<double> > mx = st_nodes[i].matrix;
    long id = st_nodes[i].id;
    this->planning_graph_.addNode(id, coord, mx);

    // Load the links
    for (unsigned int j = 0; j < st_nodes[i].nodesConnected.size(); j++)
    {
      if (st_nodes[i].nodesConnected[j] < i)
      {
        this->planning_graph_.addLinkBetweenNodesById(id,
            st_nodes[i].nodesConnected[j]);
      }
    }
  }
}

void Graph::setAStarAlgorithm()
{
  planning_graph_.setAlgorithm(Util::AStar);
}
void Graph::setDijkstraAlgorithm()
{
  planning_graph_.setAlgorithm(Util::Dijkstra);
}

