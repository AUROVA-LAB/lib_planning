#include "../src/util.cpp"
#include "../src/link.cpp"
#include "../src/planning_graph.cpp"
#include "../src/node.cpp"
#include "../src/position.cpp"
#include "../src/latlong_utm.cpp"


#include <gtest/gtest.h>

vector<double> newVector(double x, double y, double z, double yaw){
	vector<double> newVector;
	newVector.push_back(x);
	newVector.push_back(y);
	newVector.push_back(z);
	newVector.push_back(yaw);
	return newVector;
}


TEST(PlanningGraphTest, addNodes)
{ 
	PlanningGraph graph;
	vector<double> coordinates = Util::newVector();
	vector<vector<double> > matrix = Util::newMatrix();
	graph.addNode(coordinates,matrix);
	Node node = graph.getNodes()[0];
    ASSERT_EQ(node.getX(), 0);
    ASSERT_EQ(node.getY(), 0);
    ASSERT_EQ(node.getZ(), 0);
    ASSERT_EQ(node.getYaw(), 0);
}
 
TEST(PlanningGraphTest, addNodes2)
{
	PlanningGraph graph;
	vector<double> coordinates = Util::newVector();
	vector<vector<double> > matrix = Util::newMatrix();
	graph.addNode(coordinates,matrix);

	graph.addNode(newVector(1,1,1,1),matrix);
	graph.addNode(newVector(2,2,2,2),matrix);
	Node node1 = graph.getNodes()[1];
	Node node2 = graph.getNodes()[2];

    ASSERT_EQ(node1.getX(), 1);
    ASSERT_EQ(node1.getY(), 1);
    ASSERT_EQ(node2.getZ(), 2);
    ASSERT_EQ(node2.getYaw(), 2);

}

TEST(PlanningGraphTest, equalsNodes)
{
	PlanningGraph graph;
	vector<double> coordinates = Util::newVector();
	vector<vector<double> > matrix = Util::newMatrix();
	graph.addNode(coordinates,matrix);
	vector<double> coordinates1 = newVector(1,1,1,1);
	vector<double> coordinates2 = newVector(1,1,1,1);
	vector<double> coordinates3 = newVector(1,2,1,1);
	vector<double> coordinates4 = newVector(1,1,2,1);
	vector<double> coordinates5 = newVector(1,1,1,2);
	vector<double> coordinates6 = newVector(2,2,2,2);

	graph.addNode(coordinates1,matrix);
	graph.addNode(coordinates2,matrix);
	graph.addNode(coordinates3,matrix);
	graph.addNode(coordinates4,matrix);	
	graph.addNode(coordinates5,matrix);
	graph.addNode(coordinates6,matrix);

	Node node0 = graph.getNodes()[0];
	Node node1 = graph.getNodes()[1];
	Node node2 = graph.getNodes()[2];
	Node node3 = graph.getNodes()[3];
	Node node4 = graph.getNodes()[4];
	Node node5 = graph.getNodes()[5];
	Node node6 = graph.getNodes()[6];

    ASSERT_TRUE(node1.equals(node2));
	ASSERT_TRUE(node1.equals(node5));
	ASSERT_TRUE(node2.equals(node5));
    ASSERT_FALSE(node1.equals(node0));
    ASSERT_FALSE(node1.equals(node0));
    ASSERT_FALSE(node2.equals(node3));
    ASSERT_FALSE(node2.equals(node4));
    ASSERT_FALSE(node1.equals(node6));
    ASSERT_FALSE(node4.equals(node5));

}

TEST(PlanningGraphTest, addLink)
{
	PlanningGraph graph;
	vector<double> coordinates = Util::newVector();
	vector<vector<double> > matrix = Util::newMatrix();
	graph.addNode(coordinates,matrix);
	vector<double> coordinates1 = newVector(1,1,1,1);
	vector<double> coordinates2 = newVector(2,2,2,2);


	graph.addNode(coordinates1,matrix);
	graph.addNode(coordinates2,matrix);
	Node node1 = graph.getNodes()[1];
	Node node2 = graph.getNodes()[2];
	graph.addLinkBetweenNodes(coordinates1,coordinates2);

    ASSERT_TRUE(graph.getLinks()[0].getNodes()[0]->equals(node1));
    ASSERT_TRUE(graph.getLinks()[0].getNodes()[1]->equals(node2));
}

TEST(PlanningGraphTest, addLink2)
{
	PlanningGraph graph;

	for(int i=0; i<10; i++){
		graph.addNode(newVector(i,i,0,0), Util::newMatrix());
		if(i>0){
			int j=i-1;
			graph.addLinkBetweenNodes(newVector(j,j,0,0),newVector(i,i,0,0));
		}
	}
	Node node0 = graph.getNodes()[0];
	Node node1 = graph.getNodes()[1];
	Node node2 = graph.getNodes()[2];
	Node node3 = graph.getNodes()[3];
	Node node4 = graph.getNodes()[4];
	Node node7 = graph.getNodes()[7];
	Node node9 = graph.getNodes()[9];

	// Assert_Exit check if is a null pointer
    ASSERT_EXIT((graph.getLinks()[0].getNodes()[0]->equals(node0),exit(0)),::testing::ExitedWithCode(0),".*");
    ASSERT_EXIT((graph.getLinks()[4].getNodes()[0]->equals(node4),exit(0)),::testing::ExitedWithCode(0),".*");

	ASSERT_TRUE(graph.getLinks()[0].getNodes()[0]->equals(node0));
	ASSERT_TRUE(graph.getLinks()[0].getNodes()[1]->equals(node1));
	ASSERT_TRUE(graph.getLinks()[1].getNodes()[1]->equals(node2));
	ASSERT_TRUE(graph.getLinks()[2].getNodes()[0]->equals(node2));
	ASSERT_TRUE(graph.getLinks()[2].getNodes()[1]->equals(node3));
	ASSERT_TRUE(graph.getLinks()[4].getNodes()[0]->equals(node4));
	ASSERT_TRUE(graph.getLinks()[6].getNodes()[1]->equals(node7));
	ASSERT_TRUE(graph.getLinks()[7].getNodes()[0]->equals(node7));
	ASSERT_TRUE(graph.getLinks()[8].getNodes()[1]->equals(node9));

}

TEST(PlanningGraphTest, correctPointers)
{
	PlanningGraph graph;
	vector<double> coordinates = Util::newVector();
	vector<vector<double> > matrix = Util::newMatrix();
	//graph.addNode(coordinates,matrix);
	vector<double> coordinates1 = newVector(1,1,1,1);
	vector<double> coordinates2 = newVector(2,2,2,2);
	vector<double> coordinates3 = newVector(3,3,3,3);
	vector<double> coordinates4 = newVector(4,4,4,4);
	vector<double> coordinates5 = newVector(5,5,5,5);
	vector<double> coordinates6 = newVector(6,6,6,6);
	
	graph.addNode(coordinates1,matrix);
	graph.addNode(coordinates2,matrix);
	graph.addNode(coordinates3,matrix);
	graph.addNode(coordinates4,matrix);
	graph.addNode(coordinates5,matrix);
	graph.addNode(coordinates6,matrix);
	Node node1 = graph.getNodes()[0];
	Node node2 = graph.getNodes()[1];
	graph.addLinkBetweenNodes(coordinates1,coordinates2);
	graph.addLinkBetweenNodes(coordinates2,coordinates3);
	graph.addLinkBetweenNodes(coordinates3,coordinates4);
	graph.addLinkBetweenNodes(coordinates4,coordinates5);
	graph.addLinkBetweenNodes(coordinates5,coordinates6);

	// Assert_Exit check if is a null pointer
    ASSERT_EXIT((graph.getLinks()[0].getNodes()[0]->getLinks()[0]->getNodes()[0]->equals(node1),exit(0)),::testing::ExitedWithCode(0),".*");
    ASSERT_EXIT((graph.getLinks()[0].getNodes()[1]->getLinks()[0]->getNodes()[1]->equals(node2),exit(0)),::testing::ExitedWithCode(0),".*");

    ASSERT_TRUE(graph.getLinks()[0].getNodes()[0]->getLinks()[0]->getNodes()[0]->equals(node1));
    ASSERT_TRUE(graph.getLinks()[0].getNodes()[1]->getLinks()[0]->getNodes()[1]->equals(node2));
}


int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
