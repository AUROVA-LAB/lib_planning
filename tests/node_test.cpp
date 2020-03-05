#include "../src/Util.cpp"
#include "../src/Link.cpp"
#include "../src/Node.cpp"
#include "../src/LatLongToUTM.cpp"
#include "../src/Position.cpp"

#include <gtest/gtest.h>

vector<double> newVector(double x, double y, double z, double yaw){
	vector<double> newVector;
	newVector.push_back(x);
	newVector.push_back(y);
	newVector.push_back(z);
	newVector.push_back(yaw);
	return newVector;
}

vector<vector<double> > newMatrix(double x, double y, double z, double yaw){
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



TEST(NodeTest, euclideanDistance1)
{
    Node n1(newVector(0,0,0,0),newMatrix(0,0,0,0));
    Node n2(newVector(5,0,0,0),newMatrix(0,0,0,0));

    ASSERT_EQ(n1.calculateEuclideanDistance(n2),5);
}

TEST(NodeTest, euclideanDistance2)
{
    Node n1(newVector(1,3,5,0),newMatrix(0,0,0,0));
    Node n2(newVector(10,7,12,0),newMatrix(0,0,0,0));
    double expect = 12.083045973594572;
    ASSERT_EQ(n1.calculateEuclideanDistance(n2),expect);
}

TEST(NodeTest, euclideanDistance3)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(7.5,7.5,7.5,0),newMatrix(0,0,0,0));
    double expect = 11.258330249197702;
    ASSERT_EQ(n1.calculateEuclideanDistance(n2),expect);
}

TEST(NodeTest, nodesEquals)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(1,1,1,0),newMatrix(0,0,0,0));
    ASSERT_TRUE(n1.equals(n2));
}

TEST(NodeTest, nodesNotEquals1)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(1,2,1,0),newMatrix(0,0,0,0));
    ASSERT_FALSE(n1.equals(n2));
}

TEST(NodeTest, nodesNotEquals2)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(3,1,1,0),newMatrix(0,0,0,0));
    ASSERT_FALSE(n1.equals(n2));
}
TEST(NodeTest, nodesNotEquals3)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(1,1,4,0),newMatrix(0,0,0,0));
    ASSERT_FALSE(n1.equals(n2));
}
TEST(NodeTest, nodesNotEquals4)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(4,4,4,0),newMatrix(0,0,0,0));
    ASSERT_FALSE(n1.equals(n2));
}

TEST(NodeTest, addLinks)
{
    Node n1(newVector(1,1,1,0),newMatrix(0,0,0,0));
    Node n2(newVector(4,4,4,0),newMatrix(0,0,0,0));

    Link l1(1),l2(2),l3(3);
    ASSERT_EQ(n1.getLinks().size(),0);
    n1.addLink(l1);
    ASSERT_EQ(n1.getLinks().size(),1);
    n1.addLink(l2);
    n1.addLink(l3);
    ASSERT_EQ(n1.getLinks().size(),3);
    ASSERT_EQ(n1.getLinks()[0]->getId(),1);
    ASSERT_EQ(n1.getLinks()[1]->getId(),2);
    ASSERT_EQ(n1.getLinks()[2]->getId(),3);
    ASSERT_EQ(n1.getLinks()[2]->getNodes().size(),0);
    n1.getLinks()[2]->addNodes(&n1,&n2);
    ASSERT_EQ(n1.getLinks()[2]->getNodes().size(),2);

}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
