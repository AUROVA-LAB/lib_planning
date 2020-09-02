#include "../src/graph.cpp"
#include "../src/util.cpp"
#include "../src/link.cpp"
#include "../src/planning_graph.cpp"
#include "../src/node.cpp"
#include "../src/position.cpp"
#include "../src/latlong_utm.cpp"

#include "./stubs/graphstub.h"

#include <gtest/gtest.h>

TEST(GraphTest, nextPose)
{
    string file="./testFiles/map.osm";
    string typeDistance = "M";
    double radiusDistance = 2;
	Graph g(file,Util::newMatrix(),typeDistance,radiusDistance);
    double arr0[] = {718046.54061376222,4251256.470624933,0,0};
	std::vector<double> pose0(arr0, arr0+4);
	double arr1[] = {717046.54061376222,4251456.470624933,0,0};
	std::vector<double> pose1(arr1, arr1+4);
	double arr2[] = {716594.83671693609,4251999.7478357395,0,0};
	std::vector<double> pose2(arr2, arr2+4);
    double arr3[] = {716143.19184357452,4252543.0481898841,0,0};
	std::vector<double> pose3(arr3, arr3+4);
    double arr4[] = {715691.60600210144,4253086.3716911143,0,0};
	std::vector<double> pose4(arr4, arr4+4);
    double arr5[] = {715240.08531767083,4253629.2948358618,0,0};
	std::vector<double> pose5(arr5, arr5+4);
    double arr6[] = {714046.54061376222,4253829.470624933,0,0};
	std::vector<double> pose6(arr6, arr6+4);
    
    Pose p1,p2,p3,p4,p6,p0,result;
    p1.coordinates=pose1;
    p1.matrix=Util::newMatrix();
    p2.coordinates=pose2;
    p2.matrix=Util::newMatrix();
    p3.coordinates=pose3;
    p3.matrix=Util::newMatrix();
    p4.coordinates=pose4;
    p4.matrix=Util::newMatrix();
    p0.coordinates=pose0;
    p0.matrix=Util::newMatrix();
    p6.coordinates=pose6;
    p6.matrix=Util::newMatrix();
    result = g.getNextPose(p1,p4);
    ASSERT_EQ(pose2[0],result.coordinates[0]);
    ASSERT_EQ(pose2[1],result.coordinates[1]);
    ASSERT_EQ(pose2[2],result.coordinates[2]);
    result = g.getNextPose(p2,p4);
    ASSERT_EQ(pose3[0],result.coordinates[0]);
    ASSERT_EQ(pose3[1],result.coordinates[1]);
    result = g.getNextPose(p3,p4);
    ASSERT_EQ(pose4[0],result.coordinates[0]);
    ASSERT_EQ(pose4[1],result.coordinates[1]);
    result = g.getNextPose(p4,p4);
    ASSERT_EQ(pose4[0],result.coordinates[0]);
    ASSERT_EQ(pose4[1],result.coordinates[1]);
    result = g.getNextPose(p0,p6);
    ASSERT_EQ(pose2[0],result.coordinates[0]);
    ASSERT_EQ(pose2[1],result.coordinates[1]);

}

TEST(GraphTest, nextPoseBadReference)
{
    string file="./testFiles/mapWithBadReference.osm";
    string typeDistance = "M";
    double radiusDistance = 2;
	Graph g(file,Graph::newMatrix(1,1,10000000,90),typeDistance,radiusDistance);
    double arr0[] = {718046.54061376222,4251256.470624933,0,0};
	std::vector<double> pose0(arr0, arr0+4);
	double arr1[] = {717046.54061376222,4251456.470624933,0,0};
	std::vector<double> pose1(arr1, arr1+4);
	double arr2[] = {716594.83671693609,4251999.7478357395,0,0};
	std::vector<double> pose2(arr2, arr2+4);
    double arr3[] = {716143.19184357452,4252543.0481898841,0,0};
	std::vector<double> pose3(arr3, arr3+4);
    double arr4[] = {715691.60600210144,4253086.3716911143,0,0};
	std::vector<double> pose4(arr4, arr4+4);
    double arr5[] = {715240.08531767083,4253629.2948358618,0,0};
	std::vector<double> pose5(arr5, arr5+4);
    double arr6[] = {714046.54061376222,4253829.470624933,0,0};
	std::vector<double> pose6(arr6, arr6+4);
    
    Pose p1,p2,p3,p4,p6,p0,p5,result;
    p1.coordinates=pose1;
    p1.matrix=Util::newMatrix();
    p2.coordinates=pose2;
    p2.matrix=Util::newMatrix();
    p3.coordinates=pose3;
    p3.matrix=Util::newMatrix();
    p4.coordinates=pose4;
    p4.matrix=Util::newMatrix();
    p0.coordinates=pose0;
    p0.matrix=Util::newMatrix();
    p6.coordinates=pose6;
    p6.matrix=Util::newMatrix();
    p5.coordinates=pose5;
    p5.matrix=Util::newMatrix();
    result = g.getNextPose(p2,p5);
    ASSERT_EQ(pose3[0],result.coordinates[0]);
    ASSERT_EQ(pose3[1],result.coordinates[1]);
    ASSERT_EQ(pose3[2],result.coordinates[2]);
}

TEST(GraphTest, graphStub1)
{
    string file="./testFiles/mapWithBadReference.osm";
    string typeDistance = "M";
    double radiusDistance = 2;
	Graph gs(file,Util::newMatrix(),typeDistance,radiusDistance);
    PlanningGraph pg = gs.getPlanningGraph();

    ASSERT_EQ(pg.getNodes().size(),5);
    ASSERT_EQ(pg.getLinks().size(),5);
    ASSERT_EQ(pg.getLinks()[0].getNodes()[0]->getId(),1);
    ASSERT_EQ(pg.getLinks()[0].getNodes()[1]->getId(),2);
    ASSERT_EQ(pg.getLinks()[1].getNodes()[0]->getId(),5);
    ASSERT_EQ(pg.getLinks()[1].getNodes()[1]->getId(),1);

}


TEST(GraphTest, getPathPoses)
{
    string file="./testFiles/map.osm";
    string typeDistance = "M";
    double radiusDistance = 2;
	Graph g(file,Util::newMatrix(),typeDistance,radiusDistance);
    double arr0[] = {718046.54061376222,4251256.470624933,0,0};
	std::vector<double> pose0(arr0, arr0+4);
	double arr1[] = {717046.54061376222,4251456.470624933,0,0};
	std::vector<double> pose1(arr1, arr1+4);
	double arr2[] = {716594.83671693609,4251999.7478357395,0,0};
	std::vector<double> pose2(arr2, arr2+4);
    double arr3[] = {716143.19184357452,4252543.0481898841,0,0};
	std::vector<double> pose3(arr3, arr3+4);
    double arr4[] = {715691.60600210144,4253086.3716911143,0,0};
	std::vector<double> pose4(arr4, arr4+4);
    double arr5[] = {715240.08531767083,4253629.2948358618,0,0};
	std::vector<double> pose5(arr5, arr5+4);
    double arr6[] = {714046.54061376222,4253829.470624933,0,0};
	std::vector<double> pose6(arr6, arr6+4);
	vector<Pose> result;
    Pose p1,p2,p3,p4,p6,p0;
    p1.coordinates=pose1;
    p1.matrix=Util::newMatrix();
    p2.coordinates=pose2;
    p2.matrix=Util::newMatrix();
    p3.coordinates=pose3;
    p3.matrix=Util::newMatrix();
    p4.coordinates=pose4;
    p4.matrix=Util::newMatrix();
    p0.coordinates=pose0;
    p0.matrix=Util::newMatrix();
    p6.coordinates=pose6;
    p6.matrix=Util::newMatrix();
    result = g.getPathPoses(p1,p4);
    ASSERT_EQ(pose2[0],result[0].coordinates[0]);
    ASSERT_EQ(pose2[1],result[0].coordinates[1]);
    ASSERT_EQ(pose2[2],result[0].coordinates[2]);
    /*result = g.getPathPoses(p2,p4);
    ASSERT_EQ(pose3[0],result[1].coordinates[0]);
    ASSERT_EQ(pose3[1],result[1].coordinates[1]);
    result = g.getPathPoses(p3,p4);
    ASSERT_EQ(pose4[0],result[2].coordinates[0]);
    ASSERT_EQ(pose4[1],result[2].coordinates[1]);
    result = g.getPathPoses(p4,p4);
    ASSERT_EQ(pose4[0],result[3].coordinates[0]);
    ASSERT_EQ(pose4[1],result[3].coordinates[1]);
    result = g.getPathPoses(p0,p6);
    ASSERT_EQ(pose2[0],result[0].coordinates[0]);
    ASSERT_EQ(pose2[1],result[0].coordinates[1]);*/

}

TEST(GraphTest, readAndWriteXML)
{
    string file="./testFiles/map.osm";
    string typeDistance = "M";
    double radiusDistance = 2;
	Graph gs(file,Util::newMatrix(),typeDistance,radiusDistance);
    string xmlToWrite = "./testFiles/writeXmlMap.osm";
    gs.xmlWrite(xmlToWrite,20);

    Graph newGs(typeDistance,radiusDistance);
    newGs.xmlReadUTM(xmlToWrite);

    for(unsigned int i=0; i<gs.getPlanningGraph().getNodes().size(); i++){
        ASSERT_TRUE(gs.getPlanningGraph().getNodes()[i].equals(newGs.getPlanningGraph().getNodes()[i]));
    }
    for(unsigned int i=0; i<gs.getPlanningGraph().getLinks().size(); i++){
        for(unsigned int j=0; j<gs.getPlanningGraph().getLinks()[i].getNodes().size(); j++){
            ASSERT_TRUE(gs.getPlanningGraph().getLinks()[i].getNodes()[j]->equals(*newGs.getPlanningGraph().getLinks()[i].getNodes()[j]));
        }
    }
}

TEST(GraphTest, readAndWriteStruct)
{
    string file="./testFiles/map.osm";
    string typeDistance = "M";
    double radiusDistance = 2;
	Graph gs(file,Util::newMatrix(),typeDistance,radiusDistance);

    vector<StNodes> structNodes = gs.getStructGraph();
    Graph newGs(typeDistance,radiusDistance);
    newGs.loadStructGraph(structNodes);
    
    for(unsigned int i=0; i<gs.getPlanningGraph().getNodes().size(); i++){
        ASSERT_TRUE(gs.getPlanningGraph().getNodes()[i].equals(newGs.getPlanningGraph().getNodes()[i]));
    }
    bool  sameLinks= false;
    for(unsigned int i=0; i<gs.getPlanningGraph().getLinks().size(); i++){
        sameLinks= false;
        for(unsigned int j=0; j<newGs.getPlanningGraph().getLinks().size(); j++){
            if(gs.getPlanningGraph().getLinks()[i].getNodes()[0]->equals(*newGs.getPlanningGraph().getLinks()[j].getNodes()[0]) &&
            gs.getPlanningGraph().getLinks()[i].getNodes()[1]->equals(*newGs.getPlanningGraph().getLinks()[j].getNodes()[1])){
                sameLinks=true;
            } else if(gs.getPlanningGraph().getLinks()[i].getNodes()[0]->equals(*newGs.getPlanningGraph().getLinks()[j].getNodes()[1]) &&
            gs.getPlanningGraph().getLinks()[i].getNodes()[1]->equals(*newGs.getPlanningGraph().getLinks()[j].getNodes()[0])){
                sameLinks=true;
            }
        }
        ASSERT_TRUE(sameLinks);
    }
}
 

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
