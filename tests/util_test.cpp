#include "../src/util.cpp"
#include "../src/latlong_utm.cpp"

#include <gtest/gtest.h>

 TEST(UtilTest, LLToUTM1)
{
	double lat = 38;
	double lon = -0.5;
    // 35 Decimals
    double posX = 719510.33564152033068239688873291015625000;
    double posY = 4208764.45880824793130159378051757812500000;
    vector<double> coord;

    coord=Util::LLToUTM(lat,lon);

    ASSERT_EQ(posX,coord[0]);
    ASSERT_EQ(posY,coord[1]);
}

 TEST(UtilTest, LLToUTM2)
{
	double lat = 39.555;
	double lon = -0.555;
    double posX = 710065.09044901677;
    double posY = 4381223.6977620358;
    vector<double> coord;

    coord=Util::LLToUTM(lat,lon);
    ASSERT_EQ(posX,coord[0]);
    ASSERT_EQ(posY,coord[1]);
}


int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
