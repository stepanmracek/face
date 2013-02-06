#include <vector>

//#include "kinectacquire.h"
//#include "showpointcloud.h"
#include "align.h"


int main(int argc, char *argv[])
{
    //KinectAcquire kinect;
    //kinect.run();

    //loadTransformAndSave("../test/pointcloud0.pcd", "../test/pointcloud0_.pcd", 1/1000.0, -1/1000.0, 1/1000.0, 0.5);

    Data target("../test/pointcloud0.pcd");
    Data source1("../test/pointcloud1.pcd");
    Data source2("../test/pointcloud2.pcd");
    Data source3("../test/pointcloud3.pcd");
    Data source4("../test/pointcloud4.pcd");

    std::vector<Data> sources;
    sources.push_back(source1);
    sources.push_back(source2);
    sources.push_back(source3);
    sources.push_back(source4);

    Align align(target, sources);
    align.runBruteForce();
}
