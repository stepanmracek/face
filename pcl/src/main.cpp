#include <vector>

//#include "kinectacquire.h"
//#include "showpointcloud.h"
#include "align.h"


int main(int argc, char *argv[])
{
    //KinectAcquire kinect;
    //kinect.run();
    //loadTransformAndSave("../test/pointcloud0.pcd", "../test/pointcloud0_.pcd", 1/1000.0, -1/1000.0, 1/1000.0, 0.5);

    std::vector<std::string> sources;
    sources.push_back("../test/pointcloud1.pcd");
    sources.push_back("../test/pointcloud2.pcd");
    sources.push_back("../test/pointcloud3.pcd");
    sources.push_back("../test/pointcloud4.pcd");
    alignRansac("../test/pointcloud0.pcd", sources);
    //alignICP("../test/pointcloud0.pcd", sources);
}
