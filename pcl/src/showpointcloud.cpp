#include "showpointcloud.h"

ShowPointCloud::ShowPointCloud(Data &data) : data(data), viewer("PCL")
{
    viewer.showCloud(data.cloud);
}

void ShowPointCloud::run()
{
    while (!viewer.wasStopped())
    {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
    }
}
