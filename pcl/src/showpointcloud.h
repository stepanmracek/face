#ifndef SHOWPOINTCLOUD_H
#define SHOWPOINTCLOUD_H

#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread.hpp>

#include "common.h"

class ShowPointCloud
{
private:
    pcl::visualization::CloudViewer viewer;
    Data &data;

public:
    ShowPointCloud(Data &data);

    void run();
};

#endif // SHOWPOINTCLOUD_H
