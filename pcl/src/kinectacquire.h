#ifndef KINECTACQUIRE_H
#define KINECTACQUIRE_H

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

class KinectAcquire
{
private:

public:
    pcl::visualization::CloudViewer viewer;

    void displayCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    KinectAcquire() : viewer("Kinect") //, filtered(new pcl::PointCloud<pcl::PointXYZRGBA>)
    {
    }

    void run();
};

#endif // KINECTACQUIRE_H
