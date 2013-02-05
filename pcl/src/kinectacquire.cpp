#include "kinectacquire.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <limits>

void KinectAcquire::run()
{
    pcl::OpenNIGrabber *interface = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
            boost::bind (&KinectAcquire::displayCallback, this, _1);

    interface->registerCallback (f);
    interface->start ();

    while (!viewer.wasStopped())
    {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
    }

    interface->stop ();
}

void KinectAcquire::displayCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if (!viewer.wasStopped())
    {
        pcl::PassThrough<pcl::PointXYZ> filter;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0.6, 0.8);
        filter.setInputCloud(cloud);
        filter.filter(*filtered);

        viewer.showCloud(filtered);
    }
}
