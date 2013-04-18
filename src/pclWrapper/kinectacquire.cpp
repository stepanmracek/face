/*#include "kinectacquire.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <limits>

void KinectAcquire::run()
{
    pcl::OpenNIGrabber *interface = new pcl::OpenNIGrabber();

    boost::function<void (const PointCloud::ConstPtr&)> f =
            boost::bind (&KinectAcquire::displayCallback, this, _1);

    interface->registerCallback (f);
    interface->start ();

    while (!viewer.wasStopped())
    {
        boost::this_thread::sleep (boost::posix_time::seconds (1));
    }

    interface->stop ();
}

void KinectAcquire::displayCallback(const PointCloud::ConstPtr &cloud)
{
    if (!viewer.wasStopped())
    {
        pcl::PassThrough<Point> filter;
        PointCloud::Ptr filtered(new PointCloud);
        filter.setFilterFieldName("z");
        filter.setFilterLimits(0.6, 0.8);
        filter.setInputCloud(cloud);
        filter.filter(*filtered);

        viewer.showCloud(filtered);
    }
}
*/
