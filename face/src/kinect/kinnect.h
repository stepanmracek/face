#ifndef KINECT_H_
#define KINECT_H_

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>

#include "linalg/common.h"
#include "facelib/mesh.h"
#include "facelib/map.h"

class Kinect
{
private:
    static const int n = 640*480;
    static const double alpha = 0.00307110156374;
    static const double beta = 3.33094951605675;
    static const double mindistance = -10;
    static const double scaleFactor = 0.0021;

public:
    static bool getDepth(double *depth, bool *mask = NULL, double minDistance = 200, double maxDistance = 2500);

    static bool getDepth(double *depth, int scansCount, bool *mask = NULL, double minDistance = 200, double maxDistance = 2500);

    static bool getRGB(uint8_t *rgb);

    static VectorOfPoints depthToVectorOfPoints(double *depth);

    static Mesh createMesh(double *depth, uint8_t *rgb);

    static ImageBGR RGBToColorMatrix(uint8_t *rgb);

    static ImageGrayscale RGBToGrayscale(uint8_t *rgb);

    static Mesh scanFace();

    static void PCL()
    {

    }
};

class SimpleOpenNIViewer
{
public:
    SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}

    void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
    {
        if (!viewer.wasStopped())
            viewer.showCloud (cloud);
    }

    void run()
    {
        pcl::Grabber* interface = new pcl::OpenNIGrabber();

        boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
                boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);

        interface->registerCallback (f);

        interface->start ();

        while (!viewer.wasStopped())
        {
            boost::this_thread::sleep (boost::posix_time::seconds (1));
        }

        interface->stop ();
    }

    pcl::visualization::CloudViewer viewer;
};


#endif /* KINECT_H_ */
