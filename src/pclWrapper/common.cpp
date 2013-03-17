#include "common.h"

void loadTransformAndSave(const char *in, const char *out, float mx, float my, float mz, float tz)
{
    PointCloud cloud;
    pcl::io::loadPCDFile(in, cloud);
    int n = cloud.size();
    for (int i = 0; i < n; i++)
    {
        cloud[i].x *= mx;
        cloud[i].y *= my;
        cloud[i].z = cloud[i].z * mz + tz;
    }
    pcl::io::savePCDFileBinary(out, cloud);
}
