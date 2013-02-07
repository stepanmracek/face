#include "common.h"

#include <pcl/filters/voxel_grid.h>

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

Data::Data(const char *path) :
       cloud(new PointCloud),
       normals(new Normals)
   {
       // load pcd
       std::cout << "loading " << path << std::endl;
       pcl::io::loadPCDFile(path, *cloud);

       const float voxel_grid_size = 0.01f;
       pcl::VoxelGrid<Point> vox_grid;
       vox_grid.setInputCloud (cloud);
       vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
       PointCloud::Ptr tempCloud (new PointCloud);
       vox_grid.filter (*tempCloud);
       cloud = tempCloud;

       // normals
       Tree::Ptr tree(new Tree);
       std::cout << "  normals" << std::endl;
       pcl::NormalEstimation<Point, Normal> ne;
       ne.setInputCloud(cloud);
       ne.setSearchMethod(tree);
       ne.setKSearch(10);
       ne.compute(*normals);

       std::cout << "  done" << std::endl;
   }
