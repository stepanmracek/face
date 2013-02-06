#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> Normals;
typedef pcl::search::KdTree<Point> Tree;
typedef pcl::FPFHSignature33 Descriptor;
typedef pcl::PointCloud<Descriptor> Descriptors;
typedef pcl::PointWithScale Keypoint;
typedef pcl::PointCloud<Keypoint> Keypoints;

struct Data
{
    PointCloud::Ptr cloud;
    Normals::Ptr normals;
    //Tree::Ptr tree;
    //Features::Ptr features;

    Data(const char *path);
};

void loadTransformAndSave(const char *in, const char *out, float mx, float my, float mz, float tz);

#endif // COMMON_H
