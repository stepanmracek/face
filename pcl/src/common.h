#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

typedef pcl::PointXYZRGBA Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> Normals;
typedef pcl::PointNormal PointNormal;
typedef pcl::PointCloud<PointNormal> PointNormalCloud;
typedef pcl::search::KdTree<Point> Tree;
typedef pcl::FPFHSignature33 Descriptor;
typedef pcl::PointCloud<Descriptor> Descriptors;
typedef pcl::PointWithScale Keypoint;
typedef pcl::PointCloud<Keypoint> Keypoints;

void loadTransformAndSave(const char *in, const char *out, float mx, float my, float mz, float tz);

#endif // COMMON_H
