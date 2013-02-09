#include "align.h"

#include <pcl/registration/ia_ransac.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>


void downSample(PointCloud::ConstPtr &input, float gridSize, PointCloud::Ptr &output)
{
    //const float gridSize = 0.01f;
    pcl::VoxelGrid<Point> voxGrid;
    voxGrid.setInputCloud (input);
    voxGrid.setLeafSize (gridSize, gridSize, gridSize);
    PointCloud::Ptr tempCloud (new PointCloud);
    voxGrid.filter(*output);
}

void calcNormals(PointCloud::ConstPtr &input, int kSearch, PointCloud::Ptr &output)
{
    //const int kSearch = 10;
    pcl::NormalEstimation<Point, Normal> ne;
    ne.setInputCloud(input);
    ne.setSearchMethod(Tree::Ptr(new Tree));
    ne.setKSearch(kSearch);
    ne.compute(*output);
}

void calcDescriptors(PointCloud::ConstPtr &inputCloud, Normals::ConstPtr &inputNormals,
                     int kSearch, Descriptors::Ptr &outputDescriptors)
{
    //const int kSearch = 20;
    pcl::FPFHEstimation<Point, Normal, Descriptor> featuresEstimation;
    featuresEstimation.setSearchMethod(Tree::Ptr(new Tree));
    featuresEstimation.setKSearch(kSearch);
    featuresEstimation.setInputCloud (inputCloud);
    featuresEstimation.setInputNormals(inputNormals);
    featuresEstimation.compute (*outputDescriptors);
}

Eigen::Matrix4f calcAlignTransform(PointCloud::ConstPtr &targetCloud, Descriptors::ConstPtr &targetDescriptors,
                                   PointCloud::ConstPtr &sourceCloud, Descriptors::ConstPtr &sourceDescriptors,
                                   float minSampleDistance, float maxCorDistance, int maxIterations)
{
    //minSampleDistance = 0.01;
    //maxCorDistance = 0.01*0.01;
    //maxIterations = 5000;
    pcl::SampleConsensusInitialAlignment<Point, Point, Descriptor> alignment;
    alignment.setMinSampleDistance(minSampleDistance);
    alignment.setMaxCorrespondenceDistance(maxCorDistance);
    alignment.setMaximumIterations(maxIterations);

    alignment.setInputTarget(targetCloud);
    alignment.setTargetFeatures(targetDescriptors);

    alignment.setInputCloud(sourceCloud);
    alignment.setSourceFeatures(sourceDescriptors);

    PointCloud result;
    alignment.align(result);
    return alignment.getFinalTransformation();
}


/*void runBruteForce()
{
    Descriptors::Ptr targetDescriptors(new Descriptors);
    detect(target.cloud, target.normals, targetDescriptors);

    pcl::SampleConsensusInitialAlignment<Point, Point, Descriptor> alignment;
    alignment.setMinSampleDistance(0.01);
    alignment.setMaxCorrespondenceDistance(0.01*0.01);
    alignment.setMaximumIterations(5000);
    alignment.setInputTarget(target.cloud);
    alignment.setTargetFeatures(targetDescriptors);
    int n = sources.size();
    for (int i = 0; i < n; i++)
    {
        Data &source = sources[i];
        Descriptors::Ptr sourceDescriptors(new Descriptors);
        detect(source.cloud, source.normals, sourceDescriptors);

        std::cout << "aligning " << (i+1) << std::endl;
        PointCloud::Ptr result(new PointCloud);
        alignment.setInputCloud(source.cloud);
        alignment.setSourceFeatures(sourceDescriptors);
        alignment.align(*result);

        pcl::visualization::PCLVisualizer viz;
        viz.addPointCloud(target.cloud, "target");
        viz.addPointCloud(result, "result");
        viz.spin();
    }
}*/

void align(string targetPath, std::vector<string> &sourcePaths)
{
    PointCloud::Ptr target(new PointCloud);
    pcl::io::loadPCDFile(targetPath, target);
}
