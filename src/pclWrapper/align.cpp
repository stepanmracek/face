/*#include "align.h"

void downSample(PointCloud::Ptr &input, float gridSize, PointCloud::Ptr &output)
{
    //const float gridSize = 0.01f;
    pcl::VoxelGrid<Point> voxGrid;
    voxGrid.setInputCloud (input);
    voxGrid.setLeafSize (gridSize, gridSize, gridSize);
    voxGrid.filter(*output);
}

void calcNormals(PointCloud::Ptr &input, int kSearch, Normals::Ptr &output)
{
    //const int kSearch = 10;
    pcl::NormalEstimation<Point, Normal> ne;
    ne.setInputCloud(input);
    ne.setSearchMethod(Tree::Ptr(new Tree));
    ne.setKSearch(kSearch);
    ne.compute(*output);
}

void calcDescriptors(PointCloud::Ptr &inputCloud, Normals::Ptr &inputNormals,
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

void createPointNormalsCloud(PointCloud::Ptr &cloud, Normals::Ptr &normals, PointNormalCloud::Ptr &result)
{
    int n = cloud->size();
    for (int i = 0; i < n; i++)
    {
        const Point &p = cloud->at(i);
        const Normal &n = normals->at(i);
        PointNormal pn;
        pn.x = p.x; pn.y = p.y; pn.z = p.z;
        pn.normal_x = n.normal_x; pn.normal_y = n.normal_y; pn.normal_z = n.normal_z;
        result->push_back(pn);
    }
}

Eigen::Matrix4f calcAlignTransform(PointCloud::Ptr &targetCloud, Descriptors::Ptr &targetDescriptors,
                                   PointCloud::Ptr &sourceCloud, Descriptors::Ptr &sourceDescriptors,
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

void alignRansac(std::string targetPath, std::vector<std::string> &sourcePaths)
{
    float voxelSize = 0.01f;
    int normalsK = 10;
    int descriptorsK = 15;

    PointCloud::Ptr target(new PointCloud);
    pcl::io::loadPCDFile(targetPath, *target);

    PointCloud::Ptr targetDownSampled(new PointCloud);
    Normals::Ptr targetNormals(new Normals);
    Descriptors::Ptr targetDescriptors(new Descriptors);

    downSample(target, voxelSize, targetDownSampled);
    calcNormals(targetDownSampled, normalsK, targetNormals);
    calcDescriptors(targetDownSampled, targetNormals, descriptorsK, targetDescriptors);

    std::vector<string>::iterator it;
    for (it = sourcePaths.begin(); it != sourcePaths.end(); ++it)
    {
        std::cout << (*it) << std::endl;
        PointCloud::Ptr source(new PointCloud);
        pcl::io::loadPCDFile(*it, *source);

        PointCloud::Ptr sourceDownSampled(new PointCloud);
        Normals::Ptr sourceNormals(new Normals);
        Descriptors::Ptr sourceDescriptors(new Descriptors);

        downSample(source, voxelSize, sourceDownSampled);
        calcNormals(sourceDownSampled, normalsK, sourceNormals);
        calcDescriptors(sourceDownSampled, sourceNormals, descriptorsK, sourceDescriptors);

        Eigen::Matrix4f transform = calcAlignTransform(targetDownSampled, targetDescriptors,
                                                       sourceDownSampled, sourceDescriptors,
                                                       0.1, 0.01, 50000);

        PointCloud transformedSource;
        pcl::transformPointCloud(*source, transformedSource, transform);

        (*target) += transformedSource;
    }

    pcl::io::savePCDFileBinary("../test/result.pcd", *target);
}

void alignICP(std::string targetPath, std::vector<std::string> &sourcePaths)
{
    float voxelSize = 0.02f;
    int normalsK = 10;

    PointCloud::Ptr targetCloud(new PointCloud);
    pcl::io::loadPCDFile(targetPath, *targetCloud);

    PointCloud::Ptr targetCloudDownSampled(new PointCloud);
    Normals::Ptr targetNormals(new Normals);
    PointNormalCloud::Ptr target(new PointNormalCloud);

    downSample(targetCloud, voxelSize, targetCloudDownSampled);
    calcNormals(targetCloudDownSampled, normalsK, targetNormals);
    createPointNormalsCloud(targetCloudDownSampled, targetNormals, target);

    std::vector<string>::iterator it;
    for (it = sourcePaths.begin(); it != sourcePaths.end(); ++it)
    {
        std::cout << (*it) << std::endl;
        PointCloud::Ptr sourceCloud(new PointCloud);
        pcl::io::loadPCDFile(*it, *sourceCloud);

        PointCloud::Ptr sourceCloudDownSampled(new PointCloud);
        Normals::Ptr sourceNormals(new Normals);
        PointNormalCloud::Ptr source(new PointNormalCloud);

        downSample(sourceCloud, voxelSize, sourceCloudDownSampled);
        calcNormals(sourceCloudDownSampled, normalsK, sourceNormals);
        createPointNormalsCloud(sourceCloudDownSampled, sourceNormals, source);

        pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> icp;
        icp.setTransformationEpsilon(1e-6);
        icp.setMaxCorrespondenceDistance(0.2);
        icp.setInputTarget(target);
        icp.setInputCloud(source);
        icp.setMaximumIterations(10000);
        PointNormalCloud result;
        icp.align(result);

        Eigen::Matrix4f transform = icp.getFinalTransformation();

        PointCloud transformedSource;
        pcl::transformPointCloud(*sourceCloud, transformedSource, transform);

        (*targetCloud) += transformedSource;
    }

    pcl::io::savePCDFileBinary("../test/result.pcd", *targetCloud);
}*/
