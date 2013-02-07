#include "align.h"

#include <pcl/registration/ia_ransac.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>

Align::Align(Data &target, std::vector<Data> &sources):
    target(target), sources(sources)
{
}

void detect(PointCloud::Ptr &cloud, Normals::Ptr &normals,
            Keypoints::Ptr &keypoints, PointCloud::Ptr &keypointsXYZ, Descriptors::Ptr &features)
{
    // compute keypoints
    std::cout << "keypoints..." << std::endl;
    const float minScale = 0.01;
    const int nrOctaves = 3;
    const int nrScalesPerOctave = 3;
    const float minContrast = 10.0;
    Tree::Ptr tree(new Tree);
    pcl::SIFTKeypoint<Point, Keypoint> siftDetect;
    siftDetect.setInputCloud (cloud);
    siftDetect.setSearchSurface(cloud);
    siftDetect.setSearchMethod(tree);
    siftDetect.setScales (minScale, nrOctaves, nrScalesPerOctave);
    siftDetect.setMinimumContrast (minContrast);
    siftDetect.compute(*keypoints);

    // compute features
    std::cout << "features..." << std::endl;
    pcl::FPFHEstimation<Point, Normal, Descriptor> featuresEstimation;
    featuresEstimation.setSearchMethod(Tree::Ptr(new Tree));
    featuresEstimation.setKSearch(20);

    pcl::copyPointCloud (*keypoints, *keypointsXYZ);

    featuresEstimation.setInputCloud (keypointsXYZ);
    featuresEstimation.setSearchSurface(cloud);
    featuresEstimation.setInputNormals(normals);
    featuresEstimation.compute (*features);

    std::cout << "detected " << features->size() << " features" << std::endl;
}

void findCorrespondences (Descriptors::Ptr &sourceDescriptors, Descriptors::Ptr &targetDescriptors,
                          std::vector<int> &outCorrespondences, std::vector<float> &outCorrespondenceScores)
{
    // Resize the output vector
    outCorrespondences.resize (sourceDescriptors->size ());
    outCorrespondenceScores.resize (sourceDescriptors->size ());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::KdTreeFLANN<Descriptor> descriptorKDtree;
    descriptorKDtree.setInputCloud(targetDescriptors);

    // Find the index of the best match for each keypoint, and store it in "outCorrespondences"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (size_t i = 0; i < sourceDescriptors->size (); ++i)
    {
        descriptorKDtree.nearestKSearch (*sourceDescriptors, i, k, k_indices, k_squared_distances);
        outCorrespondences[i] = k_indices[0];
        outCorrespondenceScores[i] = k_squared_distances[0];
    }
}

void visualizeKeypoints (const PointCloud::Ptr points,
                         const Keypoints::Ptr keypoints)
{
  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");

  // Draw each keypoint as a sphere
  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];

    // Pick the radius of the sphere *
    float r = 2 * p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint

    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;

    // Add a sphere at the keypoint
    viz.addSphere (p, 2*p.scale, 1.0, 0.0, 0.0, ss.str ());
  }

  // Give control over to the visualizer
  viz.spin ();
}

void visualizeCorrespondences (const PointCloud::Ptr points1,
                               const Keypoints::Ptr keypoints1,
                               const PointCloud::Ptr points2,
                               const Keypoints::Ptr keypoints2,
                               const std::vector<int> &correspondences,
                               const std::vector<float> &correspondenceScores)
{
    // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
    // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

    // Create some new point clouds to hold our transformed data
    PointCloud::Ptr points_left (new PointCloud);
    Keypoints::Ptr keypoints_left (new Keypoints);
    PointCloud::Ptr points_right (new PointCloud);
    Keypoints::Ptr keypoints_right (new Keypoints);

    // Shift the first clouds' points to the left
    //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
    const Eigen::Vector3f translate (0.4, 0.0, 0.0);
    const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
    pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
    pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

    // Shift the second clouds' points to the right
    pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
    pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

    // Add the clouds to the vizualizer
    pcl::visualization::PCLVisualizer viz;
    viz.addPointCloud (points_left, "points_left");
    viz.addPointCloud (points_right, "points_right");

    // Compute the median correspondence score
    std::vector<float> temp (correspondenceScores);
    std::sort (temp.begin (), temp.end ());
    float median_score = temp[temp.size ()/2];

    // Draw lines between the best corresponding points
    for (size_t i = 0; i < keypoints_left->size (); ++i)
    {
        if (correspondenceScores[i] > median_score)
        {
            continue; // Don't draw weak correspondences
        }

        // Get the pair of points
        const pcl::PointWithScale & p_left = keypoints_left->points[i];
        const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];

        // Generate a random (bright) color
        double r = (rand() % 100);
        double g = (rand() % 100);
        double b = (rand() % 100);
        double max_channel = std::max (r, std::max (g, b));
        r /= max_channel;
        g /= max_channel;
        b /= max_channel;

        // Generate a unique string for each line
        std::stringstream ss ("line");
        ss << i;

        // Draw the line
        viz.addLine (p_left, p_right, r, g, b, ss.str ());
    }

    // Give control over to the visualizer
    viz.spin ();
}

void detect(PointCloud::Ptr &cloud, Normals::Ptr &normals, Descriptors::Ptr &features)
{
    // compute features
    std::cout << "features..." << std::endl;
    pcl::FPFHEstimation<Point, Normal, Descriptor> featuresEstimation;
    featuresEstimation.setSearchMethod(Tree::Ptr(new Tree));
    featuresEstimation.setKSearch(20);
    featuresEstimation.setInputCloud (cloud);
    featuresEstimation.setInputNormals(normals);
    featuresEstimation.compute (*features);

    std::cout << "detected " << features->size() << " features" << std::endl;
}

void Align::runBruteForce()
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
}

void Align::runSIFT()
{
    Keypoints::Ptr targetKeypoints(new Keypoints);
    Descriptors::Ptr targetFeatures(new Descriptors);
    PointCloud::Ptr targetKeypointsXYZ(new PointCloud);

    detect(target.cloud, target.normals, targetKeypoints, targetKeypointsXYZ, targetFeatures);

    int n = sources.size();
    for (int i = 0; i < 1; i++)
    {
        Data &source = sources[i];
        Keypoints::Ptr sourceKeypoints(new Keypoints);
        Descriptors::Ptr sourceFeatures(new Descriptors);
        PointCloud::Ptr sourceKeypointsXYZ(new PointCloud);

        detect(source.cloud, source.normals, sourceKeypoints, sourceKeypointsXYZ, sourceFeatures);
        std::vector<int> correspondences;
        std::vector<float> correspondenceScores;
        findCorrespondences(sourceFeatures, targetFeatures, correspondences, correspondenceScores);
        visualizeCorrespondences(source.cloud, sourceKeypoints, target.cloud, targetKeypoints,
                                 correspondences, correspondenceScores);
    }
}
