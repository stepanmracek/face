#include "faceCommon/facedata/landmarkdetector.h"

#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/image_processing/full_object_detection.h>
#include <dlib/opencv.h>

/*#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include "faceCommon/facedata/util.h"
#include "faceCommon/facedata/map.h"
#include "faceCommon/facedata/maskedvector.h"
#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/kernelgenerator.h"*/

using namespace Face::FaceData;

LandmarkDetector::LandmarkDetector(const std::string &shapePredictorPath)
{
    shapePredictor = new dlib::shape_predictor();
    dlib::deserialize(shapePredictorPath) >> (*shapePredictor);
}

std::vector<cv::Point2d> LandmarkDetector::get(ImageGrayscale &img, cv::Rect &roi)
{
    std::vector<cv::Point2d> result;

    dlib::rectangle rect(dlib::point(roi.tl().x, roi.tl().y), dlib::point(roi.br().x, roi.br().y));
    dlib::cv_image<unsigned char> dlibImg(img);
    dlib::full_object_detection detection = (*shapePredictor)(dlibImg, rect);
    unsigned long n = detection.num_parts();
    if (n <= 48) return result;

    for (unsigned long i = 27; i < 48; i++)
    {
        const dlib::point &p = detection.part(i);
        result.push_back(cv::Point2d(p.x(), p.y()));
    }

    return result;
}

/*LandmarkDetector::LandmarkDetector(Mesh &mesh) : mesh(mesh)
{
    stripeWidth = 50; //100;
    depthGaussSize = 7;
    depthGaussIterations = 3;
    depthErode = 7;
    depthLevelSelect = 0;
    depthScale = 1; //2;
    peakDensityWindowsSize = 21;
    pitsStripeSmoothKernel = 7;

    minYDistanceFromNosetipToEyes = 20;
    maxYDistanceFromNosetipToEyes = 70;
    minXDistanceFromNosetipToEyes = 12;
    maxXDistanceFromNosetipToEyes = 20;

    // Convert to range image
    depth = SurfaceProcessor::depthmap(mesh, converter, depthScale, SurfaceProcessor::ZCoord);

    // smooth
    Matrix smoothKernel = Face::LinAlg::KernelGenerator::gaussianKernel(depthGaussSize);
    depth.applyFilter(smoothKernel, depthGaussIterations, true);

    //cv::imshow("kernel", smoothKernel);
    //cv::imshow("smoothed", depth.toMatrix());

    // select only points with z-coordinate higher or equal than some threshold
    depth.levelSelect((mesh.maxz+mesh.minz)/2);
    int cropWidth, cropHeight;
    depth.getCropParams(cropStartX, cropWidth, cropStartY, cropHeight);
    croppedDepth = depth.subMap(cropStartX, cropWidth, cropStartY, cropHeight);

    //cv::imshow("depth", croppedDepth.toMatrix());

    // calculate curvature
    curvature = SurfaceProcessor::calculateCurvatures(croppedDepth, false);

    // density maps
    peakDensity = curvature.peaks.densityMap(peakDensityWindowsSize, true);
    //pitDensity = curvature.pits.densityMap(pitDensityWindowsSize, false);

    //cv::imshow("depth", depth.toMatrix());
    //cv::imshow("croppedDepth", croppedDepth.toMatrix());
    //cv::imshow("curvature", curvature.curvatureIndex.toMatrix());
    //cv::imshow("peaks", curvature.peaks.toMatrix(0, 0, 1));
    //cv::imshow("pits", curvature.pits.toMatrix(0, 0, 1));
    //cv::imshow("peakDensity", peakDensity.toMatrix(0, 0, 1));
    //cv::imshow("pitDensity", pitDensity.toMatrix(0, 0, 1));
    //cv::waitKey();
}

Landmarks LandmarkDetector::detect()
{
    Landmarks l;

    nosetip(l);
    //innerEyeCorners(l);

    return l;
}

void LandmarkDetector::nosetip(Landmarks &l)
{
    //qDebug() << "  nose tip";
    int x = -1;
    int y = -1;
    peakDensity.maxIndex(x, y);
    if (x == -1 && y == -1)
    {
        std::cerr << "Nose coordinates not found" << std::endl;
        return;
    }

    bool ok;
    cv::Point3d nosetip = converter.MapToMeshCoords(depth, cv::Point2d(x + cropStartX, y + cropStartY), &ok);
    if (ok)
    {
        l.set(Landmarks::Nosetip, nosetip);
    }
    else
    {
        std::cerr << "Nose coordinates not found" << std::endl;
    }
}

void LandmarkDetector::innerEyeCorners(Landmarks &l)
{
    cv::Point3d nosetip = l.get(Landmarks::Nosetip);
    cv::Point2d nose2d = converter.MeshToMapCoords(depth, cv::Point2d(nosetip.x, nosetip.y));
    nose2d.x -= cropStartX;
    nose2d.y -= cropStartY;

    // Y coordinate
    int h = depth.h;
    Face::LinAlg::Vector pitsAlongY(h);
    for (int y = 0; y < h; y++)
    {
        int count = 0;
        for (int x = nose2d.x - stripeWidth/2; x <= nose2d.x + stripeWidth/2; x++)
        {
            count = curvature.pits.isSet(x, y) ? count+1 : count;
        }
        pitsAlongY(y) = count;
    }
    Face::LinAlg::Vector smoothedPits = pitsAlongY.smooth(pitsStripeSmoothKernel);

    int eyes2dY = smoothedPits.maxIndex(nose2d.y - maxYDistanceFromNosetipToEyes,
                                        nose2d.y - minYDistanceFromNosetipToEyes);
    if (eyes2dY < 0)
    {
        std::cerr << "Y coordinate of inner eye corners not found" << std::endl;
        return;
    }

    // X coordinate
    int w = depth.w;
    Face::LinAlg::Vector pitsAlongX(w);
    for (int x = 0; x < w; x++)
    {
        int count = 0;
        for (int y = eyes2dY - stripeWidth/2; y <= eyes2dY + stripeWidth/2; y++)
        {
            count = curvature.pits.isSet(x,y) ? count+1 : count;
        }
        pitsAlongX(x) = count;
    }
    smoothedPits = pitsAlongX.smooth(pitsStripeSmoothKernel);

    int leftEye2dX = smoothedPits.maxIndex(nose2d.x - maxXDistanceFromNosetipToEyes,
                                           nose2d.x - minXDistanceFromNosetipToEyes);
    if (leftEye2dX < 0)
    {
        std::cerr << "X coordinate of left inner eye corner not found" << std::endl;
    }
    else
    {
        l.set(Landmarks::LeftInnerEye,
              converter.MapToMeshCoords(depth, cv::Point2d(leftEye2dX + cropStartX, eyes2dY + cropStartY)));
    }

    int rightEye2dX = smoothedPits.maxIndex(nose2d.x + minXDistanceFromNosetipToEyes,
                                            nose2d.x + maxXDistanceFromNosetipToEyes);

    if (rightEye2dX < 0)
    {
        std::cerr << "X coordinate of right inner eye corner not found" << std::endl;
    }
    else
    {
        l.set(Landmarks::RightInnerEye,
              converter.MapToMeshCoords(depth, cv::Point2d(rightEye2dX + cropStartX, eyes2dY + cropStartY)));
    }
}*/
