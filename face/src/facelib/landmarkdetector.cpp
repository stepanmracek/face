#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

#include "surfaceprocessor.h"
#include "landmarkdetector.h"
#include "util.h"
#include "map.h"
#include "maskedvector.h"

Landmarks LandmarkDetector::Detect(Mesh &f)
{
    Landmarks s;

    s.set(Landmarks::Nosetip, Nosetip(f));

    return s;
}

cv::Point3d LandmarkDetector::Nosetip(Mesh &f)
{   
    // Convert to range image
    MapConverter converter;
    Map dmap = SurfaceProcessor::depthmap(f, converter, 2);

    // smooth
    SurfaceProcessor::smooth(dmap, 1.0, 20);

    // erode and select only points with positive z-coordinate
    dmap.erode(7);
    dmap.levelSelect(0.0);

    // calculate curvature
    CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(dmap);

    // peak density map
    Map peakDensityMap = cs.peaks.densityMap(10, true);

    //cv::imshow("peak density", peakDensityMap.toMatrix(0, 0, 1));
    //cv::waitKey();

    int i = peakDensityMap.maxIndex();
    int x = peakDensityMap.indexToX(i);
    int y = peakDensityMap.indexToY(i);
    qDebug() << "Nose:"  << x << y;

    cv::Point3d nosetip = converter.MapToMeshCoords(dmap, cv::Point2d(x,y));
    return nosetip;
}

/*Vector maxProfile = dmap.maxVerticalProfile();
Vector medianProfile = dmap.medianVerticalProfile();
Vector diffProfile = Vector::diff(&maxProfile, &medianProfile);

int noseY = diffProfile.maxIndex();

img = Converter::mapToImage(cs.peaks);
cvSaveImage("peaks.png", img);
cvReleaseImage(&img);

Vector peakDensity = cs.peaks->horizontalPointDensity(noseY, 10);
peakDensity.savePlot("peakDensity.plt");

int noseX = peakDensity.maxIndex();

qDebug() << "Nose:" << noseX << noseY;

qDebug() << "Peak Density Map";*/
