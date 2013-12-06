#ifndef SURFACEPROCESSOR_H
#define SURFACEPROCESSOR_H

#include <QDebug>
#include <opencv2/flann/flann.hpp>

#include "mesh.h"
#include "map.h"

class CurvatureStruct;
class MapConverter;

enum SurfaceDataToProcess
{
    ZCoord, Texture_I, Texture_R, Texture_G, Texture_B
};

class SurfaceProcessor
{
private:
    SurfaceProcessor() {}

public:
    static void smooth(Mesh &mesh, double alpha, int steps);
    static void zsmooth(Mesh &mesh, double alpha, int steps);

    static CurvatureStruct calculateCurvatures(Map &depthmap, bool pcl = true);

    static Map depthmap(const Mesh &mesh, MapConverter &converter, double scaleCoef, SurfaceDataToProcess dataToProcess);
    static Map depthmap(const Mesh &mesh, MapConverter &converter, cv::Point2d meshStart, cv::Point2d meshEnd, double scaleCoef, SurfaceDataToProcess dataToProcess);

    static QVector<cv::Point3d> isoGeodeticCurve(Map &map, MapConverter &converter, cv::Point3d center,
                                                 double distance, int samples, double mapScaleFactor);

    static QVector<cv::Point3d> surfaceCurve(const Map &map, const MapConverter &converter, cv::Point3d start,
                                             cv::Point3d end, int samples, double mapScaleFactor);

    static QVector<double> isoGeodeticCurveToEuclDistance(const QVector<cv::Point3d> &isoCuvre, cv::Point3d center);

private:
    static void depthmap(const Mesh &f, Map &map, cv::Point2d meshStart, cv::Point2d meshEnd, SurfaceDataToProcess dataToProcess);
};

class CurvatureStruct
{
public:
    Map curvatureK1;
    Map curvatureK2;
    Map curvatureGauss;
    Map curvatureMean;
    Map curvatureIndex;
    Map curvaturePcl;

    Map peaks;
    Map pits;
    Map saddles;
    Map valleys;
};

class MapConverter
{
public:

    cv::Point2d meshStart;
    cv::Point2d meshSize;

    cv::Point2d MeshToMapCoords(const Map &map, cv::Point3d meshCoords) const
    {
        double x = ( (meshCoords.x-meshStart.x) / meshSize.x) * map.w;
        double y = ( (meshCoords.y-meshStart.y) / meshSize.y) * map.h;
        y = (map.h - 1) - y;

        return cv::Point2d(x, y);
    }

    cv::Point2d MeshToMapCoords(const Map &map, cv::Point2d meshCoords) const
    {
        double x = ( (meshCoords.x-meshStart.x) / meshSize.x) * map.w;
        double y = ( (meshCoords.y-meshStart.y) / meshSize.y) * map.h;
        y = (map.h - 1) - y;

        return cv::Point2d(x, y);
    }

    cv::Point3d MapToMeshCoords(const Map &map, cv::Point2d mapCoords, bool *success = 0) const
    {
        double z = map.get(mapCoords.x, mapCoords.y, success);

        double realMapY = (map.h - 1) - mapCoords.y;

        double x = mapCoords.x/map.w * meshSize.x + meshStart.x;
        double y = realMapY/map.h * meshSize.y + meshStart.y;

        return cv::Point3d(x, y, z);
    }

    cv::Point3d MapToMeshCoordsSafe(const Map &map, cv::Point2d mapCoords, double safeZValue) const
    {
        double z = map.getSafe(mapCoords.x, mapCoords.y, safeZValue);

        double realMapY = (map.h - 1) - mapCoords.y;

        double x = mapCoords.x/map.w * meshSize.x + meshStart.x;
        double y = realMapY/map.h * meshSize.y + meshStart.y;

        return cv::Point3d(x, y, z);
    }
};

#endif // SURFACEPROCESSOR_H
