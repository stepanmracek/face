#ifndef SURFACEPROCESSOR_H
#define SURFACEPROCESSOR_H

#include <QDebug>

#include "mesh.h"
#include "map.h"

class CurvatureStruct;
class MapConverter;

enum SurfaceDataToProcess
{
    ZCoord, Texture, Curvature
};

class SurfaceProcessor
{
private:
    SurfaceProcessor() {}

public:
    //static void smooth(Mesh *f, double alpha, int steps);
    static void smooth(Map &map, double alpha, int steps);
    static void smooth(Mesh &mesh, double alpha, int steps);
    static void smooth(Mesh &mesh, int knn, double alpha, int steps);

    //static void interpolate(Mesh &f);
    static CurvatureStruct calculateCurvatures(Map &depthmap);

    static Map depthmap(Mesh &mesh, MapConverter &converter, double scaleCoef, SurfaceDataToProcess dataToProcess);
    static Map depthmap(Mesh &mesh, MapConverter &converter, cv::Point2d meshStart, cv::Point2d meshEnd, double scaleCoef, SurfaceDataToProcess dataToProcess);

    static QVector<cv::Point3d> isoGeodeticCurve(Mesh &f, cv::Point3d center, double distance, int samples,
                                                 double mapScaleFactor, double smoothAlpha = 0, int smoothIterations = 0);
    static QVector<cv::Point3d> isoGeodeticCurve(Map &map, MapConverter &converter, cv::Point3d center,
                                                 double distance, int samples, double mapScaleFactor);

    static QVector<double> isoGeodeticCurveToEuclDistance(QVector<cv::Point3d> &isoCuvre, cv::Point3d &center);

    static void calculateNormals(Mesh &mesh, int knn);

private:
    static void depthmap(Mesh &f, Map &map, cv::Point2d meshStart, cv::Point2d meshEnd, SurfaceDataToProcess dataToProcess);
};

class CurvatureStruct
{
public:
    Map curvatureK1;
    Map curvatureK2;
    Map curvatureGauss;
    Map curvatureMean;
    Map curvatureIndex;

    Map peaks;
    Map pits;
    Map saddles;
    Map valleys;

    /*virtual ~CurvatureStruct()
    {
        qDebug() << "deleting CurvatureStruct";
        if (curvatureK1) delete curvatureK1;
        if (curvatureK2) delete curvatureK2;
        if (curvatureGauss) delete curvatureGauss;
        if (curvatureMean) delete curvatureMean;
        if (curvatureIndex) delete curvatureIndex;
        if (peaks) delete peaks;
        if (pits) delete pits;
        if (saddles) delete saddles;
        if (valleys) delete valleys;
    }*/
};

class MapConverter
{
public:

    cv::Point2d meshStart;
    cv::Point2d meshSize;

    cv::Point2d MeshToMapCoords(Map &map, cv::Point3d meshCoords)
    {
        double x = ( (meshCoords.x-meshStart.x) / meshSize.x) * map.w;
        double y = ( (meshCoords.y-meshStart.y) / meshSize.y) * map.h;
        y = (map.h - 1) - y;

        return cv::Point2d(x, y);
    }

    cv::Point2d MeshToMapCoords(Map &map, cv::Point2d meshCoords)
    {
        double x = ( (meshCoords.x-meshStart.x) / meshSize.x) * map.w;
        double y = ( (meshCoords.y-meshStart.y) / meshSize.y) * map.h;
        y = (map.h - 1) - y;

        return cv::Point2d(x, y);
    }

    cv::Point3d MapToMeshCoords(Map &map, cv::Point2d mapCoords)
    {
        double z = map.get(mapCoords.x, mapCoords.y);

        double realMapY = (map.h - 1) - mapCoords.y;

        double x = mapCoords.x/map.w * meshSize.x + meshStart.x;
        double y = realMapY/map.h * meshSize.y + meshStart.y;

        return cv::Point3d(x, y, z);
    }
};

#endif // SURFACEPROCESSOR_H
