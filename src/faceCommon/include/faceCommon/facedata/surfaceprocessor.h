#ifndef SURFACEPROCESSOR_H
#define SURFACEPROCESSOR_H

#include <opencv2/flann/flann.hpp>

#include "mesh.h"
#include "map.h"

namespace Face {
namespace FaceData {

class CurvatureStruct;
class MapConverter;

class FACECOMMON_EXPORTS SurfaceProcessor
{
private:
    SurfaceProcessor() {}

public:
    static void smooth(Mesh &mesh, double alpha, int steps);
    static void zsmooth(Mesh &mesh, double alpha, int steps);

    enum SurfaceDataToProcess
    {
        ZCoord, Texture_I, Texture_R, Texture_G, Texture_B
    };

    /**
     * Anisotrpic diffusion of z-coordinates
     * Tried parameters on faces that looked quite good:
     *  anisotropicDiffusionSmooth(m1, SurfaceProcessor::PeronaMalic, 5, 8, 0.04);
     *  anisotropicDiffusionSmooth(m1, SurfaceProcessor::PeronaMalic, 10, 16, 0.02);
     *  anisotropicDiffusionSmooth(m1, SurfaceProcessor::Linear, 0.5, 12, 0.04);
     */
    enum AnisotropicDiffusionType {
        Linear, PeronaMalic
    };
    static void anisotropicDiffusionSmooth(Mesh &mesh, AnisotropicDiffusionType type, double edgeThresh, int steps, double dt);

    /**
     * MDenoising algorithm
     * @sigma: <0,1>; smaller more smoothed
     * Tried parameters:
     *   mdenoising(m1, 0.04f, 20, 50);
     *   mdenoising(m1, 0.06f, 20, 50);
     *   mdenoising(m1, 0.06f, 20, 20);
     */
    static void mdenoising(Mesh &mesh, float sigma, int normalIterations, int vertexIterations);

    static CurvatureStruct calculateCurvatures(Map &depthmap, bool pcl = true);

    static Map depthmap(const Mesh &mesh, MapConverter &converter, double scaleCoef, SurfaceDataToProcess dataToProcess);
    static Map depthmap(const Mesh &mesh, MapConverter &converter, cv::Point2d meshStart, cv::Point2d meshEnd, double scaleCoef, SurfaceDataToProcess dataToProcess);

    static std::vector<cv::Point3d> isoGeodeticCurve(const Map &map, const MapConverter &converter, const cv::Point3d center,
                                                 double distance, int samples, double mapScaleFactor);

    static std::vector<cv::Point3d> surfaceCurve(const Map &map, const MapConverter &converter, cv::Point3d start,
                                             cv::Point3d end, int samples, double mapScaleFactor);

    static std::vector<double> isoGeodeticCurveToEuclDistance(const std::vector<cv::Point3d> &isoCuvre, cv::Point3d center);

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

    Matrix gaussMatrix();
    Matrix meanMatrix();
    Matrix indexMatrix();
    Matrix pclMatrix();
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

}
}

#endif // SURFACEPROCESSOR_H
