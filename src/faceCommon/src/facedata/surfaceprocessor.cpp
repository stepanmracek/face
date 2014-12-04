#include "faceCommon/facedata/surfaceprocessor.h"

#include <math.h>
#include <limits>

#include "faceCommon/facedata/util.h"
#include "faceCommon/linalg/pca.h"
#include "faceCommon/facedata/mdenoise/mdenoise.h"

using namespace Face::FaceData;

void SurfaceProcessor::smooth(Mesh &mesh, double alpha, int steps)
{
    std::map<int, std::set<int>> neighbours;
    int pc = mesh.pointsMat.rows;
    int tc = mesh.triangles.size();
    for (int i = 0; i < tc; i++)
    {
        neighbours[mesh.triangles[i][0]].insert(mesh.triangles[i][1]);
        neighbours[mesh.triangles[i][0]].insert(mesh.triangles[i][2]);

        neighbours[mesh.triangles[i][1]].insert(mesh.triangles[i][0]);
        neighbours[mesh.triangles[i][1]].insert(mesh.triangles[i][2]);

        neighbours[mesh.triangles[i][2]].insert(mesh.triangles[i][0]);
        neighbours[mesh.triangles[i][2]].insert(mesh.triangles[i][1]);
    }

    double * newx = new double[pc];
    double * newy = new double[pc];
    double * newz = new double[pc];
    for (int i = 0; i < steps; i++)
    {
        for (int j = 0; j < pc; j++)
        {
            double sumx = 0.0;
            double sumy = 0.0;
            double sumz = 0.0;
            for (int neighbour : neighbours[j])
            {
                sumx += (mesh.pointsMat(neighbour, 0) - mesh.pointsMat(j, 0));
                sumy += (mesh.pointsMat(neighbour, 1) - mesh.pointsMat(j, 1));
                sumz += (mesh.pointsMat(neighbour, 2) - mesh.pointsMat(j, 2));
            }
            newx[j] = sumx/neighbours[j].size();
            newy[j] = sumy/neighbours[j].size();
            newz[j] = sumz/neighbours[j].size();
        }

        for (int j = 0; j < pc; j++)
        {
            mesh.pointsMat(j, 0) += alpha * newx[j];
            mesh.pointsMat(j, 1) += alpha * newy[j];
            mesh.pointsMat(j, 2) += alpha * newz[j];
        }
    }
    delete [] newx;
    delete [] newy;
    delete [] newz;
}

void SurfaceProcessor::zsmooth(Mesh &mesh, double alpha, int steps)
{
    std::map<int, std::set<int>> neighbours;
    int pc = mesh.pointsMat.rows;
    int tc = mesh.triangles.size();
    for (int i = 0; i < tc; i++)
    {
        neighbours[mesh.triangles[i][0]].insert(mesh.triangles[i][1]);
        neighbours[mesh.triangles[i][0]].insert(mesh.triangles[i][2]);

        neighbours[mesh.triangles[i][1]].insert(mesh.triangles[i][0]);
        neighbours[mesh.triangles[i][1]].insert(mesh.triangles[i][2]);

        neighbours[mesh.triangles[i][2]].insert(mesh.triangles[i][0]);
        neighbours[mesh.triangles[i][2]].insert(mesh.triangles[i][1]);
    }

    std::vector<double> newz(pc);
    for (int i = 0; i < steps; i++)
    {
        for (int j = 0; j < pc; j++)
        {
            double sumz = 0.0;
            for (int neighbour : neighbours[j])
            {
                sumz += (mesh.pointsMat(neighbour, 2) - mesh.pointsMat(j, 2));
            }
            newz[j] = sumz/neighbours[j].size();
        }

        for (int j = 0; j < pc; j++)
        {
            mesh.pointsMat(j, 2) += alpha * newz[j];
        }
    }
}

void SurfaceProcessor::anisotropicDiffusionSmooth(Mesh &mesh, AnisotropicDiffusionType type, double edgeThresh, int steps, double dt) {
    std::map<int, std::set<int>> neighbours;
    int pc = mesh.pointsMat.rows;
    int tc = mesh.triangles.size();
    for (int i = 0; i < tc; i++)
    {
        neighbours[mesh.triangles[i][0]].insert(mesh.triangles[i][1]);
        neighbours[mesh.triangles[i][0]].insert(mesh.triangles[i][2]);

        neighbours[mesh.triangles[i][1]].insert(mesh.triangles[i][0]);
        neighbours[mesh.triangles[i][1]].insert(mesh.triangles[i][2]);

        neighbours[mesh.triangles[i][2]].insert(mesh.triangles[i][0]);
        neighbours[mesh.triangles[i][2]].insert(mesh.triangles[i][1]);
    }

    for (int i = 0; i < steps; i++)
    {
        Mesh mesh0(mesh);
        Mesh tmpMesh(mesh);
        cv::GaussianBlur(mesh.pointsMat, tmpMesh.pointsMat, cv::Size(5, 5), 0.5);
        for (int j = 0; j < pc; j++)
        {
            double gradient;
            double diffusionCoef;
            double rawGradient;
            double stepCoef = 0.0;
            for (int neighbour : neighbours[j])
            {
                gradient = tmpMesh.pointsMat(neighbour, 2) - tmpMesh.pointsMat(j, 2);
                rawGradient = mesh0.pointsMat(neighbour, 2) - mesh0.pointsMat(j, 2);
                diffusionCoef = edgeThresh;
                if (type == PeronaMalic) {
                    diffusionCoef = 1.0 / (1.0 + (gradient*gradient) / (edgeThresh*edgeThresh));
                }
                stepCoef += rawGradient * diffusionCoef;
            }
            mesh.pointsMat(j, 2) = mesh0.pointsMat(j, 2) + dt * stepCoef;
        }
    }
}

void SurfaceProcessor::mdenoising(Mesh &mesh, float sigma, int normalIterations, int vertexIterations) {
    MDenoise::MDenoise mdenoise;
    mdenoise.importModel(mesh);
    mdenoise.denoise(true, sigma, normalIterations, vertexIterations);
    mdenoise.exportVertices(mesh);
}


inline double min(double a, double b, double c)
{
    double m = a < b ? a : b;
    return m < c ? m : c;
}

inline double max(double a, double b, double c)
{
    double m = a > b ? a : b;
    return m > c ? m : c;
}

inline int convert3DmodelToMap(double value, double minVal, double maxVal, double resultSize)
{
    return (value-minVal)/(maxVal-minVal) * resultSize;
}

inline double linearInterpolation(double x1, double y1, double z1,
                                  double x2, double y2, double z2,
                                  double x3, double y3, double z3,
                                  double x, double y)
{
    if (x1 == x2 && y1 == y2 && x1 == x3 && y1 == y3)
    {
        return (z1+z2+z3)/3;
    }

    // Ax + By + Cz + D = 0
    double A = y1*(z2 - z3) + y2*(z3 - z1) + y3*(z1 - z2);
    double B = z1*(x2 - x3) + z2*(x3 - x1) + z3*(x1 - x2);
    double C = x1*(y2 - y3) + x2*(y3 - y1) + x3*(y1 - y2);
    double D = -A*x1 - B*y1 - C*z1;

    double result = -(A/C)*x - (B/C)*y - D/C;
    if (result != result)
    {
        return (z1+z2+z3)/3;
    }
    return result;
}

void SurfaceProcessor::depthmap(const Mesh &mesh, Map &map, cv::Point2d meshStart, cv::Point2d meshEnd, SurfaceDataToProcess dataToProcess)
{
    Map zmap(map.w, map.h);
    int c = mesh.triangles.size();
    for (int i = 0; i < c; i++)
    {
        const cv::Vec3i &t = mesh.triangles[i];
        double x1d = mesh.pointsMat(t[0], 0);
        double y1d = mesh.pointsMat(t[0], 1);
        double x2d = mesh.pointsMat(t[1], 0);
        double y2d = mesh.pointsMat(t[1], 1);
        double x3d = mesh.pointsMat(t[2], 0);
        double y3d = mesh.pointsMat(t[2], 1);

        double v1d, v2d, v3d;
        switch (dataToProcess)
        {
        case ZCoord:
            v1d = mesh.pointsMat(t[0], 2);
            v2d = mesh.pointsMat(t[1], 2);
            v3d = mesh.pointsMat(t[2], 2);
            break;
        case Texture_I:
            v1d = 0.299*mesh.colors[t[0]][2] + 0.587*mesh.colors[t[0]][1] + 0.114*mesh.colors[t[0]][0];
            v2d = 0.299*mesh.colors[t[1]][2] + 0.587*mesh.colors[t[1]][1] + 0.114*mesh.colors[t[1]][0];
            v3d = 0.299*mesh.colors[t[2]][2] + 0.587*mesh.colors[t[2]][1] + 0.114*mesh.colors[t[2]][0];
            break;
        case Texture_R:
            v1d = mesh.colors[t[0]][2];
            v2d = mesh.colors[t[1]][2];
            v3d = mesh.colors[t[2]][2];
            break;
        case Texture_G:
            v1d = mesh.colors[t[0]][1];
            v2d = mesh.colors[t[1]][1];
            v3d = mesh.colors[t[2]][1];
            break;
        case Texture_B:
        default:
            v1d = mesh.colors[t[0]][0];
            v2d = mesh.colors[t[1]][0];
            v3d = mesh.colors[t[2]][0];
            break;
        }
        double z1d = mesh.pointsMat(t[0], 2);
        double z2d = mesh.pointsMat(t[1], 2);
        double z3d = mesh.pointsMat(t[2], 2);

        int x1 = convert3DmodelToMap(x1d, meshStart.x, meshEnd.x, map.w);
        int y1 = convert3DmodelToMap(y1d, meshStart.y, meshEnd.y, map.h);
        int x2 = convert3DmodelToMap(x2d, meshStart.x, meshEnd.x, map.w);
        int y2 = convert3DmodelToMap(y2d, meshStart.y, meshEnd.y, map.h);
        int x3 = convert3DmodelToMap(x3d, meshStart.x, meshEnd.x, map.w);
        int y3 = convert3DmodelToMap(y3d, meshStart.y, meshEnd.y, map.h);

        int triangleMinX = min(x1, x2, x3);
        int triangleMinY = min(y1, y2, y3);
        int triangleMaxX = max(x1, x2, x3);
        int triangleMaxY = max(y1, y2, y3);

        int dx1 = x2-x1; int dy1 = y2-y1;
        int dx2 = x3-x2; int dy2 = y3-y2;
        int dx3 = x1-x3; int dy3 = y1-y3;

        // vyplnovani...
        for (int y = triangleMinY; y <= triangleMaxY; ++y)
        {
            // inicilizace hranove fce v bode [minx, y]
            int e1 = (triangleMinX-x1)*dy1 - (y-y1)*dx1;
            int e2 = (triangleMinX-x2)*dy2 - (y-y2)*dx2;
            int e3 = (triangleMinX-x3)*dy3 - (y-y3)*dx3;

            for (int x = triangleMinX; x <= triangleMaxX; ++x)
            {
                if( e1 <= 0 && e2 <= 0 && e3 <= 0 )
                {
                    if (x >= 0 && x < map.w && y >= 0 && y < map.h)
                    {
                        double v = linearInterpolation(x1, y1, v1d,
                                                       x2, y2, v2d,
                                                       x3, y3, v3d,
                                                       x, y);
                        double z = linearInterpolation(x1, y1, z1d,
                                                       x2, y2, z2d,
                                                       x3, y3, z3d,
                                                       x, y);

                        int y2 = (map.h-1)-y;
                        if (!zmap.flags(y2, x) || (zmap.flags(y2, x) && zmap.values(y2, x) < z))
                        {
                            zmap.set(x, y2, z);
                            map.set(x, y2, v);
                        }
                    }
                }

                // hranova fce o pixel vedle
                e1 += dy1;
                e2 += dy2;
                e3 += dy3;
            }
        }
    }
}

Map SurfaceProcessor::depthmap(const Mesh &mesh, MapConverter &converter,
                               cv::Point2d meshStart, cv::Point2d meshEnd,
                               double scaleCoef, SurfaceDataToProcess dataToProcess)
{
    converter.meshStart = meshStart;
    converter.meshSize = meshEnd - meshStart;

    Map map(converter.meshSize.x * scaleCoef , converter.meshSize.y * scaleCoef);
    depthmap(mesh, map, converter.meshStart, meshEnd, dataToProcess);
    return map;
}

Map SurfaceProcessor::depthmap(const Mesh &mesh, MapConverter &converter, double scaleCoef, SurfaceDataToProcess dataToProcess)
{
    converter.meshStart = cv::Point2d(mesh.minx, mesh.miny);
    converter.meshSize = cv::Point2d(mesh.maxx - mesh.minx, mesh.maxy - mesh.miny);

    Map map(converter.meshSize.x * scaleCoef , converter.meshSize.y * scaleCoef);
    depthmap(mesh, map, converter.meshStart, cv::Point2d(mesh.maxx, mesh.maxy), dataToProcess);
    return map;
}

double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-y1, 2) + pow(x2-y2, 2));
}

CurvatureStruct SurfaceProcessor::calculateCurvatures(Map &depthmap, bool pcl)
{
    CurvatureStruct c;

    c.curvatureGauss.init(depthmap.w, depthmap.h);
    c.curvatureIndex.init(depthmap.w, depthmap.h);
    c.curvaturePcl.init(depthmap.w, depthmap.h);
    c.curvatureK1.init(depthmap.w, depthmap.h);
    c.curvatureK2.init(depthmap.w, depthmap.h);
    c.curvatureMean.init(depthmap.w, depthmap.h);
    c.peaks.init(depthmap.w, depthmap.h);
    c.pits.init(depthmap.w, depthmap.h);
    c.saddles.init(depthmap.w, depthmap.h);
    c.valleys.init(depthmap.w, depthmap.h);

    double k1;
    double k2;

    //qDebug() << "calculating curvatures - k1, k2, pcl";
    for (int x = 0; x < depthmap.w; x++)
    {
        for (int y = 0; y < depthmap.h; y++)
        {
            c.curvatureK1.unset(x, y);
            c.curvatureK2.unset(x, y);
            if (depthmap.isSet(x,y) && depthmap.has8neigbours(x,y))
            {
                double point[] = {(double)x, (double)y, depthmap.get(x, y)};

                // left - right direction
                double first[] = {(double)(x-1), (double)y, depthmap.get(x-1, y)};
                double second[] = {(double)(x+1), (double)y, depthmap.get(x+1, y)};

                double vec1[] = {first[0]-point[0], first[1]-point[1], first[2]-point[2]};
                double vec2[] = {second[0]-point[0], second[1]-point[1], second[2]-point[2]};

                double angle = Util::angle(vec1,vec2,3);
                k1 = M_PI - angle;

                double delta = (first[2]+second[2])/2;
                if (point[2] < delta)
                    k1 = -k1;

                // top - down direction
                first[0] = x; first[1] = y+1; first[2] = depthmap.get(x, y+1);
                second[0] = x; second[1] = y-1; second[2] = depthmap.get(x, y-1);

                vec1[0] = first[0]-point[0]; vec1[1]=first[1]-point[1]; vec1[2]=first[2]-point[2];
                vec2[0] = second[0]-point[0]; vec2[1]=second[1]-point[1]; vec2[2]=second[2]-point[2];

                angle = Util::angle(vec1,vec2,3);
                k2 = M_PI - angle;

                delta = (first[2]+second[2])/2;
                if (point[2] < delta)
                    k2 = -k2;

                if (k1 < k2)
                {
                    double tmp = k1;
                    k1 = k2;
                    k2 = tmp;
                }

                k1 = -k1;
                k2 = -k2;

                c.curvatureK1.set(x, y, k1);
                c.curvatureK2.set(x, y, k2);

                // PCL
                if (pcl)
                {
                    Matrix pcaData(3, 9);
                    pcaData(0, 0) = x-1; pcaData(1, 0) = y-1; pcaData(2, 0) = depthmap.get(x-1, y-1);
                    pcaData(0, 1) = x  ; pcaData(1, 1) = y-1; pcaData(2, 1) = depthmap.get(x  , y-1);
                    pcaData(0, 2) = x+1; pcaData(1, 2) = y-1; pcaData(2, 2) = depthmap.get(x+1, y-1);
                    pcaData(0, 3) = x-1; pcaData(1, 3) = y  ; pcaData(2, 3) = depthmap.get(x-1, y  );
                    pcaData(0, 4) = x  ; pcaData(1, 4) = y  ; pcaData(2, 4) = depthmap.get(x  , y  );
                    pcaData(0, 5) = x+1; pcaData(1, 5) = y  ; pcaData(2, 5) = depthmap.get(x+1, y  );
                    pcaData(0, 6) = x-1; pcaData(1, 6) = y+1; pcaData(2, 6) = depthmap.get(x-1, y+1);
                    pcaData(0, 7) = x  ; pcaData(1, 7) = y+1; pcaData(2, 7) = depthmap.get(x  , y+1);
                    pcaData(0, 8) = x+1; pcaData(1, 8) = y+1; pcaData(2, 8) = depthmap.get(x+1, y+1);
                    cv::PCA cvPca(pcaData, cv::Mat(), CV_PCA_DATA_AS_COL);
                    double l0 = cvPca.eigenvalues.at<double>(2);
                    double l1 = cvPca.eigenvalues.at<double>(1);
                    double l2 = cvPca.eigenvalues.at<double>(0);
                    double pclCurvature = l0/(l0+l1+l2);
                    c.curvaturePcl.set(x, y, pclCurvature);
                }
            }
        }
    }

    for (int x = 0; x < depthmap.w; x++)
    {
        for (int y = 0; y < depthmap.h; y++)
        {
            c.curvatureMean.unset(x, y);
            c.curvatureGauss.unset(x, y);
            c.curvatureIndex.unset(x, y);

            c.peaks.unset(x, y);
            c.pits.unset(x, y);
            c.saddles.unset(x, y);
            c.valleys.unset(x, y);

            if (c.curvatureK1.isSet(x,y) && c.curvatureK2.isSet(x,y))
            {
                double k1 = c.curvatureK1.get(x,y);
                double k2 = c.curvatureK2.get(x,y);

                double mean = (k1 + k2)/2;
                double gauss = k1 * k2;

                c.curvatureMean.set(x, y, mean);
                c.curvatureGauss.set(x, y, gauss);

                if (k1 != k2)
                {
                    double index = 0.5 - M_1_PI*atan((k1+k2)/(k2-k1));
                    c.curvatureIndex.set(x, y, index);
                }

                if (gauss > 0.004 && mean < -0.005)
                    c.peaks.set(x, y, 1);
                if (gauss > 0.001 && mean > 0.001)
                    c.pits.set(x, y, 1);
                if (gauss < -0.002)
                    c.saddles.set(x, y, 1);
                if (fabs(gauss) < 0.001 && mean > 0.006)
                    c.valleys.set(x, y, 1);
            }
        }
    }

    return c;
}

Matrix CurvatureStruct::gaussMatrix()
{
    return curvatureGauss.toMatrix(0, -0.01, 0.01);
}

Matrix CurvatureStruct::indexMatrix()
{
    return curvatureIndex.toMatrix(0, 0, 1);
}

Matrix CurvatureStruct::meanMatrix()
{
    return curvatureMean.toMatrix(0, -0.1, 0.1);
}

Matrix CurvatureStruct::pclMatrix()
{
    return curvaturePcl.toMatrix(0, 0, 0.0025);
}

std::vector<cv::Point3d> SurfaceProcessor::surfaceCurve(const Map &map, const MapConverter &converter, cv::Point3d start,
                                                    cv::Point3d end, int samples, double mapScaleFactor)
{
    if (samples <= 1) throw FACELIB_EXCEPTION("invalid samples count");
    double nan = NAN;
    std::vector<cv::Point3d> result;

    for (int i = 0; i < samples; i++)
    {
        double t = (double)i/(samples-1.0);
        cv::Point3d p = start + (t*(end-start));

        cv::Point2d mapPoint = converter.MeshToMapCoords(map, p);
        bool success;
        double z = map.get(mapPoint.x, mapPoint.y, &success) * mapScaleFactor;
        if (success)
        {
            p.z = z;
        }
        else
        {
            p == cv::Point3d(nan, nan, nan);
        }
        result.push_back(p);
    }

    return result;
}

std::vector<cv::Point3d> SurfaceProcessor::isoGeodeticCurve(const Map &map, const MapConverter &converter,
                                                        const cv::Point3d center, double distance, int samples,
                                                        double mapScaleFactor)
{
    cv::Point2d mapCenter = converter.MeshToMapCoords(map, cv::Point2d(center.x, center.y));

    std::vector<cv::Point3d> result;
    for (int i = 0; i < samples; i++)
    {
        double angle = ((double)i) / samples * 2 * M_PI;
        double dx = sin(angle);
        double dy = -cos(angle);

        cv::Point2d prevPoint(mapCenter.x, mapCenter.y);
        cv::Point2d currentPoint(mapCenter.x, mapCenter.y);
        double prevZ = map.get(mapCenter.x, mapCenter.y) * mapScaleFactor;
        double currentZ = prevZ;
        double cumulativeDistance = 0.0;

        bool hitOutsideMap = false;
        while (cumulativeDistance < distance)
        {
            prevZ = currentZ;
            prevPoint = currentPoint;

            currentPoint.x += dx;
            currentPoint.y += dy;
            bool success;
            currentZ = map.get(currentPoint.x, currentPoint.y, &success) * mapScaleFactor;
            if (!success)
            {
                hitOutsideMap = true;
                break;
            }

            double d = sqrt((currentZ-prevZ)*(currentZ-prevZ) +
                            (currentPoint.x - prevPoint.x)*(currentPoint.x - prevPoint.x) +
                            (currentPoint.y - prevPoint.y)*(currentPoint.y - prevPoint.y)) / mapScaleFactor;
            cumulativeDistance += d;
        }

        if (hitOutsideMap)
        {
            double nan = NAN;
            result.push_back(cv::Point3d(nan, nan, nan));
        }
        else
        {
            result.push_back(converter.MapToMeshCoords(map, currentPoint));
        }
    }

    return result;
}

std::vector<double> SurfaceProcessor::isoGeodeticCurveToEuclDistance(const std::vector<cv::Point3d> &isoCuvre, cv::Point3d center)
{
    std::vector<double> result;
    for (const cv::Point3d &p : isoCuvre)
    {
        result.push_back(Face::LinAlg::euclideanDistance(p, center));
    }
    return result;
}

