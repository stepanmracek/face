#include <QDebug>
#include <math.h>
#include <limits>

#include "util.h"
#include "surfaceprocessor.h"

void SurfaceProcessor::smooth(Map &map, double alpha, int steps)
{
    Map newMap(map.w, map.h);
    int n = map.w*map.h;

    for (int i = 0; i < steps; i++)
    {
        qDebug() << "Smoothing, step:" << (i+1) << "/" << steps;
        for (int j = 0; j < n; j++)
            newMap.unset(j);

        for (int y = 0; y < map.h; y++)
        {
            for (int x = 0; x < map.w; x++)
            {
                if (!map.isSet(x,y)) continue;

                double otherSum = 0.0;
                int otherCount = 0;
                //newZ.set(x, y,  map.values[j]);

                int delta = 1;
                for (int xShift = -delta; xShift <= delta; xShift++)
                {
                    for (int yShift = -delta; yShift <= delta; yShift++)
                    {
                        if (xShift == 0 && yShift == 0) continue;

                        int x2 = x+xShift; int y2 = y+yShift;
                        if (x2 < 0 || x2 >= map.w || y2 < 0 || y2 > map.h) continue;

                        if (map.isSet(x2, y2))
                        {
                            otherSum += (map.get(x2, y2) - map.get(x,y));
                            otherCount++;
                        }
                    }
                }

                if (otherCount > 0)
                {
                    newMap.set(x, y, otherSum/otherCount);
                }
                else
                {
                    newMap.set(x, y, map.get(x, y));
                }
            }
        }

        for (int j = 0; j < n; j++)
        {
            if (map.flags[j] && newMap.flags[j])
            {
                map.values[j] = map.values[j] + alpha*newMap.values[j];
            }
        }
    }
}

void SurfaceProcessor::smooth(Mesh &mesh, double alpha, int steps)
{
    // neighbours initialization
    QMap<int, QSet<int> > neighbours;
    int pc = mesh.points.count();
    for (int i = 0; i < pc; i++)
    {
        QSet<int> l;
        neighbours[i] = l;
    }

    int tc = mesh.triangles.count();
    qDebug() << "Smoothing, neighbours initialization";
    for (int i = 0; i < tc; i++)
    {
        neighbours[mesh.triangles[i][0]] << mesh.triangles[i][1];
        neighbours[mesh.triangles[i][0]] << mesh.triangles[i][2];

        neighbours[mesh.triangles[i][1]] << mesh.triangles[i][0];
        neighbours[mesh.triangles[i][1]] << mesh.triangles[i][2];

        neighbours[mesh.triangles[i][2]] << mesh.triangles[i][0];
        neighbours[mesh.triangles[i][2]] << mesh.triangles[i][1];
    }

    double newx[pc];
    double newy[pc];
    double newz[pc];
    for (int i = 0; i < steps; i++)
    {
        qDebug() << "Smoothing, step:" << (i+1) << "/" << steps;
        for (int j = 0; j < pc; j++)
        {
            double sumx = 0.0;
            double sumy = 0.0;
            double sumz = 0.0;
            foreach(int neighbour, neighbours[j])
            {
                sumx += (mesh.points[neighbour].x - mesh.points[j].x);
                sumy += (mesh.points[neighbour].y - mesh.points[j].y);
                sumz += (mesh.points[neighbour].z - mesh.points[j].z);
            }
            newx[j] = sumx/neighbours[j].count();
            newy[j] = sumy/neighbours[j].count();
            newz[j] = sumz/neighbours[j].count();
        }

        for (int j = 0; j < pc; j++)
        {
            mesh.points[j].x += alpha * newx[j];
            mesh.points[j].y += alpha * newy[j];
            mesh.points[j].z += alpha * newz[j];
        }
    }
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
        //qDebug() << "x1 == x2 && y1 == y2 && x1 == x3 && y1 == y3";
        return (z1+z2+z3)/3;
    }

    /*Ax + By + Cz + D = 0*/

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

void SurfaceProcessor::depthmap(Mesh &mesh, Map &map, cv::Point2d meshStart, cv::Point2d meshEnd)
{
    qDebug() << "Depthmap calculation";

    int emptyCounter = 0;
    int c = mesh.triangles.count();
    for (int i = 0; i < c; i++)
    {
        //qDebug() << "triangle"<<i<<"/"<<f->triangleCount;
        cv::Vec3i &t = mesh.triangles[i];
        double x1d = mesh.points[t[0]].x;
        double y1d = mesh.points[t[0]].y;
        double z1d = mesh.points[t[0]].z;
        double x2d = mesh.points[t[1]].x;
        double y2d = mesh.points[t[1]].y;
        double z2d = mesh.points[t[1]].z;
        double x3d = mesh.points[t[2]].x;
        double y3d = mesh.points[t[2]].y;
        double z3d = mesh.points[t[2]].z;

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
        int fillCounter = 0;
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
                        double val = linearInterpolation(x1, y1, z1d,
                                                         x2, y2, z2d,
                                                         x3, y3, z3d,
                                                         x, y);
                        int index = map.coordToIndex(x, (map.h-1)-y);
                        if (!map.flags[index] || (map.flags[index] && map.values[index] < val))
                        {
                            map.set(index, val);
                        }
                    }
                    fillCounter++;
                }
                else
                {
                    //map.unset(x, y);
                }

                // hranova fce o pixel vedle
                e1 += dy1;
                e2 += dy2;
                e3 += dy3;
            }
        }
        if (fillCounter == 0)
        {
            /*qDebug() << "Triangle"<< i <<"empty";
            double minx = min(x1,x2,x3);
            double miny = min(y1,y2,y3);
            qDebug() << x1-minx<<y1-miny<<";"<<x2-minx<<y2-miny<<";"<<x3-minx<<y3-miny<<";";*/
            emptyCounter ++;
        }
    }
    //qDebug() << "Total empty triangles" << emptyCounter;
    qDebug() << "..done";
}

Map SurfaceProcessor::depthmap(Mesh &mesh, MapConverter &converter, cv::Point2d meshStart, cv::Point2d meshEnd, double scaleCoef)
{
    converter.meshStart = meshStart;
    converter.meshSize = meshEnd - meshStart;

    Map map(converter.meshSize.x * scaleCoef , converter.meshSize.y * scaleCoef);
    depthmap(mesh, map, converter.meshStart, meshEnd);
    return map;
}

Map SurfaceProcessor::depthmap(Mesh &mesh, MapConverter &converter, double scaleCoef)
{
    converter.meshStart = cv::Point2d(mesh.minx, mesh.miny);
    converter.meshSize = cv::Point2d(mesh.maxx - mesh.minx, mesh.maxy - mesh.miny);

    Map map(converter.meshSize.x * scaleCoef , converter.meshSize.y * scaleCoef);
    depthmap(mesh, map, converter.meshStart, cv::Point2d(mesh.maxx, mesh.maxy));
    return map;
}

Map SurfaceProcessor::depthmap(Mesh &mesh, MapConverter &converter,
                               int mapWidth, int mapHeight)
{
    converter.meshStart = cv::Point2d(mesh.minx, mesh.miny);
    converter.meshSize = cv::Point2d(mesh.maxx - mesh.minx, mesh.maxy - mesh.miny);

    Map map(mapWidth, mapHeight);
    depthmap(mesh, map, converter.meshStart, cv::Point2d(mesh.maxx, mesh.maxy));
    return map;
}

double dist(double x1, double y1, double x2, double y2)
{
    return sqrt(pow(x1-y1, 2) + pow(x2-y2, 2));
}

CurvatureStruct SurfaceProcessor::calculateCurvatures(Map &depthmap)
{
    CurvatureStruct c;

    c.curvatureGauss.init(depthmap.w, depthmap.h);
    c.curvatureIndex.init(depthmap.w, depthmap.h);
    c.curvatureK1.init(depthmap.w, depthmap.h);
    c.curvatureK2.init(depthmap.w, depthmap.h);
    c.curvatureMean.init(depthmap.w, depthmap.h);
    c.peaks.init(depthmap.w, depthmap.h);
    c.pits.init(depthmap.w, depthmap.h);
    c.saddles.init(depthmap.w, depthmap.h);
    c.valleys.init(depthmap.w, depthmap.h);

    double k1;
    double k2;

    qDebug() << "calculating curvatures - k1 and k2";
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

                //qDebug() << x << y << angle << k1;

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
            }
        }
    }

    qDebug() << "calculating curvatures - mean, gaussian, shape index";

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

                //if (gauss > 0.002 && mean < -0.006)
                if (gauss > 0.002 && mean < -0.005)
                {
                    c.peaks.set(x, y, 1);
                }
                if (gauss > 0.002 && mean > 0.004)
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

QVector<cv::Point3d> SurfaceProcessor::isoGeodeticCurve(Map &map, MapConverter &converter, cv::Point3d center,
                                                        double distance, int samples, double mapScaleFactor)
{
    cv::Point2d mapCenter = converter.MeshToMapCoords(map, cv::Point2d(center.x, center.y));

    QVector<cv::Point3d> result;
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
        while (cumulativeDistance < distance)
        {
            prevZ = currentZ;
            prevPoint = currentPoint;

            currentPoint.x += dx;
            currentPoint.y += dy;
            currentZ = map.get(currentPoint.x, currentPoint.y) * mapScaleFactor;

            double d = sqrt((currentZ-prevZ)*(currentZ-prevZ) +
                            (currentPoint.x - prevPoint.x)*(currentPoint.x - prevPoint.x) +
                            (currentPoint.y - prevPoint.y)*(currentPoint.y - prevPoint.y)) / mapScaleFactor;
            cumulativeDistance += d;
        }
        result << converter.MapToMeshCoords(map, currentPoint);
    }

    return result;
}

QVector<cv::Point3d> SurfaceProcessor::isoGeodeticCurve(Mesh &f, cv::Point3d center, double distance, int samples,
                                                        double mapScaleFactor, double smoothAlpha, int smoothIterations)
{
    MapConverter converter;
    Map map = depthmap(f, converter, mapScaleFactor);

    if (smoothIterations > 0)
    {
        smooth(map, smoothAlpha, smoothIterations);
    }

    return isoGeodeticCurve(map, converter, center, distance, samples, mapScaleFactor);
}

QVector<double> SurfaceProcessor::isoGeodeticCurveToEuclDistance(QVector<cv::Point3d> &isoCuvre, cv::Point3d &center)
{
    QVector<double> result;
    foreach(const cv::Point3d &p, isoCuvre)
    {
        result << euclideanDistance(p, center);
    }
    return result;
}
