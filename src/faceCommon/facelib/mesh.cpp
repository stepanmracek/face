#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <QDebug>
#include <opencv/highgui.h>
#include <opencv2/flann/flann.hpp>

#include "mesh.h"
#include "surfaceprocessor.h"
#include "linalg/delaunay.h"
#include "linalg/common.h"
#include "linalg/procrustes.h"

void Mesh::recalculateMinMax()
{
    minx = 1e300;
    maxx = -1e300;
    miny = 1e300;
    maxy = -1e300;
    minz = 1e300;
    maxz = -1e300;

    int n = points.size();
    for (int i = 0; i < n; i++)
    {
        cv::Point3d &p = points[i];
        if (p.x > maxx) maxx = p.x;
        if (p.x < minx) minx = p.x;
        if (p.y > maxy) maxy = p.y;
        if (p.y < miny) miny = p.y;
        if (p.z > maxz) maxz = p.z;
        if (p.z < minz) minz = p.z;
    }
}

void Mesh::centralize()
{
    qDebug() << "Centering";
    double sumx = 0;
    double sumy = 0;
    double sumz = 0;
    double count = points.count();
    foreach(cv::Point3d p, points)
    {
        sumx += p.x;
        sumy += p.y;
        sumz += p.z;
    }

    for (int i = 0; i < points.count(); i++)
    {
        points[i].x -= sumx/count;
        points[i].y -= sumy/count;
        points[i].z -= sumz/count;
    }

    minx -= sumx/count;
    miny -= sumy/count;
    maxx -= sumx/count;
    maxy -= sumy/count;
    minz -= sumz/count;
    maxz -= sumz/count;

    qDebug() << "..done";
}

void Mesh::translate(cv::Point3d translationVector)
{
    Procrustes3D::translate(points, translationVector);
    minx += translationVector.x;
    miny += translationVector.y;
    minz += translationVector.z;
    maxx += translationVector.x;
    maxy += translationVector.y;
    maxz += translationVector.z;
}

void Mesh::scale(cv::Point3d scaleParam)
{
    Procrustes3D::scale(points, scaleParam);
    minx *= scaleParam.x;
    miny *= scaleParam.y;
    minz *= scaleParam.z;
    maxx *= scaleParam.x;
    maxy *= scaleParam.y;
    maxz *= scaleParam.z;
}

void Mesh::rotate(cv::Vec3d xyz)
{
    rotate(xyz(0), xyz(1), xyz(2));
}

void Mesh::rotate(double x, double y, double z)
{
    /*Matrix Rx = (Matrix(3,3) <<
                 1, 0, 0,
                 0, cos(x), -sin(x),
                 0, sin(x), cos(x));
    Matrix Ry = (Matrix(3,3) <<
                 cos(y), 0, sin(y),
                 0, 1, 0,
                 -sin(y), 0, cos(y));
    Matrix Rz = (Matrix(3,3) <<
                 cos(z), -sin(z), 0,
                 sin(z), cos(z), 0,
                 0, 0, 1);
    Matrix R = Rx*Ry*Rz;

    int n = points.count();
    for (int i = 0; i < n; i++)
    {
        cv::Point3d &p = points[i];

        Matrix v = (Matrix(3,1) << p.x, p.y, p.z);
        Matrix newV = R*v;

        p.x = newV(0);
        p.y = newV(1);
        p.z = newV(2);
    }*/
    Procrustes3D::rotate(points, x, y, z);
    recalculateMinMax();
}

void Mesh::transform(Matrix &m)
{
    Procrustes3D::transform(points, m);

    recalculateMinMax();
}

void Mesh::calculateTriangles()
{
    //qDebug() << "Calculating triangles";

    QVector<cv::Point2d> points2d;
    foreach(cv::Point3d p3d, points)
    {
        cv::Point2d p; p.x = p3d.x; p.y = p3d.y;
        points2d.append(p);
    }

    triangles = Delaunay::process(points2d);
    int c = triangles.count();

    QList<int> toRemove;
    for (int i = 0; i < c; i++)
    {
        double maxd = 0.0;

        cv::Point3d &p1 = points[triangles[i][0]];
        cv::Point3d &p2 = points[triangles[i][1]];
        cv::Point3d &p3 = points[triangles[i][2]];

        double d = euclideanDistance(p1, p2);
        if (d > maxd) maxd = d;
        d = euclideanDistance(p1, p3);
        if (d > maxd) maxd = d;
        d = euclideanDistance(p2, p3);
        if (d > maxd) maxd = d;

        //if (maxd > 30.0)
        //    toRemove.append(i);
    }

    for (int i = toRemove.count()-1; i >= 0; i--)
    {
        triangles.remove(toRemove.at(i));
    }

    //qDebug() << "Triangles done, |triangles| =" << triangles.count();
}

Mesh Mesh::fromXYZ(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename;
    QFile f(filename);
    bool exists = f.exists();
    assert(exists);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QTextStream in(&f);

    Mesh mesh;
    double x,y,z;
    while (!in.atEnd())
    {
        in >> x; in >> y; in >> z;

        if (in.status() == QTextStream::ReadPastEnd)
            break;

        if (x > mesh.maxx) mesh.maxx = x;
        if (x < mesh.minx) mesh.minx = x;
        if (y > mesh.maxy) mesh.maxy = y;
        if (y < mesh.miny) mesh.miny = y;
        if (z > mesh.maxz) mesh.maxz = z;
        if (z < mesh.minz) mesh.minz = z;

        cv::Point3d p;
        p.x = x; p.y = y; p.z = z;
        mesh.points.append(p);
    }
    f.close();

    if (centralizeLoadedMesh)
        mesh.centralize();

    mesh.calculateTriangles();
    return mesh;
}

Mesh Mesh::fromABS(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename;
    QFile f(filename);
    bool exists = f.exists();
    assert(exists);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QTextStream in(&f);

    int mapHeight;
    in >> mapHeight;
    in.readLine();
    int mapwidth;
    in >> mapwidth;

    Mesh mesh;

    in.readLine();
    in.readLine();

    int total = mapwidth*mapHeight;
    qDebug() << "total points" << total;

    int flags[total];
    double *xPoints = new double[total];
    double *yPoints = new double[total];
    double *zPoints = new double[total];

    for (int i = 0; i < total; i++)
    {
        in >> (flags[i]);
    }
    qDebug() << "flags loaded";

    for (int i = 0; i < total; i++)
    {
        in >> (xPoints[i]);
    }
    qDebug() << "x points loaded";

    for (int i = 0; i < total; i++)
    {
        in >> (yPoints[i]);
    }
    qDebug() << "y points loaded";

    for (int i = 0; i < total; i++)
    {
        in >> (zPoints[i]);
    }
    qDebug() << "z points loaded";

    for (int i = 0; i < total; i++)
    {
        if (flags[i])
        {
            cv::Point3d p;
            p.x = xPoints[i];
            p.y = yPoints[i];
            p.z = zPoints[i];
            mesh.points.append(p);
        }
    }

    delete [] xPoints;
    delete [] yPoints;
    delete [] zPoints;

    mesh.recalculateMinMax();
    if (centralizeLoadedMesh)
        mesh.centralize();
    mesh.calculateTriangles();

    return mesh;
}

Mesh Mesh::fromPointcloud(VectorOfPoints &pointcloud, bool centralizeLoadedMesh)
{
    Mesh m;
    m.points = VectorOfPoints(pointcloud);
    m.calculateTriangles();
    m.recalculateMinMax();

    if (centralizeLoadedMesh)
        m.centralize();

    return m;
}

Mesh Mesh::fromMap(Map &map, bool centralizeLoadedMesh)
{
    QMap<int, int> coordToIndex;

    Mesh mesh;
    int index = 0;
    for (int y = 0; y < map.h; y++)
    {
        for (int x = 0; x < map.w; x++)
        {
            if (map.isSet(x,y))
            {
                mesh.points << cv::Point3d(x, map.h-y-1, map.get(x,y));
                int coord = map.coordToIndex(x,y);
                coordToIndex[coord] = index;
                index++;
            }
        }
    }

    if (centralizeLoadedMesh)
        mesh.centralize();

    mesh.recalculateMinMax();

    // triangles
    for (int y = 0; y < map.h; y++)
    {
        for (int x = 0; x < map.w; x++)
        {
            if (map.isSet(x,y) &&
                map.isValidCoord(x, y+1) && map.isSet(x, y+1) &&
                map.isValidCoord(x+1, y+1) && map.isSet(x+1, y+1))
            {
                mesh.triangles << cv::Vec3i(coordToIndex[map.coordToIndex(x,y)], coordToIndex[map.coordToIndex(x,y+1)], coordToIndex[map.coordToIndex(x+1,y+1)]);
                //mesh.triangles.append(cv::Vec3i(map.coordToIndex(x,y), map.coordToIndex(x,y+1), map.coordToIndex(x+1,y+1)));
            }

            if (map.isSet(x,y) &&
                map.isValidCoord(x+1, y+1) && map.isSet(x+1, y+1) &&
                map.isValidCoord(x+1, y) && map.isSet(x+1, y))
            {
                mesh.triangles << cv::Vec3i(coordToIndex[map.coordToIndex(x,y)], coordToIndex[map.coordToIndex(x+1,y+1)], coordToIndex[map.coordToIndex(x+1,y)]);
                //mesh.triangles.append(cv::Vec3i(map.coordToIndex(x,y), map.coordToIndex(x+1,y+1), map.coordToIndex(x+1,y)));
            }
        }
    }

    return mesh;
}

Mesh Mesh::fromOBJ(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename;
    QFile f(filename);
    bool fileExists = f.exists();
    assert(fileExists);
    bool fileOpened = f.open(QIODevice::ReadOnly);
    assert(fileOpened);
    QTextStream in(&f);

    Mesh result;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList items = line.split(QChar(' '));

        if (items[0].compare("v") == 0)
        {
            double x = items[1].toDouble();
            double y = items[2].toDouble();
            double z = items[3].toDouble();

            result.points << cv::Point3d(x,y,z);
        }
        else if (items[0].compare("f") == 0)
        {
            int t1 = items[1].toInt()-1;
            int t2 = items[2].toInt()-1;
            int t3 = items[3].toInt()-1;

            result.triangles << cv::Vec3i(t1, t2, t3);
        }
    }

    if (centralizeLoadedMesh)
        result.centralize();

    result.recalculateMinMax();

    return result;
}

Mesh::Mesh()
{
    minx = 1e300;
    maxx = -1e300;
    miny = 1e300;
    maxy = -1e300;
    minz = 1e300;
    maxz = -1e300;
}

Mesh::Mesh(const Mesh &src)
{
    minx = src.minx;
    maxx = src.maxx;
    miny = src.miny;
    maxy = src.maxy;
    minz = src.minz;
    maxz = src.maxz;

    points = src.points;
    triangles = src.triangles;
    normals = src.normals;
    colors = src.colors;
    curvatures = src.curvatures;
}

Mesh::~Mesh()
{
    //qDebug() << "deleting mesh";
}

QString formatNumber(double n, char decimalPoint)
{
    return QString::number(n).replace('.', decimalPoint);
}

void Mesh::writeOBJ(const QString &path, char decimalPoint)
{
    QFile outFile(path);
    outFile.open(QFile::WriteOnly);
    QTextStream outStream(&outFile);

    int pointCount = points.size();
    for (int i = 0; i < pointCount; i++)
    {
        cv::Point3d &p = points[i];
        outStream << "v " << formatNumber(p.x, decimalPoint) << " " << formatNumber(p.y, decimalPoint) << " " << formatNumber(p.z, decimalPoint) << endl;
    }

    int tCount = triangles.count();
    for (int i = 0; i < tCount; i++)
    {
        cv::Vec3i &t = triangles[i];
        outStream << "f " << (t[0]+1) << " " << (t[1]+1) << " " << (t[2]+1) << endl;
    }
}

void Mesh::writeOFF(const QString &path)
{
    QFile outFile(path);
    outFile.open(QFile::WriteOnly);
    QTextStream outStream(&outFile);

    outStream << "OFF" << endl;

    outStream << points.size() << " " << triangles.size() << " 0" << endl;

    int pointCount = points.size();
    for (int i = 0; i < pointCount; i++)
    {
        cv::Point3d &p = points[i];
        outStream << p.x << " " << p.y << " " << p.z << endl;
    }

    int tCount = triangles.count();
    for (int i = 0; i < tCount; i++)
    {
        cv::Vec3i &t = triangles[i];
        outStream << "3 " << t[0] << " " << t[1] << " " << t[2] << endl;
    }
}

void Mesh::writeBIN(const QString &path)
{
    qDebug() << "writing to" << path << "...";
    QFile outFile(path);
    outFile.open(QFile::WriteOnly);
    QDataStream stream(&outFile);

    stream << points.count();
    foreach (const cv::Point3d &p, points)
    {
        stream << p.x;
        stream << p.y;
        stream << p.z;
    }

    stream << triangles.count();
    foreach (const cv::Vec3i &t, triangles)
    {
        stream << t[0];
        stream << t[1];
        stream << t[2];
    }

    stream << colors.count();
    foreach (const Color &c, colors)
    {
        stream << c[0];
        stream << c[1];
        stream << c[2];
    }

    outFile.flush();
    outFile.close();

    qDebug() << "...done";
}

Mesh Mesh::fromBIN(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename << "...";
    QFile f(filename);
    bool exists = f.exists();
    assert(exists);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QDataStream in(&f);

    Mesh result;

    int pCount;
    in >> pCount;
    result.points = VectorOfPoints(pCount);
    for (int i = 0; i < pCount; i++)
    {
        double x,y,z;
        in >> x; in >> y; in >> z;
        result.points[i] = cv::Point3d(x, y, z);
    }

    int tCount;
    in >> tCount;
    result.triangles = VectorOfTriangles(tCount);
    for (int i = 0; i < tCount; i++)
    {
        int p1, p2, p3;
        in >> p1; in >> p2; in >> p3;
        result.triangles[i] = cv::Vec3i(p1, p2, p3);
    }

    int cCount;
    in >> cCount;
    for (int i = 0; i < cCount; i++)
    {
        uchar r,g,b;
        in >> r; in >> g; in >> b;
        result.colors[i] = cv::Vec3b(r, g, b);
    }

    qDebug() << "...done";
    return result;
}

void Mesh::printStats()
{
    qDebug() << "x-range: " << minx << maxx;
    qDebug() << "y-range: "<< miny << maxy;
    qDebug() << "z-range: "<< minz << maxz;
    qDebug() << "points: "<< points.count();
    qDebug() << "triangles: "<< triangles.count();
}

VectorOfPoints Mesh::getNearestPoints(VectorOfPoints input)
{
    int n = points.size();
    cv::Mat features(n, 3, CV_32F);
    for (int i = 0; i < n; i++)
    {
        features.at<float>(i, 0) = points[i].x;
        features.at<float>(i, 1) = points[i].y;
        features.at<float>(i, 2) = points[i].z;
    }
    cv::flann::LinearIndexParams indexParams; //KDTreeIndexParams
    cv::flann::Index index(features, indexParams);

    VectorOfPoints resultPoints;
    for (int i = 0; i < input.count(); i++)
    {
        cv::Mat query(1, 3, CV_32F);
        query.at<float>(0, 0) = input[i].x;
        query.at<float>(0, 1) = input[i].y;
        query.at<float>(0, 2) = input[i].z;
        std::vector<int> resultIndicies;
        std::vector<float> resultDistances;
        index.knnSearch(query, resultIndicies, resultDistances, 1);

        int pIndex = resultIndicies[0];
        resultPoints << points[pIndex];
    }

    return resultPoints;
}

Mesh Mesh::zLevelSelect(double zValue)
{
    qDebug() << "Mesh::zLevelSelect";
    Mesh result;
    for (int i = 0; i < points.count(); i++)
    {
        if (points[i].z >= zValue)
        {
            result.points << points[i];
            if (colors.count() > 0)
            {
                result.colors << colors[i];
            }
            if (normals.count() > 0)
            {
                result.normals << normals[i];
            }
        }
    }
    result.recalculateMinMax();
    result.calculateTriangles();
    qDebug() << points.count() << result.points.count();
    return result;
}
