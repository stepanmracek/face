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
    cv::minMaxIdx(pointsMat.colRange(0,1), &minx, &maxx);
    cv::minMaxIdx(pointsMat.colRange(1,2), &miny, &maxy);
    cv::minMaxIdx(pointsMat.colRange(2,3), &minz, &maxz);
}

void Mesh::centralize()
{
    int count = pointsMat.rows;
    double sumx = cv::sum(pointsMat.colRange(0,1))[0];
    double sumy = cv::sum(pointsMat.colRange(1,2))[0];
    double sumz = cv::sum(pointsMat.colRange(2,3))[0];

    for (int i = 0; i < count; i++)
    {
        pointsMat(i, 0) -= sumx/count;
        pointsMat(i, 1) -= sumy/count;
        pointsMat(i, 2) -= sumz/count;
    }

    minx -= sumx/count;
    miny -= sumy/count;
    maxx -= sumx/count;
    maxy -= sumy/count;
    minz -= sumz/count;
    maxz -= sumz/count;
}

void Mesh::translate(cv::Point3d translationVector)
{
    Procrustes3D::translate(pointsMat, translationVector);
    minx += translationVector.x;
    miny += translationVector.y;
    minz += translationVector.z;
    maxx += translationVector.x;
    maxy += translationVector.y;
    maxz += translationVector.z;
}

void Mesh::scale(cv::Point3d scaleParam)
{
    Procrustes3D::scale(pointsMat, scaleParam);
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
    Procrustes3D::rotate(pointsMat, x, y, z);
    recalculateMinMax();
}

void Mesh::transform(Matrix &m)
{
    Procrustes3D::transform(pointsMat, m);
    recalculateMinMax();
}

void Mesh::calculateTriangles()
{
    QVector<cv::Point2d> points2d;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        cv::Point2d p;
        p.x = pointsMat(r, 0);
        p.y = pointsMat(r, 1);
        points2d.append(p);
    }

    triangles = Delaunay::process(points2d);
}

Mesh Mesh::fromXYZ(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename;
    assert(filename.endsWith(".xyz", Qt::CaseInsensitive));
    QFile f(filename);
    bool exists = f.exists();
    assert(exists);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QTextStream in(&f);

    double x,y,z;
    VectorOfPoints points;
    while (!in.atEnd())
    {
        in >> x; in >> y; in >> z;

        if (in.status() == QTextStream::ReadPastEnd)
            break;

        cv::Point3d p;
        p.x = x; p.y = y; p.z = z;
        points.append(p);
    }
    f.close();

    return Mesh::fromPointcloud(points, centralizeLoadedMesh);
}

Mesh Mesh::fromABS(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename;
    assert(filename.endsWith(".abs", Qt::CaseInsensitive));
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

    VectorOfPoints points;
    for (int i = 0; i < total; i++)
    {
        if (flags[i])
        {
            cv::Point3d p;
            p.x = xPoints[i];
            p.y = yPoints[i];
            p.z = zPoints[i];
            points.append(p);
        }
    }

    delete [] xPoints;
    delete [] yPoints;
    delete [] zPoints;

    return Mesh::fromPointcloud(points, centralizeLoadedMesh);
}

Mesh Mesh::fromABS(const QString &filename, const QString &texture, bool centralizeLoadedMesh)
{
    assert(filename.endsWith(".abs", Qt::CaseInsensitive));

    cv::Mat_<cv::Vec3b> image = cv::imread(texture.toStdString());

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

    VectorOfPoints points;
    VectorOfColors colors;
    for (int i = 0; i < total; i++)
    {
        if (flags[i])
        {
            cv::Point3d p;
            p.x = xPoints[i];
            p.y = yPoints[i];
            p.z = zPoints[i];
            points.append(p);

            int x = i % 640;
            int y = i / 640;
            colors << image(y, x);
        }
    }

    delete [] xPoints;
    delete [] yPoints;
    delete [] zPoints;

    Mesh mesh = Mesh::fromPointcloud(points, centralizeLoadedMesh);
    mesh.colors = colors;
    return mesh;
}

Mesh Mesh::fromPointcloud(VectorOfPoints &pointcloud, bool centralizeLoadedMesh, bool calculateTriangles)
{
    Mesh m;

    int n = pointcloud.count();
    m.pointsMat = Matrix(n, 3);
    for (int i = 0; i < n; i++)
    {
        const cv::Point3d &p = pointcloud.at(i);
        m.pointsMat(i, 0) = p.x;
        m.pointsMat(i, 1) = p.y;
        m.pointsMat(i, 2) = p.z;
    }

    if (calculateTriangles) m.calculateTriangles();
    m.recalculateMinMax();
    if (centralizeLoadedMesh) m.centralize();

    return m;
}

Mesh Mesh::fromMap(Map &depth, Map &intensities, bool centralizeLoadedMesh)
{
    assert(depth.w == intensities.w);
    assert(depth.h == intensities.h);
    QMap<QPair<int,int>, int> coordToIndex;

    VectorOfPoints points;
    VectorOfColors colors;
    int index = 0;
    for (int y = 0; y < depth.h; y++)
    {
        for (int x = 0; x < depth.w; x++)
        {
            if (depth.isSet(x,y))
            {
                assert(intensities.isSet(x,y));

                points << cv::Point3d(x, depth.h-y-1, depth.get(x,y));

                uchar intensity = intensities.get(x, y);
                colors << cv::Vec3b(intensity, intensity, intensity);

                coordToIndex[QPair<int,int>(x,y)] = index;
                index++;
            }
        }
    }

    Mesh mesh = Mesh::fromPointcloud(points, centralizeLoadedMesh, false);
    mesh.colors = colors;

    // triangles
    for (int y = 0; y < depth.h; y++)
    {
        for (int x = 0; x < depth.w; x++)
        {
            if (depth.isSet(x,y) &&
                depth.isValidCoord(x, y+1) && depth.isSet(x, y+1) &&
                depth.isValidCoord(x+1, y+1) && depth.isSet(x+1, y+1))
            {
                mesh.triangles << cv::Vec3i(coordToIndex[QPair<int,int>(x,y)], coordToIndex[QPair<int,int>(x,y+1)], coordToIndex[QPair<int,int>(x+1,y+1)]);
            }

            if (depth.isSet(x,y) &&
                depth.isValidCoord(x+1, y+1) && depth.isSet(x+1, y+1) &&
                depth.isValidCoord(x+1, y) && depth.isSet(x+1, y))
            {
                mesh.triangles << cv::Vec3i(coordToIndex[QPair<int,int>(x,y)], coordToIndex[QPair<int,int>(x+1,y+1)], coordToIndex[QPair<int,int>(x+1,y)]);
            }
        }
    }

    return mesh;
}

Mesh Mesh::fromOBJ(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename;
    assert(filename.endsWith(".obj", Qt::CaseInsensitive));
    QFile f(filename);
    bool fileExists = f.exists();
    assert(fileExists);
    bool fileOpened = f.open(QIODevice::ReadOnly);
    assert(fileOpened);
    QTextStream in(&f);

    VectorOfPoints points;
    VectorOfTriangles triangles;
    while (!in.atEnd())
    {
        QString line = in.readLine();
        QStringList items = line.split(QChar(' '));

        if (items[0].compare("v") == 0)
        {
            double x = items[1].toDouble();
            double y = items[2].toDouble();
            double z = items[3].toDouble();

            points << cv::Point3d(x,y,z);
        }
        else if (items[0].compare("f") == 0)
        {
            int t1 = items[1].toInt()-1;
            int t2 = items[2].toInt()-1;
            int t3 = items[3].toInt()-1;

            triangles << cv::Vec3i(t1, t2, t3);
        }
    }

    Mesh result = Mesh::fromPointcloud(points, centralizeLoadedMesh, false);
    result.triangles = triangles;
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

    pointsMat = src.pointsMat.clone();
    triangles = src.triangles;
    colors = src.colors;
}

bool Mesh::equals(const Mesh &other)
{
    return cv::countNonZero(pointsMat - other.pointsMat) == 0 &&
            triangles == other.triangles &&
            colors == other.colors;
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

    for (int r = 0; r < pointsMat.rows; r++)
    {
        outStream << "v " << formatNumber(pointsMat(r, 0), decimalPoint)
                  << " " << formatNumber(pointsMat(r, 1), decimalPoint)
                  << " " << formatNumber(pointsMat(r, 2), decimalPoint) << endl;
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

    outStream << pointsMat.rows << " " << triangles.size() << " 0" << endl;

    for (int r = 0; r < pointsMat.rows; r++)
    {;
        outStream << pointsMat(r, 0) << " " << pointsMat(r, 1) << " " << pointsMat(r, 2) << endl;
    }

    int tCount = triangles.count();
    for (int i = 0; i < tCount; i++)
    {
        cv::Vec3i &t = triangles[i];
        outStream << "3 " << t[0] << " " << t[1] << " " << t[2] << endl;
    }
}

void Mesh::writeToDataStream(QDataStream &stream)
{
    stream << pointsMat.rows;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        stream << pointsMat(r, 0);
        stream << pointsMat(r, 1);
        stream << pointsMat(r, 2);
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
}

void Mesh::writeBIN(const QString &path)
{
    qDebug() << "writing to" << path << "...";
    QFile outFile(path);
    outFile.open(QFile::WriteOnly);
    QDataStream stream(&outFile);

    writeToDataStream(stream);

    qDebug() << "...done";
}

void Mesh::writeBINZ(const QString &path)
{
    qDebug() << "writing to buffer";
    QByteArray uncompressed;
    QDataStream stream(&uncompressed, QIODevice::WriteOnly);
    writeToDataStream(stream);

    qDebug() << "compressing...";
    QByteArray compressed = qCompress(uncompressed, 1);
    qDebug() << "compress ratio" << uncompressed.size() << "/" << compressed.size();

    qDebug() << "writing to" << path << "...";
    QFile outFile(path);
    outFile.open(QFile::WriteOnly);
    outFile.write(compressed);

    qDebug() << "...done";
}

char *Mesh::toCharArray(int *resultLength)
{
    // write to datastream
    QByteArray byteArray;
    QDataStream stream(&byteArray, QIODevice::WriteOnly);
    writeToDataStream(stream);

    // copy to array of bytes
    *resultLength = byteArray.size();
    char *array = new char[*resultLength];
    memcpy(array, byteArray.constData(), sizeof(char) * (*resultLength));

    return array;
}

Mesh Mesh::fromCharArray(char *data, int length)
{
    QByteArray byteArray(data, length);
    QDataStream stream(&byteArray, QIODevice::ReadOnly);
    Mesh result;
    fromDataStream(stream, result);
    return result;
}

void Mesh::fromDataStream(QDataStream &stream, Mesh &mesh)
{
    int pCount;
    stream >> pCount;

    mesh.pointsMat = Matrix(pCount, 3);
    for (int r = 0; r < pCount; r++)
    {
        double x,y,z;
        stream >> x; stream >> y; stream >> z;

        mesh.pointsMat(r, 0) = x;
        mesh.pointsMat(r, 1) = y;
        mesh.pointsMat(r, 2) = z;
    }

    int tCount;
    stream >> tCount;
    mesh.triangles = VectorOfTriangles(tCount);
    for (int i = 0; i < tCount; i++)
    {
        int p1, p2, p3;
        stream >> p1; stream >> p2; stream >> p3;
        mesh.triangles[i] = cv::Vec3i(p1, p2, p3);
    }

    int cCount;
    stream >> cCount;
    mesh.colors = VectorOfColors(cCount);
    for (int i = 0; i < cCount; i++)
    {
        uchar r,g,b;
        stream >> r; stream >> g; stream >> b;
        mesh.colors[i] = cv::Vec3b(r, g, b);
    }
}

Mesh Mesh::fromBIN(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename << "...";
    assert(filename.endsWith(".bin", Qt::CaseInsensitive));
    QFile f(filename);
    bool exists = f.exists();
    assert(exists);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QDataStream in(&f);

    Mesh result;
    fromDataStream(in, result);

    if (centralizeLoadedMesh)
    {
        result.centralize();
    }

    result.recalculateMinMax();
    qDebug() << "...done";
    return result;
}

Mesh Mesh::fromBINZ(const QString &filename, bool centralizeLoadedMesh)
{
    qDebug() << "loading" << filename << "...";
    assert(filename.endsWith(".binz", Qt::CaseInsensitive));
    QFile f(filename);
    bool exists = f.exists();
    assert(exists);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QByteArray compressed = f.readAll();

    qDebug() << "uncompressing...";
    QByteArray uncompressed = qUncompress(compressed);

    qDebug() << "reading from buffer...";
    QDataStream stream(&uncompressed, QIODevice::ReadOnly);
    Mesh result;
    fromDataStream(stream, result);

    if (centralizeLoadedMesh)
    {
        result.centralize();
    }

    result.recalculateMinMax();
    qDebug() << "...done";
    return result;
}

void Mesh::printStats()
{
    qDebug() << "x-range: " << minx << maxx;
    qDebug() << "y-range: "<< miny << maxy;
    qDebug() << "z-range: "<< minz << maxz;
    qDebug() << "points: "<< pointsMat.rows;
    qDebug() << "triangles: "<< triangles.count();
}

void Mesh::trainPointIndex(cv::flann::Index &index, cv::Mat &features, const cv::flann::IndexParams &params) const
{
    features = cv::Mat(pointsMat.rows, 3, CV_32F);
    pointsMat.convertTo(features, CV_32F);
    index.build(features, params);
}

void Mesh::getNearestPoints(const Matrix &input, cv::flann::Index &index, Matrix &output) const
{
    for (int r = 0; r < input.rows; r++)
    {
        cv::Mat query;
        input.row(r).convertTo(query, CV_32F);

        std::vector<int> resultIndicies;
        std::vector<float> resultDistances;
        index.knnSearch(query, resultIndicies, resultDistances, 1);

        int pIndex = resultIndicies[0];
        output(r, 0) = pointsMat(pIndex, 0);
        output(r, 1) = pointsMat(pIndex, 1);
        output(r, 2) = pointsMat(pIndex, 2);
    }
}

void Mesh::getNearestPoints(const Matrix &input, Matrix output) const
{
    cv::flann::Index index;
    cv::Mat features;
    trainPointIndex(index, features, cv::flann::LinearIndexParams());
    getNearestPoints(input, index, output);
}

Mesh Mesh::zLevelSelect(double zValue)
{
    VectorOfPoints newPoints;
    VectorOfColors newColors;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        if (pointsMat(r, 2) >= zValue)
        {
            newPoints << cv::Point3d(pointsMat(r, 0), pointsMat(r, 1), pointsMat(r, 2));
            if (colors.count() > 0)
            {
                newColors << colors[r];
            }
        }
    }

    Mesh result = Mesh::fromPointcloud(newPoints, false, true);
    result.colors = newColors;
    return result;
}

Mesh Mesh::radiusSelect(double radius, cv::Point3d center)
{
    VectorOfPoints newPoints;
    VectorOfColors newColors;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        cv::Point3d p(pointsMat(r, 0), pointsMat(r, 1), pointsMat(r, 2));
        if (euclideanDistance(p, center) <= radius)
        {
            newPoints << p;
            if (colors.count() > 0)
            {
                newColors << colors[r];
            }
        }
    }

    Mesh result = Mesh::fromPointcloud(newPoints, false, true);
    result.colors = newColors;
    return result;
}
