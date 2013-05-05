#ifndef MESH_H
#define MESH_H

#include <QVector>
#include <QtGlobal>
#include <QString>

#include "opencv/cv.h"

#include "map.h"

typedef QVector<cv::Point3d> VectorOfPoints;
typedef QVector<cv::Vec3i> VectorOfTriangles;
typedef cv::Vec3b Color;
typedef QVector<Color> VectorOfColors;

class Mesh;

class Mesh
{
public:
    VectorOfPoints points;
    VectorOfTriangles triangles;
    VectorOfColors colors;
    VectorOfPoints normals;
    QVector<double> curvatures;

    double minx, maxx, miny, maxy, minz, maxz;
    void calculateTriangles();
    void recalculateMinMax();
    void centralize();
    void translate(cv::Point3d translationVector);
    void rotate(double x, double y, double z);
    void rotate(cv::Vec3d xyz);
    void scale(cv::Point3d scaleParam);
    void transform(Matrix &m);
    void printStats();
    VectorOfPoints getNearestPoints(VectorOfPoints input);
    Mesh zLevelSelect(double zValue);

    Mesh();
    Mesh(const Mesh &src);
    virtual ~Mesh();

    void writeOFF(const QString &path);
    void writeOBJ(const QString &path, char decimalPoint);
    void writeBIN(const QString &path);

    static Mesh fromABS(const QString &filename, bool centralizeLoadedMesh = false);
    static Mesh fromBIN(const QString &filename, bool centralizeLoadedMesh = false);
    static Mesh fromOBJ(const QString &filename, bool centralizeLoadedMesh = false);
    static Mesh fromXYZ(const QString &filename, bool centralizeLoadedMesh = false);
    static Mesh fromMap(Map &map, bool centralizeLoadedMesh = false);
    static Mesh fromPointcloud(VectorOfPoints &pointcloud, bool centralizeLoadedMesh = false);
};

#endif // MESH_H
