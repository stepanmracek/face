#pragma once

#include <opencv2/opencv.hpp>
#include <Poco/BinaryReader.h>
#include <Poco/BinaryWriter.h>

#include "faceCommon/faceCommon.h"
#include "faceCommon/linalg/common.h"

namespace Face {
namespace FaceData {

typedef std::vector<cv::Point3d> VectorOfPoints;

class Map;
class MapConverter;
class FACECOMMON_EXPORTS Mesh;

class FACECOMMON_EXPORTS Mesh
{
public:
    Matrix pointsMat;

    typedef cv::Vec3i Triangle;
    typedef std::vector<Triangle> Triangles;
    Triangles triangles;

    typedef cv::Vec3b Color;
    typedef std::vector<Color> Colors;
    Colors colors;

	typedef std::vector<cv::Vec2d> UVMap;
	UVMap uvmap;

    double minx, maxx, miny, maxy, minz, maxz;
	void clear();
    void calculateTriangles();
    void recalculateMinMax();
    cv::Point3d centralize();
    void translate(cv::Point3d translationVector);
    void rotate(double x, double y, double z);
    void rotate(cv::Vec3d xyz);
    void scale(cv::Point3d scaleParam);
    void transform(Matrix &m);
    void printStats() const;
    void getNearestPoints(const Matrix &input, Matrix output) const;
    void getNearestPoints(const Matrix &input, cv::flann::Index &index, Matrix &output) const;
    void trainPointIndex(cv::flann::Index &index, cv::Mat &features, const cv::flann::IndexParams &params) const;
    Mesh zLevelSelect(double zValue) const;
    Mesh radiusSelect(double radius, cv::Point3d center = cv::Point3d(0,0,0)) const;
    Mesh bandPassSelect(double minX, double maxX, double minY, double maxY, double minZ, double maxZ) const;
    ImageGrayscale preview(bool rotate = true) const;

    Mesh();
    Mesh(const Mesh &src);
    virtual ~Mesh();
    bool equals(const Mesh &other);
    Mesh &operator =(const Mesh &src);

    void writeOBJ(const std::string &path, const std::string &textureName = "") const;
    void writeBIN(const std::string &path) const;
    void writeBINZ(const std::string &path) const;

    static Mesh fromABS(const std::string &filename, const std::string &texture, bool centralizeLoadedMesh = false);
    static Mesh fromBIN(const std::string &filename, bool centralizeLoadedMesh = false);
    static Mesh fromBINZ(const std::string &filename, bool centralizeLoadedMesh = false);
    static Mesh fromOBJ(const std::string &filename, bool centralizeLoadedMesh = false);
    static Mesh fromXYZ(const std::string &filename, bool centralizeLoadedMesh = false);
    static Mesh fromFile(const std::string &filename, bool centralizeLoadedMesh = false);
    static Mesh fromMap(const Map &depth, const Map &intensities, bool centralizeLoadedMesh = false);
	static Mesh fromMap(const Map &depth, const MapConverter &converter);
    static Mesh fromPointcloud(const VectorOfPoints &pointcloud, bool centralizeLoadedMesh = false, bool calculateTriangles = true);

private:
    static void fromDataStream(Poco::BinaryReader &stream, Mesh &mesh);
    void writeToDataStream(Poco::BinaryWriter &stream) const;
};

}
}
