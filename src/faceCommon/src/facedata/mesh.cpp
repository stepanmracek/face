#include "faceCommon/facedata/mesh.h"

#include <Poco/Path.h>
#include <Poco/InflatingStream.h>
#include <Poco/FileStream.h>
#include <Poco/MemoryStream.h>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include <fstream>
#include <opencv/highgui.h>
#include <opencv2/flann/flann.hpp>
#include <zlib.h>

#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/linalg/delaunay.h"
#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/procrustes.h"
#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::FaceData;

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
    Face::LinAlg::Procrustes3D::translate(pointsMat, translationVector);
    minx += translationVector.x;
    miny += translationVector.y;
    minz += translationVector.z;
    maxx += translationVector.x;
    maxy += translationVector.y;
    maxz += translationVector.z;
}

void Mesh::scale(cv::Point3d scaleParam)
{
    Face::LinAlg::Procrustes3D::scale(pointsMat, scaleParam);
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
    Face::LinAlg::Procrustes3D::rotate(pointsMat, x, y, z);
    recalculateMinMax();
}

void Mesh::transform(Matrix &m)
{
    Face::LinAlg::Procrustes3D::transform(pointsMat, m);
    recalculateMinMax();
}

void Mesh::calculateTriangles()
{
    std::vector<cv::Point2d> points2d;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        cv::Point2d p;
        p.x = pointsMat(r, 0);
        p.y = pointsMat(r, 1);
        points2d.push_back(p);
    }

    triangles = Face::LinAlg::Delaunay::process(points2d);
}

Mesh Mesh::fromFile(const std::string &filename, bool centralizeLoadedMesh)
{
    Poco::Path path(filename);
    auto extension = path.getExtension();

    if (extension.compare("xyz") == 0)       return Mesh::fromXYZ(filename, centralizeLoadedMesh);
    else if (extension.compare("abs") == 0)  return Mesh::fromABS(filename, std::string(), centralizeLoadedMesh);
    else if (extension.compare("obj") == 0)  return Mesh::fromOBJ(filename, centralizeLoadedMesh);
    else if (extension.compare("bin") == 0)  return Mesh::fromBIN(filename, centralizeLoadedMesh);
    else if (extension.compare("binz") == 0) return Mesh::fromBINZ(filename, centralizeLoadedMesh);

    std::cerr << "unknown suffix: " << filename << std::endl;
    return Mesh();
}

Mesh Mesh::fromXYZ(const std::string &filename, bool centralizeLoadedMesh)
{
    std::ifstream in(filename);
    if (!in.is_open())
        throw FACELIB_EXCEPTION("Can't open file " + filename);

    double x,y,z;
    VectorOfPoints points;
    while (in >> x >> y >> z)
    {
        cv::Point3d p(x, y, z);
        points.push_back(p);
    }

    return Mesh::fromPointcloud(points, centralizeLoadedMesh);
}

Mesh Mesh::fromABS(const std::string &filename, const std::string &texture, bool centralizeLoadedMesh)
{
    cv::Mat_<cv::Vec3b> image;
    if (!texture.empty())
        image = cv::imread(texture);

    std::ifstream in(filename);
    if (!in.is_open())
        throw FACELIB_EXCEPTION("can't open file " + filename);

    std::string line;

    int mapHeight;
    in >> mapHeight;
    std::getline(in, line);

    int mapwidth;
    in >> mapwidth;    
    std::getline(in, line);

    std::getline(in, line);
    int total = mapwidth*mapHeight;
    std::vector<int> flags(total);
    std::vector<double> xPoints(total);
    std::vector<double> yPoints(total);
    std::vector<double> zPoints(total);

    for (int i = 0; i < total; i++)
        in >> (flags[i]);
    for (int i = 0; i < total; i++)
        in >> (xPoints[i]);
    for (int i = 0; i < total; i++)
        in >> (yPoints[i]);
    for (int i = 0; i < total; i++)
        in >> (zPoints[i]);

    VectorOfPoints points;
    Colors colors;
    for (int i = 0; i < total; i++)
    {
        if (flags[i])
        {
            cv::Point3d p;
            p.x = xPoints[i];
            p.y = yPoints[i];
            p.z = zPoints[i];
            points.push_back(p);

            if (!texture.empty())
            {
                int x = i % 640;
                int y = i / 640;
                colors.push_back(image(y, x));
            }
        }
    }

    Mesh mesh = Mesh::fromPointcloud(points, centralizeLoadedMesh);
    mesh.colors = colors;
    return mesh;
}

Mesh Mesh::fromPointcloud(VectorOfPoints &pointcloud, bool centralizeLoadedMesh, bool calculateTriangles)
{
    Mesh m;

    int n = pointcloud.size();
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
    if ((depth.w != intensities.w) || (depth.h != intensities.h))
        throw FACELIB_EXCEPTION("incompatibile input map sizes");
    std::map<std::pair<int,int>, int> coordToIndex;

    VectorOfPoints points;
    Colors colors;
    int index = 0;
    for (int y = 0; y < depth.h; y++)
    {
        for (int x = 0; x < depth.w; x++)
        {
            if (depth.isSet(x,y))
            {
                if (!intensities.isSet(x,y)) throw FACELIB_EXCEPTION("intensities point is not valid");

                points.push_back(cv::Point3d(x, depth.h-y-1, depth.get(x,y)));

                uchar intensity = intensities.get(x, y);
                colors.push_back(cv::Vec3b(intensity, intensity, intensity));

                coordToIndex[std::pair<int,int>(x,y)] = index;
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
                mesh.triangles.push_back(cv::Vec3i(coordToIndex[std::pair<int,int>(x,y)], coordToIndex[std::pair<int,int>(x,y+1)], coordToIndex[std::pair<int,int>(x+1,y+1)]));
            }

            if (depth.isSet(x,y) &&
                depth.isValidCoord(x+1, y+1) && depth.isSet(x+1, y+1) &&
                depth.isValidCoord(x+1, y) && depth.isSet(x+1, y))
            {
                mesh.triangles.push_back(cv::Vec3i(coordToIndex[std::pair<int,int>(x,y)], coordToIndex[std::pair<int,int>(x+1,y+1)], coordToIndex[std::pair<int,int>(x+1,y)]));
            }
        }
    }

    return mesh;
}

Mesh Mesh::fromOBJ(const std::string &filename, bool centralizeLoadedMesh)
{
    std::cout << "loading " << filename << std::endl;
    std::ifstream in(filename);
    if (!in.is_open())
        throw FACELIB_EXCEPTION("Can't open file " + filename);

    VectorOfPoints points;
    Triangles triangles;
    std::string line;
    while (std::getline(in, line))
    {
        if (line.empty()) continue;

        if (line[0] == 'v')
        {
            Poco::StringTokenizer tokens(line, " ");
            double x = Poco::NumberParser::parseFloat(tokens[1]);
            double y = Poco::NumberParser::parseFloat(tokens[2]);
            double z = Poco::NumberParser::parseFloat(tokens[3]);

            points.push_back(cv::Point3d(x,y,z));
        }
        else if (line[0] == 'f')
        {
            Poco::StringTokenizer tokens(line, " ");
            int t1 = Poco::NumberParser::parse(tokens[1]) - 1;
            int t2 = Poco::NumberParser::parse(tokens[2]) - 1;
            int t3 = Poco::NumberParser::parse(tokens[3]) - 1;

            triangles.push_back(cv::Vec3i(t1, t2, t3));
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

/*QString formatNumber(double n, char decimalPoint)
{
    return QString::number(n).replace('.', decimalPoint);
}*/

void Mesh::writeOBJ(const std::string &path)
{
    std::ofstream outStream(path);

    for (int r = 0; r < pointsMat.rows; r++)
    {
        outStream << "v " << pointsMat(r, 0)
                  << " " << pointsMat(r, 1)
                  << " " << pointsMat(r, 2) << std::endl;
    }

    int tCount = triangles.size();
    for (int i = 0; i < tCount; i++)
    {
        cv::Vec3i &t = triangles[i];
        outStream << "f " << (t[0]+1) << " " << (t[1]+1) << " " << (t[2]+1) << std::endl;
    }
}

/*void Mesh::writeOFF(const std::string &path)
{
    QFile outFile(QString::fromStdString(path));
    outFile.open(QFile::WriteOnly);
    QTextStream outStream(&outFile);

    outStream << "OFF" << endl;

    outStream << pointsMat.rows << " " << triangles.size() << " 0" << endl;

    for (int r = 0; r < pointsMat.rows; r++)
    {;
        outStream << pointsMat(r, 0) << " " << pointsMat(r, 1) << " " << pointsMat(r, 2) << endl;
    }

    int tCount = triangles.size();
    for (int i = 0; i < tCount; i++)
    {
        cv::Vec3i &t = triangles[i];
        outStream << "3 " << t[0] << " " << t[1] << " " << t[2] << endl;
    }
}*/

void Mesh::writeToDataStream(Poco::BinaryWriter &stream)
{
    stream << pointsMat.rows;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        stream << pointsMat(r, 0);
        stream << pointsMat(r, 1);
        stream << pointsMat(r, 2);
    }

    stream << (int)triangles.size();
    for (const cv::Vec3i &t : triangles)
    {
        stream << t[0];
        stream << t[1];
        stream << t[2];
    }

    stream << (int)colors.size();
    for (const Color &c : colors)
    {
        stream << c[0];
        stream << c[1];
        stream << c[2];
    }
}

void Mesh::writeBIN(const std::string &path)
{
    std::ofstream ofstream(path, std::ios::binary);
    Poco::BinaryWriter writer(ofstream, Poco::BinaryWriter::BIG_ENDIAN_BYTE_ORDER);
    writeToDataStream(writer);
}

void Mesh::writeBINZ(const std::string &path)
{
    // Calculate input buffer size
    size_t size = 3*sizeof(int)                   // # of points, triangles, and colors
            + sizeof(double) * 3 * pointsMat.rows // points
            + sizeof(int) * 3 * triangles.size()  // triangles
            + sizeof(uchar) * 3 * colors.size();  // colors
    char *buffer = new char[size];
    //std::cout << "calculated size: " << size << std::endl;

    // Write data to memory stream
    Poco::MemoryOutputStream mostream(buffer, size);
    Poco::BinaryWriter writer(mostream, Poco::BinaryWriter::BIG_ENDIAN_BYTE_ORDER);
    writeToDataStream(writer);

    // Compress
    uLongf compSize = size + size / 100 + 13;
    unsigned char *compressed = new unsigned char[compSize+4];
    //std::cout << "compressed size: " << compSize << std::endl;
    compress2(compressed + 4, &compSize, (unsigned char*)buffer, size, 1);
    //std::cout << "compressed size: " << compSize << std::endl;

    // Header
    compressed[0] = (size & 0xff000000) >> 24;
    compressed[1] = (size & 0x00ff0000) >> 16;
    compressed[2] = (size & 0x0000ff00) >> 8;
    compressed[3] = (size & 0x000000ff);

    // Write Compressed data to file
    std::ofstream ofstream(path, std::ios::binary);
    ofstream.write((char *)compressed, compSize + 4);
    delete [] compressed;
}

void Mesh::fromDataStream(Poco::BinaryReader &stream, Mesh &mesh)
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
    mesh.triangles = Triangles(tCount);
    for (int i = 0; i < tCount; i++)
    {
        int p1, p2, p3;
        stream >> p1; stream >> p2; stream >> p3;
        mesh.triangles[i] = cv::Vec3i(p1, p2, p3);
    }

    int cCount;
    stream >> cCount;
    mesh.colors = Colors(cCount);
    for (int i = 0; i < cCount; i++)
    {
        uchar r,g,b;
        stream >> r; stream >> g; stream >> b;
        mesh.colors[i] = cv::Vec3b(r, g, b);
    }
}

Mesh Mesh::fromBIN(const std::string &filename, bool centralizeLoadedMesh)
{
    std::ifstream ifstream(filename, std::ios::binary);
    Poco::BinaryReader reader(ifstream, Poco::BinaryReader::BIG_ENDIAN_BYTE_ORDER);
    Mesh result;
    fromDataStream(reader, result);

    if (centralizeLoadedMesh)
    {
        result.centralize();
    }

    result.recalculateMinMax();
    return result;
}

Mesh Mesh::fromBINZ(const std::string &filename, bool centralizeLoadedMesh)
{
    // Get file size
    std::ifstream ifstream(filename, std::ios::binary);
    ifstream.seekg(0, std::ios::end);
    std::streamsize size = ifstream.tellg();
    ifstream.seekg(0, std::ios::beg);
    //std::cout << "compressed size: " << size - 4 << std::endl;

    // read file
    std::vector<char> vec(size);
    if (!ifstream.read(vec.data(), size))
        throw FACELIB_EXCEPTION("Can't read from file " + filename);

    // decompress
    unsigned char *compData = (unsigned char *)vec.data();
    uLongf expectedSize = ((compData[0] << 24) | (compData[1] << 16) | (compData[2] << 8) | compData[3]);
    //std::cout << "expectedSize: " << expectedSize << std::endl;
    unsigned char *decompData = new unsigned char[expectedSize];
    if (int code = uncompress(decompData, &expectedSize, compData + 4, vec.size() - 4) != Z_OK)
        throw FACELIB_EXCEPTION("Decompress error " + filename + ": " + std::to_string(code));

    // create Poco::BinaryReader
    Poco::MemoryInputStream mistream((char *)decompData, expectedSize);
    Poco::BinaryReader reader(mistream, Poco::BinaryReader::BIG_ENDIAN_BYTE_ORDER);

    Mesh result;
    fromDataStream(reader, result);
    delete[] decompData;

    if (centralizeLoadedMesh) result.centralize();
    result.recalculateMinMax();
    return result;
}

void Mesh::printStats() const
{
    std::cout << "x-range: " << minx << " " << maxx << std::endl;
    std::cout << "y-range: " << miny << " " << maxy << std::endl;
    std::cout << "z-range: " << minz << " " << maxz << std::endl;
    std::cout << "points: " << pointsMat.rows << std::endl;
    std::cout << "triangles: " << triangles.size() << std::endl;
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

Mesh Mesh::zLevelSelect(double zValue) const
{
    VectorOfPoints newPoints;
    Colors newColors;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        if (pointsMat(r, 2) >= zValue)
        {
            newPoints.push_back(cv::Point3d(pointsMat(r, 0), pointsMat(r, 1), pointsMat(r, 2)));
            if (colors.size() > 0)
            {
                newColors.push_back(colors[r]);
            }
        }
    }

    Mesh result = Mesh::fromPointcloud(newPoints, false, true);
    result.colors = newColors;
    return result;
}

Mesh Mesh::radiusSelect(double radius, cv::Point3d center) const
{
    VectorOfPoints newPoints;
    Colors newColors;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        cv::Point3d p(pointsMat(r, 0), pointsMat(r, 1), pointsMat(r, 2));
        if (Face::LinAlg::euclideanDistance(p, center) <= radius)
        {
            newPoints.push_back(p);
            if (colors.size() > 0)
            {
                newColors.push_back(colors[r]);
            }
        }
    }

    Mesh result = Mesh::fromPointcloud(newPoints, false, true);
    result.colors = newColors;
    return result;
}

Mesh Mesh::bandPassSelect(double minX, double maxX, double minY, double maxY, double minZ, double maxZ) const
{
    VectorOfPoints newPoints;
    Colors newColors;
    for (int r = 0; r < pointsMat.rows; r++)
    {
        cv::Point3d p(pointsMat(r, 0), pointsMat(r, 1), pointsMat(r, 2));
        if (p.x >= minX && p.x <= maxX &&
            p.y >= minY && p.y <= maxY &&
            p.z >= minZ && p.z <= maxZ)
        {
            newPoints.push_back(p);
            if (colors.size() > 0)
            {
                newColors.push_back(colors[r]);
            }
        }
    }

    Mesh result = Mesh::fromPointcloud(newPoints, false, true);
    result.colors = newColors;
    return result;
}

ImageGrayscale Mesh::preview() const
{
    Mesh mesh = Mesh(*this);
    mesh.centralize();
    mesh = mesh.radiusSelect(150, cv::Point3d((mesh.minx + mesh.maxx)/2, (mesh.miny + mesh.maxy)/2, mesh.maxz));
    mesh.centralize();
    mesh.rotate(0.3, -0.5, 0);

    Face::FaceData::MapConverter c;
    Face::FaceData::Map texture =
            Face::FaceData::SurfaceProcessor::depthmap(mesh, c, cv::Point2d(-160, -240),
                                                       cv::Point2d(160, 240), 2.0,
                                                       Face::FaceData::SurfaceProcessor::Texture_I);

    Matrix textureMatrix = texture.toMatrix(0, 0, 255);
    textureMatrix = textureMatrix(cv::Rect(160, 240, 320, 480));
    ImageGrayscale textureGrayscaleMatrix =
            Face::LinAlg::MatrixConverter::DoubleMatrixToGrayscaleImage(textureMatrix);

    return textureGrayscaleMatrix;
}
