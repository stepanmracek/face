#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "vector.h"
#include "matrixconverter.h"

namespace Face {

namespace FaceData {
class FaceAlignerIcp;
class FaceAlignerLandmark;
class Mesh;
}

namespace LinAlg {

class FACECOMMON_EXPORTS Loader
{
private:
    static void loadMeshesAllocate(const std::string &dir, std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes, std::vector<std::string> &fileNames);

public:
    enum PathType { AbsoluteFull, Filename, BaseFilename };

    static std::vector<Vector> loadShapes(const std::string &dirPath);

    static std::vector<Matrix> loadImages(const std::string &dirPath);

    static void loadImages(const std::string &dirPath, std::vector<Vector> &vectors, std::vector<int> &classes,
                           const std::string &extension = ".png", const std::string &classSeparator = "-",
                           int maxImages = -1, cv::Rect roi = cv::Rect(), bool qdebug = false);

    static void loadImages(const std::string &dirPath, std::vector<Matrix> &matrices, std::vector<int> &classes,
                           const std::string &extension = ".png", const std::string &classSeparator = "-",
                           int maxImages = -1, cv::Rect roi = cv::Rect(), bool qdebug = false);

    static void loadVectors(const std::string &dirPath, std::vector<Vector> &vectors, std::vector<int> &classes,
                            const std::string &classSeparator = "-", const std::string &nameFilter = "*");

    static void loadMatrices(const std::string &dirPath, std::vector<Matrix> &matrices, std::vector<int> &classes,
                             const std::string &classSeparator = "-", const std::string &nameFilter = "*",
                             int maxCount = -1, const std::string &storageKey = "m");

    static void loadMeshes(const std::string &dir,
                           std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes,
                           int smoothIterations, float smoothCoef, const std::string &idSeparator);

	static void loadMeshes(const std::string &dir, const Face::FaceData::FaceAlignerIcp &aligner,
                           std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes, int icpIterations,
                           int smoothIterations, float smoothCoef, const std::string &idSeparator);

    static void loadMeshes(const std::string &dir, const Face::FaceData::FaceAlignerLandmark &aligner,
                           std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes,
                           int smoothIterations, float smoothCoef, const std::string &idSeparator);

    static std::vector<std::string> listFiles(const std::string &path, const std::string &filter, PathType pathType);
    static std::vector<std::string> listFiles(const std::string &path, const std::vector<std::string> &filters, PathType pathType);

    static std::vector<int> getClasses(const std::vector<std::string> &fileNames, const std::string &separator);
};

}
}
