#ifndef LOADER_H
#define LOADER_H

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "vector.h"
#include "matrixconverter.h"

namespace Face {
namespace LinAlg {

class Loader
{
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

    static std::vector<std::string> listFiles(const std::string &path, const std::string &filter, PathType pathType);
    static std::vector<std::string> listFiles(const std::string &path, const std::vector<std::string> &filters, PathType pathType);

    static std::vector<int> getClasses(const std::vector<std::string> &fileNames, const std::string &separator);
};

}
}

#endif // LOADER_H
