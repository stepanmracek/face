#include "faceCommon/linalg/loader.h"

#include <Poco/File.h>
#include <Poco/String.h>
#include <Poco/Glob.h>
#include <Poco/Path.h>
#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

#include <faceCommon/facedata/mesh.h>
#include <faceCommon/facedata/facealigner.h>
#include <faceCommon/facedata/surfaceprocessor.h>

using namespace Face::LinAlg;

std::vector<Vector> Loader::loadShapes(const std::string &dirPath)
{
    std::set<std::string> files;
    std::vector<Vector> shapes;

    Poco::Glob::glob(dirPath + Poco::Path::separator() + "*.shape", files);
    for (const std::string &file : files)
    {
        std::cout << "loading " << file.c_str() << std::endl;
        Vector v = Vector::fromTwoColsFile(file);
        shapes.push_back(v);
    }

    return shapes;
}

std::vector<Matrix> Loader::loadImages(const std::string &dirPath)
{
    std::vector<Matrix> images;
    std::set<std::string> files;

    Poco::Glob::glob(dirPath + Poco::Path::separator() + "*.png", files);
    for (const std::string &file : files)
    {
        std::cout << "loading " << file.c_str() << std::endl;

        Matrix img = MatrixConverter::imageToMatrix(file);
        images.push_back(img);
    }

    return images;
}

void Loader::loadImages(const std::string &dirPath, std::vector<Vector> &vectors, std::vector<int> &classes,
                        const std::string &extension, const std::string &classSeparator, int maxImages,
                        cv::Rect roi, bool qdebug)
{
    std::cout << "loading files from " << dirPath.c_str() << std::endl;

    std::string mask = dirPath + Poco::Path::separator() + "*" + classSeparator + "*" + extension;
    std::set<std::string> files;
    Poco::Glob::glob(mask, files);

    for (const std::string &file : files)
    {
        if (qdebug)
            std::cout << "loading " << file.c_str() << std::endl;

        Matrix img = MatrixConverter::imageToMatrix(file);
        if (roi.width != 0 && roi.height != 0)
        {
            img = img(roi);
        }
        Vector vec = MatrixConverter::matrixToColumnVector(img);
        vectors.push_back(vec);

        Poco::Path p(file);
        Poco::StringTokenizer tokens(p.getBaseName(), classSeparator);
        int classNumber = Poco::NumberParser::parse(tokens[0]);
        classes.push_back(classNumber);

        if (maxImages > 0 && (int)vectors.size() >= maxImages)
        {
            break;
        }
    }
}

void Loader::loadImages(const std::string &dirPath, std::vector<Matrix> &matrices, std::vector<int> &classes,
                        const std::string &extension, const std::string &classSeparator, int maxImages,
                        cv::Rect roi, bool qdebug)
{
    std::cout << "loading files from " << dirPath.c_str() << std::endl;

    std::string mask = dirPath + Poco::Path::separator() + "*" + classSeparator + "*" + extension;
    std::set<std::string> files;
    Poco::Glob::glob(mask, files);

    for (const std::string &file : files)
    {
        if (qdebug)
            std::cout << "loading " << file.c_str() << std::endl;

        Matrix img = MatrixConverter::imageToMatrix(file);
        if (roi.width != 0 && roi.height != 0)
        {
            img = img(roi);
        }
        matrices.push_back(img);

        Poco::Path p(file);
        Poco::StringTokenizer tokens(p.getBaseName(), classSeparator);
        int classNumber = Poco::NumberParser::parse(tokens[0]);
        classes.push_back(classNumber);

        if (maxImages > 0 && (int)matrices.size() >= maxImages)
        {
            break;
        }
    }
}

void Loader::loadMatrices(const std::string &dirPath, std::vector<Matrix> &matrices,
                          std::vector<int> &classes, const std::string &classSeparator,
                          const std::string &nameFilter, int maxCount, const std::string &storageKey)
{
    matrices.clear();
    classes.clear();

    std::vector<std::string> filenames = listFiles(dirPath, nameFilter, Filename);
    for (const std::string &f : filenames)
    {
        Matrix m = Face::LinAlg::Common::loadMatrix(dirPath + Poco::Path::separator() + f, storageKey);
        matrices.push_back(m);

        if (Face::LinAlg::Common::matrixContainsNan(m))
        {
            std::cerr << f << " contains NaN" << std::endl;
        }

        Poco::StringTokenizer tokenizer(f, classSeparator);
        int classNumber = Poco::NumberParser::parse(tokenizer[0]);
        classes.push_back(classNumber);

        if (maxCount > 0 && (int)classes.size() >= maxCount) break;
    }
}

void Loader::loadVectors(const std::string &dirPath, std::vector<Vector> &vectors,
                         std::vector<int> &classes, const std::string &classSeparator, const std::string &nameFilter)
{
    vectors.clear();
    classes.clear();

    std::vector<std::string> filenames = listFiles(dirPath, nameFilter, Filename);
    for (const std::string &f : filenames)
    {
        Vector vec = Vector::fromFile(dirPath + Poco::Path::separator() + f);
        vectors.push_back(vec);

        Poco::StringTokenizer tokenizer(f, classSeparator);
        int classNumber = Poco::NumberParser::parse(tokenizer[0]);
        classes.push_back(classNumber);
    }
}

std::vector<std::string> Loader::listFiles(const std::string &path, const std::string &filter, PathType pathType)
{
    std::set<std::string> files;
    std::vector<std::string> result;

    std::string mask = path + Poco::Path::separator() + filter;
    Poco::Glob::glob(mask, files);
    for (const std::string &file : files)
    {
        Poco::Path path(file);
        switch (pathType)
        {
        case AbsoluteFull:
            result.push_back(path.absolute().toString());
            break;
        case Filename:
            result.push_back(path.getFileName());
            break;
        default:
            result.push_back(path.getBaseName());
            break;
        }
    }

    std::sort(result.begin(), result.end());
    return result;
}

std::vector<std::string> Loader::listFiles(const std::string &path, const std::vector<std::string> &filters, PathType pathType)
{
    std::vector<std::string> result;

    for (const std::string &filter : filters)
    {
        auto files = listFiles(path, filter, pathType);
        result.insert(result.end(), files.begin(), files.end());
    }

    std::sort(result.begin(), result.end());
    return result;
}

std::vector<int> Loader::getClasses(const std::vector<std::string> &fileNames, const std::string &separator)
{
    std::vector<int> result;
    for(const std::string &f : fileNames)
    {
        Poco::StringTokenizer tokenizer(f, separator);
        result.push_back(Poco::NumberParser::parse(tokenizer[0]));
    }
    return result;
}

void Loader::loadMeshesAllocate(const std::string &dir, std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes, std::vector<std::string> &fileNames)
{
    std::vector<std::string> filters;
    filters.push_back("*.binz");
    filters.push_back("*.bin");
    filters.push_back("*.obj");

    fileNames = listFiles(dir, filters, Filename);
    int n = fileNames.size();
    ids.resize(n);
    meshes.resize(n);
}

void Loader::loadMeshes(const std::string &dir, std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes,
                        int smoothIterations, float smoothCoef, const std::string &idSeparator)
{
    std::vector<std::string> fileNames;
    loadMeshesAllocate(dir, ids, meshes, fileNames);
    int n = fileNames.size();

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(dir + Poco::Path::separator() + fileNames[i]);

        if (smoothIterations > 0)
        {
            Face::FaceData::SurfaceProcessor::mdenoising(m, smoothCoef, smoothIterations, smoothIterations);
        }

        ids[i] = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], idSeparator)[0]);
        meshes[i] = m;
    }
}

void Loader::loadMeshes(const std::string &dir, const Face::FaceData::FaceAlignerIcp &aligner, std::vector<int> &ids,
                        std::vector<Face::FaceData::Mesh> &meshes, int icpIterations, int smoothIterations,
                        float smoothCoef, const std::string &idSeparator)
{
    std::vector<std::string> fileNames;
    loadMeshesAllocate(dir, ids, meshes, fileNames);
    int n = fileNames.size();

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(dir + Poco::Path::separator() + fileNames[i]);

        if (smoothIterations > 0)
        {
            Face::FaceData::SurfaceProcessor::mdenoising(m, smoothCoef, smoothIterations, smoothIterations);
        }

        if (icpIterations > 0)
        {
			aligner.align(m, icpIterations, Face::FaceData::FaceAlignerIcp::CVTemplateMatching);
        }

        ids[i] = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], idSeparator)[0]);
        meshes[i] = m;
    }
}

void Loader::loadMeshes(const std::string &dir, const Face::FaceData::FaceAlignerLandmark &aligner,
                        std::vector<int> &ids, std::vector<Face::FaceData::Mesh> &meshes,
                        int smoothIterations, float smoothCoef, const std::string &idSeparator)
{
    std::vector<std::string> fileNames;
    loadMeshesAllocate(dir, ids, meshes, fileNames);
    int n = fileNames.size();

    #pragma omp parallel for
    for (int i = 0; i < n; i++)
    {
        Poco::Path path(dir + Poco::Path::separator() + fileNames[i]);
        Face::FaceData::Mesh m = Face::FaceData::Mesh::fromFile(path.toString());

        std::string lmPath = dir + Poco::Path::separator() + path.getBaseName() + "-landmarks.yml";
        Face::FaceData::Landmarks lm(lmPath);

        if (smoothIterations > 0)
        {
            Face::FaceData::SurfaceProcessor::mdenoising(m, smoothCoef, smoothIterations, smoothIterations);
        }

        aligner.align(m, lm);

        ids[i] = Poco::NumberParser::parse(Poco::StringTokenizer(fileNames[i], idSeparator)[0]);
        meshes[i] = m;
    }
}
