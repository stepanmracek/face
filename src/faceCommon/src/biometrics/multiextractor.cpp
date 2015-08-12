#include "faceCommon/biometrics/multiextractor.h"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>
#include <Poco/Path.h>
#include <Poco/File.h>
#include <fstream>

#include "faceCommon/linalg/gabor.h"
#include "faceCommon/linalg/gausslaguerre.h"
#include "faceCommon/linalg/matrixconverter.h"
#include "faceCommon/facedata/surfaceprocessor.h"
#include "faceCommon/linalg/differenceofgaussians.h"
#include "faceCommon/linalg/imagefilterfactory.h"
#include "faceCommon/biometrics/isocurveprocessing.h"
#include "faceCommon/linalg/imagefilter.h"
#include "faceCommon/biometrics/extractorthreadpool.h"
#include "faceCommon/biometrics/imagedatathreadpool.h"

using namespace Face::Biometrics;

MultiExtractor::ImageData::ImageData(const Face::FaceData::Mesh &mesh)
{
    cv::Rect roi(25, 15, 100, 90);
    FaceData::MapConverter converter;
    FaceData::Map textureMap = FaceData::SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-75, -75),
                                                                    cv::Point2d(75, 75), 1,
                                                                    FaceData::SurfaceProcessor::Texture_I);
    Matrix texture = textureMap.toMatrix(0, 0, 255)(roi);
    typeDict["texture"] = texture;

    FaceData::Map depthmap = FaceData::SurfaceProcessor::depthmap(mesh, converter, cv::Point2d(-75, -75),
                                                                  cv::Point2d(75, 75), 1,
                                                                  FaceData::SurfaceProcessor::ZCoord);
    depthmap.bandPass(-70, 10, false, false);
    typeDict["depth"] = depthmap.toMatrix(0, -70, 10)(roi);

    FaceData::Map smoothedDepthmap = depthmap;
    smoothedDepthmap.applyCvGaussBlur(7, 3);
    FaceData::CurvatureStruct cs = FaceData::SurfaceProcessor::calculateCurvatures(smoothedDepthmap);

    cs.curvatureMean.bandPass(-0.1, 0.1, false, false);
    typeDict["mean"] = cs.meanMatrix()(roi);

    cs.curvatureGauss.bandPass(-0.01, 0.01, false, false);
    typeDict["gauss"] = cs.gaussMatrix()(roi);

    cs.curvatureIndex.bandPass(0, 1, false, false);
    typeDict["index"] = cs.indexMatrix()(roi);

    cs.curvaturePcl.bandPass(0, 0.0025, false, false);
    typeDict["eigencur"] = cs.pclMatrix()(roi);

    this->largeDepth = FaceData::SurfaceProcessor::depthmap(mesh, this->depthConverter, 2.0,
                                                            FaceData::SurfaceProcessor::ZCoord);

    /*for (const std::pair<std::string, Matrix> &kvp : typeDict)
    {
        cv::imshow(kvp.first, kvp.second);
    }
    cv::waitKey(0);*/
}

MultiExtractor::ImageData::ImageData(const ImageGrayscale &textureImage)
{
    typeDict["texture"] = LinAlg::MatrixConverter::grayscaleImageToDoubleMatrix(textureImage);
}

std::vector<std::string> MultiExtractor::ImageData::getTypes()
{
    std::vector<std::string> types;
    types.push_back("depth");
    types.push_back("index");
    types.push_back("mean");
    types.push_back("gauss");
    types.push_back("eigencur");
    types.push_back("texture");
    return types;
}

MultiExtractor::Unit::Ptr MultiExtractor::Unit::parse(const std::string &params)
{
    Poco::StringTokenizer items(params, " ");
    if (items.count() < 1) throw FACELIB_EXCEPTION("wrong unit params: " + params);
    const std::string unitType = items[0];

    if (unitType.compare("image") == 0)
        return ImageUnit::parse(params);
    else if (unitType.compare("curve") == 0)
        return CurveUnit::parse(params);

    throw FACELIB_EXCEPTION("unknown unity type: " + unitType);
}

double MultiExtractor::Unit::compare(const Face::LinAlg::Vector &first, const Face::LinAlg::Vector &second)
{
    return metrics->distance(first, second);
}

void MultiExtractor::ImageUnit::train(const std::vector<int> &ids, const std::vector<ImageData> &imageData)
{
    std::vector<Face::LinAlg::Vector> rawVecs;
    for (const ImageData &data : imageData)
    {
        Matrix inputImg = data.typeDict.at(type);
        Matrix processed = Face::LinAlg::ImageFilter::batchProcess(inputImg, this->imageFilters);
        rawVecs.push_back(Face::LinAlg::MatrixConverter::matrixToColumnVector(processed));
    }

    featureExtractor->train(ids, rawVecs);
}


MultiExtractor::Unit::Ptr MultiExtractor::ImageUnit::parse(const std::string &params)
{
    Poco::StringTokenizer items(params, " ");
    if (items.count() != 5) throw FACELIB_EXCEPTION("wrong unit params: " + params);

    ImageUnit * result = new ImageUnit();
    result->type = items[1];
    result->imageFilters = Face::LinAlg::ImageFilterFactory::create(items[2], ";");
    result->featureExtractor = FeatureExtractorFactory::create(items[3]);
    result->metrics = Face::LinAlg::MetricsFactory::create(items[4]);
    return result;
}

std::string MultiExtractor::ImageUnit::writeParams() const
{
    std::string filters;
    for (Face::LinAlg::ImageFilter::Ptr f : imageFilters)
    {
        filters += f->writeParams() + ";";
    }
    return "image " + type + " " + filters + " " + featureExtractor->writeParams() + " " + metrics->writeParams();
}

Face::LinAlg::Vector MultiExtractor::ImageUnit::extract(const ImageData &data) const
{
    Matrix input = data.typeDict.at(type);
    Face::LinAlg::Vector vec = Face::LinAlg::MatrixConverter::matrixToColumnVector(
                Face::LinAlg::ImageFilter::batchProcess(input, imageFilters));

    return featureExtractor->extract(vec);
}

void MultiExtractor::CurveUnit::train(const std::vector<int> &ids, const std::vector<ImageData> &imageData)
{
    std::vector<Face::LinAlg::Vector> rawVecs;
    int count = ids.size();
    for (int i = 0; i < count; i++)
    {
        int n = this->curveCenters.size();
        Face::Biometrics::VectorOfCurves curves;
        for (int j = 0; j < n; j++)
        {
            curves.push_back(Face::FaceData::SurfaceProcessor::isoGeodeticCurve(imageData[i].largeDepth, imageData[i].depthConverter,
                                                                                curveCenters[j], radiuses[j], pointCounts[j], 2.0));
        }
        rawVecs.push_back(Face::Biometrics::IsoCurveProcessing::generateFeatureVector(curves, false));
    }
    featureExtractor->train(ids, rawVecs);
}

MultiExtractor::Unit::Ptr MultiExtractor::CurveUnit::parse(const std::string &params)
{
    Poco::StringTokenizer items(params, " ");
    if (items.count() != 6) throw FACELIB_EXCEPTION("wrong unit params: " + params);

    CurveUnit * result = new CurveUnit();

    Poco::StringTokenizer centrerValues(items[1], ";");
    int n = centrerValues.count() / 3;
    for (int i = 0; i < n; i++)
        result->curveCenters.push_back(cv::Point3d(Poco::NumberParser::parseFloat(centrerValues[3*i]),
                                                   Poco::NumberParser::parseFloat(centrerValues[3*i + 1]),
                                                   Poco::NumberParser::parseFloat(centrerValues[3*i + 2])));

    Poco::StringTokenizer radiusValues(items[2], ";");
    for (int i = 0; i < n; i++)
        result->radiuses.push_back(Poco::NumberParser::parse(radiusValues[i]));

    Poco::StringTokenizer countValues(items[3], ";");
    for (int i = 0; i < n; i++)
        result->pointCounts.push_back(Poco::NumberParser::parse(countValues[i]));

    result->featureExtractor = FeatureExtractorFactory::create(items[4]);
    result->metrics = Face::LinAlg::MetricsFactory::create(items[5]);
    return result;
}

std::string MultiExtractor::CurveUnit::writeParams() const
{
    std::string centersStr;
    for (const cv::Point3d &c : curveCenters)
        centersStr += std::to_string(c.x) + ";" + std::to_string(c.y) + ";" + std::to_string(c.z) + ";";

    std::string radiusesStr;
    for (int r : radiuses)
        radiusesStr += std::to_string(r) + ";";

    std::string countsStr;
    for (int c : pointCounts)
        countsStr += std::to_string(c) + ";";

    return "curve " + centersStr + " " + radiusesStr + " " + countsStr + " "
            + featureExtractor->writeParams() + " " + metrics->writeParams();
}

Face::LinAlg::Vector MultiExtractor::CurveUnit::extract(const ImageData &data) const
{
    int n = this->curveCenters.size();
    Face::Biometrics::VectorOfCurves curves;
    for (int i = 0; i < n; i++)
    {
        curves.push_back(Face::FaceData::SurfaceProcessor::isoGeodeticCurve(data.largeDepth, data.depthConverter,
                                                                            curveCenters[i], radiuses[i], pointCounts[i], 2.0));
    }
    Face::LinAlg::Vector rawVec = Face::Biometrics::IsoCurveProcessing::generateFeatureVector(curves, false);
    return featureExtractor->extract(rawVec);
}

MultiExtractor::MultiExtractor() : extractorThreadPool(0), imageDataThreadPool(0), fusion(0)
{
}

MultiExtractor::MultiExtractor(std::string directoryPath) : extractorThreadPool(0), imageDataThreadPool(0)
{
    if (directoryPath.back() != Poco::Path::separator())
        directoryPath.push_back(Poco::Path::separator());

    auto dir = Poco::File(directoryPath);
    if (!dir.isDirectory() || !dir.exists())
    {
        throw FACELIB_EXCEPTION("directory " + directoryPath + " does not exist");
    }

    // load 'units' file
    std::string unitsPath = directoryPath + "units";
    auto f = Poco::File(unitsPath);
    if (!f.exists())
    {
        throw FACELIB_EXCEPTION("units file " + unitsPath + " does not exist");
    }

    std::ifstream in(unitsPath);
    std::string fusionType;
    std::getline(in, fusionType);
    if (fusionType.empty())
    {
        throw FACELIB_EXCEPTION(unitsPath + " is empty");
    }

    fusion = Face::Biometrics::ScoreLevelFusionFactory::create(fusionType);
    std::string fusionFile = directoryPath + "fusion";
    if (!Poco::File(fusionFile).exists())
    {
        throw FACELIB_EXCEPTION("fusion file " + fusionFile + " does not exist");
    }
    fusion->deserialize(fusionFile);

    int index = 0;
    std::string line;
    while (std::getline(in, line))
    {
        std::cout << line << std::endl;

        Unit::Ptr unit = Unit::parse(line);

        std::string filename = directoryPath + "unit-" + std::to_string(index);
        if (!Poco::File(filename).exists())
        {
            throw FACELIB_EXCEPTION("unit file " + filename + " does not exist");
        }

        cv::FileStorage storage(filename, cv::FileStorage::READ);
        if (!storage.isOpened())
        {
            throw FACELIB_EXCEPTION("can't open units file " + unitsPath);
        }
        unit->featureExtractor->deserialize(storage);

        units.push_back(unit);
        index++;
    }
}

MultiExtractor::~MultiExtractor()
{
    if (imageDataThreadPool) delete imageDataThreadPool;
    if (extractorThreadPool) delete extractorThreadPool;
}

void MultiExtractor::setEnableThreadPool(bool enable)
{
    if (enable)
    {
        if (!imageDataThreadPool) imageDataThreadPool = new ImageDataThreadPool();
        if (!extractorThreadPool) extractorThreadPool = new ExtractorThreadPool();
    }
    else
    {
        if (imageDataThreadPool) delete imageDataThreadPool;
        if (extractorThreadPool) delete extractorThreadPool;
    }
}

void MultiExtractor::serialize(std::string directoryPath) const
{
    if (directoryPath.back() != Poco::Path::separator())
        directoryPath.push_back(Poco::Path::separator());

    // create directory if it doesn't exist
    Poco::File dir(directoryPath);
    if (!dir.exists())
    {
        std::cout << "creating path " << dir.path() << std::endl;
        dir.createDirectories();
    }   

    fusion->serialize(directoryPath + "fusion");

    std::string unitsPath = directoryPath + "units";
    std::ofstream out(unitsPath);

    out << fusion->writeName() << std::endl;
    for (unsigned int i = 0; i < units.size(); i++)
    {
        Unit::Ptr p = units[i];
        out << p->writeParams() << std::endl;

        std::string filename = directoryPath + "unit-" + std::to_string(i);
        cv::FileStorage storage(filename, cv::FileStorage::WRITE);

        p->featureExtractor->serialize(storage);
    }
}

MultiTemplate MultiExtractor::extract(const ImageData &data, int version, int id) const
{
    MultiTemplate t; t.id = id; t.version = version;
    if (extractorThreadPool)
    {
        t.featureVectors.resize(units.size());
        extractorThreadPool->extract(&data, &t, this);
    }
    else
    {
        for (const MultiExtractor::Unit::Ptr &u : units)
        {
            t.featureVectors.push_back(u->extract(data));
        }
    }

    if (data.typeDict.find("depth") != data.typeDict.end())
    {
        const Matrix &depth = data.typeDict.at("depth");
        t.depthCoverage = ((float)cv::countNonZero(depth)) / (depth.rows * depth.cols);
    }
    else
    {
        t.depthCoverage = 0.0;
    }

    return t;
}

MultiTemplate MultiExtractor::extract(const Face::FaceData::Mesh &mesh, int version, int id) const
{
    if (imageDataThreadPool)
    {
        ImageData data = imageDataThreadPool->process(&mesh);
        return extract(data, version, id);
    }
    else
    {
        ImageData data(mesh);
        return extract(data, version, id);
    }
}

std::vector<MultiTemplate> MultiExtractor::extract(const std::vector<Face::FaceData::Mesh> &meshes,
                                                   const std::vector<int> &ids, int version) const
{
    std::vector<Face::Biometrics::MultiTemplate> templates(meshes.size());

    #pragma omp parallel for
    for (int i = 0; i < ids.size(); i++)
    {
        templates[i] = extract(meshes[i], version, ids[i]);
    }

    return templates;
}

bool MultiExtractor::checkTemplate(const MultiTemplate &t)
{
	unsigned int n = t.featureVectors.size();
    if (n != this->units.size()) return false;
    for (unsigned int i = 0; i < n; i++)
    {
        if (units[i]->featureExtractor->outputLen() == -1) continue;
        if (t.featureVectors[i].rows != units[i]->featureExtractor->outputLen())
        {
            return false;
        }
    }
    return true;
}

ScoreLevelFusionBase::Result MultiExtractor::compare(const MultiTemplate &first, const MultiTemplate &second) const
{
    if(first.version != second.version) throw FACELIB_EXCEPTION("templates have different versions");

    unsigned int n = first.featureVectors.size();

    if (n != second.featureVectors.size()){
        std::string str = "feature vector components count mismatch: " + std::to_string(n) + " vs " + std::to_string(second.featureVectors.size());
        throw FACELIB_EXCEPTION(str);
    }
    if (n != units.size()){
        std::string str = "feature vector and units count mismatch: " + std::to_string(n) + " vs " + std::to_string(units.size());
        throw FACELIB_EXCEPTION(str);
    }

    std::vector<double> scores;
    for (unsigned int i = 0; i < n; i++)
    {
        int fvOuputLen = units[i]->featureExtractor->outputLen();
        if (fvOuputLen > 0 && (fvOuputLen != first.featureVectors[i].rows ||
                               fvOuputLen != second.featureVectors[i].rows))
        {
            throw FACELIB_EXCEPTION("feature vector component " + std::to_string(i) + " length mismatch");
        }

        scores.push_back(units[i]->metrics->distance(first.featureVectors[i], second.featureVectors[i]));
    }
    return fusion->fuse(scores);
}

MultiExtractor::ComparisonResult MultiExtractor::compare(const std::vector<MultiTemplate> &reference, const MultiTemplate &probe, int count) const
{
    MultiExtractor::ComparisonResult result;
    if (count <= 0 || count > (int)reference.size()) count = reference.size();

    std::vector<double> scores;
    for (const MultiTemplate &ref : reference)
    {
        result.perReferenceResults.push_back(compare(ref, probe));
        scores.push_back(result.perReferenceResults.back().score);
    }
    std::sort(scores.begin(), scores.end());

    for (int i = 0; i < count; i++)
    {
        result.distance += scores[i];
    }
    result.distance /= count;

    return result;
}

Evaluation MultiExtractor::evaluate(const std::vector<MultiTemplate> &templates) const
{
    std::vector<double> genScores;
    std::vector<double> impScores;

    int n = templates.size();
    for (int i = 0; i < (n-1); i++)
    {
        for (int j = i+1; j < n; j++)
        {
            double s = compare(templates[i], templates[j]).score;
            if (templates[i].id == templates[j].id)
            {
                genScores.push_back(s);
            }
            else
            {
                impScores.push_back(s);
            }
        }
    }
    return Evaluation(genScores, impScores);
}

double MultiExtractor::rankOneIdentification(const std::vector<MultiTemplate> &templates) const
{
    std::map<int, MultiTemplate> referenceTemplatesDict;
    std::vector<MultiTemplate> probes;
    for (const MultiTemplate &t : templates)
    {
        int id = t.id;
        if (referenceTemplatesDict.count(id) == 0)
        {
            referenceTemplatesDict[id] = t;
        }
        else
        {
            probes.push_back(t);
        }
    }

    std::vector<MultiTemplate> references;
    for (const auto &pair : referenceTemplatesDict)
        references.push_back(pair.second);

    int matchCount = 0;
    for (const MultiTemplate &probe : probes)
    {
        double minScore = DBL_MAX;
        int minId = -1;
        for (const MultiTemplate &reference : references)
        {
            double s = compare(reference, probe).score;
            if (s < minScore)
            {
                minScore = s;
                minId = reference.id;
            }
        }

        if (minId == probe.id)
        {
            matchCount++;
        }
    }

    return ((double)matchCount)/probes.size();
}

void saveScoreMap(const std::map<int, std::vector<double> > &map, const std::string &path)
{
    std::ofstream out(path);

    for (auto mapIter = map.begin(); mapIter != map.end(); ++mapIter)
    {
        for (auto vecIter = mapIter->second.begin(); vecIter != mapIter->second.end(); ++vecIter)
        {
            out << mapIter->first << " " << (*vecIter) << std::endl;
        }
    }
}

void MultiExtractor::createPerSubjectScoreCharts(const std::vector<MultiTemplate> &templates,
                                                 const std::string &pathPrefix) const
{
    std::map<int, std::vector<double> > impScores;
    std::map<int, std::vector<double> > genScores;
    int n = templates.size();

    for (int i = 0; i < n; i++)
    {
        impScores[templates[i].id] = std::vector<double>();
        genScores[templates[i].id] = std::vector<double>();
    }

    for (int i = 0; i < (n-1); i++)
    {
        for (int j = (i+1); j < n; j++)
        {
            double s = compare(templates[i], templates[j]).score;
            if (templates[i].id == templates[j].id)
            {
                genScores[templates[i].id].push_back(s);
            }
            else
            {
                impScores[templates[i].id].push_back(s);
                impScores[templates[j].id].push_back(s);
            }
        }
    }

    saveScoreMap(impScores, pathPrefix + "-imp");
    saveScoreMap(genScores, pathPrefix + "-gen");
}
