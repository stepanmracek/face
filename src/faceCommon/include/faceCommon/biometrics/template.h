#ifndef TEMPLATE_H
#define TEMPLATE_H

#include <vector>

#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/normalization.h"

namespace Face {
namespace Biometrics {

class FeatureExtractor;

class Template
{
public:
    int subjectID;
    Face::LinAlg::Vector featureVector;

    Template() {}
    Template(int id, const Face::LinAlg::Vector &featureVector) : subjectID(id), featureVector(featureVector) { }

    static std::vector<Template> loadTemplates(const std::string &dirPath, const std::string &classSeparator);

    static void normalizeFeatureVectorComponents(std::vector<Template> &templates,
                                                 std::vector<double> &minValues, std::vector<double> &maxValues);

    static void splitVectorsAndClasses(std::vector<Template> &templates,
                                       std::vector<Face::LinAlg::Vector> &featureVectors,
                                       std::vector<int> &classMemberships);

    static std::vector<Template> joinVectorsAndClasses(const std::vector<LinAlg::Vector> &featureVectors,
                                                   const std::vector<int> &classMemberships);

    static void saveTemplates(std::vector<std::vector<Template> > &clusters, const std::string &path,
                              const std::string &classSeparator = "-");

    static Face::LinAlg::ZScoreNormalizationResult zScoreNorm(std::vector<Template> &templates);

    static void zScoreNorm(std::vector<Template> &templates, Face::LinAlg::ZScoreNormalizationResult &normParams);

    static std::vector<Face::LinAlg::Vector> getVectors(std::vector<Template> &templates);

    static void stats(std::vector<Template> &templates, const std::string &outPath);

    static std::vector<Template> createTemplates(const std::vector<Face::LinAlg::Vector> &rawData,
                                             const std::vector<int> &IDs,
                                             const Face::Biometrics::FeatureExtractor &extractor);

    static std::vector<Template> clone(const std::vector<Template> &src);
};

}
}

#endif // TEMPLATE_H
