#ifndef EVALUATION_H
#define EVALUATION_H

#include <map>

#include "faceCommon/linalg/metrics.h"

namespace Face {
namespace Biometrics {

class Evaluation;
class BatchEvaluationResult;
class FeatureExtractor;
class Template;

class Evaluation
{
private:
    void commonEvaluation();
    void commonTemplatesEvaluation(const std::vector<Template> &templates, const Face::LinAlg::Metrics &metrics);
    void commonInit();
    void commonInitOfScores();

public:
    enum SCORE_TYPE { SCORE_UNKNOWN, SCORE_DISTANCE, SCORE_SIMILARITY};
    SCORE_TYPE scoreType;

    double minGenuineScore;
    double maxGenuineScore;
    double meanGenuineScore;
    double minImpostorScore;
    double maxImpostorScore;
    double meanImpostorScore;
    double minScore;
    double maxScore;

    std::vector<double> genuineScores;
    std::vector<double> impostorScores;

    std::vector<double> fmr;
    std::vector<double> fnmr;
    std::vector<double> distances;

    double eer;
    double eerScore;

    Evaluation();

    Evaluation(const std::vector<double> &genuineScores, const std::vector<double> &impostorScores);

    Evaluation(const std::vector<Template> &templates, const Face::LinAlg::Metrics &metrics);

    Evaluation(std::map<std::pair<int, int>, std::vector<double> > &distances);

    Evaluation(const std::vector<Face::LinAlg::Vector> &rawData, const std::vector<int> &classes,
               const Face::Biometrics::FeatureExtractor &extractor, const Face::LinAlg::Metrics &metric);


    void fnmrAndFmrAtDistance(const double inDistance, double &outFnmr, double &outFmr) const;
    void fnmrAtFmr(const double inFmr, double &outFnmr, double &outDistance) const;
    void printStats() const;

    void outputResults(const std::string &path, int histogramBins) const;
    void outputResultsDET(const std::string &path) const;
    void outputResultsGenuineDistribution(const std::string &path, int bins) const;
    void outputResultsImpostorDistribution(const std::string &path, int bins) const;
    void outputResultsGenuineScores(const std::string &path) const;
    void outputResultsImpostorScores(const std::string &path) const;
    void outputResultsFMR(const std::string &path) const;
    void outputResultsFNMR(const std::string &path) const;

    static BatchEvaluationResult batch(std::vector<std::vector<Template> > &templates, const Face::LinAlg::Metrics &metrics, int startIndex = 0);

    static BatchEvaluationResult batch(std::vector<std::vector<Face::LinAlg::Vector> > &images, std::vector<std::vector<int> > &classes,
                                       const Face::Biometrics::FeatureExtractor &extractor, const Face::LinAlg::Metrics &metrics, int startIndex = 0);
};

class BatchEvaluationResult
{
public:
    std::vector<Evaluation> results;
    double meanEER;
    double stdDevOfEER;
};

}
}

#endif // EVALUATION_H
