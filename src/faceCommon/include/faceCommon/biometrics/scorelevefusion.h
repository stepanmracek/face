#ifndef SCORELEVELFUSION_H
#define SCORELEVELFUSION_H

#include <opencv2/opencv.hpp>

#include "featureextractor.h"
#include "faceCommon/linalg/common.h"
#include "evaluation.h"
#include "faceCommon/linalg/lda.h"
#include "faceCommon/linalg/logisticregression.h"
#include "faceCommon/linalg/vector.h"
#include "scorenormalizer.h"

namespace Face {
namespace Biometrics {

class ScoreLevelFusionBase
{
protected:
    std::vector<Evaluation> components;

    bool learned;

    void prepareDataForClassification(std::vector<Face::LinAlg::Vector> &scores, std::vector<int> &classes,
                                      int genuineLabel, int impostorLabel);

    virtual void learnImplementation() = 0;

    ScoreLevelFusionBase() : scoreNormalizer(new ScoreNormalizerPass()) { learned = false; }

public:
    struct Result
    {
        Result() : score(0) {}
        double score;
        std::vector<double> preNormalized;
        std::vector<double> normalized;
    };

    ScoreNormalizerBase::Ptr scoreNormalizer;

    typedef cv::Ptr<ScoreLevelFusionBase> Ptr;

    virtual ~ScoreLevelFusionBase() {}

    void learn();

    virtual Result fuse(const std::vector<double> &scores) const = 0;

    virtual void serialize(const std::string &path) const = 0;

    virtual void deserialize(const std::string &path) = 0;

    ScoreLevelFusionBase & addComponent(const Evaluation &component);

    void popComponent();

    Evaluation evaluate(const std::vector<std::vector<Face::Biometrics::Template> > &templates, const std::vector<Face::LinAlg::Metrics *> &);
    Evaluation evaluate(const std::vector<Evaluation> &evaluations);

    virtual std::string writeName() const = 0;
};

class ScoreLevelFusionFactory
{
public:
    static ScoreLevelFusionBase::Ptr create(const std::string &name);
};

class ScoreLDAFusion : public ScoreLevelFusionBase
{
private:
    /*double maxScore;
    double minScore;
    bool swapResultScore;*/

    Face::LinAlg::LDA lda;

public:
    void learnImplementation();
    Result fuse(const std::vector<double> &scores) const;
    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    static std::string name() { return "ldaFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }
};

class ScoreLogisticRegressionFusion : public ScoreLevelFusionBase
{
private:
    Face::LinAlg::LogisticRegression logR;

public:
    void learnImplementation();
    Result fuse(const std::vector<double> &scores) const;
    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    static std::string name() { return "logRFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }
};

class ScoreWeightedSumFusion : public ScoreLevelFusionBase
{
private:
    std::vector<double> eer;
    double weightDenominator;

public:
    void learnImplementation();
    Result fuse(const std::vector<double> &scores) const;

    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    static std::string name() { return "wsumFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }
};

class ScoreSumFusion : public ScoreLevelFusionBase
{
public:
    void learnImplementation();
    Result fuse(const std::vector<double> &scores) const;

    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    static std::string name() { return "sumFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }

};

class ScoreProductFusion : public ScoreLevelFusionBase
{
public:
    void learnImplementation();
    Result fuse(const std::vector<double> &scores) const;
    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    static std::string name() { return "productFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }
};

class ScoreSVMFusion : public ScoreLevelFusionBase
{
private:
    cv::Ptr<cv::SVM> svm;

    cv::Mat colVectorsToFPMatrix(std::vector<Face::LinAlg::Vector> &vectors) const;
    cv::Mat colVectorToColFPMatrix(std::vector<int> &vector) const;
    cv::Mat colVectorToColFPMatrix(std::vector<double> &vector) const;

    void init();

public:   
    void learnImplementation();
    Result fuse(const std::vector<double> &scores) const;

    ScoreSVMFusion();

    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    static std::string name() { return "svmFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }
};

class ScoreGMMFusion : public ScoreLevelFusionBase
{
private:
    cv::Ptr<cv::EM> impostorScoresModel;
    cv::Ptr<cv::EM> genuineScoresModel;

    void init();
public:
    ScoreGMMFusion();
    void serialize(const std::string &path) const;
    void deserialize(const std::string &path);

    void learnImplementation();

    Result fuse(const std::vector<double> &scores) const;

    static std::string name() { return "gmmFusion"; }
    std::string writeName() const { return name() + "-" + scoreNormalizer->writeParams(); }
};

}
}

#endif // SCORELEVELFUSION_H
