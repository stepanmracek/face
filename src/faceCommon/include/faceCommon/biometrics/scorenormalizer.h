#ifndef SCORENORMALIZER_H
#define SCORENORMALIZER_H

#include "faceCommon/biometrics/evaluation.h"
#include "faceCommon/linalg/iserializable.h"
#include "faceCommon/faceCommon.h"

namespace Face {
namespace Biometrics {

class ScoreNormalizerBase : public Face::LinAlg::ISerializable
{
public:
    typedef cv::Ptr<ScoreNormalizerBase> Ptr;

    virtual void learn(const std::vector<Face::Biometrics::Evaluation> &evaluations) = 0;
    virtual std::vector<double> normalize(const std::vector<double> &inputScores) const = 0;
    virtual std::string writeParams() const = 0;
};

class FACECOMMON_EXPORTS ScoreNormalizerFactory
{
public:
    static ScoreNormalizerBase::Ptr create(const std::string &name);
};

class FACECOMMON_EXPORTS ScoreNormalizerPass : public ScoreNormalizerBase
{
public:
    void learn(const std::vector<Face::Biometrics::Evaluation> &/*evaluations*/) {}
    std::vector<double> normalize(const std::vector<double> &inputScores) const { return inputScores; }
    static std::string name() { return "pass"; }
    std::string writeParams() const { return name(); }
    void serialize(cv::FileStorage &/*storage*/) const {}
    void deserialize(cv::FileStorage &/*storage*/) {}
};

class FACECOMMON_EXPORTS ScoreNormalizerMean : public ScoreNormalizerBase
{
private:
    std::vector<double> impostorMeans;
    std::vector<double> genuineMeans;

public:
    void learn(const std::vector<Face::Biometrics::Evaluation> &evaluations);
    std::vector<double> normalize(const std::vector<double> &inputScores) const;
    static std::string name() { return "mean"; }
    std::string writeParams() const { return name(); }
    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};


class FACECOMMON_EXPORTS ScoreNormalizerMedian : public ScoreNormalizerBase
{
private:
    std::vector<double> impostorMedians;
    std::vector<double> genuineMedians;

public:
    void learn(const std::vector<Face::Biometrics::Evaluation> &evaluations);
    std::vector<double> normalize(const std::vector<double> &inputScores) const;
    static std::string name() { return "median"; }
    std::string writeParams() const { return name(); }
    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};

class FACECOMMON_EXPORTS ScoreNormalizerZScore : public ScoreNormalizerBase
{
private:
    bool compensateGenImpCount;
    std::vector<double> means;
    std::vector<double> stdDevs;

public:
    ScoreNormalizerZScore(bool compensateGenImpCount) : compensateGenImpCount(compensateGenImpCount) {}

    void learn(const std::vector<Face::Biometrics::Evaluation> &evaluations);
    std::vector<double> normalize(const std::vector<double> &inputScores) const;
    static std::string name() { return "zscore"; }
    std::string writeParams() const { return compensateGenImpCount ? name()+"Comp" : name(); }
    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};

class FACECOMMON_EXPORTS ScoreNormalizerMAD : public ScoreNormalizerBase
{
private:
    bool compensateGenImpCount;
    std::vector<double> medians;
    std::vector<double> mads;

public:
    ScoreNormalizerMAD(bool compensateGenImpCount) : compensateGenImpCount(compensateGenImpCount) {}

    void learn(const std::vector<Face::Biometrics::Evaluation> &evaluations);
    std::vector<double> normalize(const std::vector<double> &inputScores) const;
    static std::string name() { return "mad"; }
    std::string writeParams() const { return compensateGenImpCount ? name()+"Comp" : name(); }
    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};

class FACECOMMON_EXPORTS ScoreNormalizerTanh : public ScoreNormalizerBase
{
private:
    bool compensateGenImpCount;
    std::vector<double> means;
    std::vector<double> stdDevs;

public:
    ScoreNormalizerTanh(bool compensateGenImpCount) : compensateGenImpCount(compensateGenImpCount) {}

    void learn(const std::vector<Face::Biometrics::Evaluation> &evaluations);
    std::vector<double> normalize(const std::vector<double> &inputScores) const;
    static std::string name() { return "tanh"; }
    std::string writeParams() const { return compensateGenImpCount ? name()+"Comp" : name(); }
    void serialize(cv::FileStorage &storage) const;
    void deserialize(cv::FileStorage &storage);
};

}
}

#endif // SCORENORMALIZER_H
