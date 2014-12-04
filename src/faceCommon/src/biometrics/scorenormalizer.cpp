#include "faceCommon/biometrics/scorenormalizer.h"
#include "faceCommon/linalg/common.h"

using namespace Face::Biometrics;

ScoreNormalizerBase::Ptr ScoreNormalizerFactory::create(const std::string &name)
{
    if (name == ScoreNormalizerPass::name())
        return new ScoreNormalizerPass();

    else if (name == ScoreNormalizerMean::name())
        return new ScoreNormalizerMean();

    else if (name == ScoreNormalizerMedian::name())
        return new ScoreNormalizerMedian();

    else if (name == ScoreNormalizerZScore::name())
        return new ScoreNormalizerZScore(false);
    else if (name == ScoreNormalizerZScore::name() + "Comp")
        return new ScoreNormalizerZScore(true);

    else if (name == ScoreNormalizerMAD::name())
        return new ScoreNormalizerMAD(false);
    else if (name == ScoreNormalizerMAD::name() + "Comp")
        return new ScoreNormalizerMAD(true);

    else if (name == ScoreNormalizerTanh::name())
        return new ScoreNormalizerTanh(false);
    else if (name == ScoreNormalizerTanh::name() + "Comp")
        return new ScoreNormalizerTanh(true);

    throw FACELIB_EXCEPTION("unknown score normalizer " + name);
}

// --- ScoreNormalizerMean ---

void ScoreNormalizerMean::learn(const std::vector<Evaluation> &evaluations)
{
    genuineMeans.clear();
    impostorMeans.clear();

    for (const Evaluation &e : evaluations)
    {
        genuineMeans.push_back(Face::LinAlg::Vector(e.genuineScores).meanValue());
        impostorMeans.push_back(Face::LinAlg::Vector(e.impostorScores).meanValue());
    }
}

std::vector<double> ScoreNormalizerMean::normalize(const std::vector<double> &inputScores) const
{
	unsigned int n = inputScores.size();
    if (n != genuineMeans.size()) throw FACELIB_EXCEPTION("invalid scores count");
    std::vector<double> result(n);
    for (unsigned int i = 0; i < n; i++)
    {
        result[i] = (inputScores[i] - genuineMeans[i])/(impostorMeans[i] - genuineMeans[i]);
    }
    return result;
}

void ScoreNormalizerMean::serialize(cv::FileStorage &storage) const
{
    storage << "genuineMeans" << Face::LinAlg::Vector(genuineMeans);
    storage << "impostorMeans" << Face::LinAlg::Vector(impostorMeans);
}

void ScoreNormalizerMean::deserialize(cv::FileStorage &storage)
{
    Face::LinAlg::Vector genMeansMat;
    storage["genuineMeans"] >> genMeansMat;
    genuineMeans = genMeansMat.toStdVector();

    Face::LinAlg::Vector impMeansMat;
    storage["impostorMeans"] >> impMeansMat;
    impostorMeans = impMeansMat.toStdVector();
}

// --- ScoreNormalizerMedian ---

void ScoreNormalizerMedian::learn(const std::vector<Evaluation> &evaluations)
{
    genuineMedians.clear();
    impostorMedians.clear();

    for (const Evaluation &e : evaluations)
    {
        auto genScores = e.genuineScores;
        auto impScores = e.impostorScores;
        genuineMedians.push_back(Face::LinAlg::Common::median(genScores));
        impostorMedians.push_back(Face::LinAlg::Common::median(impScores));
    }
}

std::vector<double> ScoreNormalizerMedian::normalize(const std::vector<double> &inputScores) const
{
	unsigned int n = inputScores.size();
    if (n != genuineMedians.size()) throw FACELIB_EXCEPTION("invalid scores count");
    std::vector<double> result(n);
    for (unsigned int i = 0; i < n; i++)
    {
        result[i] = (inputScores[i] - genuineMedians[i])/(impostorMedians[i] - genuineMedians[i]);
    }
    return result;
}

void ScoreNormalizerMedian::serialize(cv::FileStorage &storage) const
{
    storage << "genuineMedians" << Face::LinAlg::Vector(genuineMedians);
    storage << "impostorMedians" << Face::LinAlg::Vector(impostorMedians);
}

void ScoreNormalizerMedian::deserialize(cv::FileStorage &storage)
{
    Face::LinAlg::Vector genMediansMat;
    storage["genuineMedians"] >> genMediansMat;
    genuineMedians = genMediansMat.toStdVector();

    Face::LinAlg::Vector impMediansMat;
    storage["impostorMedians"] >> impMediansMat;
    impostorMedians = impMediansMat.toStdVector();
}

// --- ScoreNormalizerZScore ---

void ScoreNormalizerZScore::learn(const std::vector<Evaluation> &evaluations)
{
    means.clear();
    stdDevs.clear();

    for (const Evaluation &e : evaluations)
    {
        std::vector<double> scoresVec;
        if (compensateGenImpCount)
        {
            scoresVec = Face::LinAlg::Common::balanceSizesAndJoin(e.genuineScores, e.impostorScores);
        }
        else
        {
            scoresVec.insert(scoresVec.end(), e.genuineScores.begin(), e.genuineScores.end());
            scoresVec.insert(scoresVec.end(), e.impostorScores.begin(), e.impostorScores.end());
        }
        Face::LinAlg::Vector scores(scoresVec);
        means.push_back(scores.meanValue());
        stdDevs.push_back(scores.stdDeviation());
    }
}

std::vector<double> ScoreNormalizerZScore::normalize(const std::vector<double> &inputScores) const
{
	unsigned int n = means.size();
    if (inputScores.size() != n) throw FACELIB_EXCEPTION("invalid scores count");
    std::vector<double> result(n);
    for (unsigned int i = 0; i < n; i++)
    {
        result[i] = (inputScores[i]-means[i])/stdDevs[i];
    }
    return result;
}

void ScoreNormalizerZScore::serialize(cv::FileStorage &storage) const
{
    storage << "means" << Face::LinAlg::Vector(means);
    storage << "stdDevs" << Face::LinAlg::Vector(stdDevs);
}

void ScoreNormalizerZScore::deserialize(cv::FileStorage &storage)
{
    Face::LinAlg::Vector meansMat;
    storage["means"] >> meansMat;
    means = meansMat.toStdVector();

    Face::LinAlg::Vector stdDevsMat;
    storage["stdDevs"] >> stdDevsMat;
    stdDevs = stdDevsMat.toStdVector();
}

// --- ScoreNormalizerMAD ---

void ScoreNormalizerMAD::learn(const std::vector<Evaluation> &evaluations)
{
    medians.clear();
    mads.clear();

    for (const Evaluation &e : evaluations)
    {
        std::vector<double> scoresVec;
        if (compensateGenImpCount)
        {
            scoresVec = Face::LinAlg::Common::balanceSizesAndJoin(e.genuineScores, e.impostorScores);
        }
        else
        {
            scoresVec.insert(scoresVec.end(), e.genuineScores.begin(), e.genuineScores.end());
            scoresVec.insert(scoresVec.end(), e.impostorScores.begin(), e.impostorScores.end());
        }

        double m = Face::LinAlg::Common::median(scoresVec);
        std::vector<double> deviations;
        for (double s : scoresVec)
        {
            deviations.push_back(fabs(m - s));
        }

        medians.push_back(m);
        mads.push_back(Face::LinAlg::Common::median(deviations));
    }
}

std::vector<double> ScoreNormalizerMAD::normalize(const std::vector<double> &inputScores) const
{
	unsigned int n = medians.size();
    if (inputScores.size() != n) throw FACELIB_EXCEPTION("invalid scores count");
    std::vector<double> result(n);
    for (unsigned int i = 0; i < n; i++)
    {
        result[i] = (inputScores[i] - medians[i])/mads[i];
    }
    return result;
}

void ScoreNormalizerMAD::serialize(cv::FileStorage &storage) const
{
    storage << "medians" << Face::LinAlg::Vector(medians);
    storage << "mads" << Face::LinAlg::Vector(mads);
}

void ScoreNormalizerMAD::deserialize(cv::FileStorage &storage)
{
    Face::LinAlg::Vector mediansMat;
    storage["medians"] >> mediansMat;
    medians = mediansMat.toStdVector();

    Face::LinAlg::Vector madsMat;
    storage["mads"] >> madsMat;
    mads = madsMat.toStdVector();
}

// --- ScoreNormalizerTanh ---

void ScoreNormalizerTanh::learn(const std::vector<Evaluation> &evaluations)
{
    means.clear();
    stdDevs.clear();

    for (const Evaluation &e : evaluations)
    {
        std::vector<double> scoresVec;
        if (compensateGenImpCount)
        {
            scoresVec = Face::LinAlg::Common::balanceSizesAndJoin(e.genuineScores, e.impostorScores);
        }
        else
        {
            scoresVec.insert(scoresVec.end(), e.genuineScores.begin(), e.genuineScores.end());
            scoresVec.insert(scoresVec.end(), e.impostorScores.begin(), e.impostorScores.end());
        }
        Face::LinAlg::Vector scores(scoresVec);
        means.push_back(scores.meanValue());
        stdDevs.push_back(scores.stdDeviation());
    }
}

std::vector<double> ScoreNormalizerTanh::normalize(const std::vector<double> &inputScores) const
{
	unsigned int n = means.size();
    if (inputScores.size() != n) throw FACELIB_EXCEPTION("invalid scores count");
    std::vector<double> result(n);
    for (unsigned int i = 0; i < n; i++)
    {
        result[i] = 0.5 * (tanh(0.01 * (inputScores[i] - means[i])/stdDevs[i]) + 1);
    }
    return result;
}

void ScoreNormalizerTanh::serialize(cv::FileStorage &storage) const
{
    storage << "means" << Face::LinAlg::Vector(means);
    storage << "stdDevs" << Face::LinAlg::Vector(stdDevs);
}

void ScoreNormalizerTanh::deserialize(cv::FileStorage &storage)
{
    Face::LinAlg::Vector meansMat;
    storage["means"] >> meansMat;
    means = meansMat.toStdVector();

    Face::LinAlg::Vector stdDevsMat;
    storage["stdDevs"] >> stdDevsMat;
    stdDevs = stdDevsMat.toStdVector();
}
