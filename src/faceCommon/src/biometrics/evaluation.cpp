#include "faceCommon/biometrics/evaluation.h"

#include <fstream>
#include <cmath>

#include "faceCommon/linalg/histogram.h"
#include "faceCommon/biometrics/template.h"

using namespace Face::Biometrics;

void Evaluation::commonInit()
{
    eer = 1.0;
    eerScore = 0.0;

    minGenuineScore = 1e300;
    minImpostorScore = 1e300;
    minScore = 1e300;

    maxGenuineScore = -1e300;
    maxImpostorScore = -1e300;
    maxScore = -1e300;

    meanGenuineScore = meanImpostorScore = 0;

    scoreType = SCORE_UNKNOWN;
}

void Evaluation::commonInitOfScores()
{
    Face::LinAlg::Vector genVec(genuineScores);
    minGenuineScore = genVec.minValue();
    maxGenuineScore = genVec.maxValue();
    meanGenuineScore = genVec.meanValue();

    Face::LinAlg::Vector impVec(impostorScores);
    minImpostorScore = impVec.minValue();
    maxImpostorScore = impVec.maxValue();
    meanImpostorScore = impVec.meanValue();

    minScore = minGenuineScore < minImpostorScore ? minGenuineScore : minImpostorScore;
    maxScore = maxImpostorScore > maxGenuineScore ? maxImpostorScore : maxGenuineScore;

    if (meanImpostorScore > meanGenuineScore) scoreType = SCORE_DISTANCE;
    if (meanImpostorScore < meanGenuineScore) scoreType = SCORE_SIMILARITY;
}

Evaluation::Evaluation()
{
    commonInit();
}

Evaluation::Evaluation(const std::vector<double> &genuineScores, const std::vector<double> &impostorScores) :
    genuineScores(genuineScores), impostorScores(impostorScores)
{
    commonInit();
    commonInitOfScores();
    commonEvaluation();
}

Evaluation::Evaluation(std::map<std::pair<int, int>, std::vector<double> > &distances)
{
    commonInit();

    for (auto i = distances.begin(); i != distances.end(); ++i)
    {
        bool genuine = i->first.first == i->first.second;

        for (auto j = i->second.begin(); j != i->second.end(); ++j)
        {
            double d = *j;
            if (d != d) throw FACELIB_EXCEPTION("NaN");

            // same or different?
            if (genuine)
            {
                genuineScores.push_back(d);
            }
            else
            {
                impostorScores.push_back(d);
            }
        }
    }

    commonInitOfScores();
    commonEvaluation();
}

Evaluation::Evaluation(const std::vector<Template> &templates, const Face::LinAlg::Metrics &metrics)
{
    commonInit();
    commonTemplatesEvaluation(templates, metrics);
    commonEvaluation();
}

Evaluation::Evaluation(const std::vector<Face::LinAlg::Vector> &rawData, const std::vector<int> &classes,
                       const Face::Biometrics::FeatureExtractor &extractor,
                       const Face::LinAlg::Metrics &metric)
{
    commonInit();
    std::vector<Template> templates = Template::createTemplates(rawData, classes, extractor);

    commonTemplatesEvaluation(templates, metric);
    commonEvaluation();
}

void Evaluation::commonEvaluation()
{
    // DET, EER
    double delta = maxScore - minScore;
    double step = delta/1000.0;
    if (step == 0) return;

    double minDiff = 1e300;

    for (double t = minScore; t <= maxScore; t += step)
    {
        distances.push_back(t);

        // FNMR
        double sameMismatchCount = 0.0;
        for (unsigned int i = 0; i < genuineScores.size(); i++)
        {
            if ((scoreType == SCORE_DISTANCE) && (genuineScores[i] > t)) sameMismatchCount+=1.0;
            if ((scoreType == SCORE_SIMILARITY) && (genuineScores[i] < t)) sameMismatchCount+=1.0;
        }
        sameMismatchCount /= genuineScores.size();
        fnmr.push_back(sameMismatchCount);

        // FMR
        double differentMatchCount = 0.0;
        for (unsigned int i = 0; i < impostorScores.size(); i++)
        {
            if ((scoreType == SCORE_DISTANCE) && (impostorScores[i] < t)) differentMatchCount+=1.0;
            if ((scoreType == SCORE_SIMILARITY) && (impostorScores[i] > t)) differentMatchCount+=1.0;
        }
        differentMatchCount /= impostorScores.size();
        fmr.push_back(differentMatchCount);

        // EER
        double diff = fabs(sameMismatchCount - differentMatchCount);
        if (diff < minDiff)
        {
            minDiff = diff;
            eer = (sameMismatchCount+differentMatchCount)/2;
            eerScore = t;
        }
    }
}

void Evaluation::fnmrAndFmrAtDistance(const double inDistance, double &outFnmr, double &outFmr) const
{
    int n = fmr.size();
    double minDiff = 1e300;
    for (int i = 0; i < n; i++)
    {
        double diff = fabs(distances[i]-inDistance);
        if (diff < minDiff)
        {
            minDiff = diff;
            outFnmr = fnmr[i];
            outFmr = fmr[i];
        }
    }
}

void Evaluation::fnmrAtFmr(const double inFmr, double &outFnmr, double &outDistance) const
{
    int n = fmr.size();
	double minDiff = 1e300;
    outFnmr = 1.0;
	for (int i = 0; i < n; i++)
	{
        double diff = fabs(fmr[i]-inFmr);
		if (diff < minDiff)
		{
			minDiff = diff;
            outFnmr = fnmr[i];
            outDistance = distances[i];
		}
	}
}

void Evaluation::printStats() const
{
    std::cout << "EER: " << eer << "; distance: " << eerScore << std::endl;

    std::vector<double> fmrs;
    fmrs.push_back(0.01);
    fmrs.push_back(0.001);
    fmrs.push_back(0.0001);
    fmrs.push_back(0.00001);

    for (double desiredFmr : fmrs)
    {
        double fnmr, distance;
        fnmrAtFmr(desiredFmr, fnmr, distance);
        std::cout << "FNMR @ FMR = " << desiredFmr << ": " << fnmr << "; distance: " << distance << std::endl;
    }
}

void Evaluation::commonTemplatesEvaluation(const std::vector<Template> &templates, const Face::LinAlg::Metrics &metrics)
{
    int n = templates.size();
    for (int i = 0; i < (n-1); i++)
    {
        const Template &ti = templates[i];
        if (Face::LinAlg::Common::matrixContainsNan(ti.featureVector))
            throw FACELIB_EXCEPTION("feature vector contains NaN");

        for (int j = i+1; j < n; j++)
        {
            const Template &tj = templates[j];
            if (Face::LinAlg::Common::matrixContainsNan(tj.featureVector))
                throw FACELIB_EXCEPTION("feature vector contains NaN");
            double d = metrics.distance(ti.featureVector, tj.featureVector);
            if (d != d)
                throw FACELIB_EXCEPTION("NaN");

            // same or different?
            if (ti.subjectID == tj.subjectID)
            {
                genuineScores.push_back(d);
                if (d > maxGenuineScore) maxGenuineScore = d;
                if (d < minGenuineScore) minGenuineScore = d;
            }
            else
            {
                impostorScores.push_back(d);
                if (d > maxImpostorScore) maxImpostorScore = d;
                if (d < minImpostorScore) minImpostorScore = d;
            }

            if (d > maxScore) maxScore = d;
            if (d < minScore) minScore = d;
        }
    }
    commonInitOfScores();
}

void Evaluation::outputResultsDET(const std::string &path) const
{
    std::ofstream out(path);

    for (unsigned int i = 0; i < distances.size(); i++)
    {
        out << fmr[i] << ' ' << fnmr[i] << std::endl;
    }
    out.close();
}

void Evaluation::outputResultsFMR(const std::string &path) const
{
    Face::LinAlg::Common::savePlot(distances, fmr, path);
}

void Evaluation::outputResultsFNMR(const std::string &path) const
{
    Face::LinAlg::Common::savePlot(distances, fnmr, path);
}

void Evaluation::outputResultsGenuineDistribution(const std::string &path, int bins) const
{
    Face::LinAlg::Histogram(genuineScores, bins, true, minScore, maxScore).savePlot(path);
}

void Evaluation::outputResultsGenuineScores(const std::string &path) const
{
    Face::LinAlg::Vector(genuineScores).toFile(path);
}

void Evaluation::outputResultsImpostorDistribution(const std::string &path, int bins) const
{
    Face::LinAlg::Histogram(impostorScores, bins, true, minScore, maxScore).savePlot(path);
}

void Evaluation::outputResultsImpostorScores(const std::string &path) const
{
    Face::LinAlg::Vector(impostorScores).toFile(path);
}

void Evaluation::outputResults(const std::string &path, int histogramBins) const
{
    // DET
    outputResultsFMR(path+"-fmr");
    outputResultsFNMR(path+"-fnmr");
    outputResultsDET(path+"-det");

    // Impostor, Genuine
    outputResultsImpostorDistribution(path+"-imp-distrib", histogramBins);
    outputResultsImpostorScores(path+"-imp-scores");
    outputResultsGenuineDistribution(path+"-gen-distrib", histogramBins);
    outputResultsGenuineScores(path+"-gen-scores");
}

BatchEvaluationResult Evaluation::batch(std::vector<std::vector<Template> > &templates, const Face::LinAlg::Metrics &metrics, int startIndex)
{
    BatchEvaluationResult batchResult;
    std::vector<double> eer;

    int cCount = templates.size();
    for (int i = startIndex; i < cCount; i++)
    {
        Evaluation e(templates[i], metrics);
        batchResult.results.push_back(e);
        eer.push_back(e.eer);
    }

    Face::LinAlg::Vector eerVec = Face::LinAlg::Vector(eer);
    batchResult.meanEER = eerVec.meanValue();
    batchResult.stdDevOfEER = eerVec.stdDeviation();

    return batchResult;
}

BatchEvaluationResult Evaluation::batch(std::vector<std::vector<Face::LinAlg::Vector> > &images, std::vector<std::vector<int> > &classes,
        const Face::Biometrics::FeatureExtractor &extractor, const Face::LinAlg::Metrics &metrics, int startIndex)
{
    BatchEvaluationResult batchResult;
    std::vector<double> eer;

    int cCount = images.size();
    for (int i = startIndex; i < cCount; i++)
    {
        Evaluation e(images[i], classes[i], extractor, metrics);
        batchResult.results.push_back(e);
        eer.push_back(e.eer);
    }

    Face::LinAlg::Vector eerVec = Face::LinAlg::Vector(eer);
    batchResult.meanEER = eerVec.meanValue();
    batchResult.stdDevOfEER = eerVec.stdDeviation();

    return batchResult;
}
