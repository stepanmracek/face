#ifndef EVALUATION_H
#define EVALUATION_H

#include <QVector>
#include <QList>
#include <QString>
#include <QHash>

#include "linalg/common.h"
#include "biometrics/template.h"
#include "linalg/metrics.h"
#include "linalg/vector.h"

class Evaluation;
class BatchEvaluationResult;

class Evaluation
{
private:
    void commonEvaluation(bool debugOutput);
    bool commonTemplatesEvaluation(QVector<Template> &templates, const Metrics &metrics, bool debugOutput);
    void commonInit();

public:
    double minSameDistance;
    double maxSameDistance;
    double minDifferentDistance;
    double maxDifferentDistance;
    double minDistance;
    double maxDistance;

    QVector<double> genuineScores;
    QVector<double> impostorScores;
    QVector<double> scores;

    QVector<double> fmr;
    QVector<double> fnmr;
    QVector<double> thresholds;

    double eer;
    double eerDistance;

    Evaluation();

    Evaluation(QVector<Template> &templates, const Metrics &metrics, bool debugOutput = false);

    Evaluation(QHash<QPair<int, int>, double> &distances, bool debugOutput = false);

    Evaluation(QVector<Vector> &rawData, QVector<int> &classes,
               const FeatureExtractor &extractor, const Metrics &metric, bool debugOutput = false);

    double fnmrAtFmr(double fmr);

    void outputResults(const QString &path, int histogramBins) const;
    void outputResultsDET(const QString &path) const;
    void outputResultsGenuineDistribution(const QString &path, int bins) const;
    void outputResultsImpostorDistribution(const QString &path, int bins) const;
    void outputResultsGenuineScores(const QString &path) const;
    void outputResultsImpostorScores(const QString &path) const;
    void outputResultsFMR(const QString &path) const;
    void outputResultsFNMR(const QString &path) const;

    static BatchEvaluationResult batch(QList<QVector<Template> > &templates, const Metrics &metrics, int startIndex = 0);

    static BatchEvaluationResult batch(QList<QVector<Vector> > &images, QList<QVector<int> > &classes,
                                       const FeatureExtractor &extractor, const Metrics &metrics, int startIndex = 0);
};

class BatchEvaluationResult
{
public:
    QList<Evaluation> results;
    double meanEER;
    double stdDevOfEER;
};

#endif // EVALUATION_H
