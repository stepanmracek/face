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
    void commonTemplatesEvaluation(QVector<Template> &templates, const Metrics &metrics, bool debugOutput);

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
    QVector<double> impostorDistribution;
    QVector<double> genuineDistribution;

    double eer;
    double eerDistance;

    Evaluation() {}

    Evaluation(QVector<Template> &templates, const Metrics &metrics, bool debugOutput = false);

    Evaluation(QHash<QPair<int, int>, double> &distances, bool debugOutput = false);

    Evaluation(QVector<Vector> &rawData, QVector<int> &classes,
               const FeatureExtractor &extractor, const Metrics &metric, bool debugOutput = false);

    double fnmrAtFmr(double fmr);

    void outputResults(const QString &path);
    void outputResultsDET(const QString &path);
    void outputResultsGenuine(const QString &path);
    void outputResultsImpostor(const QString &path);

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
