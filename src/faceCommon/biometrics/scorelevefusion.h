#ifndef SCORELEVELFUSION_H
#define SCORELEVELFUSION_H

#include <QVector>

#include <opencv/ml.h>

#include "featureextractor.h"
#include "linalg/common.h"
#include "evaluation.h"
#include "linalg/lda.h"
#include "linalg/logisticregression.h"
#include "linalg/vector.h"

class ScoreLevelFusionComponent
{
public:
    const QVector<Vector> trainRawData;
    const QVector<int> trainClasses;
    const FeatureExtractor *featureExtractor;
    const Metrics *metrics;

    ScoreLevelFusionComponent() {}

    ScoreLevelFusionComponent(const QVector<Vector> &trainRawData,
                              const QVector<int> &trainClasses,
                              const FeatureExtractor *featureExtractor,
                              const Metrics *metrics) :
        trainRawData(trainRawData), trainClasses(trainClasses),
        featureExtractor(featureExtractor), metrics(metrics) {}
};

class ScoreLevelFusionBase
{
private:
    QList<ScoreLevelFusionComponent> components;
    /*QList<QVector<Vector> *> trainRawData;
	QList<QVector<int> *> trainClasses;
    QVector<const FeatureExtractor *> extractors;
    QVector<const Metrics *> metrics;*/

protected:
    bool learned;
    QVector<double> genuineMeans;
    QVector<double> impostorMeans;

    void prepareDataForClassification(QList<Evaluation> &evaluationResults,
                                      QVector<Vector> &scores, QVector<int> &classes,
                                      int genuineLabel, int impostorLabel);

    Vector normalizeScore(QVector<double> &score);

    virtual void learnImplementation(QList<Evaluation> &evaluationResults) = 0;

public:
    ScoreLevelFusionBase() { learned = false; }

    void learn();

    virtual double fuse(QVector<double> &scores) = 0;

    ScoreLevelFusionBase & addComponent(const ScoreLevelFusionComponent &component);

    void popComponent();

    Evaluation evaluate(const QList<QVector<Vector> > &rawData, const QVector<int> &classes, bool debugOutput = false);

    virtual ~ScoreLevelFusionBase() {}
};

class ScoreLDAFusion : public ScoreLevelFusionBase
{
private:
    double maxScore;
    double minScore;
    bool swapResultScore;

    LDA lda;

public:
    void learnImplementation(QList<Evaluation> &evaluationResults);
    double fuse(QVector<double> &scores);
    virtual ~ScoreLDAFusion() {}
};

class ScoreLogisticRegressionFusion : public ScoreLevelFusionBase
{
private:
    LogisticRegression logR;

public:
    void learnImplementation(QList<Evaluation> &evaluationResults);
    double fuse(QVector<double> &scores);
    virtual ~ScoreLogisticRegressionFusion() {}
};

class ScoreWeightedSumFusion : public ScoreLevelFusionBase
{
private:
    QVector<double> eer;
    double weightDenominator;

public:
    void learnImplementation(QList<Evaluation> &evaluationResults);
    double fuse(QVector<double> &scores);
    virtual ~ScoreWeightedSumFusion() {}
};

class ScoreProductFusion : public ScoreLevelFusionBase
{
public:
	void learnImplementation(QList<Evaluation> &evaluationResults);
    double fuse(QVector<double> &scores);
    virtual ~ScoreProductFusion() {}
};

class ScoreDummyProductFusion : public ScoreLevelFusionBase
{
public:
    virtual ~ScoreDummyProductFusion() {}
    void learnImplementation(QList<Evaluation> &evaluationResults) {}
    double fuse(QVector<double> &scores);
};

class ScoreSVMFusion : public ScoreLevelFusionBase
{
private:
    cv::SVM svm;

    cv::Mat colVectorsToFPMatrix(QVector<Vector> &vectors);
    cv::Mat colVectorToColFPMatrix(QVector<int> &vector);
    cv::Mat colVectorToColFPMatrix(QVector<double> &vector);

public:
    void learnImplementation(QList<Evaluation> &evaluationResults);
    double fuse(QVector<double> &scores);
    virtual ~ScoreSVMFusion() {}
};

#endif // MULTIBIOSYSTEM_H
