#ifndef SCORELEVELFUSION_H
#define SCORELEVELFUSION_H

#include <QVector>

#include <opencv/ml.h>

#include "featureextractor.h"
#include "linalg/common.h"
#include "evaluation.h"
#include "linalg/lda.h"
#include "linalg/logisticregression.h"

/*class ScoreLevelFusionComponent
{
public:
	QVector<Matrix> &trainRawData;
	QVector<int> &trainClasses;
	FeatureExtractor &featureExtractor;
	Metrics &metrics;

	ScoreLevelFusionComponent(
			QVector<Matrix> &trainRawData,
			QVector<int> &trainClasses,
			FeatureExtractor &featureExtractor,
			Metrics &metrics) :
				trainRawData(trainRawData),
				trainClasses(trainClasses),
				featureExtractor(featureExtractor),
				metrics(metrics)
	{}
};*/

class ScoreLevelFusionBase
{
private:
	QList<QVector<Matrix> *> trainRawData;
	QList<QVector<int> *> trainClasses;
	QVector<FeatureExtractor *> extractors;
	QVector<Metrics *> metrics;

	bool learned;

protected:
    QVector<double> genuineMeans;
    QVector<double> impostorMeans;

    void prepareDataForClassification(QList<Evaluation> &evaluationResults,
                                      QVector<Matrix> &scores, QVector<int> &classes,
                                      int genuineLabel, int impostorLabel);

    Matrix normalizeScore(QVector<double> &score);

    virtual void learnImplementation(QList<Evaluation> &evaluationResults) = 0;

public:
    ScoreLevelFusionBase() { learned = false; }

    void learn();

    virtual double fuse(QVector<double> &scores) = 0;

    ScoreLevelFusionBase & addComponent(
			QVector<Matrix> &trainRawData,
			QVector<int> &trainClasses,
			FeatureExtractor &featureExtractor,
			Metrics &metrics);

    Evaluation evaluate(QList<QVector<Matrix> > &rawData, QVector<int> &classes, bool debugOutput = false);

    virtual ~ScoreLevelFusionBase() {}
};

/*class ScoreLevelFusionSystem
{
	QVector<FeatureExtractor *> extractors;
	QVector<Metrics *> metrics;
	ScoreLeveFusionBase &fuser;

public:
	ScoreLevelFusionSystem(ScoreLeveFusionBase &fuser) : fuser(fuser) {}

    Evaluation evaluate(QList<QVector<Matrix> > &rawData, QVector<int> &classes, bool debugOutput = false);

    void addModule(FeatureExtractor &featureExtractor, Metrics& metrics);
};*/

class ScoreLDAFusion : public ScoreLevelFusionBase
{
private:
    double maxScore;
    double minScore;

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

class ScoreSVMFusion : public ScoreLevelFusionBase
{
private:
    cv::SVM svm;

    cv::Mat colVectorsToFPMatrix(QVector<Matrix> &vectors);
    cv::Mat colVectorToColFPMatrix(QVector<int> &vector);
    cv::Mat colVectorToColFPMatrix(QVector<double> &vector);

public:
    void learnImplementation(QList<Evaluation> &evaluationResults);
    double fuse(QVector<double> &scores);
    virtual ~ScoreSVMFusion() {}
};

#endif // MULTIBIOSYSTEM_H
