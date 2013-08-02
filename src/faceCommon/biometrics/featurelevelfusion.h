/*
 * featurevectorfusion.h
 *
 *  Created on: 9.5.2012
 *      Author: stepo
 */

#ifndef FEATUREVECTORFUSION_H_
#define FEATUREVECTORFUSION_H_

#include <QList>

#include "linalg/common.h"
#include "linalg/vector.h"
#include "linalg/lda.h"
#include "linalg/metrics.h"
#include "featureextractor.h"
#include "template.h"

class FeatureVectorFusionBase
{
private:
    bool learned;

protected:
    QList<QVector<Vector> *> trainRawData;
	QList<QVector<int> *> trainClasses;
    QVector<FeatureExtractor *> extractors;
	QVector<Metrics *> metrics;

	virtual void learnImplementation() = 0;

public:
	FeatureVectorFusionBase() { learned = false; }

	FeatureVectorFusionBase & addComponent(
                QVector<Vector> &trainRawData,
				QVector<int> &trainClasses,
                FeatureExtractor &featureExtractor,
				Metrics &metrics);

    QVector<Vector> batchFuse(QList<QVector<Vector> > inputMatricies);

	void learn();

    virtual Vector fuse(QVector<Vector> &inputMatricies) = 0;

	virtual ~FeatureVectorFusionBase() {}

    bool isLearned() { return learned; }
};

class FeatureVectorFusionConcatenation : public FeatureVectorFusionBase
{
public:
    Vector fuse(QVector<Vector> &inputMatricies);

	// no nead to learn for simple concatenation
	void learnImplementation() {}

	virtual ~FeatureVectorFusionConcatenation() {}
};

/*class FeatureVectorFusionLDA : public FeatureVectorFusionBase
{
public:
	LDA lda;

	void learn(QVector<Matrix> &rawData, QVector<int> &classes);

	Matrix fuse(QVector<Matrix> inputMatricies);
};

class FeatureVectorFusionPCA : public FeatureVectorFusionBase
{
public:
	PCA pca;

	void learn(QVector<Matrix> &rawData);

	Matrix fuse(QVector<Matrix> inputMatricies);
};

class FeatureVectorFusionICA : public FeatureVectorFusionBase
{
public:
	ICAofPCA icaOfPca;

	void learn(QVector<Matrix> &rawData);

	Matrix fuse(QVector<Matrix> inputMatricies);
};*/

#endif /* FEATUREVECTORFUSION_H_ */
