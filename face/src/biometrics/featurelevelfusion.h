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
#include "linalg/lda.h"
#include "linalg/metrics.h"
#include "featureextractor.h"
#include "template.h"

class FeatureVectorFusionBase
{
protected:
	QList<QVector<Matrix> *> trainRawData;
	QList<QVector<int> *> trainClasses;
	QVector<ZScoreFeatureExtractor *> extractors;
	QVector<Metrics *> metrics;

	bool learned;

	virtual void learnImplementation() = 0;

public:
	FeatureVectorFusionBase() { learned = false; }

	FeatureVectorFusionBase & addComponent(
				QVector<Matrix> &trainRawData,
				QVector<int> &trainClasses,
				ZScoreFeatureExtractor &featureExtractor,
				Metrics &metrics);

	QVector<Matrix> batchFuse(QList<QVector<Matrix> > inputMatricies);

	void learn();

	virtual Matrix fuse(QVector<Matrix> &inputMatricies) = 0;

	virtual ~FeatureVectorFusionBase() {}
};

class FeatureVectorFusionConcatenation : public FeatureVectorFusionBase
{
public:
	Matrix fuse(QVector<Matrix> &inputMatricies);

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
