/*
 * featurevectorfusion.cpp
 *
 *  Created on: 9.5.2012
 *      Author: stepo
 */
#include "featurelevelfusion.h"

#include <cassert>

#include "linalg/vector.h"

FeatureVectorFusionBase & FeatureVectorFusionBase::addComponent(
		QVector<Matrix> &trainRawData,
		QVector<int> &trainClasses,
		ZScoreFeatureExtractor &featureExtractor,
		Metrics &metrics)
{
	this->extractors << (&featureExtractor);
	this->metrics << (&metrics);
	this->trainClasses << (&trainClasses);
	this->trainRawData << (&trainRawData);

	return (*this);
}

void FeatureVectorFusionBase::learn()
{
	learnImplementation();
	learned = true;
}

QVector<Matrix> FeatureVectorFusionBase::batchFuse(QList<QVector<Matrix> > inputMatricies)
{
	assert(learned);
	int unitsCount = inputMatricies.count();
	assert(unitsCount > 0);

	int matriciesPerUnitCount = inputMatricies[0].count();

	QVector<Matrix> resultVector;
	for (int i = 0; i < matriciesPerUnitCount; i++)
	{
		QVector<Matrix> input;
		for (int j = 0; j < unitsCount; j++)
		{
			input << inputMatricies[j][i];
		}
		Matrix result = fuse(input);
		resultVector << result;
	}
	return resultVector;
}

// --- Concatenation ---

Matrix FeatureVectorFusionConcatenation::fuse(QVector<Matrix> &inputMatricies)
{
	int n = inputMatricies.count();
	QVector<double> vec;
	for (int i = 0; i < n; i++)
	{
		int r = inputMatricies[i].rows;
		for (int j = 0; j < r; j++)
		{
			vec << inputMatricies[i](j);
		}
	}

	Matrix result = Vector::fromQVector(vec);
	return result;
}

/*void FeatureVectorFusionLDA::learn(QVector<Matrix> &rawData, QVector<int> &classes)
{
	int exCount = extractors.count();
	assert(exCount > 0);

	int n = rawData.count();
	assert(n > 0);
	assert(n == classes.count());

	//double totalOutputLen = outputLen();

	// for all input data raw vectors
	QVector<Matrix> concatenatedProjectedData;
	for (int i = 0; i < n; i++)
	{
		// for each extractor
		QVector<double> concatenatedVector;
		for (int e = 0; e < exCount; e++)
		{
			// extract features
			Matrix projected = extractors[e]->extract(rawData[i]);

			// modify the FV coeficients according to the length of FV
			//double currentExtractorLen = extractors[e]->outputLen();
			//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

			QVector<double> projectedVector = Vector::toQVector(projected);
			concatenatedVector << projectedVector;
		}
		Matrix concatenated = Vector::fromQVector(concatenatedVector);
		concatenatedProjectedData << concatenated;
	}

	lda.learn(concatenatedProjectedData, classes);
}

Matrix FeatureVectorFusionLDA::extract(Matrix &rawData)
{
	int exCount = extractors.count();
	assert(exCount > 0);
	assert(lda.Wt.rows > 0);
	assert(lda.Wt.cols > 0);

	//double totalOutputLen = outputLen();
	QVector<double> concatenatedVector;
	for (int e = 0; e < exCount; e++)
	{
		Matrix projected = extractors[e]->extract(rawData);
		// modify the FV coeficients according to the length of FV
		//double currentExtractorLen = extractors[e]->outputLen();
		//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

		QVector<double> projectedVector = Vector::toQVector(projected);
		concatenatedVector << projectedVector;
	}
	Matrix concatenated = Vector::fromQVector(concatenatedVector);
	Matrix fusionProjected = lda.project(concatenated);
	return fusionProjected;
}

void FeatureVectorFusionPCA::learn(QVector<Matrix> &rawData)
{
	int exCount = extractors.count();
	assert(exCount > 0);

	int n = rawData.count();
	assert(n > 0);

	//double totalOutputLen = outputLen();

	// for all input data raw vectors
	QVector<Matrix> concatenatedProjectedData;
	for (int i = 0; i < n; i++)
	{
		// for each extractor
		QVector<double> concatenatedVector;
		for (int e = 0; e < exCount; e++)
		{
			// extract features
			Matrix projected = extractors[e]->extract(rawData[i]);

			// modify the FV coeficients according to the length of FV
			//double currentExtractorLen = extractors[e]->outputLen();
			//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

			QVector<double> projectedVector = Vector::toQVector(projected);
			concatenatedVector << projectedVector;
		}
		Matrix concatenated = Vector::fromQVector(concatenatedVector);
		concatenatedProjectedData << concatenated;
	}

	pca.learn(concatenatedProjectedData);
	pca.modesSelectionThreshold();
}

Matrix FeatureVectorFusionPCA::extract(Matrix &rawData)
{
	int exCount = extractors.count();
	assert(exCount > 0);
	assert(pca.getModes() > 0);

	//double totalOutputLen = outputLen();
	QVector<double> concatenatedVector;
	for (int e = 0; e < exCount; e++)
	{
		Matrix projected = extractors[e]->extract(rawData);
		// modify the FV coeficients according to the length of FV
		//double currentExtractorLen = extractors[e]->outputLen();
		//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

		QVector<double> projectedVector = Vector::toQVector(projected);
		concatenatedVector << projectedVector;
	}
	Matrix concatenated = Vector::fromQVector(concatenatedVector);
	Matrix fusionProjected = pca.project(concatenated);
	return fusionProjected;
}

void FeatureVectorFusionICA::learn(QVector<Matrix> &rawData)
{
	int exCount = extractors.count();
	assert(exCount > 0);

	int n = rawData.count();
	assert(n > 0);

	//double totalOutputLen = outputLen();

	// for all input data raw vectors
	QVector<Matrix> concatenatedProjectedData;
	for (int i = 0; i < n; i++)
	{
		// for each extractor
		QVector<double> concatenatedVector;
		for (int e = 0; e < exCount; e++)
		{
			// extract features
			Matrix projected = extractors[e]->extract(rawData[i]);

			// modify the FV coeficients according to the length of FV
			//double currentExtractorLen = extractors[e]->outputLen();
			//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

			QVector<double> projectedVector = Vector::toQVector(projected);
			concatenatedVector << projectedVector;
		}
		Matrix concatenated = Vector::fromQVector(concatenatedVector);
		concatenatedProjectedData << concatenated;
	}

	icaOfPca.learn(concatenatedProjectedData);
}

Matrix FeatureVectorFusionICA::extract(Matrix &rawData)
{
	int exCount = extractors.count();
	assert(exCount > 0);
	assert(icaOfPca.ica.getModes() > 0);

	//double totalOutputLen = outputLen();
	QVector<double> concatenatedVector;
	for (int e = 0; e < exCount; e++)
	{
		Matrix projected = extractors[e]->extract(rawData);
		// modify the FV coeficients according to the length of FV
		//double currentExtractorLen = extractors[e]->outputLen();
		//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

		QVector<double> projectedVector = Vector::toQVector(projected);
		concatenatedVector << projectedVector;
	}
	Matrix concatenated = Vector::fromQVector(concatenatedVector);
	Matrix fusionProjected = icaOfPca.project(concatenated);
	return fusionProjected;
}

Matrix FeatureVectorFusionConcatenation::extract(Matrix &rawData)
{
	int exCount = extractors.count();
	assert(exCount > 0);

	//double totalOutputLen = outputLen();
	QVector<double> concatenatedVector;
	for (int e = 0; e < exCount; e++)
	{
		Matrix projected = extractors[e]->extract(rawData);
		// modify the FV coeficients according to the length of FV
		//double currentExtractorLen = extractors[e]->outputLen();
		//projected *= ((1.0-currentExtractorLen)/totalOutputLen);

		QVector<double> projectedVector = Vector::toQVector(projected);
		concatenatedVector << projectedVector;
	}
	Matrix concatenated = Vector::fromQVector(concatenatedVector);
	return concatenated;
}*/
