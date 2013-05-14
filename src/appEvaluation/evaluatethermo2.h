/*
 * evaluationthermo2.h
 *
 *  Created on: 12.5.2012
 *      Author: stepo
 */

#ifndef EVALUATIONTHERMO2_H_
#define EVALUATIONTHERMO2_H_

#include <QVector>
#include <QList>
#include <QSet>
#include <QDebug>
#include <QString>

#include <cassert>

#include "linalg/loader.h"
#include "linalg/vector.h"
#include "linalg/common.h"
#include "linalg/pca.h"
#include "linalg/metrics.h"
#include "linalg/icaofpca.h"
#include "linalg/ldaofpca.h"
#include "biometrics/template.h"
#include "biometrics/evaluation.h"
#include "biometrics/biodataprocessing.h"
#include "biometrics/featureextractor.h"
#include "biometrics/scorelevefusion.h"
#include "biometrics/featurelevelfusion.h"
#include "biometrics/featureselection.h"
#include "biometrics/eerpotential.h"
#include "biometrics/discriminativepotential.h"

/*
typedef QVector<double> Doubles;

class EvaluateThermo2
{
public:
	static QString basePath() { return "/media/data/thermo/"; }

	static void evaluateFilterBanks()
	{
		QStringList databases; databases << "germany" << "fit" << "equinox" << "notre-dame";
		QStringList warpModes; warpModes << "Warp2D" << "Project3D";
		QStringList filterBanks; filterBanks << "Vectorization" << "GaborBank" << "LaguerreBank";
		QStringList intensities; intensities << "Global" << "Local";
		foreach(QString database, databases)
		{
			qDebug() << database;
			foreach(QString warpMode, warpModes)
			{
				qDebug() << " " << warpMode;
				foreach(QString intensity, intensities)
				{
					qDebug() << "  " << intensity;
					foreach(QString filterBank, filterBanks)
					{
						qDebug() << "   " << filterBank;

						QString path = basePath() + database+"/"+warpMode+"/"+intensity+"/"+filterBank;
						QVector<Matrix> allData;
						QVector<int> allClasses;
						Loader::loadVectors(path, allData, allClasses, "-", "*.vec");

						Doubles evalPca;
						Doubles evalIca;

						int runs = (database == "notre-dame") ? 3 : 10;
						for (int run = 1; run <= runs; run++)
						{
							// divide
							QList<QVector<Matrix> > data;
							QList<QVector<int> > classes;
							BioDataProcessing::divideToNClusters(allData, allClasses, 3, data, classes);

							// train pca
							PCA pca(data[0]); pca.modesSelectionThreshold();
							ZScorePCAExtractor zpcaExtractor(pca, data[1]);
							Templates trainZpcaTemplates = Template::createTemplates(data[1], classes[1], zpcaExtractor);
							DiscriminativePotential dpZpca(trainZpcaTemplates);
							CosineWeightedMetric cosPcaW; cosPcaW.w = dpZpca.createWeights();

							// train ica
							ICAofPCA ica(data[0]);
							ZScoreICAofPCAExtractor zicaExtractor(ica, data[1]);
							Templates trainZicaTemplates = Template::createTemplates(data[1], classes[1], zicaExtractor);
							DiscriminativePotential dpZica(trainZicaTemplates);
							CosineWeightedMetric cosIcaW; cosIcaW.w = dpZica.createWeights();

							evalPca << Evaluation(data[2], classes[2], zpcaExtractor, cosPcaW).eer;
							evalIca << Evaluation(data[2], classes[2], zicaExtractor, cosIcaW).eer;
						}

						qDebug("      pca %.2f", (Vector::meanValue(evalPca)*100.0));
						qDebug("      ica %.2f", (Vector::meanValue(evalIca)*100.0));
					}
				}
			}
		}
	}

	static void evaluateZScoreAndWeighting()
	{
		QStringList databases; databases << "germany" << "fit" << "equinox" << "notre-dame";
		QStringList warpModes; warpModes << "Warp2D" << "Project3D";

		CosineMetric cos;
		foreach(QString database, databases)
		{
			foreach(QString warpMode, warpModes)
			{
				QString path = "/run/media/stepo/frgc/thermo/"+database+"/"+warpMode+"/Global/Vectorization";

				QVector<Matrix> allData;
				QVector<int> allClasses;
				Loader::loadVectors(path, allData, allClasses, "-", "*.vec");

				Doubles evalPca;
				Doubles evalZPca;
				Doubles evalZPcaW;
				Doubles evalIca;
				Doubles evalZIca;
				Doubles evalZIcaW;

				int runs = (database == "notre-dame") ? 3 : 10;
				for (int run = 1; run <= runs; run++)
				{
					//qDebug(" %d", run);

					// divide
					QList<QVector<Matrix> > data;
					QList<QVector<int> > classes;
					BioDataProcessing::divideToNClusters(allData, allClasses, 3, data, classes);

					// train pca
					PCA pca(data[0]); pca.modesSelectionThreshold();
					PCAExtractor pcaExtractor(pca);
					ZScorePCAExtractor zpcaExtractor(pca, data[1]);
					Templates trainZpcaTemplates = Template::createTemplates(data[1], classes[1], zpcaExtractor);
					DiscriminativePotential dpZpca(trainZpcaTemplates);
					CosineWeightedMetric cosPcaW; cosPcaW.w = dpZpca.createWeights();

					// train ica
					ICAofPCA ica(data[0]);
					ICAofPCAExtractor icaExtractor(ica);
					ZScoreICAofPCAExtractor zicaExtractor(ica, data[1]);
					Templates trainZicaTemplates = Template::createTemplates(data[1], classes[1], zicaExtractor);
					DiscriminativePotential dpZica(trainZicaTemplates);
					CosineWeightedMetric cosIcaW; cosIcaW.w = dpZica.createWeights();

					// evaluate pca
					evalPca << Evaluation(data[2], classes[2], pcaExtractor, cos).eer;
					evalZPca << Evaluation(data[2], classes[2], zpcaExtractor, cos).eer;
					evalZPcaW << Evaluation(data[2], classes[2], zpcaExtractor, cosPcaW).eer;

					// evaluate ica
					evalIca << Evaluation(data[2], classes[2], icaExtractor, cos).eer;
					evalZIca << Evaluation(data[2], classes[2], zicaExtractor, cos).eer;
					evalZIcaW << Evaluation(data[2], classes[2], zicaExtractor, cosIcaW).eer;
				}

				qDebug() << database << warpMode
						<< (Vector::meanValue(evalPca)*100.0)
						<< (Vector::meanValue(evalZPca)*100.0)
						<< (Vector::meanValue(evalZPcaW)*100.0);
				qDebug() << database << warpMode
						<< (Vector::meanValue(evalIca)*100.0)
						<< (Vector::meanValue(evalZIca)*100.0)
						<< (Vector::meanValue(evalZIcaW)*100.0);
			}
		}
	}

	static void evaluteSingleMethods()
	{
		QString dbName = "notre-dame";
		QString warpMode = "Project3D";
		QString path = "/run/media/stepo/frgc/thermo/"+dbName+"/"+warpMode+"/Global/Vectorization";

		QVector<Matrix> allData;
		QVector<int> allClasses;
		Loader::loadVectors(path, allData, allClasses, "-", "*.vec");

		EuclideanMetric eucl;
		CityblockMetric cblock;
		CosineMetric cos;
		CorrelationMetric corr;

		Doubles pcaEucl;
		Doubles pcaCblock;
		Doubles pcaCos;
		Doubles pcaCorr;

		Doubles ldaEucl;
		Doubles ldaCblock;
		Doubles ldaCos;
		Doubles ldaCorr;

		Doubles icaEucl;
		Doubles icaCblock;
		Doubles icaCos;
		Doubles icaCorr;

		Doubles passEucl;
		Doubles passCblock;
		Doubles passCos;
		Doubles passCorr;

		double selectionThresholds[3] = {0.95, 0.98, 0.9999};
		for (int j = 0; j < 3; j++)
		{
			double selThreshold = selectionThresholds[j];

			for (int i = 1; i <= 3; i++)
			{
				QList<QVector<Matrix> > data;
				QList<QVector<int> > classes;
				BioDataProcessing::divideToNClusters(allData, allClasses, 3, data, classes);

				// train
				PCA pca(data[0]); pca.modesSelectionThreshold(selThreshold);
				PCAExtractor pcaExtractor(pca);

				LDAofPCA lda(data[0], classes[0], selThreshold);
				LDAofPCAExtractor ldaExtractor(lda);

				ICAofPCA ica(data[0], selThreshold);
				ICAofPCAExtractor icaExtractor(ica);

				PassExtractor pass;

				// evaluate
				pcaEucl << Evaluation(data[2], classes[2], pcaExtractor, eucl).eer;
				pcaCblock << Evaluation(data[2], classes[2], pcaExtractor, cblock).eer;
				pcaCos << Evaluation(data[2], classes[2], pcaExtractor, cos).eer;
				pcaCorr << Evaluation(data[2], classes[2], pcaExtractor, corr).eer;

				ldaEucl << Evaluation(data[2], classes[2], ldaExtractor, eucl).eer;
				ldaCblock << Evaluation(data[2], classes[2], ldaExtractor, cblock).eer;
				ldaCos << Evaluation(data[2], classes[2], ldaExtractor, cos).eer;
				ldaCorr << Evaluation(data[2], classes[2], ldaExtractor, corr).eer;

				icaEucl << Evaluation(data[2], classes[2], icaExtractor, eucl).eer;
				icaCblock << Evaluation(data[2], classes[2], icaExtractor, cblock).eer;
				icaCos << Evaluation(data[2], classes[2], icaExtractor, cos).eer;
				icaCorr << Evaluation(data[2], classes[2], icaExtractor, corr).eer;

				passEucl << Evaluation(data[2], classes[2], pass, eucl).eer;
				passCblock << Evaluation(data[2], classes[2], pass, cblock).eer;
				passCos << Evaluation(data[2], classes[2], pass, cos).eer;
				passCorr << Evaluation(data[2], classes[2], pass, corr).eer;
			}

			qDebug() << dbName << warpMode;
			qDebug() << "PCA Threshold:" << selThreshold;
			qDebug() << "Eucl, city-block, cosine, corr";
			qDebug() << "PCA " << Vector::meanValue(pcaEucl) << Vector::meanValue(pcaCblock) << Vector::meanValue(pcaCos) << Vector::meanValue(pcaCorr);
			qDebug() << "LDA " << Vector::meanValue(ldaEucl) << Vector::meanValue(ldaCblock) << Vector::meanValue(ldaCos) << Vector::meanValue(ldaCorr);
			qDebug() << "ICA " << Vector::meanValue(icaEucl) << Vector::meanValue(icaCblock) << Vector::meanValue(icaCos) << Vector::meanValue(icaCorr);
			qDebug() << "None" << Vector::meanValue(passEucl) << Vector::meanValue(passCblock) << Vector::meanValue(passCos) << Vector::meanValue(passCorr);
		}
	}

	static void fusion()
	{
		QStringList databases; databases << "notre-dame"; //<< "germany" << "fit" << "equinox" << "notre-dame";
		QStringList warpModes; warpModes << "Warp2D" << "Project3D";

		double selThresholdPca = 0.9999;
		double selThresholdIca = 0.9999;

		CosineMetric cos;
		foreach(QString database, databases)
		{
			foreach(QString warpMode, warpModes)
			{
				qDebug() << database << warpMode << "pca" << selThresholdPca << "ica" << selThresholdIca;

				QString pathGlobalGabor = basePath() +database+"/"+warpMode+"/Global/GaborBank";
				QString pathLocalGabor = basePath() +database+"/"+warpMode+"/Local/GaborBank";
				QString pathGlobalLaguerre = basePath() +database+"/"+warpMode+"/Global/LaguerreBank";
				QString pathLocalLaguerre = basePath() +database+"/"+warpMode+"/Local/LaguerreBank";
				QString pathGlobalVectorization = basePath() +database+"/"+warpMode+"/Global/Vectorization";

				QVector<Matrix> allGlobalGaborData;
				QVector<int> allGlobalGaborClasses;
				Loader::loadVectors(pathGlobalGabor, allGlobalGaborData, allGlobalGaborClasses, "-", "*.vec");

				QVector<Matrix> allLocalGaborData;
				QVector<int> allLocalGaborClasses;
				Loader::loadVectors(pathLocalGabor, allLocalGaborData, allLocalGaborClasses, "-", "*.vec");

				QVector<Matrix> allGlobalLaguerreData;
				QVector<int> allGlobalLaguerreClasses;
				Loader::loadVectors(pathGlobalLaguerre, allGlobalLaguerreData, allGlobalLaguerreClasses, "-", "*.vec");

				QVector<Matrix> allLocalLaguerreData;
				QVector<int> allLocalLaguerreClasses;
				Loader::loadVectors(pathLocalLaguerre, allLocalLaguerreData, allLocalLaguerreClasses, "-", "*.vec");

				QVector<Matrix> allGlobalVectorizationData;
				QVector<int> allGlobalVectorizationClasses;
				Loader::loadVectors(pathGlobalVectorization, allGlobalVectorizationData, allGlobalVectorizationClasses, "-", "*.vec");

				CosineMetric cos;

				// savedResults
				QVector<double> resultsGlobalGaborPca;
				QVector<double> resultsGlobalGaborIca;
				QVector<double> resultsLocalGaborPca;
				QVector<double> resultsLocalGaborIca;
				QVector<double> resultsGlobalLaguerrePca;
				QVector<double> resultsGlobalLaguerreIca;
				QVector<double> resultsLocalLaguerrePca;
				QVector<double> resultsLocalLaguerreIca;
				QVector<double> resultsGlobalVectorizationPca;
				QVector<double> resultsGlobalVectorizationIca;

				QVector<double> resultsScoreFusion;
				QVector<double> fnmr1;
				QVector<double> fnmr2;
				//QVector<double> resultsFeatureFusion;

				int runs = (database == "notre-dame") ? 3 : 10;
				for (int i = 0; i < runs; i++)
				{
					qDebug() << (i+1);
					// Divide data
					QList<QSet<int> > uniqueClassesInClusters = BioDataProcessing::divideToNClusters(allGlobalVectorizationClasses, 3);

					QList<QVector<Matrix> > globalGaborData;
					QList<QVector<int> > globalGaborClasses;
					BioDataProcessing::divideAccordingToUniqueClasses(allGlobalGaborData, allGlobalGaborClasses,
							uniqueClassesInClusters, globalGaborData, globalGaborClasses);

					QList<QVector<Matrix> > localGaborData;
					QList<QVector<int> > localGaborClasses;
					BioDataProcessing::divideAccordingToUniqueClasses(allLocalGaborData, allLocalGaborClasses,
							uniqueClassesInClusters, localGaborData, localGaborClasses);

					QList<QVector<Matrix> > globalLaguerreData;
					QList<QVector<int> > globalLaguerreClasses;
					BioDataProcessing::divideAccordingToUniqueClasses(allGlobalLaguerreData, allGlobalLaguerreClasses,
							uniqueClassesInClusters, globalLaguerreData, globalLaguerreClasses);

					QList<QVector<Matrix> > localLaguerreData;
					QList<QVector<int> > localLaguerreClasses;
					BioDataProcessing::divideAccordingToUniqueClasses(allLocalLaguerreData, allLocalLaguerreClasses,
							uniqueClassesInClusters, localLaguerreData, localLaguerreClasses);

					QList<QVector<Matrix> > globalVectorizationData;
					QList<QVector<int> > globalVectorizationClasses;
					BioDataProcessing::divideAccordingToUniqueClasses(allGlobalVectorizationData, allGlobalVectorizationClasses,
							uniqueClassesInClusters, globalVectorizationData, globalVectorizationClasses);

					// train
					// Global Gabor
					PCA globalGaborPca(globalGaborData[0]);
					globalGaborPca.modesSelectionThreshold(selThresholdPca);
					ZScorePCAExtractor ZPcaExtractorGlobalGabor(globalGaborPca, globalGaborData[1]);
					CosineWeightedMetric cosWGlobalGaborPca;
					Templates trainGlobalGaborTemplatesPca =
							Template::createTemplates(globalGaborData[1], globalGaborClasses[1], ZPcaExtractorGlobalGabor);
					DiscriminativePotential dpGlobalGaborPca(trainGlobalGaborTemplatesPca);
					cosWGlobalGaborPca.w = dpGlobalGaborPca.createWeights();

					ICAofPCA globalGaborIca(globalGaborData[0], selThresholdIca);
					ZScoreICAofPCAExtractor ZIcaExtractorGlobalGabor(globalGaborIca, globalGaborData[1]);
					CosineWeightedMetric cosWGlobalGaborIca;
					Templates trainGlobalGaborTemplatesIca =
							Template::createTemplates(globalGaborData[1], globalGaborClasses[1], ZIcaExtractorGlobalGabor);
					DiscriminativePotential dpGlobalGaborIca(trainGlobalGaborTemplatesIca);
					cosWGlobalGaborIca.w = dpGlobalGaborIca.createWeights();

					// Local Gabor
					PCA localGaborPca(localGaborData[0]);
					localGaborPca.modesSelectionThreshold(selThresholdPca);
					ZScorePCAExtractor ZPcaExtractorLocalGabor(localGaborPca, localGaborData[1]);
					CosineWeightedMetric cosWLocalGaborPca;
					Templates trainLocalGaborTemplatesPca =
							Template::createTemplates(localGaborData[1], localGaborClasses[1], ZPcaExtractorLocalGabor);
					DiscriminativePotential dpLocalGaborPca(trainLocalGaborTemplatesPca);
					cosWLocalGaborPca.w = dpLocalGaborPca.createWeights();

					ICAofPCA localGaborIca(localGaborData[0], selThresholdIca);
					ZScoreICAofPCAExtractor ZIcaExtractorLocalGabor(localGaborIca, localGaborData[1]);
					CosineWeightedMetric cosWLocalGaborIca;
					Templates trainLocalGaborTemplatesIca =
							Template::createTemplates(localGaborData[1], localGaborClasses[1], ZIcaExtractorLocalGabor);
					DiscriminativePotential dpLocalGaborIca(trainLocalGaborTemplatesIca);
					cosWLocalGaborIca.w = dpLocalGaborIca.createWeights();

					// Global Laguerre
					PCA globalLaguerrePca(globalLaguerreData[0]);
					globalLaguerrePca.modesSelectionThreshold(selThresholdPca);
					ZScorePCAExtractor ZPcaExtractorGlobalLaguerre(globalLaguerrePca, globalLaguerreData[1]);
					CosineWeightedMetric cosWGlobalLaguerrePca;
					Templates trainGlobalLaguerreTemplatesPca =
							Template::createTemplates(globalLaguerreData[1], globalLaguerreClasses[1], ZPcaExtractorGlobalLaguerre);
					DiscriminativePotential dpGlobalLaguerrePca(trainGlobalLaguerreTemplatesPca);
					cosWGlobalLaguerrePca.w = dpGlobalLaguerrePca.createWeights();

					ICAofPCA globalLaguerreIca(globalLaguerreData[0], selThresholdIca);
					ZScoreICAofPCAExtractor ZIcaExtractorGlobalLaguerre(globalLaguerreIca, globalLaguerreData[1]);
					CosineWeightedMetric cosWGlobalLaguerreIca;
					Templates trainGlobalLaguerreTemplatesIca =
							Template::createTemplates(globalLaguerreData[1], globalLaguerreClasses[1], ZIcaExtractorGlobalLaguerre);
					DiscriminativePotential dpGlobalLaguerreIca(trainGlobalLaguerreTemplatesIca);
					cosWGlobalLaguerreIca.w = dpGlobalLaguerreIca.createWeights();

					// Local Laguerre
					PCA localLaguerrePca(localLaguerreData[0]);
					localLaguerrePca.modesSelectionThreshold(selThresholdPca);
					ZScorePCAExtractor ZPcaExtractorLocalLaguerre(localLaguerrePca, localLaguerreData[1]);
					CosineWeightedMetric cosWLocalLaguerrePca;
					Templates trainLocalLaguerreTemplatesPca =
							Template::createTemplates(localLaguerreData[1], localLaguerreClasses[1], ZPcaExtractorLocalLaguerre);
					DiscriminativePotential dpLocalLaguerrePca(trainLocalLaguerreTemplatesPca);
					cosWLocalLaguerrePca.w = dpLocalLaguerrePca.createWeights();

					ICAofPCA localLaguerreIca(localLaguerreData[0], selThresholdIca);
					ZScoreICAofPCAExtractor ZIcaExtractorLocalLaguerre(localLaguerreIca, localLaguerreData[1]);
					CosineWeightedMetric cosWLocalLaguerreIca;
					Templates trainLocalLaguerreTemplatesIca =
							Template::createTemplates(localLaguerreData[1], localLaguerreClasses[1], ZIcaExtractorLocalLaguerre);
					DiscriminativePotential dpLocalLaguerreIca(trainLocalLaguerreTemplatesIca);
					cosWLocalLaguerreIca.w = dpLocalLaguerreIca.createWeights();

					// Simple vectorization
					PCA globalVectorizationPca(globalVectorizationData[0]);
					globalVectorizationPca.modesSelectionThreshold(selThresholdPca);
					ZScorePCAExtractor ZPcaExtractorGlobalVectorization(globalVectorizationPca, globalVectorizationData[1]);
					CosineWeightedMetric cosWGlobalVectorizationPca;
					Templates trainGlobalVectorizationTemplatesPca =
							Template::createTemplates(globalVectorizationData[1], globalVectorizationClasses[1], ZPcaExtractorGlobalVectorization);
					DiscriminativePotential dpGlobalVectorizationPca(trainGlobalVectorizationTemplatesPca);
					cosWGlobalVectorizationPca.w = dpGlobalVectorizationPca.createWeights();

					ICAofPCA globalVectorizationIca(globalVectorizationData[0], selThresholdIca);
					ZScoreICAofPCAExtractor ZIcaExtractorGlobalVectorization(globalVectorizationIca, globalVectorizationData[1]);
					CosineWeightedMetric cosWGlobalVectorizationIca;
					Templates trainGlobalVectorizationTemplatesIca =
							Template::createTemplates(globalVectorizationData[1], globalVectorizationClasses[1], ZIcaExtractorGlobalVectorization);
					DiscriminativePotential dpGlobalVectorizationIca(trainGlobalVectorizationTemplatesIca);
					cosWGlobalVectorizationIca.w = dpGlobalVectorizationIca.createWeights();

					// evaluate
					{
						Evaluation evalGlobalGaborPca(globalGaborData[2], globalGaborClasses[2], ZPcaExtractorGlobalGabor, cosWGlobalGaborPca);
						resultsGlobalGaborPca << evalGlobalGaborPca.eer;
						Evaluation evalGlobalGaborIca(globalGaborData[2], globalGaborClasses[2], ZIcaExtractorGlobalGabor, cosWGlobalGaborIca);
						resultsGlobalGaborIca << evalGlobalGaborIca.eer;
					}

					{
						Evaluation evalLocalGaborPca(localGaborData[2], localGaborClasses[2], ZPcaExtractorLocalGabor, cosWLocalGaborPca);
						resultsLocalGaborPca << evalLocalGaborPca.eer;
						Evaluation evalLocalGaborIca(localGaborData[2], localGaborClasses[2], ZIcaExtractorLocalGabor, cosWLocalGaborIca);
						resultsLocalGaborIca << evalLocalGaborIca.eer;
					}

					{
						Evaluation evalGlobalLaguerrePca(globalLaguerreData[2], globalLaguerreClasses[2], ZPcaExtractorGlobalLaguerre, cosWGlobalLaguerrePca);
						resultsGlobalLaguerrePca << evalGlobalLaguerrePca.eer;
						Evaluation evalGlobalLaguerreIca(globalLaguerreData[2], globalLaguerreClasses[2], ZIcaExtractorGlobalLaguerre, cosWGlobalLaguerreIca);
						resultsGlobalLaguerreIca << evalGlobalLaguerreIca.eer;
					}

					{
						Evaluation evalLocalLaguerrePca(localLaguerreData[2], localLaguerreClasses[2], ZPcaExtractorLocalLaguerre, cosWLocalLaguerrePca);
						resultsLocalLaguerrePca << evalLocalLaguerrePca.eer;
						Evaluation evalLocalLaguerreIca(localLaguerreData[2], localLaguerreClasses[2], ZIcaExtractorLocalLaguerre, cosWLocalLaguerreIca);
						resultsLocalLaguerreIca << evalLocalLaguerreIca.eer;
					}

					{
						Evaluation evalGlobalVectorizationPca(globalVectorizationData[2], globalVectorizationClasses[2], ZPcaExtractorGlobalVectorization, cosWGlobalVectorizationPca);
						resultsGlobalVectorizationPca << evalGlobalVectorizationPca.eer;
						Evaluation evalGlobalVectorizationIca(globalVectorizationData[2], globalVectorizationClasses[2], ZIcaExtractorGlobalVectorization, cosWGlobalVectorizationIca);
						resultsGlobalVectorizationIca << evalGlobalVectorizationIca.eer;
					}

					// Fusion
                    //QList<QVector<Matrix> > fusionTrainData;
                    //fusionTrainData << globalGaborData[1] << globalGaborData[1] << localGaborData[1] << localGaborData[1]
                    //				<< globalLaguerreData[1] << globalLaguerreData[1] << localLaguerreData[1] << localLaguerreData[1]
                    //				<< globalVectorizationData[1] << globalVectorizationData[1];

					QList<QVector<Matrix> > fusionTestData;
					fusionTestData << globalGaborData[2] << globalGaborData[2] << localGaborData[2] << localGaborData[2]
					               << globalLaguerreData[2] << globalLaguerreData[2] << localLaguerreData[2] << localLaguerreData[2]
					               << globalVectorizationData[2] << globalVectorizationData[2];

					// feature level fusion
                    //FeatureVectorFusionConcatenation featureFusion;
                    //featureFusion.addComponent(globalGaborData[1], globalGaborClasses[1], ZPcaExtractorGlobalGabor, cos);
                    //featureFusion.addComponent(globalGaborData[1], globalGaborClasses[1], ZIcaExtractorGlobalGabor, cos);
                    //featureFusion.addComponent(globalLaguerreData[1], globalLaguerreClasses[1], ZPcaExtractorGlobalLaguerre, cos);
                    //featureFusion.addComponent(globalLaguerreData[1], globalLaguerreClasses[1], ZIcaExtractorGlobalLaguerre, cos);
                    //featureFusion.addComponent(globalVectorizationData[1], globalVectorizationClasses[1], ZPcaExtractorGlobalVectorization, cos);
                    //featureFusion.addComponent(globalVectorizationData[1], globalVectorizationClasses[1], ZIcaExtractorGlobalVectorization, cos);
                    //featureFusion.learn();

                    //QVector<Matrix> fusedFeaturesTrain = featureFusion.batchFuse(fusionTrainData);
                    //QVector<Matrix> fusedFeaturesTest = featureFusion.batchFuse(fusionTestData);
                    //PassExtractor pass;
                    // //LDAofPCA ldaOfFusedVectors(fusedFeaturesTrain, globalVectorizationClasses[1]);
                    // //ZScoreLDAofPCAExtractor ldaOfFusedVectorsExtractor(ldaOfFusedVectors, fusedFeaturesTrain);
                    //CosineWeightedMetric cosWFeatureFusion;
                    //Templates featureFusionTrainTemplates =
                    //		Template::createTemplates(fusedFeaturesTrain, globalVectorizationClasses[1], pass);
                    //DiscriminativePotential dpFeatureFusion(featureFusionTrainTemplates);
                    //cosWFeatureFusion.w = dpFeatureFusion.createWeights();

                    //Evaluation featureFusionEval(fusedFeaturesTest, globalVectorizationClasses[2], pass, cosWFeatureFusion);
                    //resultsFeatureFusion << featureFusionEval.eer;

					// score level fusion
					{
						ScoreLogisticRegressionFusion scoreFusion;
						scoreFusion.addComponent(globalGaborData[1], globalGaborClasses[1], ZPcaExtractorGlobalGabor, cosWGlobalGaborPca);
						scoreFusion.addComponent(globalGaborData[1], globalGaborClasses[1], ZIcaExtractorGlobalGabor, cosWGlobalGaborIca);
						scoreFusion.addComponent(localGaborData[1], localGaborClasses[1], ZPcaExtractorLocalGabor, cosWLocalGaborPca);
						scoreFusion.addComponent(localGaborData[1], localGaborClasses[1], ZIcaExtractorLocalGabor, cosWLocalGaborIca);
						scoreFusion.addComponent(globalLaguerreData[1], globalLaguerreClasses[1], ZPcaExtractorGlobalLaguerre, cosWGlobalLaguerrePca);
						scoreFusion.addComponent(globalLaguerreData[1], globalLaguerreClasses[1], ZIcaExtractorGlobalLaguerre, cosWGlobalLaguerreIca);
						scoreFusion.addComponent(localLaguerreData[1], localLaguerreClasses[1], ZPcaExtractorLocalLaguerre, cosWLocalLaguerrePca);
						scoreFusion.addComponent(localLaguerreData[1], localLaguerreClasses[1], ZIcaExtractorLocalLaguerre, cosWLocalLaguerreIca);
						scoreFusion.addComponent(globalVectorizationData[1], globalVectorizationClasses[1], ZPcaExtractorGlobalVectorization, cosWGlobalVectorizationPca);
						scoreFusion.addComponent(globalVectorizationData[1], globalVectorizationClasses[1], ZIcaExtractorGlobalVectorization, cosWGlobalVectorizationIca);
						scoreFusion.learn();

						Evaluation scoreFusionEval = scoreFusion.evaluate(fusionTestData, globalVectorizationClasses[2]);
						resultsScoreFusion << scoreFusionEval.eer;
						qDebug("   fusion EER: %.2f", (scoreFusionEval.eer*100.0));
						fnmr1 << scoreFusionEval.fnmrAtFmr(0.001);
						fnmr2 << scoreFusionEval.fnmrAtFmr(0.01);

						QString farFrr(database + "-" + warpMode + "-run"+ QString::number(i) + "-far-frr");
						Common::savePlot(scoreFusionEval.fmr, scoreFusionEval.fnmr, farFrr);
					}

					qDebug("  global gabor pca         %.2f", (resultsGlobalGaborPca[i]*100.0));
					qDebug("  global gabor ica         %.2f", (resultsGlobalGaborIca[i]*100.0));
					qDebug("  local gabor pca          %.2f", (resultsLocalGaborPca[i]*100.0));
					qDebug("  local gabor ica          %.2f", (resultsLocalGaborIca[i]*100.0));
					qDebug("  global laguerre pca      %.2f", (resultsGlobalLaguerrePca[i]*100.0));
					qDebug("  global laguerre ica      %.2f", (resultsGlobalLaguerreIca[i]*100.0));
					qDebug("  local laguerre pca       %.2f", (resultsLocalLaguerrePca[i]*100.0));
					qDebug("  local laguerre ica       %.2f", (resultsLocalLaguerreIca[i]*100.0));
					qDebug("  global vectorization pca %.2f", (resultsGlobalVectorizationPca[i]*100.0));
					qDebug("  global vectorization ica %.2f", (resultsGlobalVectorizationIca[i]*100.0));

					//qDebug() << "feature level fusion" << Vector::meanValue(resultsFeatureFusion);
					qDebug();
					qDebug("  score level fusion       %.2f", (resultsScoreFusion[i]*100.0));
					qDebug("  FNMR at FAR = 0.1pct     %.2f", (fnmr1[i]*100.0));
					qDebug("  FNMR at FAR = 1pct       %.2f", (fnmr2[i]*100.0));
				}

				qDebug() << "means:";
				qDebug("  global gabor pca         %.2f", (Vector::meanValue(resultsGlobalGaborPca)*100.0));
				qDebug("  global gabor ica         %.2f", (Vector::meanValue(resultsGlobalGaborIca)*100.0));
				qDebug("  local gabor pca          %.2f", (Vector::meanValue(resultsLocalGaborPca)*100.0));
				qDebug("  local gabor ica          %.2f", (Vector::meanValue(resultsLocalGaborIca)*100.0));
				qDebug("  global laguerre pca      %.2f", (Vector::meanValue(resultsGlobalLaguerrePca)*100.0));
				qDebug("  global laguerre ica      %.2f", (Vector::meanValue(resultsGlobalLaguerreIca)*100.0));
				qDebug("  local laguerre pca       %.2f", (Vector::meanValue(resultsLocalLaguerrePca)*100.0));
				qDebug("  local laguerre ica       %.2f", (Vector::meanValue(resultsLocalLaguerreIca)*100.0));
				qDebug("  global vectorization pca %.2f", (Vector::meanValue(resultsGlobalVectorizationPca)*100.0));
				qDebug("  global vectorization ica %.2f", (Vector::meanValue(resultsGlobalVectorizationIca)*100.0));

				//qDebug() << "feature level fusion" << Vector::meanValue(resultsFeatureFusion);
				qDebug();
				qDebug("  score level fusion       %.2f", (Vector::meanValue(resultsScoreFusion)*100.0));
				qDebug("  FNMR at FAR = 0.1pct     %.2f", (Vector::meanValue(fnmr1)*100.0));
				qDebug("  FNMR at FAR = 1pct       %.2f", (Vector::meanValue(fnmr2)*100.0));
			}
		}
	}

	static void gaborBankFeatureSelection()
	{
		QMap<QString, QString> paths;
		paths["2d-global-gabor"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Global/GaborBank";
		paths["2d-local-gabor"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Local/GaborBank";
		paths["2d-global-laguerre"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Global/LaguerreBank";
		paths["2d-local-laguerre"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Local/LaguerreBank";

		QList<QString> keys = paths.keys();
		foreach(QString testName, keys)
		{
			QVector<Matrix> allImages;
			QVector<int> allClasses;
			Loader::loadVectors(paths[testName], allImages, allClasses, "-", "*.vec");

			CosineWeightedMetric cosW;

			// Divide data
			QList<QVector<Matrix> > images;
			QList<QVector<int> > classes;
			BioDataProcessing::divideToNClusters(allImages, allClasses, 5, images, classes);

			// train
			ICAofPCA icaOfPca(images[0]);
			ZScoreICAofPCAExtractor icaExtractor(icaOfPca, images[1]);
			Templates trainTemplates = Template::createTemplates(images[1], classes[1], icaExtractor);
			DiscriminativePotential dp(trainTemplates);

			// evaluation
			cosW.w = dp.createWeights();
			BatchEvaluationResult eval = Evaluation::batch(images, classes, icaExtractor, cosW, 2);

			qDebug() << "mean EER:" << eval.meanEER << "stdDev:" << eval.stdDevOfEER;
		}
	}

	static void evaluateFilterBanks_old()
	{
		QMap<QString, QString> paths;
		paths["2d-global-gabor"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Global/GaborBank";
		paths["2d-local-gabor"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Local/GaborBank";
		paths["2d-global-laguerre"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Global/LaguerreBank";
		paths["2d-local-laguerre"] = "/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Local/LaguerreBank";

		QList<QString> keys = paths.keys();
		foreach(QString testName, keys)
		{
			qDebug() << testName;
			QVector<Matrix> allImages;
			QVector<int> allClasses;
			Loader::loadVectors(paths[testName], allImages, allClasses, "-", "*.vec");

			CosineMetric cos;
			CorrelationMetric corr;

			// Divide data
			QList<QVector<Matrix> > images;
			QList<QVector<int> > classes;
			BioDataProcessing::divideToNClusters(allImages, allClasses, 5, images, classes);

			// Train
			PCA pca(images[0]);
			pca.modesSelectionThreshold();
			PCAExtractor pcaExtractor(pca);

			LDAofPCA ldaOfPca(images[0], classes[0]);
			LDAofPCAExtractor ldaExtractor(ldaOfPca);

			ICAofPCA icaOfPca(images[0]);
			ICAofPCAExtractor icaExtractor(icaOfPca);

			PassExtractor passExtractor;

			// evaluate
			double eerPca = Evaluation::batch(images, classes, pcaExtractor, cos, 2).meanEER;
			double eerLda = Evaluation::batch(images, classes, ldaExtractor, cos, 2).meanEER;
			double eerIca = Evaluation::batch(images, classes, icaExtractor, cos, 2).meanEER;
			double eerPass = Evaluation::batch(images, classes, passExtractor, corr, 2).meanEER;

			qDebug() << testName << eerPca << eerLda << eerIca << eerPass;
		}
	}

	static void evaluateFeatureSelection()
	{
		QVector<Matrix> allImages;
		QVector<int> allClasses;
		//Loader::loadVectors("/run/media/stepo/frgc/thermo/germany/Warp2D/Global/Vectorization", allImages, allClasses, "-", "*.vec");
		Loader::loadVectors("/run/media/stepo/frgc/thermo/notre-dame/Warp2D/Global/Vectorization", allImages, allClasses, "-", "*.vec");

		CosineMetric m;
		CosineWeightedMetric w;
		//CorrelationMetric m;
		//CorrelationWeightedMetric w;

		QVector<double> runs;
		QVector<double> rawEERs;
		QVector<double> dpWeightEERs;
		QVector<double> dpSelectionEERs;
		//QVector<double> epWeightEERs;
		//QVector<double> epSelectionEERs;

		int runsCount = 10;
		for (int run = 1; run <= runsCount; run++)
		{
			qDebug() << run;
			runs << run;

			// Divide data
			QList<QVector<Matrix> > images;
			QList<QVector<int> > classes;
			BioDataProcessing::divideToNClusters(allImages, allClasses, 3, images, classes);

			// Train
			ICAofPCA icaOfPca(images[0]);
			ZScoreICAofPCAExtractor extractor(icaOfPca, images[1]);
			//ZScorePassExtractor extractor(images[1]);
			Templates trainTemplates = Template::createTemplates(images[1], classes[1], extractor);
			Templates testTemplates = Template::createTemplates(images[2], classes[2], extractor);

			// baseline
			Evaluation rawEvaluationTrain(trainTemplates, m);
			Evaluation rawEvaluationTest(testTemplates, m);
			qDebug() << "raw" << rawEvaluationTrain.eer << rawEvaluationTest.eer;
			rawEERs << rawEvaluationTest.eer;

			// DiscriminativePotential
			DiscriminativePotential dp(trainTemplates);
			Matrix dpWeights = dp.createWeights();
			Matrix dpSelectionWeights = dp.createSelectionWeights((dp.maxScore+dp.minScore)/2.0);

			w.w = dpWeights;
			Evaluation dpWeightEvaluationTrain(trainTemplates, w);
			Evaluation dpWeightEvaluationTest(testTemplates, w);
			qDebug() << "dp weights" << dpWeightEvaluationTrain.eer << dpWeightEvaluationTest.eer;
			dpWeightEERs << dpWeightEvaluationTest.eer;

			w.w = dpSelectionWeights;
			Evaluation dpSelectionEvaluationTrain(trainTemplates, w);
			Evaluation dpSelectionEvaluationTest(testTemplates, w);
			qDebug() << "dp selection" << dpSelectionEvaluationTrain.eer << dpSelectionEvaluationTest.eer;
			dpSelectionEERs << dpSelectionEvaluationTest.eer;

			// EERPotential
            //EERPotential ep(trainTemplates);
            //Matrix epWeights = ep.createWeights();
            //Matrix epSelectionWeights = ep.createSelectionWeights((ep.maxScore+ep.minScore)/2.0);

            //w.w = epWeights;
            //Evaluation epWeightEvaluationTrain(trainTemplates, w);
            //Evaluation epWeightEvaluationTest(testTemplates, w);
            //qDebug() << "ep weights" << epWeightEvaluationTrain.eer << epWeightEvaluationTest.eer;
            //epWeightEERs << epWeightEvaluationTest.eer;

            //w.w = epSelectionWeights;
            //Evaluation epSelectionEvaluationTrain(trainTemplates, w);
            //Evaluation epSelectionEvaluationTest(testTemplates, w);
            //qDebug() << "ep selection" << epSelectionEvaluationTrain.eer << epSelectionEvaluationTest.eer;
            //epSelectionEERs << epSelectionEvaluationTest.eer;

            //if (run == 1)
            //{
            //	Matrix dpMask = MatrixConverter::columnVectorToMatrix(dpWeights, 60);
            //	cv::imshow("dp mask", dpMask);
            //	cv::imwrite("dpMask.png", dpMask*255);

            //	Matrix epMask = MatrixConverter::columnVectorToMatrix(epWeights, 60);
            //	cv::imshow("ep mask", epMask);
            //	cv::imwrite("epMask.png", epMask*255);
            //	//cv::waitKey(0);
            //	//return;
            //}
		}

		Common::savePlot(runs, rawEERs, "rawEERs");
		Common::savePlot(runs, dpWeightEERs, "dpWeightEERs");
		Common::savePlot(runs, dpSelectionEERs, "dpSelectionEERs");
		//Common::savePlot(runs, epWeightEERs, "epWeightEERs");
		//Common::savePlot(runs, epSelectionEERs, "epSelectionEERs");

		qDebug() << "raw" << Vector::meanValue(rawEERs);
		qDebug() << "dpWeightEERs" << Vector::meanValue(dpWeightEERs);
		qDebug() << "dpSelectionEERs" << Vector::meanValue(dpSelectionEERs);
		//qDebug() << "epWeightEERs" << Vector::meanValue(epWeightEERs);
		//qDebug() << "epSelectionEERs" << Vector::meanValue(epSelectionEERs);
	}
};
*/

#endif /* EVALUATIONTHERMO2_H_ */
