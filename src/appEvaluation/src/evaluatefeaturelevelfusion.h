/*
 * evaluateFVfusion.h
 *
 *  Created on: 9.5.2012
 *      Author: stepo
 */

#ifndef EVALUATEFVFUSION_H_
#define EVALUATEFVFUSION_H_

#include <QVector>
#include <QList>
#include <QDebug>

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/metrics.h"
#include "faceCommon/linalg/loader.h"
#include "faceCommon/linalg/pca.h"
#include "faceCommon/linalg/ldaofpca.h"
#include "faceCommon/linalg/icaofpca.h"
#include "faceCommon/biometrics/biodataprocessing.h"
#include "faceCommon/biometrics/featureextractor.h"
#include "faceCommon/biometrics/evaluation.h"
//#include "biometrics/featurevectorfusion.h"
#include "faceCommon/biometrics/scorelevefusion.h"

class EvaluateFeatureLevelFusion
{
public:
    /*static void evaluateBestSelectionThreshold()
	{
		std::vector<Matrix> allImages;
		std::vector<int> allClasses;
		Loader::loadImages("/run/media/stepo/frgc/thermo/germany", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/fit", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/nd", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/equinox", allImages, &allClasses, "*.png", true, "-");

		CityblockMetric cityBlock;
		EuclideanMetric eucl;
		CosineMetric cos;
		CorrelationMetric corr;

		QMap<double, std::vector<double> > thresholdToEERs;
		for (int i = 0; i < 5; i++)
		{
			PCA pca;
			double bestEer = 1.0;
			double bestThreshold = 0.0;
			std::vector<double> thresholds;
			std::vector<double> eers;
			for (double threshold = 0.8999; threshold < 1; threshold += 0.001)
			{
				thresholds << threshold;
				std::vector<double> pcaEer;
				for (int run = 0; run < 5; run++)
				{
					// Divide data
                    QList<std::vector<Vector> > images;
					QList<std::vector<int> > classes;
					BioDataProcessing::divideToNClusters(allImages, allClasses, 3, images, classes);

					PCA pca(images[0]);
					pca.modesSelectionThreshold(threshold);
					ZScorePCAExtractor extractor(pca, images[1]);

					Evaluation evaluation(images[2], classes[2], extractor, corr);
					pcaEer << evaluation.eer;
				}
				double meanEer = Vector::meanValue(pcaEer);
				eers << meanEer;
				if (meanEer < bestEer)
				{
					bestEer = meanEer;
					bestThreshold = threshold;
				}
				qDebug() << i << "t:" << threshold << "best t:" << bestThreshold << "eer:" << meanEer << "best eer:" << bestEer;

				if (!thresholdToEERs.contains(threshold))
				{
					thresholdToEERs[threshold] = std::vector<double>();
				}
				thresholdToEERs[threshold].append(meanEer);
			}
		}

		QList<double> keys = thresholdToEERs.keys();
		std::vector<double> keysVec = std::vector<double>::fromList(keys);
		std::vector<double> means;
		std::vector<double> stdDevs;
		for (int i = 0; i < keys.count(); i++)
		{
			means << Vector::meanValue(thresholdToEERs[keys[i]]);
			stdDevs << Vector::stdDeviation(thresholdToEERs[keys[i]]);
		}
		Common::savePlot(keysVec, means, stdDevs, "zPcaCorr");
    }*/
};


#endif /* EVALUATEFVFUSION_H_ */
/*static void evaluate()
	{
		std::vector<Matrix> allImages;
		std::vector<int> allClasses;
		Loader::loadImages("/run/media/stepo/frgc/thermo/germany", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/fit", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/nd", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/equinox", allImages, &allClasses, "*.png", true, "-");

		CityblockMetric cityBlock;
		EuclideanMetric eucl;
		CosineMetric cos;
		CorrelationMetric corr;

		int runsCount = 20;
		for (int run = 0; run < runsCount; run++)
		{
			// Divide data
			QList<std::vector<Matrix> > images;
			QList<std::vector<int> > classes;
			BioDataProcessing::divideToNClusters(allImages, allClasses, 3, images, classes);

			// train feature extractors
			PCA pca(images[0]);
			pca.modesSelectionThreshold();
			ZScorePCAExtractor zPcaExtractor(pca, images[1]);
			LDAofPCA ldaOfPca(images[0], classes[0]);
			ZScoreLDAofPCAExtractor zLdaOfPcaExtractor(ldaOfPca, images[1]);
			ICAofPCA icaOfPca(images[0]);
			ZScoreICAofPCAExtractor zIcaOfPcaExtractor(icaOfPca, images[1]);

			FeatureVectorFusionLDA featureVectorFusionLda;
			featureVectorFusionLda.addExtractor(zPcaExtractor)
					.addExtractor(zLdaOfPcaExtractor)
					.addExtractor(zIcaOfPcaExtractor);
			featureVectorFusionLda.learn(images[1], classes[1]);

			FeatureVectorFusionConcatenation featureVectorFusionConcatenation;
			featureVectorFusionConcatenation.addExtractor(zPcaExtractor)
					.addExtractor(zLdaOfPcaExtractor)
					.addExtractor(zIcaOfPcaExtractor);

			FeatureVectorFusionPCA featureVectorFusionPca;
			featureVectorFusionPca.addExtractor(zPcaExtractor)
					.addExtractor(zLdaOfPcaExtractor)
					.addExtractor(zIcaOfPcaExtractor);
			featureVectorFusionPca.learn(images[1]);

			FeatureVectorFusionICA featureVectorFusionIca;
			featureVectorFusionIca.addExtractor(zPcaExtractor)
					.addExtractor(zLdaOfPcaExtractor)
					.addExtractor(zIcaOfPcaExtractor);
			featureVectorFusionIca.learn(images[1]);

			//std::vector<FeatureExtractor*> fusionExtractors;
			//fusionExtractors << (&featureVectorFusionLda) << (&featureVectorFusionLda) << (&featureVectorFusionLda);
			//std::vector<Metrics*> fusionMetrics;
			//fusionMetrics << (&eucl) << (&cos) << (&corr);

			//Evaluation evalFeatureVectorFusionLdaEuclTrain(images[1], classes[1], featureVectorFusionLda, eucl);
			//Evaluation evalFeatureVectorFusionLdaCosTrain(images[1], classes[1], featureVectorFusionLda, cos);
			//Evaluation evalFeatureVectorFusionLdaCorrTrain(images[1], classes[1], featureVectorFusionLda, corr);
			//QList<Evaluation> fvFusionLdaTrainResults;
			//fvFusionLdaTrainResults << evalFeatureVectorFusionLdaEuclTrain
			//		<< evalFeatureVectorFusionLdaCosTrain
			//		<< evalFeatureVectorFusionLdaCorrTrain;
			//ScoreProductFusion productFusion(fvFusionLdaTrainResults);

			Evaluation evalZPcaCos(images[2], classes[2], zPcaExtractor, cos);
			Evaluation evalZLdaOfPcaCos(images[2], classes[2], zLdaOfPcaExtractor, cos);
			Evaluation evalZIcaOfPcaCos(images[2], classes[2], zIcaOfPcaExtractor, cos);

			Evaluation evalFeatureVectorFusionLdaEucl(images[2], classes[2], featureVectorFusionLda, eucl);
			Evaluation evalFeatureVectorFusionLdaCos(images[2], classes[2], featureVectorFusionLda, cos);
			Evaluation evalFeatureVectorFusionLdaCorr(images[2], classes[2], featureVectorFusionLda, corr);

			Evaluation evalFeatureVectorFusionConcatenationEucl(images[2], classes[2], featureVectorFusionConcatenation, eucl);
			Evaluation evalFeatureVectorFusionConcatenationCos(images[2], classes[2], featureVectorFusionConcatenation, cos);
			Evaluation evalFeatureVectorFusionConcatenationCorr(images[2], classes[2], featureVectorFusionConcatenation, corr);

			Evaluation evalFeatureVectorFusionPcaEucl(images[2], classes[2], featureVectorFusionPca, eucl);
			Evaluation evalFeatureVectorFusionPcaCos(images[2], classes[2], featureVectorFusionPca, cos);
			Evaluation evalFeatureVectorFusionPcaCorr(images[2], classes[2], featureVectorFusionPca, corr);

			Evaluation evalFeatureVectorFusionIcaEucl(images[2], classes[2], featureVectorFusionIca, eucl);
			Evaluation evalFeatureVectorFusionIcaCos(images[2], classes[2], featureVectorFusionIca, cos);
			Evaluation evalFeatureVectorFusionIcaCorr(images[2], classes[2], featureVectorFusionIca, corr);

			//Evaluation fusion = ScoreLevelFusionSystem::evaluate(images[2], classes[2], productFusion, fusionExtractors, fusionMetrics);

			qDebug() << (run+1) << "/" << runsCount;
			qDebug() << "individual"
					<< evalZPcaCos.eer
					<< evalZLdaOfPcaCos.eer
					<< evalZIcaOfPcaCos.eer;
			qDebug() << "lda FV fusion"
					<< evalFeatureVectorFusionLdaEucl.eer
					<< evalFeatureVectorFusionLdaCos.eer
					<< evalFeatureVectorFusionLdaCorr.eer;
			qDebug() << "concat FV fusion"
					<< evalFeatureVectorFusionConcatenationEucl.eer
					<< evalFeatureVectorFusionConcatenationCos.eer
					<< evalFeatureVectorFusionConcatenationCorr.eer;
			qDebug() << "pca FV fusion"
					<< evalFeatureVectorFusionPcaEucl.eer
					<< evalFeatureVectorFusionPcaCos.eer
					<< evalFeatureVectorFusionPcaCorr.eer;
			qDebug() << "ica FV fusion"
					<< evalFeatureVectorFusionIcaEucl.eer
					<< evalFeatureVectorFusionIcaCos.eer
					<< evalFeatureVectorFusionIcaCorr.eer;
			//qDebug() << "scorelevel fusion of fusions" << fusion.eer;
 		}
	}
*/
