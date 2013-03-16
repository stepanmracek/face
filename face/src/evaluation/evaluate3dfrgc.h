#ifndef EVALUATE3DFRGC_H_
#define EVALUATE3DFRGC_H_

#include <qvector.h>
#include <qstring.h>
#include <qdebug.h>
#include <cassert>

#include "linalg/common.h"
#include "linalg/loader.h"
#include "linalg/vector.h"
#include "linalg/pca.h"
#include "linalg/matrixconverter.h"
#include "biometrics/biodataprocessing.h"
#include "biometrics/featureextractor.h"
#include "biometrics/discriminativepotential.h"
#include "biometrics/scorelevefusion.h"

class Evaluate3dFrgc
{
public:
	static void compare(QVector<int> &first, QVector<int> &second)
	{
		int n = first.count();
		assert(n == second.count());
		for (int i = 0; i < n; i++)
			assert(first[i] == second[i]);
	}

	static void evaluateFusion()
	{
		QString commonPath = "/run/media/stepo/frgc/frgc-norm-iterative/";
		QString shapeIndexPath = commonPath + "shapeindex-roi2";
		QString anatomicalPath = commonPath + "anatomical";
		QString histogramPath = commonPath + "histogramRectRangeSmooth-10-6";

        QVector<Vector> allShapeImages;
		QVector<int> allClasses;
        Loader::loadImages(shapeIndexPath, allShapeImages, &allClasses, "*.png", "d");

        QVector<Vector> allAnatomicalVectors;
		QVector<int> allAnatomicalClasses;
		Loader::loadVectors(anatomicalPath, allAnatomicalVectors, allAnatomicalClasses, "-", "*-*");

        QVector<Vector> allHistogramVectors;
		QVector<int> allHistigramClasses;
		Loader::loadVectors(histogramPath, allHistogramVectors, allHistigramClasses, "-", "*-*");
		qDebug() << "loading done";

		double selThreshold = 0.9999;
		CorrelationMetric corr;

		// assertions
		compare(allClasses, allAnatomicalClasses);
		compare(allClasses, allHistigramClasses);

		for (int crossRun = 1; crossRun <= 4; crossRun++)
		{
			// divide
			qDebug() << "cross-validation run" << crossRun;
			QList<QSet<int> > uniqueClassesInClusters = BioDataProcessing::divideToNClusters(allClasses, 3);

            QList<QVector<Vector> > shapeData;
			QList<QVector<int> > shapeClasses;
			BioDataProcessing::divideAccordingToUniqueClasses(allShapeImages, allClasses,
					uniqueClassesInClusters, shapeData, shapeClasses);

			/*QList<QVector<Matrix> > anatomicalData;
			QList<QVector<int> > anatomicalClasses;
			BioDataProcessing::divideAccordingToUniqueClasses(allAnatomicalVectors, allAnatomicalClasses,
					uniqueClassesInClusters, anatomicalData, anatomicalClasses);

			QList<QVector<Matrix> > histogramData;
			QList<QVector<int> > histogramClasses;
			BioDataProcessing::divideAccordingToUniqueClasses(allHistogramVectors, allHistigramClasses,
					uniqueClassesInClusters, histogramData, histogramClasses);*/

			// train
			qDebug() << "  shape pca";
			PCA pca(shapeData[0]); pca.modesSelectionThreshold(selThreshold);
			//PCAExtractor pcaExtractor(pca);
			ZScorePCAExtractor zPcaExtractor(pca, shapeData[1]);
			QVector<Template> trainZPcaTemplates = Template::createTemplates(shapeData[1], shapeClasses[1], zPcaExtractor);
			DiscriminativePotential dpPca(trainZPcaTemplates);
			CosineWeightedMetric cosWPca;
			cosWPca.w = dpPca.createWeights();

			qDebug() << "  shape ica";
  			ICAofPCA ica(shapeData[0], selThreshold);
  			//ICAofPCAExtractor icaExtractor(ica);
  			ZScoreICAofPCAExtractor zIcaExtractor(ica, shapeData[1]);
  			QVector<Template> trainZIcaTemplates = Template::createTemplates(shapeData[1], shapeClasses[1], zIcaExtractor);
  			DiscriminativePotential dpIca(trainZIcaTemplates);
  			CosineWeightedMetric cosWIca;
  			cosWIca.w = dpIca.createWeights();

  			/*qDebug() << "  anatomical";
  			ZScorePassExtractor anatomicalExtractor(anatomicalData[1]);
  			//LDA anatomicalLda(anatomicalData[0], anatomicalClasses[0]);
  			//ZScoreLDAExtractor anatomicalExtractor(anatomicalLda, anatomicalData[1]);
  			QVector<Template> trainAnatomicalTemplates = Template::createTemplates(anatomicalData[1], anatomicalClasses[1], anatomicalExtractor);
  			DiscriminativePotential dpAnatomical(trainAnatomicalTemplates);
  			Matrix anatomicalWeights = dpAnatomical.createWeights();
  			anatomicalWeights = anatomicalWeights.mul(dpAnatomical.createSelectionWeights(0.6));
  			CityblockWeightedMetric cityWAnatomical; cityWAnatomical.w = anatomicalWeights;

  			qDebug() << "  histogram";
  			ZScorePassExtractor histogramExtractor(histogramData[1]);
  			QVector<Template> trainHistogramTemplates = Template::createTemplates(histogramData[1], histogramClasses[1], histogramExtractor);
  			DiscriminativePotential dpHistogram(trainHistogramTemplates);
  			Matrix histogramWeights = dpHistogram.createWeights();
  			//CityblockWeightedMetric cityWHistogram; cityWHistogram.w = histogramWeights;
  			//EuclideanWeightedMetric euclWHistogram; euclWHistogram.w = histogramWeights;
  			CosineWeightedMetric cosWHistogram; cosWHistogram.w = histogramWeights;
  			//CorrelationWeightedMetric corrWHistogram; corrWHistogram.w = histogramWeights;

  			qDebug() << "  direct";
  			ZScorePassExtractor zPassExtractor(shapeData[1]);*/

  			// evaluation
  			qDebug() << " evaluation";

  			// shape-pca
  			QVector<Template> zPcaTemplates = Template::createTemplates(shapeData[2], shapeClasses[2], zPcaExtractor);
  			Evaluation evalWPcaCos(zPcaTemplates, cosWPca);
  			qDebug("  wPca-cos      %.2f", (evalWPcaCos.eer*100.0));

  			// shape-ica
  			QVector<Template> zIcaTemplates = Template::createTemplates(shapeData[2], shapeClasses[2], zIcaExtractor);
  			Evaluation evalWIcaCos(zIcaTemplates, cosWIca);
  			qDebug("  wIca-cos      %.2f", (evalWIcaCos.eer*100.0));

  			/*// shape-pass
  			QVector<Template> zPassTemplates = Template::createTemplates(shapeData[2], shapeClasses[2], zPassExtractor);
  			Evaluation evalZPass(zPassTemplates, corr);
  			qDebug("  zPass-corr    %.2f", (evalZPass.eer*100.0));

  			// anatomical
  			QVector<Template> anatomicalTemplates = Template::createTemplates(anatomicalData[2], anatomicalClasses[2], anatomicalExtractor);
  			Evaluation evalAnatomicalCity(anatomicalTemplates, cityWAnatomical);
  			qDebug("  anatomical-city   %.2f", (evalAnatomicalCity.eer*100.0));

  			// histogram
  			QVector<Template> histogramTemplates = Template::createTemplates(histogramData[2], histogramClasses[2], histogramExtractor);
  			//Evaluation evalHistogramCity(histogramTemplates, cityWHistogram);
  			//qDebug("  histogram-city   %.2f", (evalHistogramCity.eer*100.0));
  			//Evaluation evalHistogramEucl(histogramTemplates, euclWHistogram);
  			//qDebug("  histogram-eucl   %.2f", (evalHistogramEucl.eer*100.0));
  			//Evaluation evalHistogramCorr(histogramTemplates, corrWHistogram);
  			//qDebug("  histogram-corr   %.2f", (evalHistogramCorr.eer*100.0));
  			Evaluation evalHistogramCos(histogramTemplates, cosWHistogram);
  			qDebug("  histogram-cos    %.2f", (evalHistogramCos.eer*100.0));*/

  			// fusion
  			qDebug() << " fusion - learning";
  			ScoreLogisticRegressionFusion fusion;
  			fusion.addComponent(shapeData[1], shapeClasses[1], zPcaExtractor, cosWPca);
  			fusion.addComponent(shapeData[1], shapeClasses[1], zIcaExtractor, cosWIca);
  			//fusion.addComponent(anatomicalData[1], anatomicalClasses[1], anatomicalExtractor, cityWAnatomical);
  			//fusion.addComponent(histogramData[1], histogramClasses[1], histogramExtractor, cosWHistogram);
  			//fusion.addComponent(shapeData[1], shapeClasses[1], zPassExtractor, corr);
  			fusion.learn();

  			qDebug() << " fusion - preparing data";
  			//QList<QVector<int> > fusionEvalClasses;
  			//fusionEvalClasses << shapeClasses[2] << shapeClasses[2] << anatomicalClasses[2] << histogramClasses[2] << shapeClasses[2];
            QList<QVector<Vector> > fusionEvalData;
  			fusionEvalData << shapeData[2] << shapeData[2]; // << anatomicalData[2] << histogramData[2] << shapeData[2];

  			qDebug() << " fusion - evaluating";
  			Evaluation evalFusion = fusion.evaluate(fusionEvalData, shapeClasses[2]);
  			qDebug("  fusion    %.2f", (evalFusion.eer*100.0));
		}
	}

	static void pcaSubspace()
	{
		QString path = "/run/media/stepo/frgc/frgc-norm-iterative/tmp";
        QVector<Vector> images;
        Loader::loadImages(path, images, 0, "*.png", "d", true);

		PCA pca(images, 5, true);

        Vector tmp = pca.getMean();
        Matrix mean = MatrixConverter::columnVectorToMatrix(tmp, 200)*255;
		cv::imwrite("mean.png", mean);
		for (int i = 0; i < 5; i++)
		{
			QString fName = "eig" + QString::number(i+1)+".png";
            Matrix m = pca.cvPca.eigenvectors.row(i).t();
            Vector eig = m;
            eig = eig.normalizeComponents();
            Matrix img = MatrixConverter::columnVectorToMatrix(eig, 200)*255;
            cv::imwrite(fName.toStdString(), img);
		}

		Matrix projected = pca.project(images[0]);
		Common::printMatrix(projected);

        Vector backProjected = pca.backProject(projected);
        Matrix backProjectedImg = MatrixConverter::columnVectorToMatrix(backProjected, 200)*255;
        cv::imwrite("backprojected.png", backProjectedImg);
	}
};

#endif
