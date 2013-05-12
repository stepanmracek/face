#ifndef EVALUATETHERMO_H
#define EVALUATETHERMO_H

#include <QVector>
#include <QList>
#include <QDebug>
#include <QString>

#include <cassert>

#include "linalg/loader.h"
#include "linalg/vector.h"
#include "linalg/common.h"
#include "biometrics/biodataprocessing.h"
#include "linalg/pca.h"
#include "biometrics/evaluation.h"
#include "linalg/metrics.h"
#include "linalg/icaofpca.h"
#include "linalg/ldaofpca.h"
#include "biometrics/featureextractor.h"
#include "biometrics/scorelevefusion.h".h"
#include "biometrics/featureselection.h"
#include "biometrics/eerpotential.h"

class EvaluateThermo
{
public:
    static void evaluateDifferentMetrics()
    {
        QVector<Vector> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany",
                           allImages, &allClasses, "*.png", "-", false);

        // Divide data
        QList<QVector<Vector> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 10, images, classes);

        // all permutations of three clusters
        QVector<QVector<int> > testRuns;
        QVector<int> test0; test0 << 0 << 1 << 2; testRuns << test0;
        QVector<int> test1; test1 << 0 << 2 << 1; testRuns << test1;
        QVector<int> test2; test2 << 1 << 0 << 2; testRuns << test2;
        QVector<int> test3; test3 << 1 << 2 << 0; testRuns << test3;
        QVector<int> test4; test4 << 2 << 0 << 1; testRuns << test4;
        QVector<int> test5; test5 << 2 << 1 << 0; testRuns << test5;

        double pcaSelThreshold = 0.9999;
        EuclideanMetric m;

        QVector<double> direct;
        QVector<double> zScoreDirect;
        QVector<double> pcaR;
        QVector<double> eigenNormPca;
        QVector<double> zScorePca;
        QVector<double> whitenedPca;
        QVector<double> zScoreWhitenedPca;
        QVector<double> lda;
        QVector<double> zScoreLda;
        QVector<double> ica;
        QVector<double> zScoreIca;
        for (int testIndex = 0; testIndex < testRuns.count(); testIndex++)
        {
            QVector<int> curTest = testRuns[testIndex];
            qDebug() << "Testrun:" << (testIndex+1);

            PassExtractor passExtractor;
            ZScorePassExtractor zscorePassExtractor(images[curTest[1]]);

            PCA pca(images[curTest[0]]);
            pca.modesSelectionThreshold(pcaSelThreshold);
            PCAExtractor pcaExtractor(pca);
            NormPCAExtractor normPCAExtractor(pca);
            ZScorePCAExtractor zscorePCAExtractor(pca, images[curTest[1]]);

            ICAofPCA icaOfPca(images[curTest[0]], pcaSelThreshold);
            ICAofPCAExtractor icaOfPcaExtractor(icaOfPca);
            ZScoreICAofPCAExtractor zscoreIcaOfPcaExtractor(icaOfPca, images[curTest[1]]);
            ICAofPCAWhiteningExtractor whitenedPCAExtractor(icaOfPca);
            ZScoreICAofPCAWhiteningExtractor zscoreWhitenedPCAExtractor(icaOfPca, images[curTest[1]]);

            LDAofPCA ldaOfPca(images[curTest[0]], classes[curTest[0]], pcaSelThreshold);
            LDAofPCAExtractor ldaOfPcaExtractor(ldaOfPca);
            ZScoreLDAofPCAExtractor zscoreLdaOfPcaExtractor(ldaOfPca, images[curTest[1]]);


            Evaluation DirectEval(images[curTest[2]], classes[curTest[2]], passExtractor, m, false);
            direct << DirectEval.eer;

            Evaluation zScoreDirectEval(images[curTest[2]], classes[curTest[2]], zscorePassExtractor, m, false);
            zScoreDirect << zScoreDirectEval.eer;

            Evaluation pcaEval(images[curTest[2]], classes[curTest[2]], pcaExtractor, m, false);
            pcaR << pcaEval.eer;

            Evaluation eigenNormPcaEval(images[curTest[2]], classes[curTest[2]], normPCAExtractor, m, false);
            eigenNormPca << eigenNormPcaEval.eer;

            Evaluation zScorePcaEval(images[curTest[2]], classes[curTest[2]], zscorePCAExtractor, m, false);
            zScorePca << zScorePcaEval.eer;

            Evaluation whitenedPcaEval(images[curTest[2]], classes[curTest[2]], whitenedPCAExtractor, m, false);
            whitenedPca << whitenedPcaEval.eer;

            Evaluation zScoreWhitenedPcaEval(images[curTest[2]], classes[curTest[2]], zscoreWhitenedPCAExtractor, m, false);
            zScoreWhitenedPca << zScoreWhitenedPcaEval.eer;

            Evaluation ldaEval(images[curTest[2]], classes[curTest[2]], ldaOfPcaExtractor, m, false);
            lda << ldaEval.eer;

            Evaluation zScoreLdaEval(images[curTest[2]], classes[curTest[2]], zscoreLdaOfPcaExtractor, m, false);
            zScoreLda << zScoreLdaEval.eer;

            Evaluation icaEval(images[curTest[2]], classes[curTest[2]], icaOfPcaExtractor, m, false);
            ica << icaEval.eer;

            Evaluation zScoreIcaEval(images[curTest[2]], classes[curTest[2]], zscoreIcaOfPcaExtractor, m, false);
            zScoreIca << zScoreIcaEval.eer;
        }
        qDebug() << Vector::meanValue(direct) << '\t' << Vector::stdDeviation(direct);
        qDebug() << Vector::meanValue(zScoreDirect) << '\t'  << Vector::stdDeviation(zScoreDirect);
        qDebug() << Vector::meanValue(pcaR) << '\t'  << Vector::stdDeviation(pcaR);
        qDebug() << Vector::meanValue(eigenNormPca) << '\t'  << Vector::stdDeviation(eigenNormPca);
        qDebug() << Vector::meanValue(zScorePca) << '\t'  << Vector::stdDeviation(zScorePca);
        qDebug() << Vector::meanValue(lda) << '\t'  << Vector::stdDeviation(lda);
        qDebug() << Vector::meanValue(zScoreLda) << '\t'  << Vector::stdDeviation(zScoreLda);
        qDebug() << Vector::meanValue(ica) << '\t'  << Vector::stdDeviation(ica);
        qDebug() << Vector::meanValue(zScoreIca) << '\t'  << Vector::stdDeviation(zScoreIca);
        qDebug() << Vector::meanValue(whitenedPca) << '\t'  << Vector::stdDeviation(whitenedPca);
        qDebug() << Vector::meanValue(zScoreWhitenedPca) << '\t'  << Vector::stdDeviation(zScoreWhitenedPca);
    }

    static void evaluateFeatureSelection()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany",
                           allImages, &allClasses, "*.png", true, "-", false);

        // Divide data
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 10, images, classes);

        // all permutations of three clusters
        QVector<QVector<int> > testRuns;
        QVector<int> test0; test0 << 0 << 1 << 2; testRuns << test0;
        QVector<int> test1; test1 << 0 << 2 << 1; testRuns << test1;
        QVector<int> test2; test2 << 1 << 0 << 2; testRuns << test2;
        QVector<int> test3; test3 << 1 << 2 << 0; testRuns << test3;
        QVector<int> test4; test4 << 2 << 0 << 1; testRuns << test4;
        QVector<int> test5; test5 << 2 << 1 << 0; testRuns << test5;

        double pcaSelThreshold = 0.99;
        QVector<double> eerValues;
        qDebug() << "ICA, cos," << pcaSelThreshold;
        for (int testIndex = 0; testIndex < testRuns.count(); testIndex++)
        {
            QVector<int> test = testRuns[testIndex];
            qDebug() << "Testrun:" << (testIndex+1);

            // Train projection
            //PCA pca(images[test[0]]);
            //pca.modesSelectionThreshold(pcaSelThreshold);
            //PCAExtractor pcaExtractor(pca);
            ICAofPCA ica(images[test[0]], pcaSelThreshold);
            ICAofPCAExtractor extractor(ica);

            // Train feature selection
            //EuclideanWeightedMetric eucl;
            CosineWeightedMetric cos;
            //QVector<Template> mahalTemplates = Template::createTemplates(images[test[1]], classes[test[1]], pcaExtractor);
            //QVector<Matrix> mahalVectors = Template::getVectors(mahalTemplates);
            //mahal.learn(mahalVectors);
            cos.w = Matrix::ones(ica.ica.getModes(), 1, CV_64F);

            QVector<Template> trainSelectionTemplates = Template::createTemplates(images[test[1]], classes[test[1]], extractor);
            FeatureSelection selection(trainSelectionTemplates, cos);
            cos.w = selection.bestTrainWeight;

            // Evaluate
            Evaluation pcaEval(images[test[2]], classes[test[2]], extractor, cos, false);

            // Compare with baseline (no feature selection)
            cos.w = Matrix::ones(ica.ica.getModes(), 1, CV_64F);
            Evaluation pcaEvalOnes(images[test[2]], classes[test[2]], extractor, cos, false);
            qDebug() << "Selected:" << pcaEval.eer << "; Baseline:" << pcaEvalOnes.eer
                     << "; diff:" << (pcaEval.eer-pcaEvalOnes.eer);
        }
    }

    static void evaluateSelectedMultiWithScoreLevelFusion()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        //Loader::loadImages("/mnt/data/thermo/germany", allImages, &allClasses, "*.png", true, "-");
        //Loader::loadImages("/mnt/data/thermo/fit", allImages, &allClasses, "*.png", true, "-");
        //Loader::loadImages("/mnt/data/thermo/nd", allImages, &allClasses, "*.png", true, "-");
        Loader::loadImages("/mnt/data/thermo/equinox", allImages, &allClasses, "*.png", true, "-");


        // Divide data
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divideToNClusters(allImages, allClasses, 3, images, classes);

        double pcaSelThreshold = 0.9999;

        CityblockMetric cityBlock;
        EuclideanMetric eucl;
        CosineMetric cos;
        CorrelationMetric corr;

        int runsCount = 20;
        QVector<double> runs;
        QVector<double> ldaFusionResults;
        QVector<double> logRegFusionResults;
        QVector<double> wSumFusionResults;
        QVector<double> productFusionResults;
        QVector<double> svmFusionResults;

        QVector<double> directResults;
        QVector<double> zScoreDirectResults;
        QVector<double> pcaResults;
        QVector<double> eigenNormPcaResults;
        QVector<double> zScorePcaResults;
        QVector<double> ldaResults;
        QVector<double> icaCosResults;
        QVector<double> icaCorrResults;

        for (int testIndex = 1; testIndex <= runsCount; testIndex++)
        {
            qDebug() << "Testrun:" << testIndex;
            runs << testIndex;

            images.clear(); classes.clear();
            BioDataProcessing::divideToNClusters(allImages, allClasses, 3, images, classes);

            QList<Evaluation> evaluations;
            QVector<FeatureExtractor*> usedExtractors;
            QVector<Metrics*> usedMetrics;

            // Create (and train) feature extractors
            PassExtractor passExtractor;
            ZScorePassExtractor zscorePassExtractor(images[1]);

            PCA pca(images[0]);
            pca.modesSelectionThreshold(pcaSelThreshold);
            PCAExtractor pcaExtractor(pca);
            NormPCAExtractor normPCAExtractor(pca);
            ZScorePCAExtractor zscorePCAExtractor(pca, images[1]);

            ICAofPCA icaOfPca(images[0], pcaSelThreshold);
            ICAofPCAExtractor icaOfPcaExtractor(icaOfPca);
            //ZScoreICAofPCAExtractor zscoreIcaOfPcaExtractor(icaOfPca, images[curTest[1]]);
            //ICAofPCAWhiteningExtractor whitenedPCAExtractor(icaOfPca);
            //ZScoreICAofPCAWhiteningExtractor zscoreWhitenedPCAExtractor(icaOfPca, images[curTest[1]]);

            LDAofPCA ldaOfPca(images[0], classes[0], 0.98);
            LDAofPCAExtractor ldaOfPcaExtractor(ldaOfPca);
            //ZScoreLDAofPCAExtractor zscoreLdaOfPcaExtractor(ldaOfPca, images[curTest[1]]);

            // train data evaluation
            // ---------------------

            Evaluation DirectEval(images[1], classes[1], passExtractor, cityBlock);
            //evaluations << DirectEval;
            //usedMetrics << &cityBlock;
            //usedExtractors << &passExtractor;
            qDebug() << "  DirectEval-cityblock" << DirectEval.eer;
            directResults << DirectEval.eer;

            Evaluation zScoreDirectEval(images[1], classes[1], zscorePassExtractor, corr);
            evaluations << zScoreDirectEval;
            usedMetrics << &corr;
            usedExtractors << &zscorePassExtractor;
            qDebug() << "  zScoreDirectEval-corr" << zScoreDirectEval.eer;
            zScoreDirectResults << zScoreDirectEval.eer;

            Evaluation pcaEval(images[1], classes[1], pcaExtractor, eucl);
            //evaluations << pcaEval;
            //usedMetrics << &eucl;
            //usedExtractors << &pcaExtractor;
            qDebug() << "  pcaEval-eucl" << pcaEval.eer;
            pcaResults << pcaEval.eer;

            Evaluation eigenNormPcaEval(images[1], classes[1], normPCAExtractor, cos);
            //evaluations << eigenNormPcaEval;
            //usedMetrics << &cos;
            //usedExtractors << &normPCAExtractor;
            qDebug() << "  eigenNormPcaEval-cos" << eigenNormPcaEval.eer;
            eigenNormPcaResults << eigenNormPcaEval.eer;

            Evaluation zScorePcaEval(images[1], classes[1], zscorePCAExtractor, cos);
            evaluations << zScorePcaEval;
            usedMetrics << &cos;
            usedExtractors << &zscorePCAExtractor;
            qDebug() << "  zScorePcaEval-cos" << zScorePcaEval.eer;
            zScorePcaResults << zScorePcaEval.eer;

            /*Evaluation whitenedPcaEval(images[1], classes[1], whitenedPCAExtractor, m);
            evaluations << whitenedPcaEval;
            usedMetrics << mPtr;
            usedExtractors << &whitenedPCAExtractor;

            Evaluation zScoreWhitenedPcaEval(images[1], classes[1], zscoreWhitenedPCAExtractor, m);
            evaluations << zScoreWhitenedPcaEval;
            usedMetrics << mPtr;
            usedExtractors << &zscoreWhitenedPCAExtractor;*/

            Evaluation ldaEval(images[1], classes[1], ldaOfPcaExtractor, eucl);
            //evaluations << ldaEval;
            //usedMetrics << &eucl;
            //usedExtractors << &ldaOfPcaExtractor;
            qDebug() << "  ldaEval-eucl" << ldaEval.eer;
            ldaResults << ldaEval.eer;

            /*Evaluation zScoreLdaEval(images[1], classes[1], zscoreLdaOfPcaExtractor, m);
            evaluations << zScoreLdaEval;
            usedMetrics << mPtr;
            usedExtractors << &zscoreLdaOfPcaExtractor;*/

            Evaluation icaEval(images[1], classes[1], icaOfPcaExtractor, cos);
            evaluations << icaEval;
            usedMetrics << &cos;
            usedExtractors << &icaOfPcaExtractor;
            qDebug() << "  icaEval-cos" << icaEval.eer;
            icaCosResults << icaEval.eer;

            Evaluation icaEval2(images[1], classes[1], icaOfPcaExtractor, corr);
            //evaluations << icaEval2;
            //usedMetrics << &corr;
            //usedExtractors << &icaOfPcaExtractor;
            qDebug() << "  icaEval2-corr" << icaEval2.eer;
            icaCorrResults << icaEval2.eer;

            /*Evaluation zScoreIcaEval(images[1], classes[1], zscoreIcaOfPcaExtractor, m);
            evaluations << zScoreIcaEval;
            usedMetrics << mPtr;
            usedExtractors << &zscoreIcaOfPcaExtractor;*/

            // output graph
            QString impPlot = QString::number(testIndex) + "-impostor";
            Common::savePlot(zScorePcaEval.impostorScores, ldaEval.impostorScores, icaEval.impostorScores, impPlot);

            QString genPlot = QString::number(testIndex) + "-genuine";
            Common::savePlot(zScorePcaEval.genuineScores, ldaEval.genuineScores, icaEval.genuineScores, genPlot);

            // train fusion classifier
            WeightedSumFusion wSumFusion(evaluations);
            LogisticRegressionFusion logRegFusion(evaluations);
            SVMFusion svmFusion(evaluations);
            ProductFusion productFusion(evaluations);
            LDAFusion ldaFusion(evaluations);


            // evaluate on testing data
            Evaluation wSumFusionEval = MultiBioSystem::evaluate(images[2], classes[2], wSumFusion,
                                                                 usedExtractors, usedMetrics);
            Evaluation logRegFusionEval = MultiBioSystem::evaluate(images[2], classes[2], logRegFusion,
                                                                 usedExtractors, usedMetrics);
            Evaluation svmFusionEval = MultiBioSystem::evaluate(images[2], classes[2], svmFusion,
                                                                 usedExtractors, usedMetrics);
            Evaluation productFusionEval = MultiBioSystem::evaluate(images[2], classes[2], productFusion,
                                                                 usedExtractors, usedMetrics);
            Evaluation ldaFusionEval = MultiBioSystem::evaluate(images[2], classes[2], ldaFusion,
                                                                 usedExtractors, usedMetrics);

            qDebug() << wSumFusionEval.eer << logRegFusionEval.eer << svmFusionEval.eer
                     << productFusionEval.eer << ldaFusionEval.eer;
            wSumFusionResults << wSumFusionEval.eer;
            logRegFusionResults << logRegFusionEval.eer;
            svmFusionResults << svmFusionEval.eer;
            productFusionResults << productFusionEval.eer;
            ldaFusionResults << ldaFusionEval.eer;
        }
        qDebug() << "weightedSumFusion:" << Vector::meanValue(wSumFusionResults) << '\t' << Vector::stdDeviation(wSumFusionResults);
        qDebug() << "productFusion:" << Vector::meanValue(productFusionResults) << '\t' << Vector::stdDeviation(productFusionResults);
        qDebug() << "LDAFusion:" << Vector::meanValue(ldaFusionResults) << '\t' << Vector::stdDeviation(ldaFusionResults);
        qDebug() << "SVMFusion:" << Vector::meanValue(svmFusionResults) << '\t' << Vector::stdDeviation(svmFusionResults);
        qDebug() << "logisticRegressionFusion:" << Vector::meanValue(logRegFusionResults) << '\t' << Vector::stdDeviation(logRegFusionResults);

        qDebug();

        qDebug() << "directResults-ssd:" << Vector::meanValue(directResults) << '\t' << Vector::stdDeviation(directResults);
        qDebug() << "zScoreDirect-corr:" << Vector::meanValue(zScoreDirectResults) << '\t' << Vector::stdDeviation(zScoreDirectResults);
        qDebug() << "pca-eucl:" << Vector::meanValue(pcaResults) << '\t' << Vector::stdDeviation(pcaResults);
        qDebug() << "eigenNormPca-cos:" << Vector::meanValue(eigenNormPcaResults) << '\t' << Vector::stdDeviation(eigenNormPcaResults);
        qDebug() << "zScorePca-cos:" << Vector::meanValue(zScorePcaResults) << '\t' << Vector::stdDeviation(zScorePcaResults);
        qDebug() << "lda-eucl:" << Vector::meanValue(ldaResults) << '\t' << Vector::stdDeviation(ldaResults);
        qDebug() << "ica-cos:" << Vector::meanValue(icaCosResults) << '\t' << Vector::stdDeviation(icaCosResults);
        qDebug() << "ica-corr:" << Vector::meanValue(icaCorrResults) << '\t' << Vector::stdDeviation(icaCorrResults);

        Common::savePlot(runs, wSumFusionResults, "weightedSumFusion");
        Common::savePlot(runs, logRegFusionResults, "logisticRegressionFusion");
        Common::savePlot(runs, svmFusionResults, "SVMFusion");
        Common::savePlot(runs, productFusionResults, "productFusion");
        Common::savePlot(runs, ldaFusionResults, "LDAFusion");

        Common::savePlot(runs, directResults, "directResults");
        Common::savePlot(runs, zScoreDirectResults, "zScoreDirectResults");
        Common::savePlot(runs, pcaResults, "pcaResults");
        Common::savePlot(runs, eigenNormPcaResults, "eigenNormPcaResults");
        Common::savePlot(runs, zScorePcaResults, "zScorePcaResults");
        Common::savePlot(runs, ldaResults, "ldaResults");
        Common::savePlot(runs, icaCosResults, "icaCosResults");
        Common::savePlot(runs, icaCorrResults, "icaCorrResults");
    }

    static void evaluateMultiWithScoreLevelFusion()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany",
                           allImages, &allClasses, "*.png", true, "-", false);

        // Divide data
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 10, images, classes);

        // all permutations of three clusters
        QVector<QVector<int> > testRuns;
        QVector<int> test0; test0 << 0 << 1 << 2; testRuns << test0;
        QVector<int> test1; test1 << 0 << 2 << 1; testRuns << test1;
        QVector<int> test2; test2 << 1 << 0 << 2; testRuns << test2;
        QVector<int> test3; test3 << 1 << 2 << 0; testRuns << test3;
        QVector<int> test4; test4 << 2 << 0 << 1; testRuns << test4;
        QVector<int> test5; test5 << 2 << 1 << 0; testRuns << test5;

        double pcaSelThreshold = 0.9999;

        CityblockMetric cityBlock;
        EuclideanMetric eucl;
        CosineMetric cos;
        CorrelationMetric corr;
        QList<Metrics*> metrics;
        metrics << (&cityBlock) << (&eucl) << (&cos) << (&corr);

        for (int testIndex = 0; testIndex < testRuns.count(); testIndex++)
        {
            QVector<int> curTest = testRuns[testIndex];
            qDebug() << "Testrun:" << (testIndex+1);

            QList<Evaluation> evaluations;
            QVector<FeatureExtractor*> usedExtractors;
            QVector<Metrics*> usedMetrics;

            // Create (and train) feature extractors
            PassExtractor passExtractor;
            ZScorePassExtractor zscorePassExtractor(images[curTest[1]]);

            PCA pca(images[curTest[0]]);
            pca.modesSelectionThreshold(pcaSelThreshold);
            PCAExtractor pcaExtractor(pca);
            NormPCAExtractor normPCAExtractor(pca);
            ZScorePCAExtractor zscorePCAExtractor(pca, images[curTest[1]]);

            ICAofPCA icaOfPca(images[curTest[0]], pcaSelThreshold);
            ICAofPCAExtractor icaOfPcaExtractor(icaOfPca);
            ZScoreICAofPCAExtractor zscoreIcaOfPcaExtractor(icaOfPca, images[curTest[1]]);
            ICAofPCAWhiteningExtractor whitenedPCAExtractor(icaOfPca);
            ZScoreICAofPCAWhiteningExtractor zscoreWhitenedPCAExtractor(icaOfPca, images[curTest[1]]);

            LDAofPCA ldaOfPca(images[curTest[0]], classes[curTest[0]], pcaSelThreshold);
            LDAofPCAExtractor ldaOfPcaExtractor(ldaOfPca);
            ZScoreLDAofPCAExtractor zscoreLdaOfPcaExtractor(ldaOfPca, images[curTest[1]]);

            // evaluate on training data for different metrics
            for (int metricIndex = 0; metricIndex < metrics.count(); metricIndex++)
            {
                Metrics *mPtr = metrics[metricIndex];
                Metrics &m = *mPtr;

                Evaluation DirectEval(images[curTest[1]], classes[curTest[1]], passExtractor, m);
                evaluations << DirectEval;
                usedMetrics << mPtr;
                usedExtractors << &passExtractor;

                Evaluation zScoreDirectEval(images[curTest[1]], classes[curTest[1]], zscorePassExtractor, m);
                evaluations << zScoreDirectEval;
                usedMetrics << mPtr;
                usedExtractors << &zscorePassExtractor;

                Evaluation pcaEval(images[curTest[1]], classes[curTest[1]], pcaExtractor, m);
                evaluations << pcaEval;
                usedMetrics << mPtr;
                usedExtractors << &pcaExtractor;

                Evaluation eigenNormPcaEval(images[curTest[1]], classes[curTest[1]], normPCAExtractor, m);
                evaluations << eigenNormPcaEval;
                usedMetrics << mPtr;
                usedExtractors << &normPCAExtractor;

                Evaluation zScorePcaEval(images[curTest[1]], classes[curTest[1]], zscorePCAExtractor, m);
                evaluations << zScorePcaEval;
                usedMetrics << mPtr;
                usedExtractors << &zscorePCAExtractor;

                Evaluation whitenedPcaEval(images[curTest[1]], classes[curTest[1]], whitenedPCAExtractor, m);
                evaluations << whitenedPcaEval;
                usedMetrics << mPtr;
                usedExtractors << &whitenedPCAExtractor;

                Evaluation zScoreWhitenedPcaEval(images[curTest[1]], classes[curTest[1]], zscoreWhitenedPCAExtractor, m);
                evaluations << zScoreWhitenedPcaEval;
                usedMetrics << mPtr;
                usedExtractors << &zscoreWhitenedPCAExtractor;

                Evaluation ldaEval(images[curTest[1]], classes[curTest[1]], ldaOfPcaExtractor, m);
                evaluations << ldaEval;
                usedMetrics << mPtr;
                usedExtractors << &ldaOfPcaExtractor;

                Evaluation zScoreLdaEval(images[curTest[1]], classes[curTest[1]], zscoreLdaOfPcaExtractor, m);
                evaluations << zScoreLdaEval;
                usedMetrics << mPtr;
                usedExtractors << &zscoreLdaOfPcaExtractor;

                Evaluation icaEval(images[curTest[1]], classes[curTest[1]], icaOfPcaExtractor, m);
                evaluations << icaEval;
                usedMetrics << mPtr;
                usedExtractors << &icaOfPcaExtractor;

                Evaluation zScoreIcaEval(images[curTest[1]], classes[curTest[1]], zscoreIcaOfPcaExtractor, m);
                evaluations << zScoreIcaEval;
                usedMetrics << mPtr;
                usedExtractors << &zscoreIcaOfPcaExtractor;
            }

            // train fusion classifier
            ProductFusion fusion(evaluations);

            // evaluate on testing data
            Evaluation fusionEval = MultiBioSystem::evaluate(images[curTest[2]], classes[curTest[2]],
                                                             fusion, usedExtractors, usedMetrics);
            qDebug() << fusionEval.eer;
        }
    }

    static void evaluateMethodCorrelation()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany",
                           allImages, &allClasses, "*.png", true, "-", false);

        // Divide data
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 15, images, classes);

        // we will use the best threshold. selection is based on evaluateForDifferentSelectionThresholds()
        double selThreshold = 0.99;

        // Train
        PCA pca(images[0]);
        pca.modesSelectionThreshold(selThreshold);
        PCAExtractor pcaExtractor(pca);

        LDAofPCA lda(images[0], classes[0], selThreshold);
        LDAofPCAExtractor ldaExtractor(lda);

        ICAofPCA ica(images[0], selThreshold, false);
        ICAofPCAExtractor icaExtractor(ica);
        ICAofPCAWhiteningExtractor whitenExtractor(ica);

        // CreateTemplates
        QVector<Template> pcaTemplates = Template::createTemplates(images[1], classes[1], pcaExtractor);
        QVector<Template> ldaTemplates = Template::createTemplates(images[1], classes[1], ldaExtractor);
        QVector<Template> icaTemplates = Template::createTemplates(images[1], classes[1], icaExtractor);
        QVector<Template> whitenTemplates = Template::createTemplates(images[1], classes[1], whitenExtractor);

        // Evaluate
        EuclideanMetric eucl;
        CosineMetric cos;

        Evaluation pcaEuclEval(pcaTemplates, eucl, false);
        Evaluation pcaCosEval(pcaTemplates, cos, false);

        Evaluation ldaEuclEval(ldaTemplates, eucl, false);
        Evaluation ldaCosEval(ldaTemplates, cos, false);

        Evaluation icaEuclEval(icaTemplates, eucl, false);
        Evaluation icaCosEval(icaTemplates, cos, false);

        Evaluation whitenEuclEval(whitenTemplates, eucl, false);
        Evaluation whitenCosEval(whitenTemplates, cos, false);

        Common::savePlot(icaCosEval.genuineScores, ldaEuclEval.genuineScores, "icaCos-ldaEucl-genuineCorrelation");
        Common::savePlot(icaCosEval.impostorScores, ldaEuclEval.impostorScores, "icaCos-ldaEucl-impostorCorrelation");

        Common::savePlot(icaCosEval.genuineScores, whitenCosEval.genuineScores, "icaCos-whitenCos-genuineCorrelation");
        Common::savePlot(icaCosEval.impostorScores, whitenCosEval.impostorScores, "icaCos-whitenCos-impostorCorrelation");

        Common::savePlot(icaCosEval.genuineScores, pcaCosEval.genuineScores, "icaCos-pcaCos-genuineCorrelation");
        Common::savePlot(icaCosEval.impostorScores, pcaCosEval.impostorScores, "icaCos-pcaCos-impostorCorrelation");

        Common::savePlot(icaCosEval.genuineScores, pcaEuclEval.genuineScores, "icaCos-pcaEucl-genuineCorrelation");
        Common::savePlot(icaCosEval.impostorScores, pcaEuclEval.impostorScores, "icaCos-pcaEucl-impostorCorrelation");

        Common::savePlot(ldaEuclEval.genuineScores, pcaEuclEval.genuineScores, "ldaEucl-pcaEucl-genuineCorrelation");
        Common::savePlot(ldaEuclEval.impostorScores, pcaEuclEval.impostorScores, "ldaEucl-pcaEucl-impostorCorrelation");
    }

    static void evaluateForDifferentSelectionThresholds()
    {
        QVector<Matrix> allImages;
        QVector<int> allClasses;
        Loader::loadImages("/home/stepo/SVN/disp-stepan-mracek/databases/thermo/germany",
                           allImages, &allClasses, "*.png", true, "-", false);

        // Divide data into 3 clusters
        QList<QVector<Matrix> > images;
        QList<QVector<int> > classes;
        BioDataProcessing::divide(allImages, allClasses, 10, images, classes);

        QVector<int> trainIndicies; trainIndicies << 0 << 1 << 2;
        QVector<QVector<int> > testIndicies;
        QVector<int> test0; test0 << 1 << 2;
        QVector<int> test1; test1 << 0 << 2;
        QVector<int> test2; test2 << 0 << 1;
        testIndicies << test0 << test1 << test2;

        for (int test = 0; test < 3; test++)
        {
            QMap<double, double> pcaEucl;
            QMap<double, double> pcaCos;
            QMap<double, double> ldaEucl;
            QMap<double, double> ldaCos;
            QMap<double, double> icaEucl;
            QMap<double, double> icaCos;
            QMap<double, double> whitenEucl;
            QMap<double, double> whitenCos;
            for (double selThreshold = 0.9999; selThreshold >= 0.5; selThreshold -= 0.002)
            {
                qDebug() << "Cluster" << (test+1) << "threshold" << selThreshold;
                // Train
                PCA pca(images[test]);
                pca.modesSelectionThreshold(selThreshold);
                PCAExtractor pcaExtractor(pca);

                if (pca.getModes() < 10)
                    break;

                LDAofPCA lda(images[test], classes[test], selThreshold);
                LDAofPCAExtractor ldaExtractor(lda);

                ICAofPCA ica(images[test], selThreshold);
                ICAofPCAExtractor icaExtractor(ica);
                ICAofPCAWhiteningExtractor whitenExtractor(ica);

                // CreateTemplates
                QVector<Matrix> testImages(images[testIndicies[test][0]]);
                testImages << images[testIndicies[test][1]];

                QVector<int> testClasses(classes[testIndicies[test][0]]);
                testClasses << classes[testIndicies[test][1]];

                QVector<Template> pcaTemplates = Template::createTemplates(testImages, testClasses, pcaExtractor);
                QVector<Template> ldaTemplates = Template::createTemplates(testImages, testClasses, ldaExtractor);
                QVector<Template> icaTemplates = Template::createTemplates(testImages, testClasses, icaExtractor);
                QVector<Template> whitenTemplates = Template::createTemplates(testImages, testClasses, whitenExtractor);

                // Evaluate
                EuclideanMetric eucl;
                CosineMetric cos;

                Evaluation pcaEuclEval(pcaTemplates, eucl, false);
                pcaEucl[selThreshold] = pcaEuclEval.eer;
                Evaluation pcaCosEval(pcaTemplates, cos, false);
                pcaCos[selThreshold] = pcaCosEval.eer;

                Evaluation ldaEuclEval(ldaTemplates, eucl, false);
                ldaEucl[selThreshold] = ldaEuclEval.eer;
                Evaluation ldaCosEval(ldaTemplates, cos, false);
                ldaCos[selThreshold] = ldaCosEval.eer;

                Evaluation icaEuclEval(icaTemplates, eucl, false);
                icaEucl[selThreshold] = icaEuclEval.eer;
                Evaluation icaCosEval(icaTemplates, cos, false);
                icaCos[selThreshold] = icaCosEval.eer;

                Evaluation whitenEuclEval(whitenTemplates, eucl, false);
                whitenEucl[selThreshold] = whitenEuclEval.eer;
                Evaluation whitenCosEval(whitenTemplates, cos, false);
                whitenCos[selThreshold] = whitenCosEval.eer;
            }

            Common::saveMap(pcaEucl,    QString("pcaEucl")    + QString::number(test));
            Common::saveMap(pcaCos,     QString("pcaCos")     + QString::number(test));
            Common::saveMap(ldaEucl,    QString("ldaEucl")    + QString::number(test));
            Common::saveMap(ldaCos,     QString("ldaCos")     + QString::number(test));
            Common::saveMap(icaEucl,    QString("icaEucl")    + QString::number(test));
            Common::saveMap(icaCos,     QString("icaCos")     + QString::number(test));
            Common::saveMap(whitenEucl, QString("whitenEucl") + QString::number(test));
            Common::saveMap(whitenCos,  QString("whitenCos")  + QString::number(test));
        }
    }
};

#endif // EVALUATETHERMO_H
