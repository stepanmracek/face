#ifndef EVALUATEICA_H
#define EVALUATEICA_H

#include <QString>

#include <opencv/cv.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/biodataprocessing.h"
#include "faceCommon/linalg/icaofpca.h"
#include "faceCommon/linalg/ica.h"
#include "faceCommon/biometrics/evaluation.h"
#include "faceCommon/linalg/metrics.h"
#include "faceCommon/linalg/pca.h"
#include "faceCommon/linalg/normalization.h"
#include "faceCommon/biometrics/eerpotential.h"

class EvaluateICA
{
public:
    static void process()
    {
        // Load images
        QString path("/home/stepo/SVN/frgc/frgc-norm-iterative/small-index");
        std::vector<Vector> allImages;
        std::vector<int> allClasses;
        Loader::loadImages(path, allImages, &allClasses, "*.png", "-", false);

        // Divide
        QList<std::vector<Vector> > images;
        QList<std::vector<int> > classes;
        BioDataProcessing::divideTemplatesToClusters(allImages, allClasses, 25, images, classes);
        std::vector<Vector> &trainSet = images[0];

        // ICA
        //ICA ica(trainSet, 50);

        // ICA of PCA
        ICAofPCA icaOfpca(trainSet, 0.98, 50);

        // PCA
        //PCA pca(trainSet, 50);

        // Last cluster consits only from one class, we don't need it
        images.removeLast(); classes.removeLast();

        // Normalize
        /*std::vector<Matrix> ICAofPCAsamples;
        for (int i = 0; i < trainSet.count(); i++)
        {
            Matrix sample = icaOfpca.project(trainSet[i]);
            ICAofPCAsamples.append(sample);
        }
        LinearNormalizationResult normResult = ScoreNormalization::linearNormalization(ICAofPCAsamples);
        MahalanobisMetric mahalanobis(ICAofPCAsamples);*/

        QList<std::vector<Template> > testICAofPCATemplates;
        //QList<std::vector<Template> > testPCATemplates;
        //QList<std::vector<Template> > testICATemplates;
        for (int i = 0; i < images.count(); i++)
        {
            qDebug() << "projecting set" << i;

            std::vector<Template> ICAofPCAtemplates;
            //std::vector<Template> PCAtemplates;
            //std::vector<Template> ICAtemplates;

            for (int j = 0; j < classes[i].count(); j++)
            {
                Template tICAofPCA;
                tICAofPCA.subjectID = classes[i][j];
                Matrix fVec = icaOfpca.project(images[i][j]);
                //Vector::normalizeComponents(fVec, normResult.minValues, normResult.maxValues);
                tICAofPCA.featureVector = fVec;
                ICAofPCAtemplates.append(tICAofPCA);

                /*Template tPCA;
                tPCA.subjectID = classes[i][j];
                tPCA.featureVector = pca.project(images[i][j]);
                PCAtemplates.append(tPCA);*/

                /*Template tICA;
                tICA.subjectID = classes[i][j];
                tICA.featureVector = ica.project(images[i][j]);
                ICAtemplates.append(tICA);*/
            }
            testICAofPCATemplates.append(ICAofPCAtemplates);
            //testPCATemplates.append(PCAtemplates);
            //testICATemplates.append(ICAtemplates);
        }

        //EERPotential potential(testICAofPCATemplates[1]);
        //CosineWeightedMetric metric;
        //metric.w = potential.createSelectionWeights(0.6);
        EuclideanMetric metric;
        BatchEvaluationResult result = Evaluation::batch(testICAofPCATemplates, metric, 1);
        qDebug() << "ICA of PCA" << result.meanEER;

        //result = Evaluation::batch(testICAofPCATemplates, mahalanobis);
        //qDebug() << "ICA of PCA (mahalanobis)" << result.meanEER;

        //result = Evaluation::batch(testPCATemplates, euclidean);
        //qDebug() << "PCA" << result.meanEER;

        //result = Evaluation::batch(testICATemplates, euclidean, 1);
        //qDebug() << "ICA" << result.meanEER;
    }
};

#endif // EVALUATEICA_H
