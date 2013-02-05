#ifndef TESTLDA_H
#define TESTLDA_H

#include <QVector>
#include <QDebug>

#include <opencv/cv.h>

#include "linalg/lda.h"
#include "linalg/ldaofpca.h"
#include "linalg/common.h"
#include "linalg/matrixconverter.h"
#include "linalg/loader.h"
#include "linalg/metrics.h"
#include "biometrics/template.h"
#include "biometrics/evaluation.h"

class TestLDA
{
public:
    static void testLDASimple2D()
    {
        LDA lda;

        QVector<Matrix> vectors;
        QVector<int> classMembership;

        // first class
        Matrix a1 = Matrix::zeros(2, 1); a1(0) = 0; a1(1) = 0;
        vectors.append(a1);
        classMembership.append(1);

        Matrix b1 = Matrix::zeros(2, 1); b1(0) = 0; b1(1) = 1;
        vectors.append(b1);
        classMembership.append(1);

        Matrix c1 = Matrix::zeros(2, 1); c1(0) = 1; c1(1) = 1;
        vectors.append(c1);
        classMembership.append(1);

        // second class
        Matrix a2 = Matrix::zeros(2, 1); a2(0) = 1; a2(1) = 0;
        vectors.append(a2);
        classMembership.append(2);

        Matrix b2 = Matrix::zeros(2, 1); b2(0) = 2; b2(1) = 0;
        vectors.append(b2);
        classMembership.append(2);

        Matrix c2 = Matrix::zeros(2, 1); c2(0) = 2; c2(1) = 1;
        vectors.append(c2);
        classMembership.append(2);

        // train
        lda.learn(vectors, classMembership);

        qDebug() << "a1" << lda.project(a1)(0);
        qDebug() << "b1" << lda.project(b1)(0);
        qDebug() << "c1" << lda.project(c1)(0);

        qDebug() << "a2" << lda.project(a2)(0);
        qDebug() << "b2" << lda.project(b2)(0);
        qDebug() << "c2" << lda.project(c2)(0);
    }

    static void testLDAFaces()
    {
        QVector<Matrix> vectors;
        QVector<Matrix> vectors2;
        QVector<int> classMembership;
        QVector<int> classMembership2;
        QString path("/media/data/SVN/disp-stepan-mracek/databases/orl");
        QString path2("/media/data/SVN/disp-stepan-mracek/databases/orl2");

        // load images
        Loader::loadImages(path, vectors, &classMembership, "*.pgm", true);
        Loader::loadImages(path2, vectors2, &classMembership2, "*.pgm", true);

        // train
        LDAofPCA fisher(vectors, classMembership);

        // project
        qDebug() << "Creating templates";
        int n = vectors.count();
        QVector<Template> ldaTemplates;
        QVector<Template> pcaTemplates;
        QVector<Template> ldaTemplates2;
        QVector<Template> pcaTemplates2;
        QVector<Matrix> mahalSamples;
        for (int i = 0; i < n; i++)
        {
            Template ldaTemplate;
            ldaTemplate.subjectID = classMembership[i];
            ldaTemplate.featureVector = fisher.project(vectors[i]);
            ldaTemplates.append(ldaTemplate);

            mahalSamples.append(ldaTemplate.featureVector);

            Template pcaTemplate;
            pcaTemplate.subjectID = classMembership[i];
            pcaTemplate.featureVector = fisher.pca.project(vectors[i]);
            pcaTemplates.append(pcaTemplate);

            Template ldaTemplate2;
            ldaTemplate2.subjectID = classMembership2[i];
            ldaTemplate2.featureVector = fisher.project(vectors2[i]);
            ldaTemplates2.append(ldaTemplate2);

            Template pcaTemplate2;
            pcaTemplate2.subjectID = classMembership2[i];
            pcaTemplate2.featureVector = fisher.pca.project(vectors2[i]);
            pcaTemplates2.append(pcaTemplate2);
        }

        EuclideanMetric eucl;
        Evaluation evalLDA(ldaTemplates, eucl);
        Evaluation evalPCA(pcaTemplates, eucl);
        Evaluation evalLDA2(ldaTemplates2, eucl);
        Evaluation evalPCA2(pcaTemplates2, eucl);

        MahalanobisMetric mahal(mahalSamples);
        Evaluation evalLDAonTestWithMahal(ldaTemplates2, mahal);

        qDebug() << "PCA on training set:" << evalPCA.eer;
        qDebug() << "LDA on training set:" << evalLDA.eer;
        qDebug() << "PCA on test set:" << evalPCA2.eer;
        qDebug() << "LDA on test set:" << evalLDA2.eer;
        qDebug() << "LDA on test set using Mahalanobis distance:" << evalLDAonTestWithMahal.eer;

        // save results to file
        evalLDA2.outputResults("fisherface");
    }
};

#endif // TESTLDA_H
