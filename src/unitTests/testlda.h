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
        Face::LinAlg::LDA lda;

        QVector<Face::LinAlg::Vector> vectors;
        QVector<int> classMembership;

        // first class
        Face::LinAlg::Vector a1(2); a1(0) = 0; a1(1) = 0;
        vectors.append(a1);
        classMembership.append(1);

        Face::LinAlg::Vector b1(2); b1(0) = 0; b1(1) = 1;
        vectors.append(b1);
        classMembership.append(1);

        Face::LinAlg::Vector c1(2); c1(0) = 1; c1(1) = 1;
        vectors.append(c1);
        classMembership.append(1);

        // second class
        Face::LinAlg::Vector a2(2); a2(0) = 1; a2(1) = 0;
        vectors.append(a2);
        classMembership.append(2);

        Face::LinAlg::Vector b2(2); b2(0) = 2; b2(1) = 0;
        vectors.append(b2);
        classMembership.append(2);

        Face::LinAlg::Vector c2(2); c2(0) = 2; c2(1) = 1;
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
        QVector<Face::LinAlg::Vector> vectors;
        QVector<Face::LinAlg::Vector> vectors2;
        QVector<int> classMembership;
        QVector<int> classMembership2;
        QString path("/media/data/SVN/disp-stepan-mracek/databases/orl");
        QString path2("/media/data/SVN/disp-stepan-mracek/databases/orl2");

        // load images
        Face::LinAlg::Loader::loadImages(path, vectors, &classMembership, "*.pgm");
        Face::LinAlg::Loader::loadImages(path2, vectors2, &classMembership2, "*.pgm");

        // train
        Face::LinAlg::LDAofPCA fisher(vectors, classMembership);

        // project
        qDebug() << "Creating templates";
        int n = vectors.count();
        QVector<Face::Biometrics::Template> ldaTemplates;
        QVector<Face::Biometrics::Template> pcaTemplates;
        QVector<Face::Biometrics::Template> ldaTemplates2;
        QVector<Face::Biometrics::Template> pcaTemplates2;
        QVector<Face::LinAlg::Vector> mahalSamples;
        for (int i = 0; i < n; i++)
        {
            Face::Biometrics::Template ldaTemplate;
            ldaTemplate.subjectID = classMembership[i];
            ldaTemplate.featureVector = fisher.project(vectors[i]);
            ldaTemplates.append(ldaTemplate);

            mahalSamples.append(ldaTemplate.featureVector);

            Face::Biometrics::Template pcaTemplate;
            pcaTemplate.subjectID = classMembership[i];
            pcaTemplate.featureVector = fisher.pca.project(vectors[i]);
            pcaTemplates.append(pcaTemplate);

            Face::Biometrics::Template ldaTemplate2;
            ldaTemplate2.subjectID = classMembership2[i];
            ldaTemplate2.featureVector = fisher.project(vectors2[i]);
            ldaTemplates2.append(ldaTemplate2);

            Face::Biometrics::Template pcaTemplate2;
            pcaTemplate2.subjectID = classMembership2[i];
            pcaTemplate2.featureVector = fisher.pca.project(vectors2[i]);
            pcaTemplates2.append(pcaTemplate2);
        }

        Face::LinAlg::EuclideanMetric eucl;
        Face::Biometrics::Evaluation evalLDA(ldaTemplates, eucl);
        Face::Biometrics::Evaluation evalPCA(pcaTemplates, eucl);
        Face::Biometrics::Evaluation evalLDA2(ldaTemplates2, eucl);
        Face::Biometrics::Evaluation evalPCA2(pcaTemplates2, eucl);

        Face::LinAlg::MahalanobisMetric mahal(mahalSamples);
        Face::Biometrics::Evaluation evalLDAonTestWithMahal(ldaTemplates2, mahal);

        qDebug() << "PCA on training set:" << evalPCA.eer;
        qDebug() << "LDA on training set:" << evalLDA.eer;
        qDebug() << "PCA on test set:" << evalPCA2.eer;
        qDebug() << "LDA on test set:" << evalLDA2.eer;
        qDebug() << "LDA on test set using Mahalanobis distance:" << evalLDAonTestWithMahal.eer;

        // save results to file
        evalLDA2.outputResults("fisherface", 20);
    }
};

#endif // TESTLDA_H
