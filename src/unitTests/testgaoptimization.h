#ifndef TESTGAOPTIMIZATION_H
#define TESTGAOPTIMIZATION_H

#include <QVector>
#include <QString>
#include <QDebug>

#include <opencv/cv.h>

#include "linalg/common.h"
#include "biometrics/template.h"
#include "linalg/loader.h"
#include "linalg/metrics.h"
#include "biometrics/evaluation.h"
#include "biometrics/geneticweightoptimization.h"
#include "linalg/ldaofpca.h"
#include "linalg/pca.h"

class TestGAOptimization
{
public:
    static void testGAOptimizationOnFisherface()
    {
        QVector<Face::LinAlg::Vector> vectors;
        QVector<Face::LinAlg::Vector> vectors2;
        QVector<int> classMembership;
        QVector<int> classMembership2;
        QString path("/home/stepo/SVN/disp-stepan-mracek/databases/orl");
        QString path2("/home/stepo/SVN/disp-stepan-mracek/databases/orl2");

        // load images
        Face::LinAlg::Loader::loadImages(path, vectors, &classMembership, "*.pgm");
        Face::LinAlg::Loader::loadImages(path2, vectors2, &classMembership2, "*.pgm");

        // train
        //LDAofPCA fisher(vectors, classMembership);
        Face::LinAlg::PCA pca(vectors);
        pca.setModes(60);

        // project
        qDebug() << "Creating templates";
        int n = vectors.count();
        QVector<Face::Biometrics::Template> templates;
        QVector<Face::Biometrics::Template> templates2;
        for (int i = 0; i < n; i++)
        {
            Face::Biometrics::Template t;
            t.subjectID = classMembership[i];
            t.featureVector = pca.project(vectors[i]);
            templates.append(t);

            Face::Biometrics::Template t2;
            t2.subjectID = classMembership2[i];
            t2.featureVector = pca.project(vectors2[i]);
            templates2.append(t2);
        }

        Face::LinAlg::EuclideanWeightedMetric euclW;
        Face::Biometrics::GeneticWeightOptimizationSettings settings(1000, 30, 15, 0.1, 200);
        Face::Biometrics::GeneticWeightOptimization optimization(templates, templates2, euclW, settings);
        Face::Biometrics::GeneticWeightOptimizationResult result = optimization.trainWeights();

        Face::LinAlg::Common::savePlot(result.iterations, result.trainEER, "GA-train1");
        Face::LinAlg::Common::savePlot(result.iterations, result.validationEER, "GA-validation1");
        Face::LinAlg::Common::savePlot(result.iterations, result.diversity, "GA-popdiversity1");

        result.bestWeightsOnTrain.toFile("GA-train-weight1");
        result.bestWeightsOnValidation.toFile("GA-validation-weight1");
    }
};

#endif // TESTGAOPTIMIZATION_H
