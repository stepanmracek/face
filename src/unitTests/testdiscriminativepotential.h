#ifndef TESTDISCRIMINATIVEPOTENTIAL_H
#define TESTDISCRIMINATIVEPOTENTIAL_H

#include <QVector>
#include <QDebug>

#include "biometrics/template.h"
#include "biometrics/discriminativepotential.h"
#include "biometrics/biodataprocessing.h"
#include "linalg/vector.h"
#include "biometrics/evaluation.h"
#include "linalg/metrics.h"
#include "linalg/lda.h"

class TestDiscriminativePotential
{
public:
    static void TestOnFRGC()
    {
        QVector<Template> templates = Template::loadTemplates("/home/stepo/SVN/frgc/frgc-norm-iterative/anatomical","-");
        QList< QVector<Template> > clusters = BioDataProcessing::divide(templates, 25);
        clusters.removeLast(); // we do not want that small one with just one class
        DiscriminativePotential dp(clusters[0]);
        CityblockWeightedMetric m;

        // normalize
        //for (int i = 0; i < clusters.count(); i++)
        //    Template::normalizeFeatureVectorComponents(clusters[i], dp.minValues, dp.maxValues);

        QVector<Vector> featureVectors;
        QVector<int> classMembership;
        Template::splitVectorsAndClasses(clusters[0], featureVectors, classMembership);
        LDA lda(featureVectors, classMembership);

        // Train
        double step = (dp.maxScore-dp.minScore)/20;
        double bestThreshold;
        double bestTrainEER = 1.0;
        for (double t = dp.minScore; t <= dp.maxScore; t += step)
        {
            m.w = dp.createSelectionWeights(t);
            Evaluation e(clusters[0], m, false);

            if (e.eer < bestTrainEER)
            {
                bestTrainEER = e.eer;
                bestThreshold = t;
            }
        }
        qDebug() << "train EER:" << bestTrainEER << "; train Threshold:" << bestThreshold;

        // test data
        m.w = dp.createSelectionWeights(bestThreshold);
        BatchEvaluationResult testResults = Evaluation::batch(clusters, m, 1);

        CityblockWeightedMetric m2;
        m2.w = dp.createWeights();
        BatchEvaluationResult testResults2 = Evaluation::batch(clusters, m2, 1);

        for (int i = 0; i < testResults.results.count(); i++)
            qDebug() << "cluster:" << i << "; EER:" << testResults.results[i].eer << testResults2.results[i].eer;
    }
};

#endif // TESTDISCRIMINATIVEPOTENTIAL_H
