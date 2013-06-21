#include "eerpotential.h"

#include <QDebug>

#include <cassert>

#include "evaluation.h"

EERPotential::EERPotential(QVector<Template> &templates)
{
    //qDebug() << "EER potential evaluation...";
    int n = templates.count();
    assert(n > 0);

    int m = templates.at(0).featureVector.rows;
    assert(m > 0);

    CityblockMetric metrics;

    // Calculate EER potential
    // for each component
    QVector<double> scoresQVec;
    for (int i = 0; i < m; i++)
    {
        //qDebug() << i << "/" << m;;
        QVector<Template> currentComponentTemplates;

        // for each template
        for (int j = 0; j < n; j++)
        {
            Template t;
            t.subjectID = templates[j].subjectID;
            t.featureVector = Vector(1);
            t.featureVector(0) = templates[j].featureVector(i);
            currentComponentTemplates.append(t);
        }

        if (i == 27 || i == 63)
        {
            QVector<double> curValues;
            foreach(const Template &t, currentComponentTemplates)
                curValues << t.featureVector(0);
        }

        Evaluation eval(currentComponentTemplates, metrics, false);
        double score = (eval.eer < 0.5) ? 2.0*(0.5 - eval.eer) : 0.0;
        //score = score*score;
        scoresQVec.append(score);
    }

    scores = Vector(scoresQVec);

    minScore = scores.minValue();
    maxScore = scores.maxValue();

    //qDebug() << "...done";
}
