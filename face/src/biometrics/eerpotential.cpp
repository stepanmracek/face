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
    for (int i = 0; i < m; i++)
    {
    	//qDebug() << i << "/" << m;;
        QVector<Template> currentComponentTemplates;

        // for each template
        for (int j = 0; j < n; j++)
        {
            Template t;
            t.subjectID = templates[j].subjectID;
            t.featureVector = Matrix::zeros(1, 1);
            t.featureVector(0) = templates[j].featureVector(i);
            currentComponentTemplates.append(t);
        }

        Evaluation eval(currentComponentTemplates, metrics, false);
        double score = (1.0 - eval.eer);
        scores.append(score);
    }

    minScore = Vector::minValue(scores);
    maxScore = Vector::maxValue(scores);

    //qDebug() << "...done";
}
