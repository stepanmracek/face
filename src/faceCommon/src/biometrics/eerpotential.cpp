#include "faceCommon/biometrics/eerpotential.h"

#include "faceCommon/biometrics/template.h"
#include "faceCommon/biometrics/evaluation.h"

using namespace Face::Biometrics;

EERPotential::EERPotential(const std::vector<Template> &templates)
{
    int n = templates.size();
    if (n == 0) throw FACELIB_EXCEPTION("templates vector is empty");

    int m = templates.at(0).featureVector.rows;
    if (m == 0) throw FACELIB_EXCEPTION("feature vector is empty");

    Face::LinAlg::CityblockMetric metrics;

    // Calculate EER potential
    // for each component
    std::vector<double> scoresVec;
    for (int i = 0; i < m; i++)
    {
        //qDebug() << i << "/" << m;;
        std::vector<Template> currentComponentTemplates;

        // for each template
        for (int j = 0; j < n; j++)
        {
            Template t;
            t.subjectID = templates[j].subjectID;
            t.featureVector = Face::LinAlg::Vector(1);
            t.featureVector(0) = templates[j].featureVector(i);
            currentComponentTemplates.push_back(t);
        }

        /*if (i == 27 || i == 63) ?!???!?
        {
            std::vector<double> curValues;
            foreach(const Template &t, currentComponentTemplates)
                curValues << t.featureVector(0);
        }*/

        Evaluation eval(currentComponentTemplates, metrics);
        double score = (eval.eer < 0.5) ? 2.0*(0.5 - eval.eer) : 0.0;
        //score = score*score;
        scoresVec.push_back(score);
    }

    scores = Face::LinAlg::Vector(scoresVec);

    minScore = scores.minValue();
    maxScore = scores.maxValue();

    //qDebug() << "...done";
}
