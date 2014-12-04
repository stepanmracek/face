#include "faceCommon/biometrics/discriminativepotential.h"

#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/metrics.h"
#include "faceCommon/biometrics/template.h"

using namespace Face::Biometrics;

DiscriminativePotential::DiscriminativePotential(const std::vector<Template> &templates)
{
    int n = templates.size();
    if (n == 0) throw FACELIB_EXCEPTION("templates vector is empty");

    int m = templates.at(0).featureVector.rows;
    if (m == 0) throw FACELIB_EXCEPTION("feature vector is empty");

    std::vector<Template> normalizedTemplates = Template::clone(templates);
    Template::zScoreNorm(normalizedTemplates);

    // Calculate DP
    // for each component
    scores = Face::LinAlg::Vector(m);
    for (int i = 0; i < m; i++)
    {
        std::map<int, std::vector<double> > valuesForSubj;

        // for each template
        for (int j = 0; j < n; j++)
        {
            const Template &t = normalizedTemplates.at(j);
            double val = t.featureVector(i);

            // nan guard
            if (val != val)
            {
                std::cerr << "subj " << t.subjectID << " index " << i << " nan " << std::endl;
            }

            valuesForSubj[t.subjectID].push_back(val);
        }

        // DP for specific FV component i:
        // dp(i) = stdDev of class means - mean of std deviation within each class
        std::vector<double> classMeans;
        std::vector<double> stdDeviationForClass;

        for (auto i = valuesForSubj.begin(); i != valuesForSubj.end(); ++i)
        {
            if (i->second.size() <= 1)
        	{
                //qDebug() << "Warning, subject" << subjID << "has only one scan!";
                continue;
        	}

            Face::LinAlg::Vector valuesForSubjVec(i->second);
            classMeans.push_back(valuesForSubjVec.meanValue());
            stdDeviationForClass.push_back(valuesForSubjVec.stdDeviation());
        }

        Face::LinAlg::Vector classMeansVec(classMeans);
        Face::LinAlg::Vector stdDeviationForClassVec(stdDeviationForClass);
        scores(i) = classMeansVec.stdDeviation() - stdDeviationForClassVec.meanValue();
    }

    minScore = scores.minValue();
    maxScore = scores.maxValue();
}

