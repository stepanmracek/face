#include "faceCommon/biometrics/scorelevelfusionwrapper.h"

using namespace Face::Biometrics;

ScoreLevelFusionWrapper::Result ScoreLevelFusionWrapper::trainClassifier(const std::string &fusionName,
                                                                         const std::vector<Evaluation> &components,
                                                                         bool debugOutput)
{
    Result result;
    int n = components.size();

    if (debugOutput)
        std::cout << "Training score-level fusion classifier: " << n << " candidate components" << std::endl;

    // select the best component as the base for the classifier
    double bestEER = 1;
    int bestIndex = -1;
    for (int i = 0; i < n; i++)
    {
        double eer = components[i].eer;
        if (eer < bestEER)
        {
            bestEER = eer;
            bestIndex = i;
        }
    }

    if (bestIndex == -1) throw FACELIB_EXCEPTION("no bestIndex selected");
    result.selectedComponents.push_back(bestIndex);

    if (debugOutput)
        std::cout << "Selected " << bestIndex << " with EER " << bestEER << " as the base for the classifier" << std::endl;

    // iteratively add the best remaining components while there is some improvement
    bool improvement = true;
    while (improvement)
    {
        improvement = false;
        bestIndex = -1;

        std::vector<double> eers(n, 1.0);
        #pragma omp parallel for
        for (int i = 0; i < n; i++)
        {
            // if the component was already selected skip it
            if (std::find(result.selectedComponents.begin(), result.selectedComponents.end(), i) != result.selectedComponents.end()) continue;

            // relearn the classifier using all current selected components...
            ScoreLevelFusionBase::Ptr fusion = ScoreLevelFusionFactory::create(fusionName);
            for (int c : result.selectedComponents)
            {
                fusion->addComponent(components[c]);
            }
            // ...and the candidate one
            fusion->addComponent(components[i]);
            fusion->learn();

            // create the input
            std::vector<Evaluation> inputEvaluationResults;
            for (int component : result.selectedComponents)
            {
                inputEvaluationResults.push_back(components[component]);
            }
            inputEvaluationResults.push_back(components[i]);

            // evaluate
            double eer = fusion->evaluate(inputEvaluationResults).eer;
            eers[i] = eer;
            //if (debugOutput)
            //    qDebug() << "  trying to add classifier" << i << ", fusion EER:" << eer;
        }

        for (int i = 0; i < n; i++)
        {
            double eer = eers[i];
            if (eer < bestEER)
            {
                bestEER = eer;
                improvement = true;
                bestIndex = i;
            }
        }

        if (improvement)
        {
            result.selectedComponents.push_back(bestIndex);
            if (debugOutput)
                std::cout << "added classifier " << bestIndex << ", fusion EER: " << bestEER << std::endl;
        }
    }

    result.fusion = ScoreLevelFusionFactory::create(fusionName);
    for (int c : result.selectedComponents)
    {
        result.fusion->addComponent(components[c]);
    }
    result.fusion->learn();

    if (debugOutput)
    {
        std::cout << "final result {";
        for (int c : result.selectedComponents)
            std::cout << c << " ";

        std::cout << "} EER: " << bestEER << std::endl;
    }

    return result;
}
