#include "faceCommon/biometrics/featureselection.h"

#include "faceCommon/biometrics/evaluation.h"
#include "faceCommon/biometrics/template.h"

using namespace Face::Biometrics;

/*Matrix appendIndex(Matrix &source, Matrix &target, int index)
{
    Matrix result(target.rows+1, 1, CV_64F);
    for (int r = 0; r < target.rows; r++)
        result(r) = target(r);
    result(target.rows) = source(index);

    return result;
}*/

FeatureSelection::FeatureSelection(std::vector<Template> &trainTemplates,
                                   Face::LinAlg::WeightedMetric &metrics,
                                   std::vector<std::vector<Template> > *validationTemplates)
{
    std::cout << "Feature Selection" << std::endl;

    //bestValidationEER = 1.0;
    //bestValidationStep = -1;
    bestTrainEER = 1.0;
    bestTrainStep = -1;

    int n = trainTemplates.size();
    if (n == 0) throw FACELIB_EXCEPTION("trainTemplates vector is empty");

    std::vector<int> availableIndicies;
    for (int i = 0; i < trainTemplates[0].featureVector.rows; i++)
        availableIndicies.push_back(i);
    Face::LinAlg::Vector weights(trainTemplates[0].featureVector.rows);

    int candidatesCount;
    int step = 1;
    //bool improved = true;
    while ((candidatesCount = availableIndicies.size()) > 0) // || !improved)
    {
        int indexToRemove = -1;
        double bestCandidateEER = 1;
        int bestCandidateIndex;

        // for each candidate
        for (int i = 0; i < candidatesCount; i++)
        {
            // create candidate
            Face::LinAlg::Vector candidateWeight = weights;
            candidateWeight(availableIndicies.at(i)) = 1.0;
            metrics.w = candidateWeight;

            // evaluate candidate
            Evaluation e(trainTemplates, metrics);
            if (e.eer < bestCandidateEER)
            {
                indexToRemove = i;
                bestCandidateEER = e.eer;
                bestCandidateIndex = availableIndicies.at(i);
            }
        }

        // append best index to the bestCandidate
        availableIndicies.erase(availableIndicies.begin() + indexToRemove); //.remove(indexToRemove);
        selectedIndicies.push_back(bestCandidateIndex);

        weights(bestCandidateIndex) = 1;
        metrics.w = weights;

        if (bestCandidateEER <= bestTrainEER)
        {
            bestTrainEER = bestCandidateEER;
            bestTrainWeight = weights.clone();
            bestTrainStep = step;
        }
        //else
        //{
        //    break;
        //}

        if (validationTemplates != 0)
        {
            BatchEvaluationResult batchResult = Evaluation::batch(*validationTemplates, metrics);
            //qDebug() << step << bestCandidateEER << "validation:" << batchResult.meanEER << batchResult.stdDevOfEER;
        }
        //else
        //{
        //    //qDebug() << step << bestCandidateEER;
        //}

        step++;
    }
}
