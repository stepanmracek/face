#include "featureselection.h"

#include <cassert>

#include "evaluation.h"
#include "linalg/common.h"

/*Matrix appendIndex(Matrix &source, Matrix &target, int index)
{
    Matrix result(target.rows+1, 1, CV_64F);
    for (int r = 0; r < target.rows; r++)
        result(r) = target(r);
    result(target.rows) = source(index);

    return result;
}*/

FeatureSelection::FeatureSelection(QVector<Template> &trainTemplates,
                                   WeightedMetric &metrics,
                                   QList<QVector<Template> > *validationTemplates)
{
    qDebug() << "Feature Selection";

    //bestValidationEER = 1.0;
    //bestValidationStep = -1;
    bestTrainEER = 1.0;
    bestTrainStep = -1;

    int n = trainTemplates.count();
    assert(n > 1);

    QVector<int> availableIndicies;
    for (int i = 0; i < trainTemplates[0].featureVector.rows; i++)
        availableIndicies.append(i);
    Matrix weights = Matrix::zeros(trainTemplates[0].featureVector.rows, 1);

    int candidatesCount;
    int step = 1;
    //bool improved = true;
    while ((candidatesCount = availableIndicies.count()) > 0) // || !improved)
    {
        int indexToRemove = -1;
        double bestCandidateEER = 1;
        int bestCandidateIndex;

        // for each candidate
        for (int i = 0; i < candidatesCount; i++)
        {
            // create candidate
            Matrix candidateWeight = weights.clone();
            candidateWeight(availableIndicies.at(i)) = 1.0;
            metrics.w = candidateWeight;

            // evaluate candidate
            Evaluation e(trainTemplates, metrics, false);
            if (e.eer < bestCandidateEER)
            {
                indexToRemove = i;
                bestCandidateEER = e.eer;
                bestCandidateIndex = availableIndicies.at(i);
            }
        }

        // append best index to the bestCandidate
        availableIndicies.remove(indexToRemove);
        selectedIndicies.append(bestCandidateIndex);

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
