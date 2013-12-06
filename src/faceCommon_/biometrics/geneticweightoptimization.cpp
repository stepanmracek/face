#include "geneticweightoptimization.h"

#include <ctime>

void mutateForTrainWeights(Vector &vec)
{
    for (int j = 0; j < vec.rows; j++)
    {
        vec(j) += (((double)rand())/RAND_MAX - 0.5);
    }
    WeightedMetric::normalizeWeights(vec);
}

void mutateForFeatureSelection(Vector &vec)
{
    for (int j = 0; j < vec.rows; j++)
    {
        double p = ((double)rand())/RAND_MAX;
        if (p < 0.5)
        {
            double val = vec(j);
            vec(j) = 1 - val;
        }
    }
}

Vector createForTrainWeights(int len)
{
    Vector result(len);
    for (int j = 0; j < len; j++)
        result(j) = 1 + (((double)rand())/RAND_MAX - 0.5);
    WeightedMetric::normalizeWeights(result);
    return result;
}

Vector createForFeatureSelection(int len)
{
    Vector result(len);
    for (int j = 0; j < len; j++)
    {
        double p = ((double)rand())/RAND_MAX;
        if (p < 0.5)
            result(j) = 0;
        else
            result(j) = 1;
    }
    return result;
}

Vector combineForTrainWeights(Vector &first, Vector &second, int crosspoint)
{
    int len = first.rows;
    Vector result(len);

    for (int j = 0; j < len; j++)
    {
        if (j < crosspoint)
        {
            // from first
            result(j) = first(j);
        }
        else
        {
            // from second
            result(j) = second(j);
        }
    }

    // normalize;
    WeightedMetric::normalizeWeights(result);
    return result;
}

Vector combineForFeatureSelection(Vector &first, Vector &second, int crosspoint)
{
    int len = first.rows;
    Vector result(len);

    for (int j = 0; j < len; j++)
    {
        if (j < crosspoint)
        {
            // from first
            result(j) = first(j);
        }
        else
        {
            // from second
            result(j) = second(j);
        }
    }
    return result;
}

GeneticWeightOptimizationResult GeneticWeightOptimization::trainCommon(
        void (*mutate)(Vector &),
        Vector (*create)(int len),
        Vector (*combine)(Vector &, Vector &, int))
{
    assert(trainSet.count() > 0);

    // results
    GeneticWeightOptimizationResult result;

    // set seed
    srand(time(NULL));

    // init population
    int fvLen = trainSet.at(0).featureVector.rows;
    QVector<Vector> population(settings.populationSize);
    QVector<double> eer(settings.populationSize);
    QVector<bool> selected(settings.populationSize);
    int selectedCount = settings.populationSize - settings.newGenerationSize;
    QVector<int> selectedIndicies(selectedCount);
    for (int i = 0; i < settings.populationSize; i++)
    {
        population[i] = create(fvLen);
    }

    double validationEER = 0;
    bool end = false;
    int iteration = 0;
    int iterationsWithoutImprovement = 0;
    int i,j;

    QVector<Matrix> populationDiversityArray(fvLen);
    for (i = 0; i < fvLen; i++)
    {
        populationDiversityArray[i] = Matrix::zeros(settings.populationSize, 1);
    }

    while (!end)
    {
        // evaluate population
        iterationsWithoutImprovement++;
        for (i = 0; i < settings.populationSize; i++)
        {
            metric.w = population[i];
            Evaluation eval(trainSet, metric, false);
            eer[i] = eval.eer;

            if (eer[i] < result.bestEERonTrain)
            {
                result.bestEERonTrain = eer[i];
                result.bestWeightsOnTrain = Vector(population[i]);
                iterationsWithoutImprovement = 0;
            }

            // calculate population diversity
            for (j = 0; j < fvLen; j++)
            {
                populationDiversityArray[j](i) = population[i](j);
            }
        }

        double diversity = 0;
        for (j = 0; j < fvLen; j++)
        {
            cv::Scalar std; cv::Scalar mean;
            cv::meanStdDev(populationDiversityArray[j], mean, std);
            diversity += std[0];
        }
        diversity /= fvLen;

        // output results
        result.iterations.append(iteration);
        result.trainEER.append(result.bestEERonTrain);
        result.diversity.append(diversity);
        //if (iterationsWithoutImprovement == 0)
        //{
            metric.w = result.bestWeightsOnTrain;
            Evaluation eval(validationSet, metric, false);
            validationEER = eval.eer;

            if (validationEER < result.bestEERonValidation)
            {
                result.bestEERonValidation = validationEER;
                result.bestWeightsOnValidation = Vector(metric.w);
            }
        //}
        result.validationEER.append(validationEER);


        // select only the best
        int selectedBestIndex;
        double selectedBestEER;
        for (i = 0; i < settings.populationSize; i++)
            selected[i] = false;
        for (j = 0; j < selectedCount; j++)
        {
            selectedBestIndex = -1;
            selectedBestEER = 1.0;
            for (i = 0; i < settings.populationSize; i++)
            {
                if (eer[i] < selectedBestEER)
                {
                    selectedBestEER = eer[i];
                    selectedBestIndex = i;
                }
            }
            assert(selectedBestIndex != -1);

            selected[selectedBestIndex] = true;
        }
        j = 0;
        for (i = 0; i < settings.populationSize; i++)
        {
            if (selected[i])
            {
                selectedIndicies[j] = i;
                j++;
            }
        }

        // cross
        for (i = 0; i < settings.populationSize; i++)
        {
            if (!selected[i])
            {
                // create new population member from two another selected members
                int first = selectedIndicies[rand() % selectedCount];
                int second = selectedIndicies[rand() % selectedCount];
                // crossing point
                int cpoint = rand() % fvLen;

                population[i] = combine(population[first], population[second], cpoint);
            }
        }

        // mutate
        for (i = 0; i < settings.populationSize; i++)
        {
            double p = ((double)rand())/RAND_MAX;
            if (p < settings.mutationProbability)
            {
                mutate(population[i]);
            }
        }


        // end?
        iteration++;
        if ((iteration >= settings.maxIterations) || (iterationsWithoutImprovement >= settings.maximumIterationsWithoutImprovement))
        {
            end = true;
        }

        qDebug() << "iteration:" << iteration << "; current best EER:" << result.bestEERonTrain
                 << "; generations without improvement" << iterationsWithoutImprovement
                 << "; best EER on validation set" << result.bestEERonValidation
                 << "; current EER on validation set" << validationEER
                 << "; population diversity" << diversity;
    }

    metric.w = result.bestWeightsOnTrain;
    return result;
}

GeneticWeightOptimizationResult GeneticWeightOptimization::trainFeatureSelection()
{
    return trainCommon(mutateForFeatureSelection, createForFeatureSelection, combineForFeatureSelection);
}

GeneticWeightOptimizationResult GeneticWeightOptimization::trainWeights()
{
    return trainCommon(mutateForTrainWeights, createForTrainWeights, combineForTrainWeights);
}
