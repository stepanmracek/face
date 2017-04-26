#include "faceCommon/biometrics/geneticweightoptimization.h"

#include <ctime>

#include "faceCommon/biometrics/template.h"
#include "faceCommon/biometrics/evaluation.h"

using namespace Face::Biometrics;

void mutateForTrainWeights(Face::LinAlg::Vector &vec)
{
    for (int j = 0; j < vec.rows; j++)
    {
        vec(j) += (((double)rand())/RAND_MAX - 0.5);
    }
    Face::LinAlg::WeightedMetric::normalizeWeights(vec);
}

void mutateForFeatureSelection(Face::LinAlg::Vector &vec)
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

Face::LinAlg::Vector createForTrainWeights(int len)
{
    Face::LinAlg::Vector result(len);
    for (int j = 0; j < len; j++)
        result(j) = 1 + (((double)rand())/RAND_MAX - 0.5);
    Face::LinAlg::WeightedMetric::normalizeWeights(result);
    return result;
}

Face::LinAlg::Vector createForFeatureSelection(int len)
{
    Face::LinAlg::Vector result(len);
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

Face::LinAlg::Vector combineForTrainWeights(Face::LinAlg::Vector &first, Face::LinAlg::Vector &second, int crosspoint)
{
    int len = first.rows;
    Face::LinAlg::Vector result(len);

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
    Face::LinAlg::WeightedMetric::normalizeWeights(result);
    return result;
}

Face::LinAlg::Vector combineForFeatureSelection(Face::LinAlg::Vector &first, Face::LinAlg::Vector &second, int crosspoint)
{
    int len = first.rows;
    Face::LinAlg::Vector result(len);

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
        void (*mutate)(Face::LinAlg::Vector &),
        Face::LinAlg::Vector (*create)(int len),
        Face::LinAlg::Vector (*combine)(Face::LinAlg::Vector &, Face::LinAlg::Vector &, int))
{
    // results
    GeneticWeightOptimizationResult result;

    // set seed
    srand(time(NULL));

    // init population
    int fvLen = trainSet.at(0).featureVector.rows;
    std::vector<Face::LinAlg::Vector> population(settings.populationSize);
    std::vector<double> eer(settings.populationSize);
    std::vector<bool> selected(settings.populationSize);
    int selectedCount = settings.populationSize - settings.newGenerationSize;
    std::vector<int> selectedIndicies(selectedCount);
    for (int i = 0; i < settings.populationSize; i++)
    {
        population[i] = create(fvLen);
    }

    double validationEER = 0;
    bool end = false;
    int iteration = 0;
    int iterationsWithoutImprovement = 0;
    int i,j;

    std::vector<Matrix> populationDiversityArray(fvLen);
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
            Evaluation eval(trainSet, metric);
            eer[i] = eval.eer;

            if (eer[i] < result.bestEERonTrain)
            {
                result.bestEERonTrain = eer[i];
                result.bestWeightsOnTrain = Face::LinAlg::Vector(population[i]);
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
        result.iterations.push_back(iteration);
        result.trainEER.push_back(result.bestEERonTrain);
        result.diversity.push_back(diversity);
        //if (iterationsWithoutImprovement == 0)
        //{
            metric.w = result.bestWeightsOnTrain;
            Evaluation eval(validationSet, metric);
            validationEER = eval.eer;

            if (validationEER < result.bestEERonValidation)
            {
                result.bestEERonValidation = validationEER;
                result.bestWeightsOnValidation = Face::LinAlg::Vector(metric.w);
            }
        //}
        result.validationEER.push_back(validationEER);


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
            if (selectedBestIndex == -1) throw FACELIB_EXCEPTION("no best index selected");
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

        std::cout << "iteration: " << iteration << "; current best EER: " << result.bestEERonTrain
                  << "; generations without improvement: " << iterationsWithoutImprovement
                  << "; best EER on validation set: " << result.bestEERonValidation
                  << "; current EER on validation set: " << validationEER
                  << "; population diversity: " << diversity << std::endl;
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
