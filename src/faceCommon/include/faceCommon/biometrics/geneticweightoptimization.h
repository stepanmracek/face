#ifndef GENETICWEIGHTOPTIMIZATION_H
#define GENETICWEIGHTOPTIMIZATION_H

#include "faceCommon/linalg/metrics.h"
#include "faceCommon/linalg/vector.h"

namespace Face {
namespace Biometrics {

class Template;
class Evaluation;

class GeneticWeightOptimizationSettings
{
public:

    GeneticWeightOptimizationSettings(int maxIterations = 1000, int populationSize = 50,
                                      int newGenerationSize = 30, double mutationProbability = 0.1,
                                      int maximumIterationsWithoutImprovement = 100):
        maxIterations(maxIterations), populationSize(populationSize), newGenerationSize(newGenerationSize),
        mutationProbability(mutationProbability), maximumIterationsWithoutImprovement(maximumIterationsWithoutImprovement)
    {
        if ((maxIterations <= 0) ||
            (populationSize <= 0) ||
            (newGenerationSize <= 0) ||
            (populationSize <= newGenerationSize) ||
            (mutationProbability < 0) ||
            (mutationProbability >= 1) ||
            (maximumIterationsWithoutImprovement <= 0))
            throw FACELIB_EXCEPTION("invalid parameters");
    }

    int maxIterations;
    int populationSize;
    int newGenerationSize;
    double mutationProbability;
    int maximumIterationsWithoutImprovement;
};

class GeneticWeightOptimizationResult
{
public:
    GeneticWeightOptimizationResult()
    {
        bestEERonTrain = 1.0;
        bestEERonValidation = 1.0;
    }

    std::vector<double> iterations;
    std::vector<double> trainEER;
    std::vector<double> validationEER;
    std::vector<double> diversity;
    Face::LinAlg::Vector bestWeightsOnTrain;
    Face::LinAlg::Vector bestWeightsOnValidation;
    double bestEERonTrain;
    double bestEERonValidation;
};

class GeneticWeightOptimization
{
public:
	GeneticWeightOptimization(std::vector<Template> &trainSet, std::vector<Template> &validationSet,
            Face::LinAlg::WeightedMetric &metric, GeneticWeightOptimizationSettings settings) :
	        trainSet(trainSet), validationSet(validationSet), metric(metric), settings(settings) {}

	std::vector<Template> &trainSet;
	std::vector<Template> &validationSet;
    Face::LinAlg::WeightedMetric &metric;
	GeneticWeightOptimizationSettings settings;

    GeneticWeightOptimizationResult trainWeights();
    GeneticWeightOptimizationResult trainFeatureSelection();

private:
    GeneticWeightOptimizationResult trainCommon(void (*mutate)(Face::LinAlg::Vector &vec),
                                                Face::LinAlg::Vector (*create)(int len),
                                                Face::LinAlg::Vector (*combine)(Face::LinAlg::Vector &first, Face::LinAlg::Vector &second, int crossPoint));
};

}
}

#endif // GENETICWEIGHTOPTIMIZATION_H
