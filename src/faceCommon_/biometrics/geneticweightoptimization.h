#ifndef GENETICWEIGHTOPTIMIZATION_H
#define GENETICWEIGHTOPTIMIZATION_H

#include <QVector>

#include "linalg/metrics.h"
#include "evaluation.h"
#include "template.h"
#include "linalg/vector.h"

class GeneticWeightOptimizationSettings
{
public:

    GeneticWeightOptimizationSettings(int maxIterations = 1000, int populationSize = 50,
                                      int newGenerationSize = 30, double mutationProbability = 0.1,
                                      int maximumIterationsWithoutImprovement = 100):
        maxIterations(maxIterations), populationSize(populationSize), newGenerationSize(newGenerationSize),
        mutationProbability(mutationProbability), maximumIterationsWithoutImprovement(maximumIterationsWithoutImprovement)
    {
        assert(maxIterations > 0);
        assert(populationSize > 0);
        assert(newGenerationSize > 0);
        assert(populationSize > newGenerationSize);
        assert(mutationProbability >= 0);
        assert(mutationProbability < 1);
        assert(maximumIterationsWithoutImprovement > 0);
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

    QVector<double> iterations;
    QVector<double> trainEER;
    QVector<double> validationEER;
    QVector<double> diversity;
    Vector bestWeightsOnTrain;
    Vector bestWeightsOnValidation;
    double bestEERonTrain;
    double bestEERonValidation;
};

class GeneticWeightOptimization
{
public:
	GeneticWeightOptimization(QVector<Template> &trainSet, QVector<Template> &validationSet,
			WeightedMetric &metric, GeneticWeightOptimizationSettings settings) :
	        trainSet(trainSet), validationSet(validationSet), metric(metric), settings(settings) {}

	QVector<Template> &trainSet;
	QVector<Template> &validationSet;
	WeightedMetric &metric;
	GeneticWeightOptimizationSettings settings;

    GeneticWeightOptimizationResult trainWeights();
    GeneticWeightOptimizationResult trainFeatureSelection();

private:
    GeneticWeightOptimizationResult trainCommon(void (*mutate)(Vector &vec),
                                                Vector (*create)(int len),
                                                Vector (*combine)(Vector &first, Vector &second, int crossPoint));
};

#endif // GENETICWEIGHTOPTIMIZATION_H
