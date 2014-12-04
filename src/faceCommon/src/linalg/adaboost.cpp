#include "faceCommon/linalg/adaboost.h"

using namespace Face::LinAlg;

void AdaBoost::learn(std::vector<Vector> &trainVectors, std::vector<int> &labels)
{
    unsigned int n = trainVectors.size();
    if (n != labels.size() || n <= 1)
    {
    	throw FACELIB_EXCEPTION("input trainVectors count mismatch");
    }

    std::cout << "training weak classifiers..." << std::endl;
    trainWeakClassifiers(trainVectors, labels);
    std::cout << "initiating weights..." << std::endl;
    initiateWeights(labels);

    int T = 10;
    for (int t = 0; t < T; t++)
    {
        std::cout << "normalizing weights" << std::endl;
        normalizeWeights();

        std::cout << "selecting best classifier" << std::endl;
        int bestWeak; double error;
        getBestWeakClassifier(trainVectors, labels, bestWeak, error);
        hIndicies.push_back(bestWeak);

        std::cout << "updating weights" << std::endl;
        double beta = updateWeights(trainVectors, labels, bestWeak, error);
        alpha.push_back(log(1.0 / beta));

        std::cout << "t: " << t << "classifier: " << bestWeak << "error: " << error << std::endl;
    }
}

void AdaBoost::trainWeakClassifiers(std::vector<Vector> &trainVectors, std::vector<int> &labels)
{
    int n = trainVectors.size();
    int wCount = weakCount();
    weakClassifiers.resize(wCount);

    for (int classifier = 0; classifier < wCount; classifier++)
    {
        std::vector<Vector> trainData;
        for (int i = 0; i < n; i++)
        {
            trainData.push_back(createWeakInput(classifier, trainVectors[i]));
        }
        weakClassifiers[classifier].learn(trainData, labels);
    }
}

void AdaBoost::initiateWeights(std::vector<int> &labels)
{
    int n = labels.size();
    weights.resize(n);

    int m = std::count(labels.begin(), labels.end(), 0); // count of negatives
    int l = std::count(labels.begin(), labels.end(), 1); // count of positives
    assert(m > 0);
    assert(l > 0);

    for (int i = 0; i < n; i++)
    {
        int label = labels[i];
        assert((label == 0) || (label == 1));
        weights[i] = label ? 1.0/l : 1.0/m;
    }
}

void AdaBoost::normalizeWeights()
{
    int n = weights.size();
    double sum = 0.0;
    for (double w : weights)
    {
        sum += w;
    }
    for (int i = 0; i < n; i++)
    {
        weights[i] = weights[i]/sum;
    }
}

void AdaBoost::getBestWeakClassifier(std::vector<Vector> &trainVectors, std::vector<int> &labels, int &index, double &error)
{
    int n = trainVectors.size();
    int wCount = weakCount();

    index = -1;
    error = 1e300;
    for (int classifier = 0; classifier < wCount; classifier++)
    {
        double e = 0.0;
        for (int i = 0; i < n; i++)
        {
            e += weights[i] * fabs(h(classifier, trainVectors[i]) - labels[i]);
        }

        //qDebug() << e;

        if (e < error)
        {
            error = e;
            index = classifier;
        }
    }
}

double AdaBoost::updateWeights(std::vector<Vector> &trainVectors, std::vector<int> &labels, int classifier, double error)
{
    double beta = error / (1 - error);

    int n = trainVectors.size();
    for (int i = 0; i < n; i++)
    {
        double classifierResult = h(classifier, trainVectors[i]);
        double e;
        if ((classifierResult < 0.5 && labels[i] == 0) || (classifierResult >= 0.5 && labels[i] == 1))
            e = 0.0; // good classification
        else
            e = 1.0; // wrong classification
        weights[i] = weights[i] * pow(beta, 1-e);
    }

    return beta;
}

double AdaBoost::h(int classifier, Vector &input)
{
    Vector weakInput = createWeakInput(classifier, input);
    return weakClassifiers[classifier].classify(weakInput);
}

double AdaBoost::classify(Vector &input)
{
    double sum = 0.0;
    for (int classifier : hIndicies)
    {
        sum += alpha[classifier] * h(classifier, input);
    }

    return sum;
}
