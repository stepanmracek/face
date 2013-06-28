#include "adaboost.h"

void AdaBoost::learn(QVector<Vector> &trainVectors, QVector<int> &labels)
{
    int n = trainVectors.count();
    assert(n == labels.count());
    assert(n > 1);

    qDebug() << "training weak classifiers...";
    trainWeakClassifiers(trainVectors, labels);
    qDebug() << "initiating weights...";
    initiateWeights(labels);

    int T = 10;
    for (int t = 0; t < T; t++)
    {
        qDebug() << "normalizing weights";
        normalizeWeights();

        qDebug() << "selecting best classifier";
        int bestWeak; double error;
        getBestWeakClassifier(trainVectors, labels, bestWeak, error);
        hIndicies << bestWeak;

        qDebug() << "updating weights";
        double beta = updateWeights(trainVectors, labels, bestWeak, error);
        alpha << log(1.0 / beta);

        qDebug() << "t:" << t << "classifier:" << bestWeak << "error:" << error;
    }
}

void AdaBoost::trainWeakClassifiers(QVector<Vector> &trainVectors, QVector<int> &labels)
{
    int n = trainVectors.count();
    int wCount = weakCount();
    weakClassifiers.resize(wCount);

    for (int classifier = 0; classifier < wCount; classifier++)
    {
        QVector<Vector> trainData;
        for (int i = 0; i < n; i++)
        {
            trainData << createWeakInput(trainVectors[i]);
        }

        weakClassifiers[classifier].learn(trainData, labels);
    }
}

void AdaBoost::initiateWeights(QVector<int> &labels)
{
    int n = labels.count();
    weights.resize(n);

    int m = labels.count(0); // count of negatives
    int l = labels.count(1); // count of positives
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
    int n = weights.count();
    double sum = 0.0;
    foreach(double w, weights)
    {
        sum += w;
    }
    for (int i = 0; i < n; i++)
    {
        weights[i] = weights[i]/sum;
    }
}

void AdaBoost::getBestWeakClassifier(QVector<Vector> &trainVectors, QVector<int> &labels, int &index, double &error)
{
    int n = trainVectors.count();
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

        qDebug() << e;

        if (e < error)
        {
            error = e;
            index = classifier;
        }
    }
}

double AdaBoost::updateWeights(QVector<Vector> &trainVectors, QVector<int> &labels, int classifier, double error)
{
    double beta = error / (1 - error);

    int n = trainVectors.count();
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
    Vector weakInput = createWeakInput(input);
    return weakClassifiers[classifier].classify(weakInput);
}

double AdaBoost::classify(Vector &input)
{
    double sum = 0.0;
    foreach(int classifier, hIndicies)
    {
        sum += alpha[classifier] * h(classifier, input);
    }
}
