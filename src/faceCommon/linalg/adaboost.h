#ifndef ADABOOST_H
#define ADABOOST_H

#include <QVector>
#include <assert.h>

#include "common.h"
#include "vector.h"
#include "logisticregression.h"

class AdaBoost
{
private:
    void trainWeakClassifiers(QVector<Vector> &trainVectors, QVector<int> &labels);
    void initiateWeights(QVector<int> &labels);
    void normalizeWeights();
    void getBestWeakClassifier(QVector<Vector> &trainVectors, QVector<int> &labels, int &index, double &error);
    double updateWeights(QVector<Vector> &trainVectors, QVector<int> &labels, int classifier, double error);
    double h(int classifier, Vector &input);
    QVector<LogisticRegression> weakClassifiers;
    QVector<double> alpha;
    QVector<int> hIndicies;
    QVector<double> weights;

    virtual int weakCount() = 0;
    virtual Vector createWeakInput(Vector &input) = 0;

public:
    void learn(QVector<Vector> &trainVectors, QVector<int> &labels);

    double classify(Vector &input);
};

class AdaBoostSimple : public AdaBoost
{
private:
    int weakCount() { return 1; }
    Vector createWeakInput(Vector &input)
    {
        int rows = input.rows;
        Matrix first = input.rowRange(0, rows/2);
        Matrix second = input.rowRange(rows/2, rows);
        return first - second;
    }
};

#endif // ADABOOST_H
