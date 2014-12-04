#ifndef ADABOOST_H
#define ADABOOST_H

#include "common.h"
#include "vector.h"
#include "logisticregression.h"

namespace Face {
namespace LinAlg {

class AdaBoost
{
private:
    void trainWeakClassifiers(std::vector<Vector> &trainVectors, std::vector<int> &labels);
    void initiateWeights(std::vector<int> &labels);
    void normalizeWeights();
    void getBestWeakClassifier(std::vector<Vector> &trainVectors, std::vector<int> &labels, int &index, double &error);
    double updateWeights(std::vector<Vector> &trainVectors, std::vector<int> &labels, int classifier, double error);
    double h(int classifier, Vector &input);
    std::vector<LogisticRegression> weakClassifiers;
    std::vector<double> alpha;
    std::vector<int> hIndicies;
    std::vector<double> weights;

    virtual int weakCount() = 0;
    virtual Vector createWeakInput(int weakClassifierIndex, Vector &input) = 0;

public:
    void learn(std::vector<Vector> &trainVectors, std::vector<int> &labels);

    double classify(Vector &input);
};

class AdaBoostSimple : public AdaBoost
{
public:
    AdaBoostSimple(int stripes, int bins)
    {
        wCount = stripes*bins;
    }

private:
    int wCount;
    int weakCount() { return wCount; }

    Vector createWeakInput(int weakClassifierIndex, Vector &input)
    {
        int rows = input.rows;
        Matrix first = input.rowRange(0, rows/2);
        Matrix second = input.rowRange(rows/2, rows);
        Matrix diff = first - second;
        Vector result(1);
        result(0) = fabs(diff(weakClassifierIndex));
        return result;
    }
};

}
}

#endif // ADABOOST_H
