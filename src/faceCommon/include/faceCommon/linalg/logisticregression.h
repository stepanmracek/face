#ifndef LOGISTICREGRESSION_H
#define LOGISTICREGRESSION_H

#include "common.h"
#include "vector.h"

namespace Face {
namespace LinAlg {

class LogisticRegression
{
private:
    Vector prependOne(const Vector &in) const;

public:
    LogisticRegression() {}
    LogisticRegression(const std::vector<Vector> &data, const std::vector<int> &classLabels);
    LogisticRegression(const std::string &path);

    void learn(const std::vector<Vector> &data, const std::vector<int> &classLabels);
    void serialize(const std::string &path) const;

    Matrix w;

    inline double sigma(double x) const;
    //Matrix sigma(const Matrix &x) const;

    Matrix createDesignMatrix(const std::vector<Vector> &data) const;

    double classify(const Vector &x) const;
};

}
}

#endif // LOGISTICREGRESSION_H
