#include "logisticregression.h"

#include <QDebug>

#include <cmath>
#include <cassert>

#include "matrixconverter.h"

LogisticRegression::LogisticRegression(const QVector<Vector> &data, const QVector<int> &classLabels)
{
    learn(data, classLabels);
}

void LogisticRegression::learn(const QVector<Vector> &data, const QVector<int> &classLabels)
{
    // initial assertion
    int N = data.count();
    assert(N == classLabels.count());

    // design matrix
    Matrix Phi = createDesignMatrix(data);

    // class labels
    Matrix target = Matrix::zeros(N, 1);
    for (int i = 0; i < N; i++)
    {
        int label = classLabels[i];
        assert(label == 0 || label == 1);
        target(i) = label;
    }

    w = (Phi.t() * Phi).inv() * Phi.t() * target;
}

double LogisticRegression::sigma(double x) const
{
    double s =  1.0 / (1.0 + exp(-x) );
    return s;
}

/*Matrix LogisticRegression::sigma(const Matrix &x) const
{
    // assuming input column vector
    int M = x.rows;
    assert(M >= 1);
    assert(x.cols == 1);

    Matrix result = Matrix::zeros(M, 1);
    for (int row = 0; row < M; row++)
    {
        result(row) = sigma(x(row));
    }

    return result;
}*/

Matrix LogisticRegression::createDesignMatrix(const QVector<Vector> &data) const
{
    QVector<Vector> zeroPadding;
    for (int i = 0; i < data.count(); i++)
    {
        Vector modified = prependOne(data[i]);
        zeroPadding << modified;
    }

    return MatrixConverter::columnVectorsToDataMatrix(zeroPadding).t();
}

double LogisticRegression::classify(const Vector &x) const
{
    Vector psi = prependOne(x);
    Matrix product = w.t() * psi;
    assert (product.rows == 1);
    assert (product.cols == 1);
    return sigma(product(0));
}

Vector LogisticRegression::prependOne(const Vector &in) const
{
    int M = in.rows;
    Vector modified(M+1);
    modified(0) = 1;
    for (int r = 0; r < M; r++)
    {
        modified(r+1) = in(r);
    }
    return modified;
}
