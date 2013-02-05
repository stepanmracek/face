#include "logisticregression.h"

#include <QDebug>

#include <cmath>
#include <cassert>

#include "matrixconverter.h"

LogisticRegression::LogisticRegression(QVector<Matrix> &data, QVector<int> &classLabels)
{
    learn(data, classLabels);
}

void LogisticRegression::learn(QVector<Matrix> &data, QVector<int> &classLabels)
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

double LogisticRegression::sigma(double x)
{
    double s =  1.0 / (1.0 + exp(-x) );
    return s;
}

Matrix LogisticRegression::sigma(Matrix x)
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
}

Matrix LogisticRegression::createDesignMatrix(QVector<Matrix> &data)
{
    QVector<Matrix> zeroPadding;
    for (int i = 0; i < data.count(); i++)
    {
        Matrix modified = prependOne(data[i]);
        zeroPadding << modified;
    }

    return MatrixConverter::columnVectorsToDataMatrix(zeroPadding).t();
}

double LogisticRegression::classify(Matrix x)
{
    Matrix psi = prependOne(x);
    Matrix product = w.t() * psi;
    return sigma(product(0));
}

Matrix LogisticRegression::prependOne(Matrix &in)
{
    int M = in.rows;
    Matrix modified = Matrix::ones(M+1, 1);
    for (int r = 0; r < M; r++)
    {
        modified(r+1) = in(r);
    }
    return modified;
}
