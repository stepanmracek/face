#include "faceCommon/linalg/logisticregression.h"

#include <cmath>

#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::LinAlg;

LogisticRegression::LogisticRegression(const std::vector<Vector> &data, const std::vector<int> &classLabels)
{
    learn(data, classLabels);
}

LogisticRegression::LogisticRegression(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    storage["W"] >> w;
}

void LogisticRegression::serialize(const std::string &path) const
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "W" << w;
}

void LogisticRegression::learn(const std::vector<Vector> &data, const std::vector<int> &classLabels)
{
	unsigned int N = data.size();
    if (N != classLabels.size()) throw FACELIB_EXCEPTION("data and classLabels count mismatch");

    // design matrix
    Matrix Phi = createDesignMatrix(data);

    // class labels
    Matrix target = Matrix::zeros(N, 1);
    for (unsigned int i = 0; i < N; i++)
    {
        int label = classLabels[i];
        if ((label != 0) && (label != 1))
            throw FACELIB_EXCEPTION("invalid label " + std::to_string(label) + " at index " + std::to_string(i));
        target(i) = label;
    }

    w = (Phi.t() * Phi).inv() * Phi.t() * target;
}

inline double LogisticRegression::sigma(double x) const
{
    return 1.0 / (1.0 + exp(-x) );
}

/*Matrix LogisticRegression::sigma(const Matrix &x) const
{
    // assuming input column vector
    int M = x.rows;
    ass*rt(M >= 1);
    ass*rt(x.cols == 1);

    Matrix result = Matrix::zeros(M, 1);
    for (int row = 0; row < M; row++)
    {
        result(row) = sigma(x(row));
    }

    return result;
}*/

Matrix LogisticRegression::createDesignMatrix(const std::vector<Vector> &data) const
{
    std::vector<Vector> zeroPadding;
    for (unsigned int i = 0; i < data.size(); i++)
    {
        Vector modified = prependOne(data[i]);
        zeroPadding.push_back(modified);
    }

    return MatrixConverter::columnVectorsToDataMatrix(zeroPadding).t();
}

double LogisticRegression::classify(const Vector &x) const
{
    Vector psi = prependOne(x);
    Matrix product = w.t() * psi;
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
