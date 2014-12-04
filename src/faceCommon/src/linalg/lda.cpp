#include "faceCommon/linalg/lda.h"

using namespace Face::LinAlg;

LDA::LDA()
{

}

void LDA::serialize(const std::string &path) const
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "Wt" << Wt;
    storage << "mean" << mean;
}

LDA::LDA(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    storage["Wt"] >> Wt;
    storage["mean"] >> mean;
}

LDA::LDA(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, bool debug)
{
    learn(vectors, classMembership, debug);
}

void LDA::learn(const std::vector<Vector> &vectors, const std::vector<int> &classMembership, bool debug)
{
    if (debug) std::cout << "LDA" << std::endl;

    // number of all vectors;
    unsigned int N = vectors.size();
    if (N != (classMembership.size())) throw FACELIB_EXCEPTION("vectors and classMembership count mismatch");

    int s = vectors[0].rows;

    // number of classes
    int K = 0;
    std::map<int, int> classCounter;
    std::set<int> classes;

    // class means
    mean = Matrix::zeros(s, 1);
    std::map<int, Matrix> means;
    std::map<int, std::vector<int> > classToVector;

    for (unsigned int i = 0; i < N; i++)
    {
        int c = classMembership[i];
        classes.insert(c);
        if (classCounter.count(c) == 0)
        {
            K++;
            classCounter[c] = 0;
            means[c] = Matrix::zeros(s, 1);
        }

        classCounter[c] += 1;
        means[c] += vectors[i];
        classToVector[c].push_back(i);
        mean += vectors[i];
    }

    if (debug) std::cout << "  classes: " << K << std::endl;
    if (debug) std::cout << "  total vectors " << N << std::endl;

    mean = mean/N;
    for (auto i = classes.begin(); i != classes.end(); ++i)
    {
        int c = *i;
        means[c] = means[c] / classCounter[c];
    }

    if (debug) std::cout << "  Within class distribution matrix..." << std::endl;
    // Within class distribution matrix
    Matrix WithinClass = Matrix::zeros(s, s);
    for (auto i = classes.begin(); i != classes.end(); ++i)
    {
        int c = *i;
        for (int j = 0; j < classCounter[c]; j++)
        {
            Matrix diff = vectors[classToVector[c][j]] - means[c];
            //Sk += (diff * diff.t());
            WithinClass += (diff * diff.t());
        }
    }
    if (debug) std::cout << "  ...done" << std::endl;

    if (debug) std::cout << "  Between class distribution matrix" << std::endl;
    // Between class
    Matrix BetweenClass = Matrix::zeros(s, s);
    for (auto i = classes.begin(); i != classes.end(); ++i)
    {
        int c = *i;
        Matrix diff = means[c] - mean;

        BetweenClass += classCounter[c] * (diff * diff.t());
    }
    if (debug) std::cout << "  ...done" << std::endl;

    if (debug) std::cout << "  Calculating S = inv(Sw) * Sb" << std::endl;
    Matrix S = WithinClass.inv() * BetweenClass;
    Matrix eigenvalues;

    if (debug) std::cout << "  Calculating eigenvectors and eigenvalues of S" << std::endl;
    cv::eigen(S, eigenvalues, Wt);
    if ((K-1) < Wt.rows)
        Wt = Wt.rowRange(0, K-1);

    if (debug) std::cout << "  LDA done" << std::endl;
}

Vector LDA::project(const Vector &vector) const
{
    if (vector.rows != mean.rows) throw FACELIB_EXCEPTION("invalid input size");
    return Wt * (vector - mean);
}
