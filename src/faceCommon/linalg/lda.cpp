#include "lda.h"

#include <QMap>
#include <QSet>
#include <QDebug>

#include <cassert>

LDA::LDA()
{

}

void LDA::serialize(const char *path)
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "Wt" << Wt;
    storage << "mean" << mean;
}

LDA::LDA(const char *path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    storage["Wt"] >> Wt;
    storage["mean"] >> mean;
}

LDA::LDA(QVector<Vector> &vectors, QVector<int> &classMembership, bool debug)
{
    learn(vectors, classMembership, debug);
}

void LDA::learn(QVector<Vector> &vectors, QVector<int> &classMembership, bool debug)
{
    if (debug) qDebug() << "LDA";

    // number of all vectors;
    int N = vectors.count();
    assert(N == (classMembership.count()));
    assert(N > 0);

    int s = vectors[0].rows;

    // number of classes
    int K = 0;
    QMap<int, int> classCounter;

    // class means
    mean = Matrix::zeros(s, 1);
    QMap<int, Matrix> means;
    QMap<int, QList<int> > classToVector;

    for (int i = 0; i < N; i++)
    {
        assert(vectors[i].rows == s);
        assert(vectors[i].cols == 1);

        int c = classMembership[i];
        if (!classCounter.contains(c))
        {
            K++;
            classCounter[c] = 0;
            means[c] = Matrix::zeros(s, 1);
            classToVector[c] = QList<int>();
        }

        classCounter[c] += 1;
        means[c] += vectors[i];
        classToVector[c].append(i);
        mean += vectors[i];
    }

    if (debug) qDebug() << "  classes:" << K;
    if (debug) qDebug() << "  total vectors" << N;

    mean = mean/N;
    QList<int> classes = classCounter.keys();
    assert(K == classes.count());
    assert(K >= 2);
    for (int i = 0; i < K; i++)
    {
        int c = classes[i];
        means[c] = means[c] / classCounter[c];
    }

    if (debug) qDebug() << "  Within class distribution matrix...";
    // Within class distribution matrix
    Matrix WithinClass = Matrix::zeros(s, s);
    for (int i = 0; i < K; i++)
    {
        //qDebug() << "  " << (i+1) << "/" << K;

        int c = classes[i];
        //Matrix Sk = Matrix::zeros(s, s, CV_64F);

        assert(classCounter[c] == classToVector[c].count());
        for (int j = 0; j < classCounter[c]; j++)
        {
            Matrix diff = vectors[classToVector[c][j]] - means[c];
            //Sk += (diff * diff.t());
            WithinClass += (diff * diff.t());
        }

        //WithinClass += Sk;
    }
    if (debug) qDebug() << "  ...done";

    if (debug) qDebug() << "  Between class distribution matrix";
    // Between class
    Matrix BetweenClass = Matrix::zeros(s, s);
    for (int i = 0; i < K; i++)
    {
        int c = classes[i];
        Matrix diff = means[c] - mean;

        BetweenClass += classCounter[c] * (diff * diff.t());
    }
    if (debug) qDebug() << "  ...done";

    if (debug) qDebug() << "  Calculating S = inv(Sw) * Sb";
    Matrix S = WithinClass.inv() * BetweenClass;
    Matrix eigenvalues;

    if (debug) qDebug() << "  Calculating eigenvectors and eigenvalues of S";
    cv::eigen(S, eigenvalues, Wt);
    if ((K-1) < Wt.rows)
        Wt = Wt.rowRange(0, K-1);

    if (debug) qDebug() << "  LDA done";
}

Vector LDA::project(const Vector &vector) const
{
    assert(vector.rows == mean.rows);
    assert(vector.cols == 1);

    return Wt * (vector - mean);
}
