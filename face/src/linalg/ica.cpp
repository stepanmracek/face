#include "ica.h"

#include <cassert>

#include "linalg/vector.h"
#include "pca.h"
#include "matrixconverter.h"

double g1(double u)
{
    return tanh(u);
}

double g2(double u)
{
    return u*exp(-(u*u)/2.0);
}

void ICA::learn(QVector<Vector> &vectors, int independentComponentCount, double eps, int maxIterations, bool debug)
{
    int n = vectors.count();
    assert(n > 0);
    int m = vectors[0].rows;

    if (independentComponentCount <= 0)
        independentComponentCount = m-1;
    assert(independentComponentCount <= m);

    // mean value
    mean = Matrix::zeros(m, 1);
    for (int i = 0; i < n; i++)
        mean += vectors[i];
    mean = mean / n;

    for (int i = 0; i < n; i++)
    {
        Matrix m = vectors[i] - mean;
        vectors[i] = m;
    }

    // whitening; x <- E * D^(-1/2) * E^T * x
    // E - eigenvectors
    // D - eigenvalues
    PCA pca(vectors);

    int eValCount = pca.cvPca.eigenvalues.rows;
    Matrix eVals = Matrix::zeros(eValCount, eValCount);
    for (int i = 0; i < eValCount; i++)
    {
        double eval = pca.cvPca.eigenvalues.at<double>(i);
    	if (eval < 0) eval = 0;
        eVals(i,i) = 1.0/sqrt(eval);
    }

    EDET = pca.cvPca.eigenvectors * eVals * pca.cvPca.eigenvectors.t();
    for (int i = 0; i < n; i++)
    {
        Matrix m = EDET * vectors[i];
        vectors[i] = m;
    }
    EDETinv = EDET.inv();

    // FAST ICA
    W = Matrix::eye(m, independentComponentCount);
    for (int p = 0; p < independentComponentCount; p++)
    {
        double oldErr = 1.0;
        double sameErrCount = 0;

        // initicalization
        Matrix colMat = W.col(p);
        Vector w = colMat; // Matrix::ones(m, 1, CV_64F);
        Matrix mat = w/w.magnitude();
        w = mat;

        int iteration = 1;
        while(1)
        {
            // w <- E{x * g(w^T * x)} - E{g'(w^T * x)} * w
            // w <- w / |w|
            Matrix expected1 = Matrix::zeros(m, 1);
            double expected2 = 0;
            for (int i = 0; i < n; i++)
            {
                Matrix wtxMat = w.t()*vectors[i];
                double wtx = wtxMat(0);
                expected1 += vectors[i] * g1(wtx);
                expected2 += g2(wtx);
            }
            expected1 = expected1/n;
            Matrix expected2Mat = expected2/n * w;
            Vector wOld(w);
            Matrix diffMat = expected1 - expected2Mat;
            w = diffMat;
            Matrix normalized = w/w.magnitude();
            w = normalized;

            /*if (p > 0)
            {
                // decorrelate
                Matrix sum = Matrix::zeros(m, 1, CV_64F);
                for (int j = 0; j < p; j++)
                {
                    sum += W.col(j) * W.col(j).t() * w;
                    w = w-sum;
                    w = w/Vector::magnitude(w);
                }
            }*/

            double err = fabs(1-fabs(Vector::dot(w, wOld)));
            if (err == oldErr) sameErrCount++;
            if (err < eps || iteration > maxIterations || sameErrCount > 10)
            {
                //qDebug() << p << iteration << err;
                break;
            }
            //qDebug() << " " << p << iteration << err << Vector::dot(w, wOld);
            oldErr = err;
            iteration++;
        }

        if (debug)
            qDebug() << " " << (p+1) << "/" << independentComponentCount;
        //Common::printMatrix(W);
    }
    W = W.t();

    if (Common::matrixContainsNan(W))
    {
    	qDebug() << "ICA: W before decorrelation contains NaN";
    	exit(1);
    }

    return;

    // decorrelation, repeat until convergence
    if (debug)
        qDebug() << "ICA - projection matrix decorrelation";
    Matrix oldW;
    Matrix diffW;
    W.copyTo(oldW);
    W = W / sqrt(cv::norm(W*W.t()));
    for (int i = 0; i < 10000; i++)
    {
        W = 1.5*W - 0.5*W*W.t()*W;
        diffW = oldW-W;
        double diff = Common::absSum(diffW);

        if (Common::matrixContainsNan(W))
        {
        	qDebug() << "ICA: W during decorrelation step" << i << "contains NaN";
        	oldW.copyTo(W);
        	assert(!Common::matrixContainsNan(oldW));
        	break;
        }

        if (debug)
            qDebug() << " decorrelation" << i << diff;

        if (diff != diff || diff < eps)
            break;

        W.copyTo(oldW);
    }

    if (debug)
        qDebug() << "ICA done";
    //Common::printMatrix(W);
}

ICA::ICA(QVector<Vector> &vectors, int independentComponentCount, double eps, int maxIterations, bool debug)
{
    learn(vectors, independentComponentCount, eps, maxIterations, debug);
}

Vector ICA::whiten(const Vector &vector)
{
    Matrix m = EDET * (vector - mean);
    //Vector result = m;
    return m;
}

Vector ICA::project(const Vector &vector)
{
    Vector whitened = whiten(vector);
    Matrix m = W * whitened;
    //Vector result = m;
    return m;
}

QVector<Vector> ICA::project(const QVector<Vector> &vectors)
{
    QVector<Vector> result;
    for (int i = 0; i < vectors.count(); i++)
    {
        Matrix out = project(vectors[i]);
        result.append(out);
    }
    return result;
}

Vector ICA::backProject(const Vector &vector)
{
    Matrix result = W.t() * vector;
    result = EDETinv * result;
    result = result + mean;
    return result;
}
