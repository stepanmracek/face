#include "faceCommon/linalg/ica.h"

#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/pca.h"
#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::LinAlg;

double g1(double u)
{
    return tanh(u);
}

double g2(double u)
{
    return u*exp(-(u*u)/2.0);
}

void ICA::learn(std::vector<Vector> &vectors, int independentComponentCount, double eps, int maxIterations, bool debug)
{
    int n = vectors.size();
    int m = vectors[0].rows;

    if (independentComponentCount <= 0)
        independentComponentCount = m-1;
    //ass*rt(independentComponentCount <= m);

    // mean value
    mean = Matrix::zeros(m, 1);
    for (int i = 0; i < n; i++)
        mean += vectors[i];
    mean = mean / n;

    for (int i = 0; i < n; i++)
    {
        vectors[i] = vectors[i] - mean;
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
        vectors[i] = EDET * vectors[i];
    }
    EDETinv = EDET.inv();

    // FAST ICA
    W = Matrix::eye(m, independentComponentCount);
    for (int p = 0; p < independentComponentCount; p++)
    {
        double oldErr = 1.0;
        double sameErrCount = 0;

        // initicalization
        Vector w = W.col(p); // Matrix::ones(m, 1, CV_64F);
        w = w/w.magnitude();

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
            w = expected1 - expected2Mat;
            w = w/w.magnitude();

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
            std::cout << " " << (p+1) << "/" << independentComponentCount << std::endl;
        //Common::printMatrix(W);
    }
    W = W.t();

    if (Common::matrixContainsNan(W))
    {
        std::cout << "ICA: W before decorrelation contains NaN" << std::endl;
    	exit(1);
    }

    return;

    // decorrelation, repeat until convergence
    if (debug)
        std::cout << "ICA - projection matrix decorrelation" << std::endl;
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
            std::cerr << "ICA: W during decorrelation step" << i << "contains NaN" << std::endl;
        	oldW.copyTo(W);
        	break;
        }

        if (debug)
            std::cout << " decorrelation" << i << diff << std::endl;

        if (diff != diff || diff < eps)
            break;

        W.copyTo(oldW);
    }

    if (debug)
        std::cout << "ICA done" << std::endl;
    //Common::printMatrix(W);
}

ICA::ICA(std::vector<Vector> &vectors, int independentComponentCount, double eps, int maxIterations, bool debug)
{
    learn(vectors, independentComponentCount, eps, maxIterations, debug);
}

Vector ICA::whiten(const Vector &vector) const
{
    return EDET * (vector - mean);
}

Vector ICA::project(const Vector &vector) const
{
    Vector whitened = whiten(vector);
    return W * whitened;
}

Vector ICA::backProject(const Vector &vector) const
{
    return (EDETinv * (W.t() * vector)) + mean;
}
