#include "faceCommon/linalg/pca.h"

#include "faceCommon/linalg/matrixconverter.h"

using namespace Face::LinAlg;

PCA::PCA(const std::vector<Vector> &vectors, int maxComponents, bool debug)
{
    learn(vectors, maxComponents, debug);
}

PCA::PCA(const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    deserialize(storage);
}

void PCA::learn(const std::vector<Vector> &vectors, int maxComponents, bool debug)
{
    if (debug) std::cout << "PCA" << std::endl;
    if (debug) std::cout << "Creating input data matrix" << std::endl;

    std::vector<Vector> input;
    for (const Vector &v : vectors)
    {
        if (!v.containsNan())
        {
            input.push_back(v);
        }
    }
    if (input.size() != vectors.size())
    {
        std::cout << "PCA: Using " << input.size() << " of " << vectors.size() << " input vectors" << std::endl;
    }

    Matrix data = MatrixConverter::columnVectorsToDataMatrix(input);
    if (debug) std::cout << "Calculating eigenvectors and eigenvalues" << std::endl;
    cvPca = cv::PCA(data, cv::Mat(), CV_PCA_DATA_AS_COL, maxComponents);
    if (debug) std::cout << "PCA done" << std::endl;
}

void PCA::serialize(cv::FileStorage &storage) const
{
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("PCA::serialize - cv::FileStorage is not opened");
    }

    storage << "eigenvalues" << cvPca.eigenvalues;
    storage << "eigenvectors" << cvPca.eigenvectors;
    storage << "mean" << cvPca.mean;
}

void PCA::deserialize(cv::FileStorage &storage)
{
    if (!storage.isOpened())
    {
        throw FACELIB_EXCEPTION("PCA::deserialize - cv::FileStorage is not opened");
    }

    storage["eigenvalues"] >> cvPca.eigenvalues;
    storage["eigenvectors"] >> cvPca.eigenvectors;
    storage["mean"] >> cvPca.mean;
}

Vector PCA::project(const Vector &in) const
{
    if (!in.containsNan())
    {
        Matrix m = cvPca.project(in);
        return m;
    }
    else
    {
        Vector mean = getMean();
        Vector filled = in;
        int n = in.rows;
        std::map<int, double> valid;
        for (int i = 0; i < n; i++)
        {
            double v = in(i);
            if (v != v)
            {
                filled(i) = mean(i);
            }
            else
            {
                valid[i] = v;
            }
        }

        for (int iter = 0; iter < 5; iter++)
        {
            Matrix projected = cvPca.project(filled);
            filled = backProject(projected);

            for (auto i : valid)
            {
                filled(i.first) = i.second;
            }
        }
        Matrix result = cvPca.project(filled);
        return result;
    }
}

Vector PCA::scaledProject(const Vector &vector) const
{
    Vector out = project(vector);
    for (int i = 0; i < cvPca.eigenvalues.rows; i++)
        out(i) = out(i) / cvPca.eigenvalues.at<double>(i);
    return out;
}

Vector PCA::backProject(const Vector &in) const
{
    Matrix m = cvPca.backProject(in);
    return m;
}

void PCA::modesSelectionThreshold(double t)
{
	if (t >= 1) return; // nothing to do here

    double sum = 0.0;
    int r;
    for (r = 0; r < cvPca.eigenvalues.rows; r++)
    {
        sum += cvPca.eigenvalues.at<double>(r);
    }

    double actualSum = 0.0;
    for (r = 0; r < cvPca.eigenvalues.rows; r++)
    {
        actualSum += cvPca.eigenvalues.at<double>(r);

        if (actualSum/sum > t)
            break;
    }

    setModes(r+1);
}

double PCA::getVariation(int mode)
{
    double val = cvPca.eigenvalues.at<double>(mode);
    if (val != val) val = 0.0;
    if (val < 0) val = - val;
    return val;
}

Vector PCA::getMean() const
{
    Matrix mean = cvPca.mean;
    return mean;
}

Vector PCA::normalizeParams(const Vector &params)
{
    return normalizeParams(params, 3);
}

Vector PCA::normalizeParams(const Vector &params, double stdMultiplier)
{
    int n = getModes();
    if (params.rows != n) throw FACELIB_EXCEPTION("invalid params");
    Vector result(n);

    for (int i = 0; i < n; i++)
    {
        double p = params(i);
        double limit = stdMultiplier*sqrt(getVariation(i));
        if (p > limit)
            p = limit;
        else if (p < -limit)
            p = -limit;
        result(i) = p;
    }
    return result;
}
