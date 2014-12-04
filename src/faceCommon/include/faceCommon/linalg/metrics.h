#ifndef METRICS_H
#define METRICS_H

#include <cmath>
#include <opencv/cv.h>

#include "vector.h"

namespace Face {
namespace LinAlg {

class Metrics
{
public:
    Metrics() {}

    typedef cv::Ptr<Metrics> Ptr;

    virtual double distance(const Vector &v1, const Vector &v2) const = 0;

    virtual ~Metrics() {}

    virtual std::string writeParams() const = 0;
};

class MetricsFactory
{
public:
    static Metrics::Ptr create(const std::string &name);
};

class WeightedMetric : public Metrics
{
public:
    Vector w;

    virtual double distance(const Vector &v1, const Vector &v2) const = 0;

    void normalizeWeights()
    {
        normalizeWeights(w);
    }

    static void normalizeWeights(Vector &w)
    {
        int n = w.rows;
        if (n < 1 || w.cols != 1) throw FACELIB_EXCEPTION("invalid input size");

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double val = w(i);
            if (val < 0.0)
            {
                val = 0.0;
                w(i) = 0.0;
            }
            sum += val;
        }

        for (int i = 0; i < n; i++)
            w(i) = w(i)/sum*n;
    }
};

class EuclideanMetric : public Metrics
{
public:
    static std::string name() { return "euclidean"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double v = (v1(i, 0) - v2(i, 0));
            sum += v*v;
        }

        return sqrt(sum);
    }

    std::string writeParams() const { return name(); }
};

class EuclideanWeightedMetric : public WeightedMetric
{
public:
    static std::string name() { return "euclideanW"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");
        if (w.rows < n) throw FACELIB_EXCEPTION("weights size mismatch");

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double v = w(i) * (v1(i) - v2(i));
            sum += v*v;
        }

        return sqrt(sum);
    }

    std::string writeParams() const { return name(); }
};

class CityblockMetric : public Metrics
{
public:
    static std::string name() { return "cityblock"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += fabs(v1(i, 0) - v2(i, 0));
        }

        return sum;
    }

    std::string writeParams() const { return name(); }
};

class CityBlockNaNSafeMetric : public Metrics
{
public:
    static std::string name() { return "cityblockN"; }

    virtual double distance(const Vector &vec1, const Vector &vec2) const
    {
        int n = vec1.rows;
        if (n != vec2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        double sum = 0.0;
        int nans = 0;
        for (int i = 0; i < n; i++)
        {
            double v1 = vec1(i);
            double v2 = vec2(i);
            if (v1 != v1 || v2 != v2)
            {
                nans++;
            }
            else
            {
                sum += fabs(v1 - v2);
            }
        }

        if (nans == n) return NAN;
        if (nans == 0) return sum;
        double mean = sum / (n - nans);
        return sum + nans / sum * mean;
    }

    std::string writeParams() const { return name(); }
};

class CityblockWeightedMetric : public WeightedMetric
{
public:
    static std::string name() { return "cityblockW"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");
        if (w.rows < n) throw FACELIB_EXCEPTION("weights size mismatch");

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += w(i) * fabs(v1(i, 0) - v2(i, 0));
        }

        return sum;
    }

    std::string writeParams() const { return name(); }
};

class CorrelationMetric : public Metrics
{
public:
    static std::string name() { return "correlation"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        return 1.0 - correlation(v1, v2);
    }

    static double correlation(const Vector &v1, const Vector &v2)
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        double mean1 = v1.meanValue();
        double mean2 = v2.meanValue();
        double std1 = v1.stdDeviation();
        double std2 = v2.stdDeviation();

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += ((v1(i) - mean1)/std1)*((v2(i) - mean2)/std2);
        }

        return (1.0/(n-1.0)) * sum;
        //return sum/n;
    }

    std::string writeParams() const { return name(); }
};

class CorrelationWeightedMetric : public WeightedMetric
{
public:
    static std::string name() { return "correlationW"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        return 1.0 - correlation(v1, v2);
    }

    double correlation(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");
        if (w.rows < n) throw FACELIB_EXCEPTION("weights size mismatch");

        Vector v1w = v1.mul(w);
        Vector v2w = v2.mul(w);

        double mean1 = v1w.meanValue();
        double mean2 = v2w.meanValue();
        double std1 = v1w.stdDeviation();
        double std2 = v2w.stdDeviation();

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += ((v1w(i) - mean1)/std1)*((v2w(i) - mean2)/std2);
        }

        return (1.0/(n-1.0)) * sum;
    }

    std::string writeParams() const { return name(); }
};

class CosineMetric : public Metrics
{
public:
    static std::string name() { return "cosine"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        Matrix mul = (v1.t()*v2);
        double dist = 1.0 - (mul(0))/(v1.magnitude() * v2.magnitude());

        if(dist < 0.0)
        {
            //qDebug() << "CosineMetric" << dist;
            dist = 0.0;
        }

        return dist;
    }

    std::string writeParams() const { return name(); }
};

class CosineWeightedMetric : public WeightedMetric
{
public:
    static std::string name() { return "cosineW"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");
        if (w.rows < n) throw FACELIB_EXCEPTION("weights size mismatch");

        Vector v1w = v1.mul(w);
        Vector v2w = v2.mul(w);

        Matrix mul = (v1w.t()*v2w);
        double dist = 1.0 - (mul(0))/(v1w.magnitude() * v2w.magnitude());

        if(dist < 0.0)
        {
            // qDebug() << "CosineWeightedMetric" << dist;
            dist = 0.0;
        }
        //qDebug() << dist;

        return dist;
    }

    std::string writeParams() const { return name(); }
};

class MahalanobisMetric : public Metrics
{
public:
    static std::string name() { return "mahalanobis"; }

    Matrix invCov;
    Vector mean;

    MahalanobisMetric() {}

    MahalanobisMetric(Matrix &invCov, Vector &mean) : invCov(invCov), mean(mean) {}

    MahalanobisMetric(std::vector<Vector> &samples);

    void learn(std::vector<Vector> &samples);

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        return cv::Mahalonobis(v1, v2, invCov);
    }

    double distance(const Vector &v)
    {
        //qDebug() << "mean:" << mean.rows << mean.cols << "v:" << v.rows << v.cols << "invCov" << invCov.rows << invCov.cols;
        return cv::Mahalanobis(v, mean, invCov);
    }

    std::string writeParams() const { return name(); }
};

class MahalanobisWeightedMetric : public WeightedMetric
{
public:
    static std::string name() { return "mahalanobisW"; }

    Matrix invCov;
    Matrix mean;

    MahalanobisWeightedMetric() {}

    MahalanobisWeightedMetric(Matrix &invCov, Matrix &mean) : invCov(invCov), mean(mean) {}

    MahalanobisWeightedMetric(std::vector<Vector> &samples);

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");
        if (w.rows < n) throw FACELIB_EXCEPTION("weights size mismatch");

        return cv::Mahalonobis(v1.mul(w), v2.mul(w), invCov);
    }

    double distance(Vector &v)
    {
        return cv::Mahalanobis(v.mul(w), mean.mul(w), invCov);
    }

    std::string writeParams() const { return name(); }
};

class SSDMetric : public Metrics
{
public:
    static std::string name() { return "ssd"; }

    double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");

        double sum = 0.0;
        for (int i = 0; i < n;i++)
        {
            double d = (v1(i) - v2(i));
            sum += d*d;
        }
        return sum;
    }

    std::string writeParams() const { return name(); }
};

class SSDWeightedMetric : public WeightedMetric
{
public:
    static std::string name() { return "ssdW"; }

    virtual double distance(const Vector &v1, const Vector &v2) const
    {
        int n = v1.rows;
        if (n != v2.rows) throw FACELIB_EXCEPTION("input vector sizes mismatch");
        if (w.rows < n) throw FACELIB_EXCEPTION("weights size mismatch");

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double v = w(i) * (v1(i) - v2(i));
            sum += v*v;
        }

        return sum;
    }

    std::string writeParams() const { return name(); }
};

class HammingMetric : public Metrics
{
public:
    static std::string name() { return "hamming"; }

    virtual double distance(const Vector &v1, const Vector &v2) const;

    std::string writeParams() const { return name(); }
};

class IntersectionMetric : public Metrics
{
public:
    static std::string name() { return "intersection"; }

    virtual double distance(const Vector &v1, const Vector &v2) const;

    std::string writeParams() const { return name(); }
};

class ChiSquareMetric : public Metrics
{
public:
    static std::string name() { return "chi2"; }

    virtual double distance(const Vector &v1, const Vector &v2) const;

    std::string writeParams() const { return name(); }
};

}
}

#endif // METRICS_H
