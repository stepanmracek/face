#ifndef METRICS_H
#define METRICS_H

#include <QVector>

#include <cassert>
#include <cmath>
#include <opencv/cv.h>

#include "linalg/vector.h"

class Metrics
{
public:
    Metrics() {}

    virtual double distance(Matrix &v1, Matrix &v2) = 0;

    virtual ~Metrics() {}
};

class WeightedMetric : public Metrics
{
public:
    Matrix w;

    virtual double distance(Matrix &v1, Matrix &v2) = 0;

    void normalizeWeights()
    {
        normalizeWeights(w);
    }

    static void normalizeWeights(Matrix &w)
    {
        int n = w.rows;
        assert(n >= 1);
        assert(w.cols == 1);

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

    virtual ~WeightedMetric() {}
};

class EuclideanMetric : public Metrics
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double v = (v1(i, 0) - v2(i, 0));
            sum += v*v;
        }

        return sqrt(sum);
    }

    virtual ~EuclideanMetric() {}
};

class EuclideanWeightedMetric : public WeightedMetric
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);
        assert(w.rows >= n);

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double v = w(i) * (v1(i) - v2(i));
            sum += v*v;
        }

        return sqrt(sum);
    }

    virtual ~EuclideanWeightedMetric() {}
};

class CityblockMetric : public Metrics
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += fabs(v1(i, 0) - v2(i, 0));
        }

        return sum;
    }

    virtual ~CityblockMetric() {}
};

class CityblockWeightedMetric : public WeightedMetric
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);
        assert(w.rows >= n);

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += w(i) * fabs(v1(i, 0) - v2(i, 0));
        }

        return sum;
    }

    virtual ~CityblockWeightedMetric() {}
};

class CorrelationMetric : public Metrics
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        return 1.0 - correlation(v1, v2);
    }

    double correlation(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);

        double mean1 = Vector::meanValue(v1);
        double mean2 = Vector::meanValue(v2);
        double std1 = Vector::stdDeviation(v1);
        double std2 = Vector::stdDeviation(v2);

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += ((v1(i) - mean1)/std1)*((v2(i) - mean2)/std2);
        }

        return (1.0/(n-1.0)) * sum;
    }

    virtual ~CorrelationMetric() {}
};

class CorrelationWeightedMetric : public WeightedMetric
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        return 1.0 - correlation(v1, v2);
    }

    double correlation(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);

        Matrix v1w = v1.mul(w);
        Matrix v2w = v2.mul(w);

        double mean1 = Vector::meanValue(v1w);
        double mean2 = Vector::meanValue(v2w);
        double std1 = Vector::stdDeviation(v1w);
        double std2 = Vector::stdDeviation(v2w);

        double sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            sum += ((v1w(i) - mean1)/std1)*((v2w(i) - mean2)/std2);
        }

        return (1.0/(n-1.0)) * sum;
    }

    virtual ~CorrelationWeightedMetric() {}
};

class CosineMetric : public Metrics
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);

        Matrix mul = (v1.t()*v2);
        double dist = 1.0 - (mul(0))/(Vector::magnitude(v1) * Vector::magnitude(v2));

        if(dist < 0.0)
        {
        	// qDebug() << "CosineMetric" << dist;
        	dist = 0.0;
        }

        return dist;
    }

    virtual ~CosineMetric() {}
};

class CosineWeightedMetric : public WeightedMetric
{
public:
    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);

        Matrix v1w = v1.mul(w);
        Matrix v2w = v2.mul(w);

        Matrix mul = (v1w.t()*v2w);
        double dist = 1.0 - (mul(0))/(Vector::magnitude(v1w) * Vector::magnitude(v2w));

        if(dist < 0.0)
        {
            // qDebug() << "CosineWeightedMetric" << dist;
            dist = 0.0;
        }

        return dist;
    }

    virtual ~CosineWeightedMetric() {}
};

class MahalanobisMetric : public Metrics
{
public:
    Matrix invCov;
    Matrix mean;

    MahalanobisMetric() {}

    MahalanobisMetric(Matrix &invCov, Matrix &mean) : invCov(invCov), mean(mean) {}

    MahalanobisMetric(QVector<Matrix> &samples);

    void learn(QVector<Matrix> &samples);

    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);
        assert(v1.cols == 1);
        assert(v2.cols == 1);

        return cv::Mahalonobis(v1, v2, invCov);
    }

    double distance(Matrix &v)
    {
        //qDebug() << "mean:" << mean.rows << mean.cols << "v:" << v.rows << v.cols << "invCov" << invCov.rows << invCov.cols;
        return cv::Mahalanobis(v, mean, invCov);
    }

    virtual ~MahalanobisMetric() {}
};

class MahalanobisWeightedMetric : public WeightedMetric
{
public:
    Matrix invCov;
    Matrix mean;

    MahalanobisWeightedMetric() {}

    MahalanobisWeightedMetric(Matrix &invCov, Matrix &mean) : invCov(invCov), mean(mean) {}

    MahalanobisWeightedMetric(QVector<Matrix> &samples);

    virtual double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);
        assert(v1.cols == 1);
        assert(v2.cols == 1);

        return cv::Mahalonobis(v1.mul(w), v2.mul(w), invCov);
    }

    double distance(Matrix &v)
    {
        return cv::Mahalanobis(v.mul(w), mean.mul(w), invCov);
    }

    virtual ~MahalanobisWeightedMetric() {}
};

class SumOfSquareDifferences : public Metrics
{
    double distance(Matrix &v1, Matrix &v2)
    {
        int n = v1.rows;
        assert(n == v2.rows);
        assert(v1.cols == 1);
        assert(v2.cols == 1);

        double sum = 0.0;
        for (int i = 0; i < n;i++)
        {
            double d = (v1(i) - v2(i));
            sum += d*d;
        }
        return sum;
    }

    virtual ~SumOfSquareDifferences() {}
};

#endif // METRICS_H
