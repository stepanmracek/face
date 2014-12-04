#ifndef IMAGEFILTER_H
#define IMAGEFILTER_H

#include <string>

#include "common.h"

namespace Face {
namespace LinAlg {

class ImageFilter
{
public:
    typedef cv::Ptr<ImageFilter> Ptr;

    virtual ~ImageFilter() {}
    virtual Matrix process(const Matrix &input) const = 0;
    virtual std::string writeParams() const = 0;

    static Matrix batchProcess(const Matrix &input, const std::vector<ImageFilter::Ptr> &filters);
};

// ---------- Gabor filters ----------

class GaborFilter : public ImageFilter
{
protected:
    Matrix realKernel;
    Matrix imagKernel;
    int frequency;
    int orientation;

public:
    GaborFilter(int frequency, int orientation);
};

class GaborFilterReal : public GaborFilter
{
public:
    GaborFilterReal(int frequency, int orientation);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaborReal"; }
};

class GaborFilterImag : public GaborFilter
{
public:
    GaborFilterImag(int frequency, int orientation);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaborImag"; }
};

class GaborFilterAbs : public GaborFilter
{
public:
    GaborFilterAbs(int frequency, int orientation);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaborAbs"; }
};

// ---------- G-L filters ----------

class GaussLaguerreFilter : public ImageFilter
{
protected:
    Matrix realKernel;
    Matrix imagKernel;
    int kernelSize;
    int n;
    int k;

public:
    GaussLaguerreFilter(int kernelSize, int n, int k);
};

class GaussLaguerreFilterReal : public GaussLaguerreFilter
{
public:
    GaussLaguerreFilterReal(int kernelSize, int n, int k);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussLaguerreReal"; }
};

class GaussLaguerreFilterImag : public GaussLaguerreFilter
{
public:
    GaussLaguerreFilterImag(int kernelSize, int n, int k);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussLaguerreImag"; }
};

class GaussLaguerreFilterAbs : public GaussLaguerreFilter
{
public:
    GaussLaguerreFilterAbs(int kernelSize, int n, int k);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussLaguerreAbs"; }
};

// ---------- other filters ----------

class DifferenceOfGaussiansFilter : public ImageFilter
{
    int kernel1size;
    int kernel2size;

public:
    DifferenceOfGaussiansFilter(int kernel1size, int kernel2size);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "dog"; }
};

class EqualizeFilter : public ImageFilter
{
public:
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "equalize"; }
};

class GaussianBlurFilter : public ImageFilter
{
    int kernelSize;

public:
    GaussianBlurFilter(int kernelSize);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussBlur"; }
};

class ScaleFilter : public ImageFilter
{
protected:
    double scale;

public:
    ScaleFilter(double scale);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "scale"; }
};

class LBPFilter : public ImageFilter
{
protected:
    inline double getValue(const Matrix &in, int row, int col) const;

public:
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "lbp"; }
};

class GammaFilter : public ImageFilter
{
protected:
    double gamma;

public:
    GammaFilter(double gamma);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "gamma"; }
};

class HistogramFilter : public ImageFilter
{
protected:
    int gridSizeX;
    int gridSizeY;

public:
    HistogramFilter(int gridSizeX, int gridSizeY);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "histogram"; }

    std::vector<double> histogram(const Matrix &cell) const;
};

class ContrastEqualizationFilter : public ImageFilter
{
protected:
    double alpha;
    double tau;

public:
    ContrastEqualizationFilter() : alpha(0.1), tau(10.0) {}

    Matrix process(const Matrix &input) const;
    std::string writeParams() const { return name(); }
    static std::string name() { return "contrast"; }
};

}
}

#endif // IMAGEFILTER_H
