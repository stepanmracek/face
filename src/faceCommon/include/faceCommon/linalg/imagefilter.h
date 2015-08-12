#pragma once

#include <string>

#include "common.h"

namespace Face {
namespace LinAlg {

class FACECOMMON_EXPORTS ImageFilter
{
public:
    typedef cv::Ptr<ImageFilter> Ptr;

    virtual ~ImageFilter() {}
    virtual Matrix process(const Matrix &input) const = 0;
    virtual std::string writeParams() const = 0;

    static Matrix batchProcess(const Matrix &input, const std::vector<ImageFilter::Ptr> &filters);
};

// ---------- Gabor filters ----------

class FACECOMMON_EXPORTS GaborFilter : public ImageFilter
{
protected:
    Matrix realKernel;
    Matrix imagKernel;
    int frequency;
    int orientation;

public:
    GaborFilter(int frequency, int orientation);
};

class FACECOMMON_EXPORTS GaborFilterReal : public GaborFilter
{
public:
    GaborFilterReal(int frequency, int orientation);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaborReal"; }
};

class FACECOMMON_EXPORTS GaborFilterImag : public GaborFilter
{
public:
    GaborFilterImag(int frequency, int orientation);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaborImag"; }
};

class FACECOMMON_EXPORTS GaborFilterAbs : public GaborFilter
{
public:
    GaborFilterAbs(int frequency, int orientation);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaborAbs"; }
};

// ---------- G-L filters ----------

class FACECOMMON_EXPORTS GaussLaguerreFilter : public ImageFilter
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

class FACECOMMON_EXPORTS GaussLaguerreFilterReal : public GaussLaguerreFilter
{
public:
    GaussLaguerreFilterReal(int kernelSize, int n, int k);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussLaguerreReal"; }
};

class FACECOMMON_EXPORTS GaussLaguerreFilterImag : public GaussLaguerreFilter
{
public:
    GaussLaguerreFilterImag(int kernelSize, int n, int k);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussLaguerreImag"; }
};

class FACECOMMON_EXPORTS GaussLaguerreFilterAbs : public GaussLaguerreFilter
{
public:
    GaussLaguerreFilterAbs(int kernelSize, int n, int k);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussLaguerreAbs"; }
};

// ---------- other filters ----------

class FACECOMMON_EXPORTS DifferenceOfGaussiansFilter : public ImageFilter
{
    int kernel1size;
    int kernel2size;

public:
    DifferenceOfGaussiansFilter(int kernel1size, int kernel2size);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "dog"; }
};

class FACECOMMON_EXPORTS EqualizeFilter : public ImageFilter
{
public:
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "equalize"; }
};

class FACECOMMON_EXPORTS GaussianBlurFilter : public ImageFilter
{
    int kernelSize;

public:
    GaussianBlurFilter(int kernelSize);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "gaussBlur"; }
};

class FACECOMMON_EXPORTS ScaleFilter : public ImageFilter
{
protected:
    double scale;

public:
    ScaleFilter(double scale);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;

    static std::string name() { return "scale"; }
};

class FACECOMMON_EXPORTS LBPFilter : public ImageFilter
{
protected:
    inline double getValue(const Matrix &in, int row, int col) const;

public:
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "lbp"; }
};

class FACECOMMON_EXPORTS GammaFilter : public ImageFilter
{
protected:
    double gamma;

public:
    GammaFilter(double gamma);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "gamma"; }
};

class FACECOMMON_EXPORTS HistogramFilter : public ImageFilter
{
protected:
    int gridSizeX;
    int gridSizeY;

public:
    HistogramFilter(int gridSizeX, int gridSizeY);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "histogram"; }

    //std::vector<double> histogram(const Matrix &cell) const;
};

class FACECOMMON_EXPORTS HistogramBinsFilter : public HistogramFilter
{
protected:
    int binsCount;

public:
    HistogramBinsFilter(int gridSizeX, int gridSizeY, int binsCount);
    Matrix process(const Matrix &input) const;
    std::string writeParams() const;
    static std::string name() { return "histogramBins"; }
};

class FACECOMMON_EXPORTS ContrastEqualizationFilter : public ImageFilter
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

class FACECOMMON_EXPORTS OrientedGradientsFilter : public ImageFilter
{
protected:
    Matrix horizontalKernel;
    Matrix verticalKernel;

public:
    OrientedGradientsFilter();

    Matrix process(const Matrix &input) const;
    std::string writeParams() const { return name(); }
    static std::string name() { return "orientedGradients"; }
};

}
}
