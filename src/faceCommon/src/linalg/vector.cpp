#include "faceCommon/linalg/vector.h"

#include <fstream>

using namespace Face::LinAlg;

Vector::Vector(int size) : Matrix(size, 1)
{
    for(int i = 0; i < size; i++)
    {
        (*this)(i) = 0.0;
    }
}

Vector::Vector(int size, double values) : Matrix(size, 1)
{
    for(int i = 0; i < size; i++)
    {
        (*this)(i) = values;
    }
}

Vector::Vector(const Matrix &m) : Matrix(m.rows, 1)
{
    if (m.cols != 1) throw FACELIB_EXCEPTION("invalid input size");
    for (int i = 0; i < m.rows; i++)
    {
        (*this)(i) = m(i, 0);
    }
}

Vector::Vector(const cv::MatExpr &expr) : Matrix(expr)
{
    if (cols != 1) throw FACELIB_EXCEPTION("invalid input size");
}


Vector::Vector(const Vector &src) : Matrix(src.rows, 1)
{
    for (int i = 0; i < src.rows; i++)
    {
        (*this)(i) = src(i, 0);
    }
}

Vector::Vector(const std::vector<double> &vec) : Matrix(vec.size(), 1)
{
    int r = vec.size();
    for (int i = 0; i < r; i++)
    {
        (*this)(i) = vec.at(i);
    }
}

Vector Vector::concatenate(const std::vector<Vector> &vectors)
{
    std::vector<double> resultValues;
    for(const Vector &v : vectors)
    {
        auto vec = v.toStdVector();
        resultValues.insert(resultValues.end(), vec.begin(), vec.end());
    }

    return Vector(resultValues);
}

Vector Vector::fromFile(const std::string &path)
{
    std::ifstream in(path);
    std::vector<double> data;
    double value;
    while (in >> value)
        data.push_back(value);
    in.close();

    Vector vector(data);
    if(Face::LinAlg::Common::matrixContainsNan(vector))
        std::cerr << path << " contains Nan" << std::endl;

    return vector;
}

double Vector::sqrMagnitude() const
{
    int n = this->rows;
    double sum = 0;
    for (int i = 0; i < n; i++)
    {
        double v = (*this)(i);
        sum +=  (v * v);
    }
    return sum;
}

double Vector::magnitude() const
{
    return sqrt(sqrMagnitude());
}

double Vector::dot(const Vector &v1, const Vector &v2)
{
    int n = v1.rows;
    if (n != v2.rows) throw FACELIB_EXCEPTION("invalid input size");
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += (v1(i) * v2(i));
    }
    return sum;
}

Vector &Vector::normalize()
{
    int n = this->rows;

    double mag = magnitude();
    for (int i = 0; i < n; i++)
    {
        double v = (*this)(i);
        (*this)(i, 0) = v/mag;
    }

    return (*this);
}

Vector Vector::normalized() const
{
    int n = this->rows;
    double mag = magnitude();
    Vector newVector(n);
    for (int i = 0; i < n; i++)
    {
        double v = (*this)(i);
        newVector(i) = v/mag;
    }

    return newVector;
}

Vector Vector::mul(double value) const
{
    int n = this->rows;
    Vector result(n);

    for (int i = 0; i < n; i++)
    {
        result(i) = (*this)(i) * value;
    }

    return result;
}

Vector Vector::mul(const Vector &other) const
{
    int r = this->rows;
    if (r != other.rows) throw FACELIB_EXCEPTION("invalid input size");
    Vector result(r);

    for (int i = 0; i < r; i++)
    {
        result(i) = (*this)(i) * other(i);
    }

    return result;
}

bool Vector::isZero() const
{
    int n = this->rows;
    for (int i = 0; i < n; i++)
    {
        if ((*this)(i) != 0) return false;
    }
    return true;
}

void Vector::toFile(const std::string &path, bool append) const
{
    std::ofstream out(path, append ? std::ofstream::app | std::ofstream::out | std::ofstream::trunc : std::ofstream::out);

    int n = this->rows;
    for (int i = 0; i < n; i++)
    {
        out << (*this)(i) << std::endl;
    }
    out << std::endl;

    out.flush();
    out.close();
}

void Vector::toFileWithIndicies(const std::string &path, bool append) const
{
    std::ofstream out(path, append ? std::ofstream::app | std::ofstream::out | std::ofstream::trunc : std::ofstream::out);

    int n = this->rows;
    for (int i = 0; i < n; i++)
    {
        out << i << ' ' << (*this)(i) << std::endl;
    }
    out << std::endl;

    out.flush();
    out.close();
}

void Vector::toFileTwoCols(const std::string &path, bool append) const
{
    int n = this->rows;
    if (n % 2 != 0) throw FACELIB_EXCEPTION("invalid input size");
    n = n/2;

    std::ofstream out(path, append ? std::ofstream::app | std::ofstream::out | std::ofstream::trunc : std::ofstream::out);

    for (int i = 0; i < n; i++)
    {
        out << (*this)(i) << ' ' <<  (*this)(i+n) << std::endl;
    }
    out << std::endl;

    out.flush();
    out.close();
}

Vector Vector::fromTwoColsFile(const std::string &path)
{
    std::ifstream in(path);

    std::vector<double> xvals;
    std::vector<double> yvals;

    double x, y;
    while (in >> x >> y)
    {
        xvals.push_back(y);
        yvals.push_back(y);
    }
    in.close();

    int r = xvals.size() + yvals.size();
    Vector vector(r);

    for (unsigned int i = 0; i < xvals.size(); i++)
    {
        vector(i) = xvals[i];
    }
    for (unsigned int i = 0; i < yvals.size(); i++)
    {
        vector(xvals.size()+i) = yvals[i];
    }

    return vector;
}

int Vector::maxIndex() const
{
    double max = -1e300;
    int index = -1;
    int r = this->rows;
    for (int i = 0; i < r; i++)
    {
        double v = (*this)(i);
        if (v > max)
        {
            index = i;
            max = v;
        }
    }
    return index;
}

int Vector::maxIndex(int from, int to) const
{
    double max = -1e300;
    int index = -1;
    for (int i = from; i <= to; i++)
    {
        if (i < 0 || i >= this->rows) continue;

        double v = (*this)(i);
        if (v > max)
        {
            index = i;
            max = v;
        }
    }

    return index;
}

double Vector::maxValue() const
{
    return (*this)(maxIndex());
}

int Vector::minIndex() const
{
    double min = 1e300;
    int index = -1;
    int r = this->rows;
    for (int i = 0; i < r; i++)
    {
        double v = (*this)(i);
        if (v < min)
        {
            min = v;
            index = i;
        }
    }
    return index;
}

double Vector::minValue() const
{
    return (*this)(minIndex());
}

double Vector::meanValue() const
{
    double sum = 0;
    int r = this->rows;
    for (int i = 0; i < r; i++)
    {
        sum += (*this)(i);
    }

    return sum/r;
}

double Vector::stdDeviation() const
{
    double mean = meanValue();
    double sum = 0;
    int r = this->rows;
    for (int i = 0; i < r; i++)
    {
        sum += (((*this)(i) - mean)*((*this)(i) - mean));
    }

    return sqrt(sum/r);
    //return sqrt((1.0/(r - 1.0)) * sum);
}

std::vector<double> Vector::toStdVector() const
{
    std::vector<double> result;
    for (int i = 0; i < this->rows; i++)
        result.push_back((*this)(i));
    return result;
}

Vector Vector::normalizeComponents(const std::vector<double> &minValues, const std::vector<double> &maxValues, bool fixedBounds) const
{
    int r = this->rows;
    Vector result(r);
    for (int i = 0; i < r; i++)
    {
        double val = (*this)(i);

        if (fixedBounds)
        {
            if (val > maxValues[i]) val = maxValues[i];
            if (val < minValues[i]) val = minValues[i];
        }

        val = (val - minValues[i])/(maxValues[i]-minValues[i]);
        result(i) = val;
    }
    return result;
}

Vector Vector::normalizeComponents() const
{
    double min = minValue();
    double max = maxValue();

    Vector result = ((*this)-min)/(max-min);
    return result;
}

Vector Vector::meanVector(const std::vector<Vector> &vectors)
{
    Vector result(vectors[0].rows);
    int n = vectors.size();
    for (int i = 0; i < n; i++)
        result += vectors[i];

    result = result / ((double)n);
    return result;
}

Vector Vector::smooth(int kernelSize) const
{
    if (kernelSize <= 1 || kernelSize % 2 == 0) throw FACELIB_EXCEPTION("invalid kernel size");

    int n = this->rows;
    Vector result(n);
    for (int i = 0; i < n; i++)
    {
        double sum = 0;
        for (int j = i - kernelSize/2; j <= i + kernelSize/2; j++)
        {
            if (j < 0 || j >= n)
            {
                sum += 0;
            }
            else
            {
                sum += (*this)(j);
            }
        }
        result(i) = sum/kernelSize;
    }

    return result;
}

bool Vector::containsNan() const
{
    int n = rows;
    for (int i = 0; i < n; i++)
    {
        double v = (*this)(i);
        if (v != v) return true;
    }
    return false;
}
