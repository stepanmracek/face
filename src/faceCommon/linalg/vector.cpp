#include "linalg/vector.h"

#include <QFile>

#include <cassert>

Vector::Vector(int size) : Matrix(size, 1)
{
    for(int i = 0; i < size; i++)
    {
        (*this)(i) = 0.0;
    }
}

Vector::Vector(const Matrix &m) : Matrix(m.rows, 1)
{
    assert(m.cols == 1);
    for (int i = 0; i < m.rows; i++)
    {
        (*this)(i) = m(i, 0);
    }
}

Vector::Vector(const cv::MatExpr &expr) : Matrix(expr)
{
    assert(cols == 1);
}


Vector::Vector(const Vector &src) : Matrix(src.rows, 1)
{
    for (int i = 0; i < src.rows; i++)
    {
        (*this)(i) = src(i, 0);
    }
}

Vector::Vector(QVector<double> &vec) : Matrix(vec.count(), 1)
{
    int r = vec.count();
    for (int i = 0; i < r; i++)
    {
        (*this)(i) = vec.at(i);
    }
}

Vector Vector::fromFile(const QString &path)
{
	assert(QFile::exists(path));
    QFile f(path);
    f.open(QIODevice::ReadOnly);
    QTextStream in(&f);

    QVector<double> data;
    while (!in.atEnd())
    {
        double value;
        in >> value;

        if (in.status() == QTextStream::ReadPastEnd)
            break;

        data.append(value);
    }

    f.close();

    int r = data.count();
    Vector vector(r);
    for (int i = 0; i < r; i++)
    {
        vector(i) = data.at(i);
    }

    if(Common::matrixContainsNan(vector))
    {
        qDebug() << path << "contains Nan";
    }
    return vector;
}

double Vector::sqrMagnitude()
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

double Vector::magnitude()
{
    return sqrt(sqrMagnitude());
}

double Vector::dot(Vector &v1, Vector &v2)
{
    int n = v1.rows;
    assert(n == v2.rows);
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

Vector Vector::normalized()
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

Vector &Vector::mul(double value)
{
    int n = this->rows;

    for (int i = 0; i < n; i++)
    {
        (*this)(i) = (*this)(i) * value;
    }

    return (*this);
}

Vector &Vector::mul(Vector &other)
{
    int r = this->rows;
    assert(r == other.rows);

    for (int i = 0; i < r; i++)
    {
        (*this)(i) = (*this)(i) * other(i);
    }

    return (*this);
}

bool Vector::isZero()
{
    int n = this->rows;
    for (int i = 0; i < n; i++)
    {
        if ((*this)(i) != 0) return false;
    }
    return true;
}

void Vector::toFile(const QString &path, bool append)
{
    QFile f(path);
    if (append)
        f.open(QIODevice::WriteOnly | QIODevice::Append);
    else
        f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    int n = this->rows;
    for (int i = 0; i < n; i++)
    {
        out << (*this)(i);
        out << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}

void Vector::toFileWithIndicies(const QString &path, bool append)
{
    QFile f(path);
    if (append)
        f.open(QIODevice::WriteOnly | QIODevice::Append);
    else
        f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    int n = this->rows;
    for (int i = 0; i < n; i++)
    {
        out << i << ' ' << (*this)(i) << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}

void Vector::toFileTwoCols(const QString &path, bool append)
{
    int n = this->rows;
    assert(n % 2 == 0);
    n = n/2;

    QFile f(path);
    if (append)
        f.open(QIODevice::WriteOnly | QIODevice::Append);
    else
        f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    for (int i = 0; i < n; i++)
    {
        out << (*this)(i) << ' ' <<  (*this)(i+n) << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}

Vector Vector::fromTwoColsFile(const QString &path)
{
	assert(QFile::exists(path));
    QFile f(path);
    bool opened = f.open(QIODevice::ReadOnly);
    assert(opened);
    QTextStream in(&f);

    QVector<double> x;
    QVector<double> y;

    bool isX = true;
    while (!in.atEnd())
    {
        double value;
        in >> value;

        if (in.status() == QTextStream::ReadPastEnd)
            break;

        if (isX)
        {
            x.append(value);
        }
        else
        {
            y.append(value);
        }

        isX = !isX;
    }
    assert(x.count() == y.count());
    f.close();

    int r = x.count() + y.count();
    Vector vector(r);

    for (int i = 0; i < x.count(); i++)
    {
        vector(i) = x[i];
    }
    for (int i = 0; i < y.count(); i++)
    {
        vector(x.count()+i) = y[i];
    }

    return vector;
}

int Vector::maxIndex()
{
    double max = -1e300;
    int index = -1;
    int r = this->rows;
    assert(r > 0);
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

int Vector::maxIndex(int from, int to)
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

/*int Vector::maxIndex(QVector<double> &vector)
{
    double max = -1e300;
    int index = -1;
    int r = vector.count();
    assert(r > 0);
    for (int i = 0; i < r; i++)
    {
        double v = vector.at(i);
        if (v > max)
        {
            index = i;
            max = v;
        }
    }
    if (index == -1)
    {
    	qDebug() << vector;
    }
    return index;
}*/

double Vector::maxValue()
{
    return (*this)(maxIndex());
}

/*double Vector::maxValue(QVector<double> &vector)
{
    return vector.at(maxIndex(vector));
}*/

int Vector::minIndex()
{
    double min = 1e300;
    int index = -1;
    int r = this->rows;
    assert(r > 0);
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

/*int Vector::minIndex(QVector<double> &vector)
{
    double min = 1e300;
    int index = -1;
    int r = vector.count();
    assert(r > 0);
    for (int i = 0; i < r; i++)
    {
        double v = vector.at(i);
        if (v < min)
        {
            min = v;
            index = i;
        }
    }
    if (index == -1)
    {
    	qDebug() << vector;
    }
    return index;
}*/

double Vector::minValue()
{
    return (*this)(minIndex());
}

/*double Vector::minValue(QVector<double> &vector)
{
	int i = minIndex(vector);
    return vector.at(i);
}*/

double Vector::meanValue()
{
    double sum = 0;
    int r = this->rows;
    for (int i = 0; i < r; i++)
    {
        sum += (*this)(i);
    }

    return sum/r;
}

/*double Vector::meanValue(QVector<double> &vector)
{
    double sum = 0;
    int r = vector.count();
    for (int i = 0; i < r; i++)
    {
        sum += vector.at(i);
    }

    return sum/r;
}*/

double Vector::stdDeviation()
{
    double mean = meanValue();
    double sum = 0;
    int r = this->rows;
    for (int i = 0; i < r; i++)
    {
        sum += (((*this)(i) - mean)*((*this)(i) - mean));
    }

    return sqrt((1.0/(r - 1.0)) * sum);
}

/*ouble Vector::stdDeviation(QVector<double> &vector)
{
    double mean = meanValue(vector);
    double sum = 0;
    int r = vector.count();
    for (int i = 0; i < r; i++)
    {
        sum += ((vector.at(i) - mean)*(vector.at(i) - mean));
    }

    return sqrt((1.0/(r - 1.0)) * sum);
}*/

QVector<double> Vector::toQVector()
{
    QVector<double> result;
    for (int i = 0; i < this->rows; i++)
        result << (*this)(i);
    return result;
}

Vector Vector::normalizeComponents(QVector<double> &minValues, QVector<double> &maxValues, bool fixedBounds)
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

Vector Vector::normalizeComponents()
{
    double min = minValue();
    double max = maxValue();

    Vector result = ((*this)-min)/(max-min);
    return result;
}

Vector Vector::meanVector(QVector<Vector> &vectors)
{
    Vector result(vectors[0].rows);
    int n = vectors.count();
    for (int i = 0; i < n; i++)
        result += vectors[i];

    result = result / ((double)n);
    return result;
}

Vector Vector::smooth(int kernelSize)
{
    assert(kernelSize > 1);
    assert(kernelSize % 2 == 1);

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
