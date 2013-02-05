#include "linalg/vector.h"

#include <QFile>

#include <cassert>

Matrix Vector::fromFile(const QString &path)
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
    Matrix vector(r, 1, CV_64F);
    for (int i = 0; i < r; i++)
    {
        vector(i, 0) = data.at(i);
    }

    if(Common::matrixContainsNan(vector))
    {
        qDebug() << path << "contains Nan";
    }
    return vector;
}

double Vector::sqrMagnitude(Matrix &vector)
{
    int n = vector.rows;
    double sum = 0;
    for (int i = 0; i < n; i++)
    {
        double v = vector(i, 0);
        sum +=  (v * v);
    }
    return sum;
}

double Vector::magnitude(Matrix &vector)
{
    return sqrt(sqrMagnitude(vector));
}

double Vector::dot(Matrix &v1, Matrix &v2)
{
    int n = v1.rows;
    assert(n == v2.rows);
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += (v1(i, 0) * v2(i, 0));
    }
    return sum;
}

Matrix & Vector::normalize(Matrix &vector)
{
    int n = vector.rows;

    double mag = magnitude(vector);
    for (int i = 0; i < n; i++)
    {
        double v = vector(i, 0);
        vector(i, 0) = v/mag;
    }

    return vector;
}

Matrix Vector::normalized(Matrix &vector)
{
    int n = vector.rows;
    double mag = magnitude(vector);
    Matrix newVector(vector.rows, vector.cols, CV_64F);
    for (int i = 0; i < n; i++)
    {
        double v = vector(i, 0);
        newVector(i, 0) = v/mag;
    }

    return newVector;
}

Matrix & Vector::mul(Matrix &vector, double value)
{
    int n = vector.rows;

    for (int i = 0; i < n; i++)
    {
        double v = vector(i ,0);
        vector(i, 0) = v*value;
    }

    return vector;
}

bool Vector::isZero(Matrix &vector)
{
    int n = vector.rows;
    for (int i = 0; i < n; i++)
    {
        if (vector(i) != 0) return false;
    }
    return true;
}

void Vector::toFile(const Matrix &vector, const QString &path, bool append)
{
    QFile f(path);
    if (append)
        f.open(QIODevice::WriteOnly | QIODevice::Append);
    else
        f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    int n = vector.rows;
    for (int i = 0; i < n; i++)
    {
        out << vector(i);
        out << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}

void Vector::toFileWithIndicies(const Matrix &vector, const QString &path, bool append)
{
    QFile f(path);
    if (append)
        f.open(QIODevice::WriteOnly | QIODevice::Append);
    else
        f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    int n = vector.rows;
    for (int i = 0; i < n; i++)
    {
        out << i << ' ' << vector(i) << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}

void Vector::toFileTwoCols(Matrix &vector,const QString &path, bool append)
{
    int n = vector.rows;
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
        out <<vector(i, 0) << ' ' <<  vector(i+n, 0) << '\n';
    }
    out << '\n';

    out.flush();
    f.close();
}

Matrix Vector::fromTwoColsFile(const QString &path)
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
    Matrix vector(r, 1, CV_64F);

    for (int i = 0; i < x.count(); i++)
    {
        vector(i, 0) = x[i];
    }
    for (int i = 0; i < y.count(); i++)
    {
        vector(x.count()+i, 0) = y[i];
    }

    return vector;
}

int Vector::maxIndex(Matrix &vector)
{
    double max = -1e300;
    int index = -1;
    int r = vector.rows;
    assert(r > 0);
    for (int i = 0; i < r; i++)
    {
        double v = vector(i);
        if (v > max)
        {
            index = i;
            max = v;
        }
    }
    return index;
}

int Vector::maxIndex(QVector<double> &vector)
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
}

double Vector::maxValue(Matrix &vector)
{
    return vector(maxIndex(vector));
}

double Vector::maxValue(QVector<double> &vector)
{
    return vector.at(maxIndex(vector));
}

int Vector::minIndex(Matrix &vector)
{
    double min = 1e300;
    int index = -1;
    int r = vector.rows;
    assert(r > 0);
    for (int i = 0; i < r; i++)
    {
        double v = vector(i);
        if (v < min)
        {
            min = v;
            index = i;
        }
    }
    return index;
}

int Vector::minIndex(QVector<double> &vector)
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
}

double Vector::minValue(Matrix &vector)
{
    return vector(minIndex(vector));
}

double Vector::minValue(QVector<double> &vector)
{
	int i = minIndex(vector);
    return vector.at(i);
}

double Vector::meanValue(Matrix &vector)
{
    double sum = 0;
    int r = vector.rows;
    for (int i = 0; i < r; i++)
    {
        sum += vector(i);
    }

    return sum/r;
}

double Vector::meanValue(QVector<double> &vector)
{
    double sum = 0;
    int r = vector.count();
    for (int i = 0; i < r; i++)
    {
        sum += vector.at(i);
    }

    return sum/r;
}

double Vector::stdDeviation(Matrix &vector)
{
    double mean = meanValue(vector);
    double sum = 0;
    int r = vector.rows;
    for (int i = 0; i < r; i++)
    {
        sum += ((vector(i) - mean)*(vector(i) - mean));
    }

    return sqrt((1.0/(r - 1.0)) * sum);
}

double Vector::stdDeviation(QVector<double> &vector)
{
    double mean = meanValue(vector);
    double sum = 0;
    int r = vector.count();
    for (int i = 0; i < r; i++)
    {
        sum += ((vector.at(i) - mean)*(vector.at(i) - mean));
    }

    return sqrt((1.0/(r - 1.0)) * sum);
}

Matrix Vector::fromQVector(QVector<double> &vec)
{
    int r = vec.count();
    Matrix result = Matrix::zeros(r, 1);
    for (int i = 0; i < r; i++)
    {
        result(i) = vec.at(i);
    }
    return result;
}

QVector<double> Vector::toQVector(Matrix &vector)
{
    QVector<double> result;
    for (int i = 0; i < vector.rows; i++)
        result << vector(i);
    return result;
}

Matrix Vector::normalizeComponents(Matrix &vector, QVector<double> &minValues, QVector<double> &maxValues, bool fixedBounds)
{
    int r = vector.rows;
    Matrix result = Matrix::zeros(r, 1);
    for (int i = 0; i < r; i++)
    {
        double val = vector(i);

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

Matrix Vector::normalizeComponents(Matrix &vector)
{
    double min = minValue(vector);
    double max = maxValue(vector);

    Matrix result = (vector-min)/(max-min);
    return result;
}

Matrix Vector::meanVector(QVector<Matrix> &vectors)
{
    Matrix result = Matrix::zeros(vectors[0].rows, vectors[0].cols);
    int n = vectors.count();
    for (int i = 0; i < n; i++)
        result += vectors[i];

    result = result / ((double)n);
    return result;
}
