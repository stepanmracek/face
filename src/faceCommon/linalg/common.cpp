#include "common.h"

#include <QFile>
#include <QTextStream>

#include <iostream>
#include <assert.h>
#include <cmath>

void Common::printMatrix(CvMat *m)
{
    assert(m != NULL);
    for (int r = 0; r < m->rows; r++)
    {
        for (int c = 0; c < m->cols; c++)
            std::cout << cvmGet(m, r, c) << " ";
        std::cout << std::endl;
    }
}

void Common::printMatrix(Matrix &m)
{
    for (int r = 0; r < m.rows; r++)
    {
        for (int c = 0; c < m.cols; c++)
            std::cout << m(r,c) << " ";
        std::cout << std::endl;
    }
}

bool Common::matrixContainsNan(Matrix &m)
{
	int rows = m.rows;
	int cols = m.cols;
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
            double val = m(r,c);
			if (val != val)
			{
				return true;
			}
		}
	}
	return false;
}

void Common::savePlot(QVector<double> &x, QVector<double> &y, QVector<double> &z, const QString &path)
{
    int n = x.count();
    assert(n == y.count());
    assert(n == z.count());

    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for (int i = 0; i < n; i++)
    {
        out << x[i] << " " << y[i] << " " << z[i] << "\n";
    }
}

void Common::savePlot(QVector<double> &x, QVector<double> &y, const QString &path)
{
    int n = x.count();
    assert(n == y.count());

    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for (int i = 0; i < n; i++)
    {
        out << x[i] << " " << y[i] << "\n";
    }
}

void Common::savePlot(QVector<double> values[], int axisCount, const QString &path)
{
    int n = values[0].count();

    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < axisCount; j++)
        {
            out << values[j][i] << " ";
        }

        out <<  "\n";
    }
}

void Common::saveMap(QMap<double, double> &map, const QString &path)
{
    QFile file(path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    QList<double> keys = map.keys();
    for (int i = 0; i < keys.count(); i++)
    {
        double key = keys[i];
        double value = map[key];

        out << key << " " << value << "\n";
    }
}

double euclideanDistance(const cv::Point3d &p1, const cv::Point3d &p2)
{
    double dx = p1.x-p2.x;
    double dy = p1.y-p2.y;
    double dz = p1.z-p2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double euclideanDistance(const cv::Point &p1, const cv::Point &p2)
{
    double dx = p1.x-p2.x;
    double dy = p1.y-p2.y;
    return sqrt(dx*dx + dy*dy);
}

double Common::absSum(Matrix &m)
{
    double sum = 0;
    for (int r = 0; r < m.rows; r++)
    {
        for (int c = 0; c < m.cols; c++)
        {
            double val = m(r,c);
            sum += fabs(val);
        }
    }
    return sum;
}
