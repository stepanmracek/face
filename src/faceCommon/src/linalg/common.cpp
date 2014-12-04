#include "faceCommon/linalg/common.h"

#include <fstream>
#include <iostream>
#include <cmath>

using namespace Face::LinAlg;

void Common::printMatrix(const Matrix &m)
{
    for (int r = 0; r < m.rows; r++)
    {
        for (int c = 0; c < m.cols; c++)
            std::cout << m(r,c) << " ";
        std::cout << std::endl;
    }
}

bool Common::matrixContainsNan(const Matrix &m)
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

void Common::savePlot(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::string &path)
{
    int n = x.size();
    std::ofstream out(path);
    for (int i = 0; i < n; i++)
    {
        out << x[i] << " " << y[i] << " " << z[i] << std::endl;
    }
    out.close();
}

void Common::savePlot(const std::vector<cv::Point3d> &values, const std::string &path, bool append)
{
    std::ofstream out;
    if (append)
        out.open(path, std::ofstream::app);
    else
        out.open(path);

    for (const auto &p : values)
    {
        out << p.x << " " << p.y << " " << p.z << std::endl;
    }
    out << std::endl;
    out.close();
}

void Common::savePlot(const std::vector<double> &x, const std::vector<double> &y, const std::string &path)
{
    int n = x.size();
    std::ofstream out(path);

    for (int i = 0; i < n; i++)
    {
        out << x[i] << " " << y[i] << "\n";
    }
    out.close();
}

void Common::savePlot(const std::vector<double> values[], int axisCount, const std::string &path)
{
    int n = values[0].size();
    std::ofstream out(path);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < axisCount; j++)
        {
            out << values[j][i] << " ";
        }

        out <<  std::endl;
    }
    out.close();
}

void Common::saveMatrix(const Matrix &m, const std::string &path)
{
    cv::FileStorage storage(path, cv::FileStorage::WRITE);
    storage << "m" << m;
}

Matrix Common::loadMatrix(const std::string &path, const std::string &storageKey)
{
    cv::FileStorage storage(path, cv::FileStorage::READ);
    Matrix m;
    storage[storageKey] >> m;
    return m;
}

void Common::saveMap(std::map<double, double> &map, const std::string &path)
{
    std::ofstream out(path);
    for (auto i = map.begin(); i != map.end(); ++i)
    {
        out << i->first << " " << i->second << std::endl;
    }
    out.close();
}

double Face::LinAlg::euclideanDistance(const cv::Point3d &p1, const cv::Point3d &p2)
{
    double dx = p1.x-p2.x;
    double dy = p1.y-p2.y;
    double dz = p1.z-p2.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}

double Face::LinAlg::euclideanDistance(const cv::Point &p1, const cv::Point &p2)
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

double Common::median(std::vector<double> &values)
{
    int n = values.size();
    if (n == 2) return (values[0]+values[1])/2.0;
    if (n == 1) return values[0];
    if (n == 0) return 0;
    std::sort(values.begin(), values.end());
    return n % 2 == 1 ? values[n/2] : (values[n/2 - 1] + values[n/2])/2.0;
}

/*template <class T>
std::vector<T> Common::balanceSizesAndJoin(const std::vector<T> &first, const std::vector<T> &second)
{
    int fCount = first.count();
    int sCount = second.count();
    if (fCount == 0) throw FACELIB_EXCEPTION("first vector is empty");
    if (sCount == 0) throw FACELIB_EXCEPTION("second vector is empty");

    int prime = 31657;

    std::vector<T> result;
    if (fCount == sCount)
    {
        result << first;
        result << second;
    }
    else if (fCount > sCount)
    {
        result << fCount;
        int counter = 0;
        for (int i = 0; i < fCount; i++)
        {
            result << second[counter];
            counter = (counter + prime) % sCount;
        }
    }
    return result;
}
*/

void Common::printBacktrace()
{
    void *array[10];
    size_t size = backtrace(array, 10);
    char** strings = backtrace_symbols(array, size);
    for (unsigned int i = 0; i < size; i++)
        std::cout << strings[i] << std::endl;
}
