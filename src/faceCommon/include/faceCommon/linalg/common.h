#ifndef COMMON_H
#define COMMON_H

#include <limits>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <execinfo.h>

#ifndef NAN
    #define NAN (std::numeric_limits<double>::quiet_NaN());
#endif

#ifndef M_PI
    #define M_PI       3.14159265358979323846
    #define M_PI_2     1.57079632679489661923
    #define M_1_PI     0.318309886183790671538
    #define M_SQRT2    1.41421356237309504880
#endif

#define FACELIB_EXCEPTION(msg) std::runtime_error(__FILE__ + std::string(", line: ") + std::to_string((long long int)__LINE__) + std::string(": ") + std::string(msg))

typedef cv::Mat_<double> Matrix;
typedef cv::Mat_<cv::Vec3b> ImageBGR;
typedef cv::Mat_<unsigned char> ImageGrayscale;

namespace Face {
namespace LinAlg {

class Common
{
public:
    static double absSum(Matrix &m);

    static void printMatrix(const Matrix &m);
    static bool matrixContainsNan(const Matrix &m);

    static void savePlot(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z, const std::string &path);
    static void savePlot(const std::vector<cv::Point3d> &values, const std::string &path, bool append);
    static void savePlot(const std::vector<double> &x, const std::vector<double> &y, const std::string &path);
    static void savePlot(const std::vector<double> values[], int axisCount, const std::string &path);
    static void saveMatrix(const Matrix &m, const std::string &path);
    static void saveMap(std::map<double, double> &map, const std::string &path);

    static Matrix loadMatrix(const std::string &path, const std::string &storageKey = "m");

    static double median(std::vector<double> &values);

    template <class T>
    static std::vector<T> balanceSizesAndJoin(const std::vector<T> &first, const std::vector<T> &second)
    {
        int fCount = first.size();
        int sCount = second.size();
        if (fCount == 0) throw FACELIB_EXCEPTION("first vector is empty");
        if (sCount == 0) throw FACELIB_EXCEPTION("second vector is empty");

        int prime = 31657;

        std::vector<T> result;
        if (fCount == sCount)
        {
            result.insert(result.end(), first.begin(), first.end());
            result.insert(result.end(), second.begin(), second.end());
        }
        else if (fCount > sCount)
        {
            result.insert(result.end(), first.begin(), first.end());
            int counter = 0;
            for (int i = 0; i < fCount; i++)
            {
                result.push_back(second[counter]);
                counter = (counter + prime) % sCount;
            }
        }
        else if (sCount > fCount)
        {
            result.insert(result.end(), second.begin(), second.end());
            int counter = 0;
            for (int i = 0; i < sCount; i++)
            {
                result.push_back(first[counter]);
                counter = (counter + prime) % fCount;
            }
        }
        return result;
    }

    static void printBacktrace();
};

double euclideanDistance(const cv::Point3d &p1, const cv::Point3d &p2);
double euclideanDistance(const cv::Point &p1, const cv::Point &p2);

}
}

#endif // COMMON_H
