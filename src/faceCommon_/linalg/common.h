#ifndef COMMON_H
#define COMMON_H

#include <QVector>
#include <QMap>

#include <opencv2/opencv.hpp>

typedef cv::Mat_<double> Matrix;
typedef cv::Mat_<cv::Vec3b> ImageBGR;
typedef cv::Mat_<uint8_t> ImageGrayscale;

class Common
{
public:
    static double absSum(Matrix &m);

    static void printMatrix(CvMat *m);
    static void printMatrix(const Matrix &m);
    static bool matrixContainsNan(const Matrix &m);

    static void savePlot(const QVector<double> &x, const QVector<double> &y, const QVector<double> &z, const QString &path);
    static void savePlot(const QVector<cv::Point3d> &values, const QString &path, bool append);
    static void savePlot(const QVector<double> &x, const QVector<double> &y, const QString &path);
    static void savePlot(const QVector<double> values[], int axisCount, const QString &path);
    static void saveMatrix(const Matrix &m, const QString &path);
    static void saveMap(QMap<double, double> &map, const QString &path);

    static Matrix loadMatrix(const QString &path);
};

double euclideanDistance(const cv::Point3d &p1, const cv::Point3d &p2);
double euclideanDistance(const cv::Point &p1, const cv::Point &p2);

#endif // COMMON_H
