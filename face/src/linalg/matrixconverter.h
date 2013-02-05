#ifndef MATRIXCONVERTER_H
#define MATRIXCONVERTER_H

#include <QVector>
#include <QString>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "common.h"

class MatrixConverter
{
public:
    static Matrix grayscaleImageToDoubleMatrix(cv::Mat &in)
    {
        Matrix out(in.rows, in.cols);

        for (int r = 0; r < in.rows; r++)
        {
            for (int c = 0; c < in.cols; c++)
            {
                out(r, c) = in.at<unsigned char>(r, c)/255.0;
            }
        }
        return out;
    }

    static Matrix matrixToColumnVector(Matrix &in)
    {
        Matrix out(in.rows * in.cols, 1, CV_64F);
        int i = 0;
        for (int r = 0; r < in.rows; r++)
        {
            for (int c = 0; c < in.cols; c++)
            {
                out(i) = in(r,c);
                i++;
            }
        }
        return out;
    }

    static Matrix columnVectorToMatrix(Matrix &in, int cols)
    {
        int rows = in.rows/cols;
        Matrix out(rows, cols, CV_64F);
        int i = 0;
        for (int r = 0; r < out.rows; r++)
        {
            for (int c = 0; c < out.cols; c++)
            {
                out(r,c) = in(i);
                i++;
            }
        }
        return out;
    }

    static Matrix imageToMatrix(const QString &path)
    {
        cv::Mat img = cv::imread(path.toStdString(), CV_LOAD_IMAGE_GRAYSCALE);
        return MatrixConverter::grayscaleImageToDoubleMatrix(img);
    }

    static Matrix imageToColumnVector(const QString &path)
    {
        Matrix m = imageToMatrix(path);
        return matrixToColumnVector(m);
    }

    static Matrix columnVectorsToDataMatrix(QVector<Matrix> &vectors)
    {
        int cols = vectors.count();
        int rows = vectors[0].rows;

        assert(rows > 0);
        assert(cols > 0);

        Matrix data(rows, cols, CV_64F);

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < cols; c++)
            {
                data(r,c) = vectors[c](r,0);
            }
        }

        return data;
    }
};

#endif // MATRIXCONVERTER_H
