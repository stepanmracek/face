#include "filterbank.h"


QVector<Map> FilterBank::getRealResponse(Map &map)
{
    QVector<Map> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Map re = map;
        re.applyFilter(realKernels[i]);
        result << re;
    }
    return result;
}

QVector<Matrix> FilterBank::getRealResponse(Matrix &mat)
{
    QVector<Matrix> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Matrix re;
        cv::filter2D(mat, re, CV_64F, realKernels[i]);
        result << re;
    }

    return result;
}

QVector<Map> FilterBank::getImagResponse(Map &map)
{
    QVector<Map> result;
    int n = imagKernels.count();
    for (int i = 0; i < n; i++)
    {
        Map im = map;
        im.applyFilter(imagKernels[i]);
        result << im;
    }
    return result;
}

QVector<Matrix> FilterBank::getImagResponse(Matrix &mat)
{
    QVector<Matrix> result;
    int n = imagKernels.count();
    for (int i = 0; i < n; i++)
    {
        Matrix im;
        cv::filter2D(mat, im, CV_64F, imagKernels[i]);
        result << im;
    }

    return result;
}

QVector<Map> FilterBank::getAbsResponse(Map &map)
{
    QVector<Map> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Map re = map;
        Map im = map;
        re.applyFilter(realKernels[i]);
        im.applyFilter(imagKernels[i]);

        Map a(map.w, map.h);
        int m = a.w*a.h;
        for (int j = 0; j < m; j++)
        {
            if (!re.flags[i] || !im.flags[i]) continue;

            a.flags[i] = true;
            a.values[i] = sqrt(re.values[i]*re.values[i] + im.values[i]*im.values[i]);
        }
        result << a;
    }
    return result;
}

QVector<Matrix> FilterBank::getAbsResponse(Matrix &mat)
{
    QVector<Matrix> result;
    int n = realKernels.count();
    for (int i = 0; i < n; i++)
    {
        Matrix re;
        cv::filter2D(mat, re, CV_64F, realKernels[i]);

        Matrix im;
        cv::filter2D(mat, im, CV_64F, imagKernels[i]);

        Matrix re2;
        cv::multiply(re, re, re2);
        Matrix im2;
        cv::multiply(im, im, im2);
        Matrix ab2 = re2 + im2;
        Matrix ab;
        cv::sqrt(ab2, ab);
        result << ab;
    }

    return result;
}
