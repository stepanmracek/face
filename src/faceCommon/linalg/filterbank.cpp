#include "filterbank.h"


QVector<Map> FilterBank::getRealResponse(const Map &map) const
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

QVector<Matrix> FilterBank::getRealResponse(const Matrix &mat, const QVector<int> *selectedIndicies) const
{
    QVector<Matrix> result;
    int n = realKernels.count();

    if (selectedIndicies)
    {
        foreach(int i, *selectedIndicies)
        {
            Matrix re;
            cv::filter2D(mat, re, CV_64F, realKernels[i]);
            result << re;
        }
    }
    else
    {
        for (int i = 0; i < n; i++)
        {
            Matrix re;
            cv::filter2D(mat, re, CV_64F, realKernels[i]);
            result << re;
        }
    }

    return result;
}

QVector<Map> FilterBank::getImagResponse(const Map &map) const
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

QVector<Matrix> FilterBank::getImagResponse(const Matrix &mat, const QVector<int> *selectedIndicies) const
{
    QVector<Matrix> result;
    int n = imagKernels.count();

    if (selectedIndicies)
    {
        foreach(int i, *selectedIndicies)
        {
            Matrix re;
            cv::filter2D(mat, re, CV_64F, imagKernels[i]);
            result << re;
        }
    }
    else
    {
        for (int i = 0; i < n; i++)
        {
            Matrix im;
            cv::filter2D(mat, im, CV_64F, imagKernels[i]);
            result << im;
        }
    }

    return result;
}

QVector<Map> FilterBank::getAbsResponse(const Map &map) const
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

QVector<Matrix> FilterBank::getAbsResponse(const Matrix &mat, const QVector<int> *selectedIndicies) const
{
    QVector<Matrix> result;
    int n = realKernels.count();
    if (selectedIndicies)
    {
        foreach(int i, *selectedIndicies)
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
    }
    else
    {
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
    }

    return result;
}

QVector<Matrix> FilterBank::getAbsRealImagResponse(const Matrix &mat,
                                                   const QVector<int> *absSelectedIndicies,
                                                   const QVector<int> *realSelectedIndicies,
                                                   const QVector<int> *imagSelectedIndicies) const
{
    QVector<Matrix> result;

    result += getAbsResponse(mat, absSelectedIndicies);
    result += getRealResponse(mat, realSelectedIndicies);
    result += getImagResponse(mat, imagSelectedIndicies);

    return result;
}
