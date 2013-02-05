/*
 * texturewarper.cpp
 *
 *  Created on: 8.5.2012
 *      Author: stepo
 */

#include "texturewarper.h"

#include <qpair.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cassert>

#include <linalg/vector.h>

void minMax(double v1, double v2, double v3, double &min, double &max)
{
	max = v1 > v2 ? v1 : v2;
	max = max > v3 ? max : v3;

	min = v1 < v2 ? v1 : v2;
	min = min < v3 ? min : v3;
}

PixelIntensities TextureWarper::warp(Shape &baseShape, PixelIntensities &baseTexture, Triangles &triangles, Shape &newShape)
{
	int tCount = triangles.count();
	int vCount = baseShape.count();
	assert(vCount == newShape.count());

	PixelIntensities result;

	for (int t = 0; t < tCount; t++)
	{
		std::vector<cv::Point2f> triangle;
		int v1 = triangles[t][0];
		int v2 = triangles[t][1];
		int v3 = triangles[t][2];
		double x1 = newShape[v1].x;
		double y1 = newShape[v1].y;
		double x2 = newShape[v2].x;
		double y2 = newShape[v2].y;
		double x3 = newShape[v3].x;
		double y3 = newShape[v3].y;
		triangle.push_back(cv::Point2f(x1, y1));
		triangle.push_back(cv::Point2f(x2, y2));
		triangle.push_back(cv::Point2f(x3, y3));

		double minx, miny, maxx, maxy;
		minMax(x1, x2, x3, minx, maxx);
		minMax(y1, y2, y3, miny, maxy);
		for (int x = minx; x <= maxx; x++)
		{
			for (int y = miny; y <= maxy; y++)
			{
				cv::Point2f p(x,y);
				if (cv::pointPolygonTest(triangle, p, false) >= 0)
				{
					// linear interpolation
					double denominator = -(x2*y3)+(x2*y1)+(x1*y3)+(x3*y2)-(x3*y1)-(x1*y2);
					double beta = ((y*x3)-(x1*y)-(x3*y1)-(y3*x)+(x1*y3)+(x*y1))/denominator;
					double gamma = ((x*y2)-(x*y1)-(x1*y2)-(x2*y)+(x2*y1)+(x1*y))/denominator;
					double alfa = 1-(beta+gamma);

					// find equivalent point in the original mesh/texture
					double ox1 = baseShape[v1].x;
					double oy1 = baseShape[v1].y;
					double ox2 = baseShape[v2].x;
					double oy2 = baseShape[v2].y;
					double ox3 = baseShape[v3].x;
					double oy3 = baseShape[v3].y;
					double ox = alfa*ox1+beta*ox2+gamma*ox3;
					double oy = alfa*oy1+beta*oy2+gamma*oy3;
					QPair<int, int> originalPoint(ox, oy);

					if (baseTexture.contains(originalPoint))
					{
						QPair<int, int> newPoint(x,y);
						result[newPoint] = baseTexture[originalPoint];
					}
				}
			}
		}
	}

	return result;
}

PixelIntensities TextureWarper::matrixToPixelIntensities(Matrix &image, double nullValue)
{
	PixelIntensities result;
	for (int r = 0; r < image.rows; r++)
	{
		for (int c = 0; c < image.cols; c++)
		{
            double val = image(r,c);
			if (val != nullValue)
				result[QPair<int,int>(c,r)] = val;
		}
	}
	return result;
}


Matrix TextureWarper::pixelIntensitiesToMatrix(PixelIntensities &pixelIntensities, double defaultValue)
{
	QList<QPair<int,int> > keys = pixelIntensities.keys();
	int len = keys.count();
	int minX, minY, maxX, maxY;
	minX = minY = INT_MAX;
	maxX = maxY = INT_MIN;
	for (int i = 0; i < len; i++)
	{
		int x = keys[i].first;
		int y = keys[i].second;
		if (x > maxX) maxX = x;
		if (x < minX) minX = x;
		if (y > maxY) maxY = y;
		if (y < minY) minY = y;
	}
	int width = maxX-minX;
	int height = maxY-minY;
    Matrix result = Matrix::ones(height, width);
	result = result * defaultValue;

	for (int i = 0; i < len; i++)
	{
		int x = keys[i].first;
		int y = keys[i].second;
		int realX = x-minX;
		int realY = y-minY;
        result(realY, realX) = pixelIntensities[keys[i]];
	}
	return result;
}

Matrix TextureWarper::pixelIntensitiesToColumnVector(
		PixelIntensities &pixelIntensities,
		double defaultValue,
		QMap<int, QPair<int,int> > *inverseTransform)
{
	QList<QPair<int,int> > keys = pixelIntensities.keys();
	int len = keys.count();
	int minX, minY, maxX, maxY;
	minX = minY = INT_MAX;
	maxX = maxY = INT_MIN;
	for (int i = 0; i < len; i++)
	{
		int x = keys[i].first;
		int y = keys[i].second;
		if (x > maxX) maxX = x;
		if (x < minX) minX = x;
		if (y > maxY) maxY = y;
		if (y < minY) minY = y;
	}

	int index = 0;
	QVector<double> values;
	for (int x = minX; x <= maxX; x++)
	{
		for (int y = minY; y <= maxY; y++)
		{
			QPair<int,int> p(x,y);
			if (pixelIntensities.contains(p))
			{
				values << pixelIntensities[p];

				if (inverseTransform != NULL)
					inverseTransform->insert(index, p);
				index++;
			}
		}
	}

	/*QVector<double> values;
	Matrix image = pixelIntensitiesToMatrix(pixelIntensities, defaultValue);

	int index = 0;
	for (int r = 0; r < image.rows; r++)
	{
		for (int c = 0; c < image.cols; c++)
		{
            double val = image(r,c);
			if (val != defaultValue)
			{
				values << val;
				if (inverseTransform != NULL)
					inver
			}
		}
	}*/

	Matrix result = Vector::fromQVector(values);
	return result;
}

PixelIntensities TextureWarper::columnVectorToPixelIntensities(
		Matrix &colVector,
		QMap<int, QPair<int,int> > &inverseTransform)
{
	PixelIntensities result;
	for (int i = 0; i < colVector.rows; i++)
	{
		QPair<int,int> p = inverseTransform[i];
        result[p] = colVector(i);
	}
	return result;
}
