/*
 * annotation.cpp
 *
 *  Created on: 6.5.2012
 *      Author: stepo
 */

#include "annotation.h"

#include <QDir>
#include <QFile>
#include <QDebug>

#include "linalg/loader.h"
#include "linalg/common.h"
#include "activeappearancemodel.h"
#include "linalg/procrustes.h"

void draw(const QString &winName, Matrix &image, Shape &shape, Triangles &triangles);

Annotation::Annotation(const QString &path) : path(path)
{
	qDebug() << "Annotation" << path;
	QString aamFilePath = path + QDir::separator() + "aamTriangles";
	if (QFile::exists(aamFilePath))
	{
		QFile f(aamFilePath);
	    bool opened = f.open(QIODevice::ReadOnly);
	    assert(opened);
	    QTextStream in(&f);

		QVector<int> first;
		QVector<int> second;
		QVector<int> third;

		int firstVal;
		int secondVal;
		int thirdVal;

		while (!in.atEnd())
		{
			in >> firstVal;
			in >> secondVal;
			in >> thirdVal;

			if (in.status() == QTextStream::ReadPastEnd)
				break;

			first << firstVal;
			second << secondVal;
			third << thirdVal;
		}
		f.close();

		int n = first.count();
		assert(n == second.count());
		assert(n == third.count());

		for (int i = 0; i < n; i++)
		{
            cv::Vec3i t(first[i], second[i], third[i]);
			triangles << t;
		}
	}

	winName = "Annotation";
	cv::namedWindow(winName.toStdString());
	registerKeys();

	recalculateMeanShape();

	imageNames = Loader::listFiles(path, "*.png", false);
	assert(imageNames.count() > 0);
	currentImageIndex = 0;

	loadAndDisplayImage(currentImageIndex);

	while(1)
	{
		int key = cv::waitKey(30);
		if (key == 27) break;
		handleKey(key);
	}
}

void Annotation::handleKey(int key)
{
	switch (key) {
		case 'b':
			if (currentImageIndex > 0)
				loadAndDisplayImage(currentImageIndex - 1);
			break;
		case 'n':
			if (currentImageIndex < (imageNames.count() -1))
				loadAndDisplayImage(currentImageIndex + 1);
			break;
		case 's':
			saveCurrent();
			break;
		case 't':
			saveCurrent();
			recalculateTriangles();
			saveTriangles();
			break;
	}
}

void Annotation::saveCurrent()
{
	QString currentShapePath = path + QDir::separator() + imageNames[currentImageIndex] + ".shape";
	Matrix currentShapeVector = ActiveAppearanceModel::shapeToVector(currentShape);
	Vector::toFile(currentShapeVector, currentShapePath);
}

void Annotation::loadAndDisplayImage(int index)
{
	assert(index >= 0);
	assert(index < imageNames.count());

	currentMovingPoint = NULL;
	movingAllPoints = false;

	QString pathToImage = path + QDir::separator() + imageNames[index];
	bool fileExists = QFile::exists(pathToImage);
	assert(fileExists);

    currentImage = MatrixConverter::imageToMatrix(pathToImage);
	currentImageIndex = index;

	QString pathToShape = pathToImage + ".shape";
	if (QFile::exists(pathToShape))
	{
		Matrix curShapeVec = Vector::fromFile(pathToShape);
		currentShape = ActiveAppearanceModel::vectorToShape(curShapeVec);
	}
	else
	{
		currentShape.clear();
		for (int i = 0; i < meanShape.count(); i++)
		{
			cv::Point2d point = meanShape[i];
			point.x += currentImage.cols/2;
			point.y += currentImage.rows/2;
			currentShape << point;
		}
	}

	::draw(winName, currentImage, currentShape, triangles);
}

void Annotation::recalculateMeanShape()
{
	QStringList shapeFiles = Loader::listFiles(path, "*.png.shape", true);
	QVector<Matrix> shapeVectors;
	int len;
	for (int i = 0; i < shapeFiles.count(); i++)
	{
		Matrix shapeVec = Vector::fromFile(shapeFiles[i]);
		if (i == 0)
		{
			len = shapeVec.rows;
			shapeVectors << shapeVec;
		}
		else
		{
			if (shapeVec.rows == len)
				shapeVectors << shapeVec;
		}
	}

	if (shapeVectors.count() > 0)
	{
        Procrustes::procrustesAnalysis(shapeVectors, false, 1e-10, 10000);
		Matrix meanVec = Procrustes::getMeanShape(shapeVectors);
		meanShape = ActiveAppearanceModel::vectorToShape(meanVec);
	}
}

cv::Point2d *Annotation::getNearestPoint(cv::Point2d point)
{
	int nearestIndex = -1;
	double distance = 1e300;
	for (int i = 0; i < currentShape.count(); i++)
	{
		double d = pow(point.x-currentShape[i].x, 2) + pow(point.y-currentShape[i].y, 2);
		if (d < distance)
		{
			distance = d;
			nearestIndex = i;
		}
	}

	if (nearestIndex >= 0)
		return &(currentShape[nearestIndex]);
	else
		return NULL;
}

int lastX;
int lastY;

void onMouse(int event, int x, int y, int, void* data)
{
	Annotation *annotation = (Annotation *)data;
	cv::Point2d p(x,y);

	switch (event)
	{
	case CV_EVENT_LBUTTONUP:
		qDebug() << "newPoint" << p.x << p.y;
		annotation->currentShape << p;
		annotation->draw();
		break;

	case CV_EVENT_RBUTTONDOWN:
		annotation->currentMovingPoint = annotation->getNearestPoint(p);
		break;
	case CV_EVENT_RBUTTONUP:
		annotation->currentMovingPoint = NULL;
		break;


	case CV_EVENT_MBUTTONDOWN:
		annotation->movingAllPoints = true;
		lastX = x;
		lastY = y;
		break;
	case CV_EVENT_MBUTTONUP:
		annotation->movingAllPoints = false;
		break;

	case CV_EVENT_MOUSEMOVE:
		if (annotation->currentMovingPoint != NULL)
		{
			annotation->currentMovingPoint->x = x;
			annotation->currentMovingPoint->y = y;
			annotation->draw();
		}
		else if (annotation->movingAllPoints)
		{
			int dx = x-lastX;
			int dy = y-lastY;

			for (int i = 0; i < annotation->currentShape.count(); i++)
			{
				annotation->currentShape[i].x += dx;
				annotation->currentShape[i].y += dy;
			}

			annotation->draw();
			lastX = x;
			lastY = y;
		}
		break;
}
}

void Annotation::recalculateTriangles()
{
	recalculateMeanShape();
	triangles = Delaunay::process(meanShape);
	draw();
}

void Annotation::saveTriangles()
{
	QString trianglesPath = path + QDir::separator() + "aamTriangles";
	qDebug() << "Saving triangles to " << trianglesPath;
	QFile f(trianglesPath);
	f.open(QIODevice::WriteOnly);
	QTextStream out(&f);
	for (int t = 0; t < triangles.count(); t++)
	{
		out << triangles[t][0] << ' ' << triangles[t][1] << ' ' << triangles[t][2] << '\n';
	}

	out.flush();
	f.close();
}

void Annotation::registerKeys()
{
	cv::setMouseCallback(winName.toStdString(), onMouse, this);
}

void Annotation::draw()
{
	::draw(winName, currentImage, currentShape, triangles);
}

void draw(const QString &winName, Matrix &image, Shape &shape, Triangles &triangles)
{
	Matrix canvas = image.clone();

	for (int t = 0; t < triangles.count(); t++)
	{
		int v1	= triangles[t][0];
		int v2	= triangles[t][1];
		int v3	= triangles[t][2];

		if (v1 < shape.count() && v2 < shape.count() && v3 < shape.count())
		{
			cv::line(canvas, shape[v1], shape[v2], 1.0);
			cv::line(canvas, shape[v2], shape[v3], 1.0);
			cv::line(canvas, shape[v1], shape[v3], 1.0);
		}
	}
	for (int v = 0; v < shape.count(); v++)
	{
		cv::circle(canvas, shape[v], 3, 1.0);
	}

	cv::imshow(winName.toStdString(), canvas);
}
