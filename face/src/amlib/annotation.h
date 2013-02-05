/*
 * annotation.h
 *
 *  Created on: 6.5.2012
 *      Author: stepo
 */

#ifndef ANNOTATION_H_
#define ANNOTATION_H_

#include <QList>
#include <QString>
#include <QStringList>

#include "linalg/delaunay.h"
#include "linalg/common.h"
#include "amlibTypedefs.h"

class Annotation
{
public:
	QString winName;
	QString path;
	Triangles triangles;
	Shape meanShape;
	int currentImageIndex;
	QStringList imageNames;
	Shape currentShape;
	Matrix currentImage;
	cv::Point2d *currentMovingPoint;
	bool movingAllPoints;

	void loadAndDisplayImage(int index);
	void recalculateMeanShape();
	void recalculateTriangles();
	void registerKeys();
	void draw();
	void handleKey(int key);
	void saveCurrent();
	void saveTriangles();
	cv::Point2d *getNearestPoint(cv::Point2d point);

public:
	Annotation(const QString &path);
	//virtual ~Annotation() {};
};

#endif /* ANNOTATION_H_ */
