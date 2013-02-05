/*
 * amlibTypedefs.h
 *
 *  Created on: 8.5.2012
 *      Author: stepo
 */

#ifndef AMLIBTYPEDEFS_H_
#define AMLIBTYPEDEFS_H_

#include <qmap.h>
#include <qpair.h>
#include <qvector.h>
#include <opencv2/opencv.hpp>

typedef QMap<QPair<int, int>, double> PixelIntensities;
typedef QVector<cv::Vec3i> Triangles;
typedef QVector<cv::Point2d> Shape;

#endif /* AMLIBTYPEDEFS_H_ */
