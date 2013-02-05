/*
 * texturewarper.h
 *
 *  Created on: 8.5.2012
 *      Author: stepo
 */

#ifndef TEXTUREWARPER_H_
#define TEXTUREWARPER_H_

#include <qmap.h>
#include <qpair.h>
#include <qvector.h>
#include <opencv2/opencv.hpp>

#include <linalg/common.h>
#include "amlibTypedefs.h"

class TextureWarper
{
public:
	static PixelIntensities warp(
			Shape &baseShape,
			PixelIntensities &baseTexture,
			Triangles &triangles,
			Shape &newShape);

	static PixelIntensities matrixToPixelIntensities(
			Matrix &image,
			double nullValue = -1.0);

	static Matrix pixelIntensitiesToMatrix(
			PixelIntensities &pixelIntensities,
			double defaultValue = -1.0);

	static Matrix pixelIntensitiesToColumnVector(
			PixelIntensities &pixelIntensities,
			double defaultValue,
			QMap<int, QPair<int,int> > *inverseTransform = NULL);

	static PixelIntensities columnVectorToPixelIntensities(
			Matrix &colVector,
			QMap<int, QPair<int,int> > &inverseTransform);
};

#endif /* TEXTUREWARPER_H_ */

