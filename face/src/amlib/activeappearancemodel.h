#ifndef ACTIVEAPPEARANCEMODEL_H
#define ACTIVEAPPEARANCEMODEL_H

#include <QVector>
#include <QPair>
#include <QMap>

#include <opencv/cv.h>

#include "linalg/pca.h"
#include "linalg/procrustes.h"
#include "amlibTypedefs.h"
#include "texturewarper.h"

class ActiveAppearanceModelInstance;

class ActiveAppearanceModel
{
private:
	QMap<int, QPair<int, int> > textureIndexToPoint;

public:
	PCA pcaOfShape;
	PCA pcaOfTexture;

	Shape meanUnscaledShape;
	Triangles triangleMesh;

	ActiveAppearanceModel(QVector<Shape> &shapes, QVector<Matrix> &textures);

	ActiveAppearanceModelInstance createInstantiate(
			Matrix &shapeParametres,
			Matrix &textureParametres,
			RotateAndScaleCoefs rotAndScale = RotateAndScaleCoefs(),
			TranslationCoefs translation = TranslationCoefs());

	static Matrix shapeToVector(Shape &points);
	static Shape vectorToShape(Matrix &vector);

	// For fitting
	void gradientOfTemplateA0();
	void jacobians();
	// For fitting

	/**
	 * Scale between mean normalized shape and unscaled mean texture shape
	 */
	double scaleCoef;
};

class ActiveAppearanceModelInstance
{
private:
	ActiveAppearanceModel &model;

public:
	Shape shape;

	PixelIntensities pixelIntensities;

	ActiveAppearanceModelInstance(ActiveAppearanceModel &model) :
		model(model) {}

	Matrix getBaseTextureImage()
	{
		return TextureWarper::pixelIntensitiesToMatrix(pixelIntensities, 0.0);
	}

	Matrix getWarpedTextureImage()
	{
		PixelIntensities warped = TextureWarper::warp(
				model.meanUnscaledShape, pixelIntensities, model.triangleMesh, shape);
		return TextureWarper::pixelIntensitiesToMatrix(warped, 0.0);
	}
	//Matrix textureMatrix;
	//Matrix textureVector;
};

#endif // ACTIVEAPPEARANCEMODEL_H
