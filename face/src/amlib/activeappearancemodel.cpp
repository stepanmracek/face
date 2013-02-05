#include "activeappearancemodel.h"

#include <cassert>

#include "linalg/procrustes.h"
#include "linalg/delaunay.h"
#include "amlib/texturewarper.h"

ActiveAppearanceModel::ActiveAppearanceModel(QVector<Shape> &shapes, QVector<Matrix> &textures)
{
    qDebug() << "Active Appearance Model";

    // size of training data
    int n = shapes.count();
    assert(n == textures.count());
    assert(n > 1);

    // convert 2d shapes to column vectors
    QVector <Matrix> unitShapeVectors;
    QVector <Matrix> unscaledShapeVectors;
    for (int i = 0; i < n; i++)
    {
        Matrix colVector1 = shapeToVector(shapes[i]);
        unitShapeVectors.append(colVector1);

        Matrix colVector2 = shapeToVector(shapes[i]);
        unscaledShapeVectors.append(colVector2);
    }

    // -- Shape --
    // procrustes analysis of shape vectors
    Procrustes::procrustesAnalysis(unitShapeVectors, true, 1e-10, 10000);
    Matrix meanShapeVector = Procrustes::getMeanShape(unitShapeVectors);

    // pca of aligned shape vectors
    qDebug() << "Learning shape PCA";
    pcaOfShape.learn(unitShapeVectors, 0.98);

    // -- Texture --
    // procrustes analysis of unscaled shape vectors
    Procrustes::procrustesAnalysis(unscaledShapeVectors, false, 1e-10, 10000);

    // create mean unscaled shape and triangle mesh
    Matrix meanUnscaledVector = Procrustes::getMeanShape(unscaledShapeVectors);
    meanUnscaledShape = vectorToShape(meanUnscaledVector);
    triangleMesh = Delaunay::process(meanUnscaledShape);

    // calculate scale difference between scaled and unscaled mean shape
    RotateAndScaleCoefs rAnds = Procrustes::align(meanShapeVector, meanUnscaledVector);
    scaleCoef = rAnds.s;

    QVector<Matrix> normalizedTextures;
    int textureVecLen = -1;
    for (int i = 0; i < n; i++)
    {
    	qDebug() << "Normalizing texture" << (i+1) << "/" << "n";
    	// Warp each input shape to the unscaled mean
    	PixelIntensities piTexture = TextureWarper::matrixToPixelIntensities(textures[i], 0.0);
    	PixelIntensities pixelIntensities = TextureWarper::warp(shapes[i], piTexture, triangleMesh, meanUnscaledShape);

    	// read all point from the warped texture
    	Matrix textureVector = TextureWarper::pixelIntensitiesToColumnVector(pixelIntensities, 0.0);

    	// texture points count assertion (all should be the same)
    	if (textureVecLen == -1)
    		textureVecLen = textureVector.cols;
    	else
    		assert(textureVecLen == textureVector.cols);

    	// append to learning data
    	normalizedTextures << textureVector;
    }

    // pca of aligned warped textures
    qDebug() << "Learning texture PCA";
    pcaOfTexture.learn(normalizedTextures, 0.98);

    // fill helper textureIndexToPoint map
    PixelIntensities piTexture = TextureWarper::matrixToPixelIntensities(textures[0], 0.0);
    PixelIntensities pixelIntensities = TextureWarper::warp(shapes[0], piTexture, triangleMesh, meanUnscaledShape);
    TextureWarper::pixelIntensitiesToColumnVector(pixelIntensities, 0.0, &textureIndexToPoint);
}

ActiveAppearanceModelInstance ActiveAppearanceModel::createInstantiate(
    		Matrix &shapeParametres,
    		Matrix &textureParametres,
    		RotateAndScaleCoefs rotAndScale,
    		TranslationCoefs translation)
{
	assert(shapeParametres.rows == pcaOfShape.getModes());
	assert(textureParametres.rows == pcaOfTexture.getModes());

	ActiveAppearanceModelInstance instance(*this);

	// Shape generation
	Matrix backProjectedShape = pcaOfShape.backProject(shapeParametres);
	Procrustes::rotateAndScale(backProjectedShape, rotAndScale);
	Procrustes::translate(backProjectedShape, translation);
	instance.shape = vectorToShape(backProjectedShape);

	// Texture generation
	Matrix backProjectedTexture = pcaOfTexture.backProject(textureParametres);
	instance.pixelIntensities = TextureWarper::columnVectorToPixelIntensities(backProjectedTexture, textureIndexToPoint);
	return instance;
}


Matrix ActiveAppearanceModel::shapeToVector(Shape &points)
{
    int m = points.count();
    Matrix result = Matrix::zeros(m*2, 1);
    for (int i = 0; i < m; i++)
    {
        result(i) = points[i].x;
        result(m+i) = points[i].y;
    }
    return result;
}

Shape ActiveAppearanceModel::vectorToShape(Matrix &vector)
{
	Shape result;
	int m = vector.rows/2;
	for (int i = 0; i < m; i++)
	{
		cv::Point2d p;
        p.x = vector(i);
        p.y = vector(m+i);
		result << p;
	}
	return result;
}

void ActiveAppearanceModel::gradientOfTemplateA0()
{
    Matrix shapeParams = Matrix::zeros(pcaOfShape.getModes(), 1);
    Matrix textureParams = Matrix::zeros(pcaOfTexture.getModes(), 1);
	RotateAndScaleCoefs rotAndScale;
	rotAndScale.s = scaleCoef;

	ActiveAppearanceModelInstance instance = createInstantiate(shapeParams, textureParams, rotAndScale);
	Matrix img = instance.getBaseTextureImage();

	Matrix xgradient;
	cv::Sobel(img, xgradient, -1, 1, 0);
	cv::imshow("gradient", xgradient);
	cv::waitKey(0);

	Matrix ygradient;
	cv::Sobel(img, ygradient, -1, 0, 1);
	cv::imshow("gradient", ygradient);
	cv::waitKey(0);
}

void ActiveAppearanceModel::jacobians()
{
	//Matrix ada
}
