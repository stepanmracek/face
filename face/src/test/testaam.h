/*
 * testaam.h
 *
 *  Created on: 8.5.2012
 *      Author: stepo
 */

#ifndef TESTAAM_H_
#define TESTAAM_H_

#include <qstring.h>
#include <qstringlist.h>
#include <qdebug.h>
#include <cassert>

#include <amlib/activeappearancemodel.h>
#include <linalg/loader.h>
#include <linalg/procrustes.h>

double scale; int scaleTbVal;
double rot; int rotTbVal;
double shape1; int shape1TbVal;
double shape2; int shape2TbVal;
double texture1; int texture1TbVal;
double texture2; int texture2TbVal;

void createAndDisplayAAMInstance(ActiveAppearanceModel *aam)
{
    Matrix shapeParams = Matrix::zeros(aam->pcaOfShape.getModes(), 1);
	shapeParams(0) = shape1;
	shapeParams(1) = shape2;

    Matrix textureParams = Matrix::zeros(aam->pcaOfTexture.getModes(), 1);
	textureParams(0) = texture1;
	textureParams(1) = texture2;

	RotateAndScaleCoefs rotAndScale;
	rotAndScale.s = scale;
	rotAndScale.theta = rot;

	ActiveAppearanceModelInstance instance = aam->createInstantiate(shapeParams, textureParams, rotAndScale);
	cv::imshow("AAM instance", instance.getWarpedTextureImage());
}

void onScaleChange(int, void* aam)
{
	scale = scaleTbVal;
	if (scale <= 0) scale = 1;
	createAndDisplayAAMInstance((ActiveAppearanceModel*)aam);
}

void onRotateChange(int, void* aam)
{
	rot = (rotTbVal-50.0)/100.0;
	createAndDisplayAAMInstance((ActiveAppearanceModel*)aam);
}

void onShape1Change(int, void* aam)
{
    double l = ((ActiveAppearanceModel*)aam)->pcaOfShape.cvPca.eigenvalues.at<double>(0);
	shape1 = (shape1TbVal-50.0)/50.0*3*sqrt(l);
	createAndDisplayAAMInstance((ActiveAppearanceModel*)aam);
}

void onShape2Change(int, void* aam)
{
    double l = ((ActiveAppearanceModel*)aam)->pcaOfShape.cvPca.eigenvalues.at<double>(1);
	shape2 = (shape2TbVal-50.0)/50.0*3*sqrt(l);
	createAndDisplayAAMInstance((ActiveAppearanceModel*)aam);
}

void onTexture1Change(int, void* aam)
{
    double l = ((ActiveAppearanceModel*)aam)->pcaOfTexture.cvPca.eigenvalues.at<double>(0);
	texture1 = (texture1TbVal-50.0)/50.0*3*sqrt(l);
	createAndDisplayAAMInstance((ActiveAppearanceModel*)aam);
}

void onTexture2Change(int, void* aam)
{
    double l = ((ActiveAppearanceModel*)aam)->pcaOfTexture.cvPca.eigenvalues.at<double>(1);
	texture2 = (texture2TbVal-50.0)/50.0*3*sqrt(l);
	createAndDisplayAAMInstance((ActiveAppearanceModel*)aam);
}

class TestAAM
{
public:
	static void testLearn()
	{
		QString path("/home/stepo/SVN/disp-stepan-mracek/test/testASM");

		QStringList imageNames = Loader::listFiles(path, "*.png", true);
		QStringList shapeNames = Loader::listFiles(path, "*.png.shape", true);
		int n = imageNames.count();
		assert(n > 0);
		assert(n == shapeNames.count());

		QVector<Shape> shapes;
		QVector<Matrix> images;
		for (int i = 0; i < n; i++)
		{
            Matrix image = MatrixConverter::imageToMatrix(imageNames[i]);
			images << image;

			Matrix shapeVec = Vector::fromFile(shapeNames[i]);
			Shape shape = ActiveAppearanceModel::vectorToShape(shapeVec);
			shapes << shape;
		}

		ActiveAppearanceModel aam(shapes, images);
		qDebug() << "Done";
	}

	static void testGenerate()
	{
		QString path("/home/stepo/SVN/disp-stepan-mracek/test/testASM");

		QStringList imageNames = Loader::listFiles(path, "*.png", true);
		QStringList shapeNames = Loader::listFiles(path, "*.png.shape", true);
		int n = imageNames.count();
		assert(n > 0);
		assert(n == shapeNames.count());

		QVector<Shape> shapes;
		QVector<Matrix> images;
		for (int i = 0; i < n; i++)
		{
            Matrix image = MatrixConverter::imageToMatrix(imageNames[i]);
			images << image;

			Matrix shapeVec = Vector::fromFile(shapeNames[i]);
			Shape shape = ActiveAppearanceModel::vectorToShape(shapeVec);
			shapes << shape;
		}

		ActiveAppearanceModel aam(shapes, images);
		qDebug() << "Learning AAM Done";

		scaleTbVal = aam.scaleCoef; scale = aam.scaleCoef;
		rotTbVal = 50; rot = 0.0;
		shape1TbVal = 50; shape1 = 0.0;
		shape2TbVal = 50; shape2 = 0.0;
		texture1TbVal = 50; texture1 = 0.0;
		texture2TbVal = 50; texture2 = 0.0;

		cv::namedWindow("AAM instance");
		cv::createTrackbar("scale", "AAM instance", &scaleTbVal, aam.scaleCoef*2, onScaleChange, &aam);
		cv::createTrackbar("rotation", "AAM instance", &rotTbVal, 100, onRotateChange, &aam);
		cv::createTrackbar("shape1", "AAM instance", &shape1TbVal, 100, onShape1Change, &aam);
		cv::createTrackbar("shape2", "AAM instance", &shape2TbVal, 100, onShape2Change, &aam);
		cv::createTrackbar("texture1", "AAM instance", &texture1TbVal, 100, onTexture1Change, &aam);
		cv::createTrackbar("texture2", "AAM instance", &texture2TbVal, 100, onTexture2Change, &aam);

		createAndDisplayAAMInstance(&aam);
		while(cv::waitKey(30) != 27)
		{

		}
	}

	static void testGradient()
	{
		QString path("/home/stepo/SVN/disp-stepan-mracek/test/testASM");

		QStringList imageNames = Loader::listFiles(path, "*.png", true);
		QStringList shapeNames = Loader::listFiles(path, "*.png.shape", true);
		int n = imageNames.count();
		assert(n > 0);
		assert(n == shapeNames.count());

		QVector<Shape> shapes;
		QVector<Matrix> images;
		for (int i = 0; i < n; i++)
		{
            Matrix image = MatrixConverter::imageToMatrix(imageNames[i]);
			images << image;

			Matrix shapeVec = Vector::fromFile(shapeNames[i]);
			Shape shape = ActiveAppearanceModel::vectorToShape(shapeVec);
			shapes << shape;
		}

		ActiveAppearanceModel aam(shapes, images);
		qDebug() << "Learning AAM Done";
		aam.gradientOfTemplateA0();
	}
};


#endif /* TESTAAM_H_ */
