/*
 * evaluatefeaturestability.h
 *
 *  Created on: 10.5.2012
 *      Author: stepo
 */

#ifndef EVALUATEFEATURESTABILITY_H_
#define EVALUATEFEATURESTABILITY_H_

#include <QVector>
#include <QList>
#include <QDebug>
#include <QMap>
#include <qglobal.h>

#include <cassert>
#include <opencv2/opencv.hpp>

#include "linalg/common.h"
#include "linalg/metrics.h"
#include "linalg/loader.h"
#include "linalg/pca.h"
#include "linalg/ldaofpca.h"
#include "linalg/icaofpca.h"
#include "linalg/vector.h"
#include "biometrics/eerpotential.h"
#include "biometrics/biodataprocessing.h"
#include "biometrics/featureextractor.h"
#include "biometrics/evaluation.h"
#include "biometrics/featurevectorfusion.h"
#include "biometrics/scorelevefusion.h"

class EvaluateFeatureStability
{
public:
	static void evaluate()
	{
		QVector<Matrix> allImages;
		QVector<int> allClasses;
		Loader::loadImages("/run/media/stepo/frgc/thermo/germany", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/fit", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/nd", allImages, &allClasses, "*.png", true, "-");
		//Loader::loadImages("/run/media/stepo/frgc/thermo/equinox", allImages, &allClasses, "*.png", true, "-");

		QMap<int, QVector<Matrix> > subjectToImages;
		int n = allImages.count();
		for (int i = 0; i < n; i++)
		{
			int subjID = allClasses[i];
			if (!subjectToImages.contains(subjID))
			{
				subjectToImages[subjID] = QVector<Matrix>();
			}

			subjectToImages[subjID].append(allImages[i]);
		}

		int rows = allImages[0].rows;
		int cols = allImages[0].cols;
		Matrix meanOfStdDeviations = Matrix::zeros(rows, cols, CV_64F);
		QList<int> uniquesSubjects = subjectToImages.keys();
		foreach (int subjID, uniquesSubjects)
		{
			QVector<Matrix> &imagesForSubject = subjectToImages[subjID];
			int iCount = imagesForSubject.count();
			assert(iCount > 0);

			QMap<QPair<int, int>, QVector<double> > stdDeviationsForSubject;
			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					QVector<double> values;
					for (int i = 0; i < iCount; i++)
					{
						values << (imagesForSubject[i](r,c));
					}

					QPair<int, int> key(r,c);
					if (!stdDeviationsForSubject.contains(key))
						stdDeviationsForSubject[key] = QVector<double>();
					stdDeviationsForSubject[key] << values;
				}
			}

			for (int r = 0; r < rows; r++)
			{
				for (int c = 0; c < cols; c++)
				{
					QPair<int, int> key(r,c);
					double mean = Vector::meanValue(stdDeviationsForSubject[key]);
					meanOfStdDeviations(r,c) += mean;
				}
			}
		}

		meanOfStdDeviations /= uniquesSubjects.count();
		cv::imshow("mean of std deviations", 1-MatrixConverter::columnVectorToMatrix(meanOfStdDeviations, 50));
		cv::waitKey(0);

		PassExtractor passExtractor;
		QVector<Template> templates = Template::createTemplates(allImages, allClasses, passExtractor);
		EERPotential eerPotential(templates);
		Matrix eerPotVector = Vector::fromQVector(eerPotential.scores);
		Matrix eerPotMatrix = MatrixConverter::columnVectorToMatrix(eerPotVector, 50);
		cv::imshow("EER potential", eerPotMatrix);
		cv::waitKey(0);

	}
};

#endif /* EVALUATEFEATURESTABILITY_H_ */
