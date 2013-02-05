/*
 * testtexturewarper.h
 *
 *  Created on: 8.5.2012
 *      Author: stepo
 */

#ifndef TESTTEXTUREWARPER_H_
#define TESTTEXTUREWARPER_H_

#include <qdebug.h>

#include <linalg/matrixconverter.h>
#include <linalg/common.h>
#include <linalg/delaunay.h>
#include <amlib/texturewarper.h>

class TestTextureWarper
{
public:
	static void test()
	{
		Matrix img = MatrixConverter::imageToMatrix("/home/stepo/SVN/disp-stepan-mracek/test/testASM/02463d652-index.png");

		srand(time(0));
		cv::Scalar color;
		Shape points;
		Shape newPoints;
		int n = 50;
		for (int i = 0; i < n; i++)
		{
			cv::Point2d p;
			p.x = rand() % 200+275;
			p.y = rand() % 350+100;
			points.append(p);
			newPoints.append(p);
		}

		Triangles triangles = Delaunay::process(points);
		PixelIntensities pi = TextureWarper::matrixToPixelIntensities(img, 0.0);

		Matrix testImg = TextureWarper::pixelIntensitiesToMatrix(pi, 0.0);
		cv::imshow("test", testImg);
		cv::waitKey(0);

		for (int i = 1; i <= 20; i++)
		{
			qDebug() << "iteration, generating new point locations" << i;

			for (int j = 0; j < n; j++)
			{
				newPoints[j].x += rand() % 4 - 2;
				newPoints[j].y += rand() % 4 - 2;
			}

			qDebug() << "  warping";
			PixelIntensities newPi = TextureWarper::warp(points, pi, triangles, newPoints);
			Matrix result = TextureWarper::pixelIntensitiesToMatrix(newPi, 0);

			qDebug() << "  drawing result";
			/*cv::Scalar white; white[0] = 0.5;
			foreach(cv::Scalar_<int> t, triangles)
			{
				cv::line(result, newPoints[t[0]], newPoints[t[1]], white);
				cv::line(result, newPoints[t[1]], newPoints[t[2]], white);
				cv::line(result, newPoints[t[2]], newPoints[t[0]], white);
			}*/

			cv::imshow("test", result);
			if (cv::waitKey(1) == 27)
				break;
		}
	}
};


#endif /* TESTTEXTUREWARPER_H_ */
