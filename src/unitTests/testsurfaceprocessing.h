#ifndef TESTSURFACEPROCESSING_H
#define TESTSURFACEPROCESSING_H

#include "facelib/mesh.h"
#include "facelib/surfaceprocessor.h"

class TestSurfaceProcessing
{
public:
    static void testNormals()
    {
        Mesh m = Mesh::fromOBJ("/home/stepo/face2.obj");
        //SurfaceProcessor::smooth(m, 0.5, 20);
        SurfaceProcessor::calculateNormals(m, 20);

        MapConverter converter;
        Map map = SurfaceProcessor::depthmap(m, converter, 2, Curvature);
        qDebug() << map.minValue() << map.maxValue();
        Matrix mat = map.toMatrix(0, 0, 0.1);
        //mat = Matrix::ones(mat.rows, mat.cols) - mat;
        cv::imshow("curvature from 3d", mat);
        cv::waitKey();

        //Map depth = SurfaceProcessor::depthmap(m, converter, 2, ZCoord);
        //CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(depth);
    }
};

#endif // TESTSURFACEPROCESSING_H
