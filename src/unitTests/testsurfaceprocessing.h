#ifndef TESTSURFACEPROCESSING_H
#define TESTSURFACEPROCESSING_H

#include "facelib/mesh.h"
#include "facelib/surfaceprocessor.h"
#include "linalg/kernelgenerator.h"
#include "linalg/histogram.h"

class TestSurfaceProcessing
{
public:
    static void testNormals()
    {
        Mesh m = Mesh::fromOBJ("/home/stepo/face.obj");
        SurfaceProcessor::smooth(m, 1.0, 10);
        SurfaceProcessor::calculateNormals(m, 9);

        MapConverter converter;
        Map map = SurfaceProcessor::depthmap(m, converter, 2, Curvature);
        qDebug() << map.minValue() << map.maxValue();
        Matrix mat = map.toMatrix();
        //mat = Matrix::ones(mat.rows, mat.cols) - mat;
        cv::imshow("curvature from 3d", mat);
        cv::waitKey();

        //Map depth = SurfaceProcessor::depthmap(m, converter, 2, ZCoord);
        //CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(depth);
    }

    static void testPclCurvatures()
    {
        Mesh mesh = Mesh::fromBINZ("/home/stepo/data/frgc/spring2004/zbin-aligned/02463d652.binz");
        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(mesh, converter, 2, ZCoord);
        Matrix smoothKernel = KernelGenerator::gaussianKernel(7);
        depth.applyFilter(smoothKernel, 7, true);

        CurvatureStruct cs = SurfaceProcessor::calculateCurvatures(depth);
        cs.curvaturePcl.bandPass(0, 0.0025, false, false);
        cv::imshow("index", cs.curvatureIndex.toMatrix());
        cv::imshow("pcl", cs.curvaturePcl.toMatrix(0, 0, 0.0025));
        //Histogram pclHistogram(cs.curvaturePcl.getUsedValues(), 20, false);
        //Common::savePlot(pclHistogram.histogramValues, pclHistogram.histogramCounter, "pclHistogram");
        cv::waitKey();
    }
};

#endif // TESTSURFACEPROCESSING_H
