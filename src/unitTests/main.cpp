#include "testvector.h"
#include "testprocrustes.h"
#include "testpca.h"
#include "testmetrics.h"
#include "testdelaunay.h"
#include "testlda.h"
#include "testsvd.h"
#include "testgaoptimization.h"
#include "testbiodataprocessing.h"
#include "testdiscriminativepotential.h"
#include "testica.h"
#include "testsvm.h"
#include "testgaborwavelet.h"
#include "testfacefeaturesdetection.h"
#include "testmorphablefacemodel.h"
#include "testglwidget.h"
#include "testmesh.h"
#include "testlandmarks.h"
#include "testsurfaceprocessing.h"

#include <QString>

QString frgcPath()
{
    return "/run/media/stepo/frgc/";
}

int main(int argc, char *argv[])
{
    //TestVector::testConstructor();
    //TestVector::testBasicOperations();
    //TestProcrustes::testRotateAndScale();
    //TestProcrustes::testProcrustes();
    //TestProcrustes::testAlign();
    //TestPCA::testPCASimple();
    //TestPCA::testPCAStorage();
    //TestPCA t; t.testPCACreateFaceSpace();
    //TestLineProcessing::testLine();
    //TestLineProcessing::testMultipleLines();
    //TestMetrics::testMahalanobisDistance();

    //TestDelaunay::testSubdivision();
    //TestDelaunay::testTextureReader();
    //TestDelaunay::testWarp();

    //TestLDA::testLDASimple2D();
    //TestLDA::testLDAFaces();

    //TestSVD::testProcrustes();

    //TestGAOptimization::testGAOptimizationOnFisherface();
    //TestBioDataProcessing::testDivideVectors();

    //TestDiscriminativePotential::TestOnFRGC();

    //TestICA::testDataGeneration();
    //TestICA::ConvertGermanyThermoImages();
    //TestICA testICA;
    //testICA.DeprojectGermanyThermoImages();

    //TestSVM::testSVM();

    //TestGaborWavelet::test();

	//Annotation a("/home/stepo/SVN/disp-stepan-mracek/test/testASM");
	//TestTextureWarper::test();
	//TestAAM::testLearn();
	//TestAAM::testGenerate();
    //TestAAM::testGradient();

    //TestKinect::testFaceDetect(argc, argv);
    //TestKinect::testScan(argc, argv);
    //TestKinect::testPCL();
    //TestKinect::testPCLMerge(argc, argv);

    //TestFaceFeatuesDetection::testDepthmapProcessing(argc, argv, "/home/stepo/face.obj");
    return TestFaceFeatuesDetection::testSmoothing(argc, argv, "/home/stepo/face.obj");
    //TestFaceFeatuesDetection::testLandmarkDetection(argc, argv, frgcPath() +
    //                                                "frgc-spring2004-obj-centralized/04202d566.obj");
    //TestFaceFeatuesDetection::testBatchLandmarkDetection(argc, argv, frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::testSuccessBatchLandmarkDetection(frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::testIsoGeodeticCurves(argc, argv);
    //TestFaceFeatuesDetection::testIsoGeodeticCurvesAlign(argc, argv);
    //TestFaceFeatuesDetection::testHorizontalProfileLines(argc, argv);
    //TestFaceFeatuesDetection::testCombine();
    //bool uniqueIDs = false;
    //TestFaceFeatuesDetection::testAnotation(frgcPath() + "frgc-spring2004-obj-centralized", uniqueIDs);
    //TestFaceFeatuesDetection::exportForVOSM(frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::exportInitialEstimationsForVOSM(frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::testGoodAnotation("/media/data/frgc/obj-centralized/Spring2004");
    //TestFaceFeatuesDetection::testAlign("test/align-simple-mean.png", "test/align-simple-pca.xml", "test/align-simple-flags", simple);
    //TestFaceFeatuesDetection::testAlign("test/align-triangle-mean.png", "test/align-triangle-pca.xml", "test/align-triangle-flags", triangle);

    //TestMorphableFaceModel::testCreate("/media/data/frgc/obj-centralized/Spring2004",
    //                                   "test/align-landmarks.xml", "test/align-pca.xml", "test/align-flags");
    //TestMorphableFaceModel::testModel(argc, argv, "../test/align-pca.xml", "../test/align-flags", "../test/align-landmarks.xml");
    //TestMorphableFaceModel::testMorph(argc, argv, "test/align-pca.xml", "test/align-flags", "test/align-landmarks.xml",
    //                                  "/run/media/stepo/frgc/frgc-spring2004-obj-centralized/04200d74.obj");
    //TestMorphableFaceModel::testMorphFromKinect(argc, argv, "test/align-pca.xml", "test/align-flags", "test/align-landmarks.xml");

    //TestGlWidget::test(argc, argv, "/media/data/frgc/obj-centralized/Spring2004/");

    //TestMesh::testXYZLodaderOBJWriter("/media/data/frgc/xyz/Spring2004", "/media/data/frgc/obj-centralized/Spring2004");

    //TestLandmarks::testReadWrite();

    //TestSurfaceProcessing::testNormals();

	return 0;
}
