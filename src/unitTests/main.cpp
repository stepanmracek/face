#include "test.h"
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
#include "testanotation.h"
#include "testmap.h"
#include "testhistogramfeatures.h"
#include "testdistance.h"
#include "testlogisticregression.h"
#include "testlaguerrewavelet.h"
#include "testfacealigner.h"
#include "testtextureprocessing.h"
#include "testgmm.h"

#include <QString>

QString frgcPath()
{
    return "/home/stepo/data/frgc/spring2004/";
}

int main(int argc, char *argv[])
{
    //Test::DoG();
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

    //TestBioDataProcessing::testDivideTemplatesToClusters();
    //TestBioDataProcessing::testDivideVectorsToClusters();
    //TestBioDataProcessing::testDivideToNClusters();

    //TestDiscriminativePotential::TestOnFRGC();

    //TestICA::testDataGeneration();
    //TestICA::ConvertGermanyThermoImages();
    //TestICA testICA;
    //testICA.DeprojectGermanyThermoImages();

    //TestSVM::testSVM();

    //TestGaborWavelet::test();
    //TestLaguerreWavelet::test();

	//Annotation a("/home/stepo/SVN/disp-stepan-mracek/test/testASM");
	//TestTextureWarper::test();
	//TestAAM::testLearn();
	//TestAAM::testGenerate();
    //TestAAM::testGradient();

    //TestKinect::testFaceDetect(argc, argv);
    //TestKinect::testScan(argc, argv);

    //TestFaceFeatuesDetection::testLandmarkDetection(argc, argv, frgcPath() + "02463d652.abs.xyz");
    //TestFaceFeatuesDetection::testBatchLandmarkDetection(argc, argv, frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::testSuccessBatchLandmarkDetection(frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::testIsoGeodeticCurves(argc, argv);
    //TestFaceFeatuesDetection::testIsoGeodeticCurvesAlign(argc, argv);
    //TestFaceFeatuesDetection::testHorizontalProfileLines(argc, argv);
    //TestFaceFeatuesDetection::testCombine();
    //TestFaceFeatuesDetection::exportForVOSM(frgcPath() + "frgc-spring2004-obj-centralized");
    //TestFaceFeatuesDetection::exportInitialEstimationsForVOSM(frgcPath() + "frgc-spring2004-obj-centralized");

    //TestMorphableFaceModel::testCreate(frgcPath() + "bin/", "../../test/morph-landmarks.xml",
    //                                   "../../test/morph-pca-zcoord.xml", "../../test/morph-pca-texture.xml",
    //                                   "../../test/morph-pca.xml", "../../test/morph-flags");
    //TestMorphableFaceModel::testModel(argc, argv, "../../test/morph-pca-zcoord.xml", "../../test/morph-pca-texture.xml",
    //                                  "../../test/morph-pca.xml", "../../test/morph-flags", "../../test/morph-landmarks.xml");
    //TestMorphableFaceModel::testMorph(argc, argv, "test/align-pca.xml", "test/align-flags", "test/align-landmarks.xml",
    //                                  "/run/media/stepo/frgc/frgc-spring2004-obj-centralized/04200d74.obj");
    //TestMorphableFaceModel::testMorphFromKinect(argc, argv, "test/align-pca.xml", "test/align-flags", "test/align-landmarks.xml");

    //TestGlWidget::test(argc, argv);

    //TestMesh::testBINZLodaderOBJWriter("/media/data/frgc/spring2004/zbin-aligned2", "/home/stepo/obj");
    //TestMesh::testBinReadWrite(argc, argv, frgcPath());
    //TestMesh::testReadBinWriteBinzReadBinz(argc, argv, frgcPath());
    //TestMesh::readAbsWithTexture(argc, argv);
    //TestMesh::testReadWriteCharArray(frgcPath());

    //TestFaceAligner::test(frgcPath(), argc, argv);
    //TestFaceAligner::testOpenMP();

    //TestLandmarks::testReadWrite();

    //TestAnotation::test("/home/stepo/data/frgc/spring2004/bin");
    //TestFaceFeatuesDetection::testGoodAnotation(frgcPath() + "bin/");

    //TestMap::testSerialization(frgcPath() + "04225d402.abs.xyz");
    //TestMap::testSmoothing();

    //TestHistogramFeatures::testFeaturesGeneration(frgcPath());

    //TestDistance::testCosine();

    //TestLogisticRegression::test();

    //TestTextureProcessing::testClahe();

    //TestGMM::TestSerialization();
    //TestGMM::TestLearnScoreGMMFusion();
    TestGMM::Bug();

	return 0;
}
