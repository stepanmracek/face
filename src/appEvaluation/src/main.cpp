#include "evaluatesoftkinetic.h"
#include "evaluatemultiextractor.h"
#include "evaluatefeaturestability.h"
#include "evaluatethermo.h"

int main(int argc, char *argv[])
{
    // FRGC - multiextractor
    //Evaluate3dFrgc2::autoTuner();
    //Evaluate3dFrgc2::evaluateDoGTexture();
    //Evaluate3dFrgc2::createMultiExtractor();
    //Evaluate3dFrgc2::createTemplates();
    //Evaluate3dFrgc2::evaluateTemplates();
    //EvaluateMultiExtractor::createFRGCextractor();
    //EvaluateMultiExtractor::evaluateFRGCextractor();
    //EvaluateMultiExtractor::evaluateFRGCUnits();
    //EvaluateMultiExtractor::compareExtractors();

    //EvaluateFeatureStability::evaluate();

    //EvaluateFRGCAlign::evaluateAlignType();
    //EvaluateFRGCAlign::evaluatePCAParams();
    //EvaluateFRGCAlign::evaluateROIs();
    //EvaluateFRGCAlign::stats();

    // Kinect
    //EvaluateKinect::learnFromFrgc();
    //EvaluateKinect::evaluateKinect();
    //EvaluateKinect::evaluateSerializedKinect();
    //EvaluateKinect::createTemplates();
    //EvaluateKinect::evaluateSimple();
    //EvaluateKinect::evaluateRefeference();
    //EvaluateKinect::evaluateReferenceDistances();
    //EvaluateKinect::evaluateMultiExtractor();

    // SoftKinetic
    //EvaluateSoftKinetic::checkAligning(argc, argv);
    //EvaluateSoftKinetic::createMultiExtractor(argc, argv);
    //EvaluateSoftKinetic::createMultiExtractor();
    //EvaluateSoftKinetic::evaluateMultiExtractor();
    //EvaluateSoftKinetic::evaluateRankOneIdentification();
    //EvaluateSoftKinetic::evaluateAging();
    //EvaluateSoftKinetic::stats();

    //EvaluateSoftKinetic::createSeparatedVectors();
    //EvaluateSoftKinetic::evaluateSeparatedVectors();
    //EvaluateSoftKinetic::evaluateOptimalSmoothing();

    //EvaluateLBP::evaluate();

    //EvaluateMichalek::evaluate();
    //EvaluateMichalek::evaluateStripes();

    //EvaluateSoftKinetic skEval;
    //skEval.extractor = Face::Biometrics::MultiExtractor("../appAutoTrainer/epar/extractor/04/");
    //skEval.aligner = Face::FaceData::FaceAligner(Face::FaceData::Mesh::fromFile("../../test/meanForAlign.obj"));
    //skEval.loadDirectory("templates", 4, true);
    //skEval.loadDirectory("/mnt/data/face/softkinetic/epar/", "5001-*.binz", 4, 100, 10, 0.01);
    //skEval.evaluatePerformance();
    //skEval.problematicScans();
    //skEval.evaluateMetrics();
    //EvaluateSoftKinetic::evaluateAlignBaseline();
    //EvaluateSoftKinetic::evaluateCVAlign();
    //EvaluateSoftKinetic::deviceOpenCVtest();
    //EvaluateSoftKinetic::evaluateVariability();
    //EvaluateSoftKinetic::evaluateJenda(argc, argv);

    //EvaluateThermo::train();
    EvaluateThermo::evaluate();

    return 0;
}
