/*#include "evaluatefeatureselection.h"
#include "evaluateICA.h"
#include "evaluatethermo.h"
#include "evaluatefeaturelevelfusion.h"
#include "evaluatefeaturestability.h"
#include "evaluatethermo2.h"
#include "evaluate3dfrgc.h"*/
#include "evaluatekinect.h"
#include "evaluate3dfrgc.h"
#include "evaluatesoftkinetic.h"

int main(int argc, char *argv[])
{
    //Evaluate3dFrgc::createBIN();
    //Evaluate3dFrgc::align2();
    //Evaluate3dFrgc::createTextures();

    // Gabor
    //Evaluate3dFrgc::evaluateGaborFilterBanks();
    //Evaluate3dFrgc::trainGaborFusion();

    // Gauss-Laguerre
    //Evaluate3dFrgc::trainGaussLaguerreFusion();

    // Iso-geodesic curves
    //Evaluate3dFrgc::createIsoCurves();
    //Evaluate3dFrgc::evaluateIsoCurves();

    // Fusion
    //Evaluate3dFrgc::evaluateFilterBankFusion();
    //Evaluate3dFrgc::evaluateFusionWrapper();
    //Evaluate3dFrgc::evaluateFusionAll();
    //Evaluate3dFrgc::testSerializedClassifiers();
    //Evaluate3dFrgc::testFilterBankKernelSizes();
    //Evaluate3dFrgc::createTemplates();
    //Evaluate3dFrgc::evaluateSerializedTemplates();

    // Kinect
    //EvaluateKinect::learnFromFrgc();
    //EvaluateKinect::evaluateKinect();
    //EvaluateKinect::evaluateSerializedKinect();
    //EvaluateKinect::createTemplates();
    //EvaluateKinect::evaluateSimple();
    //EvaluateKinect::evaluateRefeference();
    //EvaluateKinect::evaluateReferenceDistances();

    // SoftKinetic
    EvaluateSoftKinetic::evaluate();

    return 0;
}
