/*#include "evaluatefeatureselection.h"
#include "evaluateICA.h"
#include "evaluatethermo.h"
#include "evaluatefeaturelevelfusion.h"
#include "evaluatefeaturestability.h"
#include "evaluatethermo2.h"
#include "evaluate3dfrgc.h"*/
#include "evaluatekinect.h"
#include "evaluate3dfrgc.h"

int main(int argc, char *argv[])
{
    //EvaluateFeatureSelection::compareEERandDPSelectionOnFRGCAnatomical();
    //EvaluateFeatureSelection::createFRGCShapeIndexPCATemplates();
    //EvaluateFeatureSelection::evaluateFRGCShapeIndexPCATemplatesCosineDist();
    //EvaluateFeatureSelection::evaluateFRGCShapeIndexPCATemplatesCorrDist();

    //EvaluateICA::process();

    //EvaluateThermo::evaluateForDifferentSelectionThresholds();
    //EvaluateThermo::evaluateMethodCorrelation();
    //EvaluateThermo::evaluateMultiWithScoreLevelFusion();
    //EvaluateThermo::evaluateFeatureSelection();
    //EvaluateThermo::evaluateDifferentMetrics();
    //EvaluateThermo::evaluateSelectedMultiWithScoreLevelFusion();

	//EvaluateFeatureLevelFusion::evaluate();
	//EvaluateFeatureLevelFusion::evaluateBestSelectionThreshold();
	//EvaluateFeatureStability::evaluate();

	//EvaluateThermo2::evaluateFeatureSelection();
	//EvaluateThermo2::evaluateFilterBanks();
	//EvaluateThermo2::gaborBankFeatureSelection();
	//EvaluateThermo2::fusion();
	//EvaluateThermo2::evaluteSingleMethods();
	//EvaluateThermo2::evaluateZScoreAndWeighting();
	//EvaluateThermo2::evaluateFilterBanks();

    //Evaluate3dFrgc::evaluateFusion();
    //Evaluate3dFrgc::createBIN();
    //Evaluate3dFrgc::align();
    //Evaluate3dFrgc::createMaps();
    //Evaluate3dFrgc::evaluateHistogramFeaturesGenerateStripesBinsMap();
    //Evaluate3dFrgc::evaluateHistogramFeatures();
    //Evaluate3dFrgc::evaluateMaps();
    //Evaluate3dFrgc::evaluateGaborFilterBanks();
    //Evaluate3dFrgc::trainGaborFusion();
    Evaluate3dFrgc::evaluateGaborFusion();
    //Evaluate3dFrgc::evaluateDirect();
    //Evaluate3dFrgc::createIsoCurves();
    //Evaluate3dFrgc::evaluateIsoCurves();
    //Evaluate3dFrgc::evaluateFusion();
    //Evaluate3dFrgc::createCurves();
    //Evaluate3dFrgc::evaluateCurves();
    //Evaluate3dFrgc::evaluateTextures();

    //EvaluateKinect::evaluate();

    return 0;
}
