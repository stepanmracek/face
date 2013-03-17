/****************************************************************************
*                                                                           *
*   IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.       *
*                                                                           *
*   By downloading, copying, installing or using the software you agree to  *
*   this license. If you do not agree to this license, do not download,     *
*   install, copy or use the software.                                      *
*                                                                           *
*                           License Agreement                               *
*                   For Vision Open Statistical Models                      *
*                                                                           *
*   Copyright (C):      2006~2012 by JIA Pei, all rights reserved.          *
*                                                                           *
*   VOSM is free software under the terms of the GNU Lesser General Public  *
*   License (GNU LGPL) as published by the Free Software Foundation; either *
*   version 3.0 of the License, or (at your option) any later version.      *
*   You can use it, modify it, redistribute it, etc; and redistribution and *
*   use in source and binary forms, with or without modification, are       *
*   permitted provided that the following conditions are met:               *
*                                                                           *
*   a) Redistribution's of source code must retain this whole paragraph of  *
*   copyright notice, including this list of conditions and all the         *
*   following contents in this  copyright paragraph.                        *
*                                                                           *
*   b) Redistribution's in binary form must reproduce this whole paragraph  *
*   of copyright notice, including this list of conditions and all the      *
*   following contents in this copyright paragraph, and/or other materials  *
*   provided with the distribution.                                         *
*                                                                           *
*   c) The name of the copyright holders may not be used to endorse or      *
*   promote products derived from this software without specific prior      *
*   written permission.                                                     *
*                                                                           *
*   Any publications based on this code must cite the following five papers,*
*   technical reports and on-line materials.                                *
*   1) P. JIA, 2D Statistical Models, Technical Report of Vision Open       *
*   Working Group, 2st Edition, October 21, 2010.                           *
*   http://www.visionopen.com/members/jiapei/publications/pei_sm2dreport2010.pdf*
*   2) P. JIA. Audio-visual based HMI for an Intelligent Wheelchair.        *
*   PhD thesis, University of Essex, February, 2011.                        *
*   http://www.visionopen.com/members/jiapei/publications/pei_phdthesis2010.pdf*
*   3) T. Cootes and C. Taylor. Statistical models of appearance for        *
*   computer vision. Technical report, Imaging Science and Biomedical       *
*   Engineering, University of Manchester, March 8, 2004.                   *
*   http://www.isbe.man.ac.uk/~bim/Models/app_models.pdf                    *
*   4) I. Matthews and S. Baker. Active appearance models revisited.        *
*   International Journal of Computer Vision, 60(2):135--164, November 2004.*
*   http://www.ri.cmu.edu/pub_files/pub4/matthews_iain_2004_2/matthews_iain_2004_2.pdf*
*   5) M. B. Stegmann, Active Appearance Models: Theory, Extensions and     *
*   Cases, 2000.                                                            *
*   http://www2.imm.dtu.dk/~aam/main/                                       *
*                                                                           *
* Version:          0.4                                                     *
* Author:           JIA Pei                                                 *
* Contact:          jp4work@gmail.com                                       *
* URL:              http://www.visionopen.com                               *
* Create Date:      2008-04-03                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/


#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <set>
#include <boost/filesystem.hpp>
#include "VO_AAMBasic.h"
#include "VO_CVCommon.h"



/**

@ note
Explained by JIA Pei, 2007-05-17
Refer to 
1) Cootes' "Comparing Variations on the Active Appearance Model Algorithm"
2) Cootes' "Statistical Models of Appearance for Computer Vision" page 56
3) Stegmann's" AAM-API, Basic Active Appearance Models"

This is for calculating the multivariate linear regression matrix
Explained by JIA Pei, 2007-05-17
According to Cootes' "Comparing Variations on the Active Appearance Model Algorithm",
there are three types of parameters, p = [c|t|u]. 
Here, we only consider the c and t parameters, why?
u is for texture transformation, so that the texture can be commensurate at the same scale 

Actually, the experiments designed in Stegmann's summary in AAM-API is just for estimating
the regression matrix R, in case of 1) the robustness; 2) make the training dataset more linear!!

It's explained in Cootes' Comparing Variations on the Active Appearance Model Algorithm that
"Delta(r)/Delta(p) is estimated by numeric differentiation, systematically displacing each parameter
from the known optimal value on typical images and computing an average over the training set."

Here, we could ignore the additional experimental samples, that's to say, the training data are just the 240
faces from the dataset. we set a variable to control this whether we need this additional man-made samples or not.
*/
void VO_AAMBasic::VO_CalcRegressionMatrices()
{
    // do model parameter experiments
    {
        // in VO_DoCParamExperiments, a matrix size of 80259*21120 is too big to be allocated
        this->VO_DoCParamExperiments();

        // estimate Rc using principal component regression    
        //this->VO_DoRegression( C, X,  m_pModel->m_R_c ); 
    }
    // do pose experiments
    {
        this->VO_DoPoseExperiments();

        // estimate Rt using principal component regression    
        //this->VO_DoRegression( C, X,  m_pModel->m_R_t );
    }

    // store negative versions to avoid any sign change at each iteration
    // m_pModel->m_R_c *= -1;
    // m_pModel->m_R_t *= -1;

    // gsl_multifit_linear_svd();
}


// Totally, there are 17280 samples for the experiments, every is of size 80259. 
// 80259*17280, too big to allocate. So, here, we use another methods for multivariate regression --
// by saving all those data in files.
void VO_AAMBasic::VO_DoCParamExperiments()
{
    bool recordIntermediateImgs = false;

    int nExperiment = 0;
    int totalExp4OneSample = this->m_vvCDisps.size() * this->m_vvCDisps[0].cols;
    Mat_<float> X = Mat_<float>::zeros(this->m_iNbOfTextures, totalExp4OneSample);          // 80259*72
    Mat_<float> C = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, totalExp4OneSample);  // 12*72
    Mat_<float> currentConcatenatedParameters;
    Mat_<float> currentShape, currentTexture;
    VO_Shape currentShapeInstance;      // built from the parameters
    VO_Texture currentTextureInstance;  // built from the parameters, but not sampled by using the shape parameters
    VO_Texture delta_g;
    Mat img;
    Mat tempImage1, tempImage2, resImage1, resImage2;

    // for each training example in the training set
    for(unsigned int i = 0; i < this->m_iNbOfSamples; i++)
    {
        if(this->m_iNbOfChannels == 1)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 0 );
        else if (this->m_iNbOfChannels == 3)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 1 );
        else
            cerr << "We can't deal with image channels not equal to 1 or 3!" << endl;

        if(recordIntermediateImgs)
        {
            img.copyTo(tempImage1);
            img.copyTo(tempImage2);
        }

        for(unsigned int j = 0; j < this->m_vvCDisps.size(); j++)           // 4
        {
            for (unsigned int k = 0; k < this->m_vvCDisps[0].cols; k++)     // 12
            {
                // do displacement measures
                currentConcatenatedParameters = this->m_MatAppearanceProject2Truncated.row(i);

                // adjust(shift) currentConcatenatedParameters to implement the experiments
                currentConcatenatedParameters(0, k) += this->m_vvCDisps[j](0,k);

                // According to Cootes' "Comparing Variations on the Active Appearance Model Algorithm" - Equation (3)

                // Build the shape instance from the combined model
//                cv::gemm(currentConcatenatedParameters, this->m_MatQs, 1, this->m_PCAAlignedShape.mean, 1, currentShape, GEMM_2_T );
                currentShape = currentConcatenatedParameters * this->m_MatQs.t() + this->m_PCAAlignedShape.mean;

                // Build the texture instance from the combined model
//                cv::gemm(currentConcatenatedParameters, this->m_MatQg, 1, this->m_PCANormalizedTexture.mean, 1, currentTexture, GEMM_2_T );
                currentTexture = currentConcatenatedParameters * this->m_MatQg.t() + this->m_PCANormalizedTexture.mean;

                // Align from the displacement alignedshape to the shape of original size
                currentShapeInstance.SetTheShape(currentShape, 2);
                currentShapeInstance.AlignTo(this->m_vShapes[i]);

                // Obtain the original texture information from shape instance
                if(!VO_TextureModel::VO_LoadOneTextureFromShape(currentShapeInstance, img, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, delta_g))
                    continue;

                currentTextureInstance.SetTheTexture(currentTexture, delta_g.GetNbOfTextureRepresentation());

                //////////////////////////////////////////////////////////////////////////
                // The following codes are just for intermediate display
                if(recordIntermediateImgs)
                {
                    // extracted from the real image
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(delta_g, this->m_vTemplateTriangle2D, tempImage1);

                    // build from the model
                    VO_TextureModel::VO_NormalizedTexture2ReferenceScale(currentTextureInstance, this->m_fAverageTextureStandardDeviation, currentTextureInstance);
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(currentTextureInstance, this->m_vTemplateTriangle2D, tempImage2);

                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstance, this->m_vTemplateTriangle2D, tempImage1, resImage1);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstance, this->m_vTemplateTriangle2D, tempImage2, resImage2);

                    stringstream ssi, ssj, ssk;
                    string stri, strj, strk;
                    ssi << i;
                    ssj << j;
                    ssk << k;
                    ssi >> stri;
                    ssj >> strj;
                    ssk >> strk;

                    string temp1Str = "CDisplaceLoadedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string temp2Str = "CDisplaceTextureImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string res1Str     = "CDisplaceLoadedWarpedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string res2Str     = "CDisplaceTextureWarpedImage" + stri + "_" + strj + "_" + strk + ".jpg";

                    imwrite(temp1Str.c_str(), tempImage1 );
                    imwrite(temp2Str.c_str(), tempImage2 );
                    imwrite(res1Str.c_str(), resImage1 );
                    imwrite(res2Str.c_str(), resImage2 );

                    ssi.clear();
                    ssj.clear();
                    ssk.clear();
                }

                //////////////////////////////////////////////////////////////////////////

                delta_g.Normalize();
                delta_g -= currentTextureInstance;

                // Explained by JIA Pei. Here, the X matrix is too big to allocate a memory for calculation, 2007-05-30
                // insert the results into X and C
                for (unsigned int n = 0; n < this->m_iNbOfTextures; n++)
                {
                    X(n, nExperiment) = delta_g.GetATexture(n);
                }
                C(k, nExperiment) = this->m_vvCDisps[j](0,k);

                nExperiment++;
            }
            //cout << "Experiment" << nExperiment << "of" << X->cols << "done (c)..." << endl;
        }

        //////////////////////////////////////////////////////////////////////////
        // just in order to save the data
        // X, C
        /*
        string filestr = this->m_vimgFiles[i].substr (14,5) + ".txt"; 
        string folderstr1 = "./cexperiment";
        string folderstrX = folderstr1 + "/" + "X";
        string folderstrC = folderstr1 + "/" + "C";
        string folderfilestrX = folderstrX + "/" + filestr;
        string folderfilestrC = folderstrC + "/" + filestr;

        boost::filesystem::create_directory( folderstr1 );
        boost::filesystem::create_directory( folderstrX );
        boost::filesystem::create_directory( folderstrC );

        fstream fp;
        fp.open(folderfilestrX.c_str (), ios::out);
        fp << this->m_vimgFiles[i].substr (14,5) << "-X" << endl;
        for (unsigned int m = 0; m < X->rows; m++)
        {
            for (unsigned int n = 0; n < X->cols; n++)
            {            
                fp << X(m, n) << " ";
            }
            fp << endl;
        }
        fp.close();fp.clear();

        fp.open(folderfilestrC.c_str (), ios::out);
        fp << this->m_vimgFiles[i].substr (14,5) << "-C" << endl;
        for (unsigned int m = 0; m < C->rows; m++)
        {
            for (unsigned int n = 0; n < C->cols; n++)
            {            
                fp << C(m, n) << " ";
            }
            fp << endl;
        }
        fp.close();fp.clear();
        */
        //////////////////////////////////////////////////////////////////////////
    }
}

/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       Carry out pose displacement experiments
 * @return      void
*/
void VO_AAMBasic::VO_DoPoseExperiments()
{
    bool recordIntermediateImgs = false;
    
    int nExperiment = 0;
    int totalExp4OneSample = this->m_vvPoseDisps.size() * this->m_vvPoseDisps[0].cols;
    Mat_<float> X = Mat_<float>::zeros(this->m_iNbOfTextures, totalExp4OneSample);                // 80259*72
    Mat_<float> P = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, totalExp4OneSample);        // 12*72
    Mat_<float> currentConcatenatedParameters;
    Mat_<float> currentShape, currentTexture;
    VO_Shape currentShapeInstance;          // built from the parameters
    VO_Texture currentTextureInstance;      // built from the parameters, but not sampled by using the shape parameters
    VO_Texture delta_g;
    //Mat_<float> disp         = Mat_<float>::zeros(1, 4);
    Mat_<float> posedisp     = Mat_<float>::zeros(1, 4);

    // just for displacement
    Mat_<float> translation, disptranslation;
    float scale = 1.0f, dispscale = 1.0f;
    vector<float> theta(1), dispangles(1);
    Mat img;
    Mat tempImage, resImage;
    

    // for each training example in the training set
    for(unsigned int i = 0; i < this->m_iNbOfSamples; i++)
    {
        nExperiment = 0;
        
        if(this->m_iNbOfChannels == 1)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 0 );
        else if (this->m_iNbOfChannels == 3)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 1 );
        else
            cerr << "We can't deal with image channels not equal to 1 or 3!" << endl;
            
        if(recordIntermediateImgs)
        {
            img.copyTo(tempImage);
        }

        for(unsigned int j = 0; j < this->m_vvPoseDisps.size(); j++)            // 4
        {
            for (unsigned int k = 0; k < this->m_vvPoseDisps[0].cols; k++)    // 4
            {
                posedisp = Mat_<float>::zeros(posedisp.size());
                posedisp(0,k) = this->m_vvPoseDisps[j](0,k);
                VO_Shape::GlobalShapeNormalization2SimilarityTrans(posedisp, dispscale, dispangles, disptranslation );

                // do displacement measures
                currentConcatenatedParameters = this->m_MatAppearanceProject2Truncated.row(i);

                // According to Cootes' "Comparing Variations on the Active Appearance Model Algorithm" - Equation (3)

                // Build the shape instance from the combined model
//                cv::gemm(currentConcatenatedParameters, this->m_MatQs, 1, this->m_PCAAlignedShape.mean, 1, currentShape, GEMM_2_T );
                currentShape = currentConcatenatedParameters * this->m_MatQs.t() + this->m_PCAAlignedShape.mean;

                // Build the texture instance from the combined model
//                cv::gemm(currentConcatenatedParameters, this->m_MatQg, 1, this->m_PCANormalizedTexture.mean, 1,currentTexture, GEMM_2_T);
                currentTexture = currentConcatenatedParameters * this->m_MatQg.t() + this->m_PCANormalizedTexture.mean;

                // Calculate the align transformation
                currentShapeInstance.SetTheShape(currentShape, 2);
                currentShapeInstance.AlignTransformation(this->m_vShapes[i], scale, theta, translation);

                currentShapeInstance.Scale(dispscale * scale);

                vector<float> tempTheta;
                tempTheta.resize(1);
                tempTheta[0] = dispangles[0] + theta[0];
                currentShapeInstance.Rotate( tempTheta );

                Mat_<float> tempTranslate = Mat_<float>::zeros(translation.size());
                tempTranslate(0,0) = disptranslation(0,0) + translation(0,0);
                tempTranslate(1,0) = disptranslation(1,0) + translation(1,0);
                currentShapeInstance.Translate(tempTranslate);

                if(!VO_TextureModel::VO_LoadOneTextureFromShape(currentShapeInstance, img, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, delta_g))
                    continue;

                currentTextureInstance.SetTheTexture(currentTexture, delta_g.GetNbOfTextureRepresentation());

                //// The following codes are just for intermediate display
                if(recordIntermediateImgs)
                {
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(delta_g, this->m_vTemplateTriangle2D, tempImage);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstance, this->m_vTemplateTriangle2D, tempImage, resImage);

                    stringstream ssi, ssj, ssk;
                    string stri, strj, strk;
                    ssi << i;
                    ssj << j;
                    ssk << k;
                    ssi >> stri;
                    ssj >> strj;
                    ssk >> strk;

                    string temp1Str = "poseDisplaceLoadedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string res1Str     = "poseDisplaceLoadedWarpedImage" + stri + "_" + strj + "_" + strk + ".jpg";

                    imwrite(temp1Str.c_str(), tempImage );
                    imwrite(res1Str.c_str(), resImage );

                    ssi.clear();
                    ssj.clear();
                    ssk.clear();
                }



                delta_g.Normalize();                
                delta_g -= currentTextureInstance;

                // Explained by JIA Pei. Here, the X matrix is too big to allocate a memory for calculation, 2007-05-30
                // insert the results into X and C
                for (unsigned int n = 0; n < this->m_iNbOfTextures; n++)
                {
                    X(n, nExperiment) = delta_g.GetATexture(n);
                }
                P(k, nExperiment) = this->m_vvPoseDisps[j](0,k);

                ++nExperiment;
            }
            //cout << "Experiment" << nExperiment << "of" << X->cols << "done (pose)..." << endl;
        }

        //////////////////////////////////////////////////////////////////////////
        // just in order to save the data
        // X, P
        /*
        string filestr = this->m_vimgFiles[i].substr (14,5) + ".txt"; 
        string folderstr1 = "./cexperiment";
        string folderstrX = folderstr1 + "/" + "X";
        string folderstrC = folderstr1 + "/" + "P";
        string folderfilestrX = folderstrX + "/" + filestr;
        string folderfilestrC = folderstrC + "/" + filestr;

        boost::filesystem::create_directory( folderstr1 );
        boost::filesystem::create_directory( folderstrX );
        boost::filesystem::create_directory( folderstrC );

        fstream fp;
        fp.open(folderfilestrX.c_str (), ios::out);
        fp << this->m_vimgFiles[i].substr (14,5) << "-X" << endl;
        for (unsigned int m = 0; m < X->rows; m++)
        {
            for (unsigned int n = 0; n < X->cols; n++)
            {            
                fp << X(m, n) << " ";
            }
            fp << endl;
        }
        fp.close();fp.clear();

        fp.open(folderfilestrC.c_str (), ios::out);
        fp << this->m_vimgFiles[i].substr (14,5) << "-C" << endl;
        for (unsigned int m = 0; m < C->rows; m++)
        {
            for (unsigned int n = 0; n < C->cols; n++)
            {            
                fp << C(m, n) << " ";
            }
            fp << endl;
        }
        fp.close();fp.clear();
        */
        //////////////////////////////////////////////////////////////////////////
    }
}


/**
 * @author         JIA Pei
 * @version        2010-02-05
 * @brief          Carry out multi variate linear regression experiments
 * @return        void
*/
void VO_AAMBasic::VO_DoRegression()
{

}


/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       Build gradient matrices
 * @return      void
*/
void VO_AAMBasic::VO_CalcGradientMatrices()
{
    // do model parameter experiments
    {
        this->VO_EstCParamGradientMatrix(this->m_MatCParamGradientMatrix);
        
        // estimate Rc
        this->m_MatRc = (this->m_MatCParamGradientMatrix.t()*this->m_MatCParamGradientMatrix).inv()
                        *(this->m_MatCParamGradientMatrix.t() );
    }
    // do pose experiments, this is for global shape normalization
    {
        this->VO_EstPoseGradientMatrix(this->m_MatPoseGradientMatrix);

        // estimate Rt
        this->m_MatRt = (this->m_MatPoseGradientMatrix.t()*this->m_MatPoseGradientMatrix).inv()
                        *(this->m_MatPoseGradientMatrix.t() );
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       Build gradient matrices in terms of C parameters
 * @param       oCParamGM   - Output    the built gradient matrix in terms of C parameters
 * @return      void
*/
void VO_AAMBasic::VO_EstCParamGradientMatrix(Mat_<float>& oCParamGM)
{
    bool recordIntermediateImgs = false;
    
    oCParamGM = Mat_<float>::zeros(this->m_iNbOfTextures, this->m_vvCDisps[0].cols );   // 80259*12
    int nExperiment = 0;
    Mat_<float> currentConcatenatedParameters, currentConcatenatedParametersPositiveDisp, currentConcatenatedParametersNegativeDisp;
    Mat_<float> currentShapePositive, currentShapeNegative, currentTexturePositive, currentTextureNegative;
    VO_Shape currentShapeInstancePositive, currentShapeInstanceNegative;
    VO_Texture currentTextureInstancePositive, currentTextureInstanceNegative;
    VO_Texture delta_g1, delta_g2, cDiff;
    Mat img;
    Mat tempImage1, tempImage2, tempImage3, tempImage4, resImage1, resImage2, resImage3, resImage4;

    // for each training example in the training set
    for(unsigned int i = 0; i < this->m_iNbOfSamples; i++)
    {
        if(this->m_iNbOfChannels == 1)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 0 );
        else if (this->m_iNbOfChannels == 3)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 1 );
        else
            cerr << "We can't deal with image channels not equal to 1 or 3!" << endl;

        if(recordIntermediateImgs)
        {
            img.copyTo(tempImage1);
            img.copyTo(tempImage2);
            img.copyTo(tempImage3);
            img.copyTo(tempImage4);
        }
        
        for(unsigned int j = 0; j < this->m_vvCDisps.size(); j = j+2)       // 4 -- number of displacements for each shape parameter
        {
            for (unsigned int k = 0; k < this->m_vvCDisps[0].cols; k++)     // 12 -- number of shape parameters
            {
                // do displacement measures
                currentConcatenatedParameters = this->m_MatAppearanceProject2Truncated.row(i);
                currentConcatenatedParameters.copyTo(currentConcatenatedParametersNegativeDisp);
                currentConcatenatedParameters.copyTo(currentConcatenatedParametersPositiveDisp);

                // adjust(shift) currentConcatenatedParameters ... for calculating the Jacobian Matrix
                currentConcatenatedParametersNegativeDisp(0, k) = currentConcatenatedParameters(0, k) + this->m_vvCDisps[j](0,k);
                currentConcatenatedParametersPositiveDisp(0, k) = currentConcatenatedParameters(0, k) + this->m_vvCDisps[j+1](0,k);

                // According to Cootes' "Comparing Variations on the Active Appearance Model Algorithm" - Equation (3)

                // Build the shape instance from the combined model
//                cv::gemm(currentConcatenatedParametersNegativeDisp, this->m_MatQs, 1, this->m_PCAAlignedShape.mean, 1, currentShapeNegative, GEMM_2_T );
//                cv::gemm(currentConcatenatedParametersPositiveDisp, this->m_MatQs, 1, this->m_PCAAlignedShape.mean, 1, currentShapePositive, GEMM_2_T );
                currentShapeNegative = currentConcatenatedParametersNegativeDisp * this->m_MatQs.t() + this->m_PCAAlignedShape.mean;
                currentShapePositive = currentConcatenatedParametersPositiveDisp * this->m_MatQs.t() + this->m_PCAAlignedShape.mean;

                // Build the texture instance from the combined model
//                cv::gemm(currentConcatenatedParametersNegativeDisp, this->m_MatQg, 1, this->m_PCANormalizedTexture.mean, 1, currentTextureNegative, GEMM_2_T );
//                cv::gemm(currentConcatenatedParametersPositiveDisp, this->m_MatQg, 1, this->m_PCANormalizedTexture.mean, 1, currentTexturePositive, GEMM_2_T );
                currentTextureNegative = currentConcatenatedParametersNegativeDisp * this->m_MatQg.t() + this->m_PCANormalizedTexture.mean;
                currentTexturePositive = currentConcatenatedParametersPositiveDisp * this->m_MatQg.t() + this->m_PCANormalizedTexture.mean;

                // Align from the displacement alignedshape to the shape of original size
                currentShapeInstanceNegative.SetTheShape(currentShapeNegative, 2);
                currentShapeInstancePositive.SetTheShape(currentShapePositive, 2);
                currentShapeInstanceNegative.AlignTo(this->m_vShapes[i]);
                currentShapeInstancePositive.AlignTo(this->m_vShapes[i]);

                // Obtain the original texture information from shape instance
                if(!VO_TextureModel::VO_LoadOneTextureFromShape(currentShapeInstanceNegative, img, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, delta_g1))
                    continue;
                if(!VO_TextureModel::VO_LoadOneTextureFromShape(currentShapeInstancePositive, img, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, delta_g2))
                    continue;

                currentTextureInstanceNegative.SetTheTexture(currentTextureNegative, delta_g1.GetNbOfTextureRepresentation());
                currentTextureInstancePositive.SetTheTexture(currentTexturePositive, delta_g1.GetNbOfTextureRepresentation());

                // The following codes are just for intermediate display
                if(recordIntermediateImgs)
                {
                    // extracted from the real image
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(delta_g1, this->m_vTemplateTriangle2D, tempImage1);
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(delta_g2, this->m_vTemplateTriangle2D, tempImage2);

                    // build from the model
                    VO_TextureModel::VO_NormalizedTexture2ReferenceScale(currentTextureInstanceNegative, this->m_fAverageTextureStandardDeviation, currentTextureInstanceNegative);
                    VO_TextureModel::VO_NormalizedTexture2ReferenceScale(currentTextureInstancePositive, this->m_fAverageTextureStandardDeviation, currentTextureInstancePositive);
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(currentTextureInstanceNegative, this->m_vTemplateTriangle2D, tempImage3);
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(currentTextureInstancePositive, this->m_vTemplateTriangle2D, tempImage4);

//    VO_Texture tempTexture;
//    Mat oImg(tempImage1);
//    cout << this->m_VOReferenceShape << endl;
//    imwrite("template.jpg", this->m_ImageTemplateFace);
//    VO_TextureModel::VO_LoadOneTextureFromShape(this->m_VOReferenceShape, this->m_ImageTemplateFace, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, tempTexture);
//    VO_TextureModel::VO_PutOneTextureToTemplateShape(tempTexture, this->m_vTemplateTriangle2D, oImg);
//    imwrite("temp.jpg", oImg);


                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstanceNegative, this->m_vTemplateTriangle2D, tempImage1, resImage1);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstancePositive, this->m_vTemplateTriangle2D, tempImage2, resImage2);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstanceNegative, this->m_vTemplateTriangle2D, tempImage3, resImage3);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstancePositive, this->m_vTemplateTriangle2D, tempImage4, resImage4);

                    stringstream ssi, ssj, ssj1, ssk;
                    string stri, strj, strj1, strk;
                    ssi << i;
                    ssj << j;
                    ssj1 << (j + 1);
                    ssk << k;
                    ssi >> stri;
                    ssj >> strj;
                    ssj1 >> strj1;
                    ssk >> strk;

                    string temp1Str = "CDisplaceLoadedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string temp2Str = "CDisplaceLoadedImage" + stri + "_" + strj1 + "_" + strk + ".jpg";
                    string temp3Str = "CDisplaceTextureImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string temp4Str = "CDisplaceTextureImage" + stri + "_" + strj1 + "_" + strk + ".jpg";
                    string res1Str     = "CDisplaceLoadedWarpedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string res2Str     = "CDisplaceLoadedWarpedImage" + stri + "_" + strj1 + "_" + strk + ".jpg";
                    string res3Str     = "CDisplaceTextureWarpedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string res4Str     = "CDisplaceTextureWarpedImage" + stri + "_" + strj1 + "_" + strk + ".jpg";

                    imwrite(temp1Str.c_str(), tempImage1 );
                    imwrite(temp2Str.c_str(), tempImage2 );
                    imwrite(temp3Str.c_str(), tempImage3 );
                    imwrite(temp4Str.c_str(), tempImage4 );
                    imwrite(res1Str.c_str(), resImage1 );
                    imwrite(res2Str.c_str(), resImage2 );
                    imwrite(res3Str.c_str(), resImage3 );
                    imwrite(res4Str.c_str(), resImage4 );

                    ssi.clear();
                    ssj.clear();
                    ssj1.clear();
                    ssk.clear();
                }



                // Normalize the extracted(loaded) textures
                delta_g1.Normalize();
                delta_g2.Normalize();
                delta_g1 -= currentTextureInstanceNegative;
                delta_g2 -= currentTextureInstancePositive;

                // form central difference
                cDiff = (delta_g2-delta_g1)/(this->m_vvCDisps[j+1](0,k) - this->m_vvCDisps[j](0,k));
                for (unsigned int n = 0; n < this->m_iNbOfTextures; n++)
                {
                    oCParamGM(n, k) += cDiff.GetATexture(n);
                }
                nExperiment++;
            }
        }
    }

    // normalize
    // this->m_MatCParamGradientMatrix is a summation for 240 pictures (size of this->m_iNbOfSamples)
    // 4 -- refer to 4 possible shifts for just one parameter (12 parameters altogether)
    // 2 -- every pair of shifts, we got only one cDiff, which is actually stored in this->m_MatCParamGradientMatrix
//    oCParamGM /= (this->m_iNbOfSamples * 4 / 2);
    // nExperiment should be equal to this->m_iNbOfSamples * 4 / 2, if every texture can be successfully loaded.
    oCParamGM /= nExperiment;
}


/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       Build gradient matrices in terms of pose
 * @param       oPoseGM   - Output    the built gradient matrix in terms of poses
 * @return      void
*/
void VO_AAMBasic::VO_EstPoseGradientMatrix(Mat_<float>& oPoseGM)
{
    bool recordIntermediateImgs = false;

    int nExperiment = 0;
    oPoseGM = Mat_<float>::zeros(this->m_iNbOfTextures, this->m_vvPoseDisps[0].cols);
    Mat_<float> currentConcatenatedParameters;
    Mat_<float> currentShape, currentTexture;
    VO_Shape currentShapeInstance, currentShapeInstance1, currentShapeInstance2;
    VO_Texture currentTextureInstance;
    VO_Texture delta_g1, delta_g2, cDiff;
    //Mat_<float> disp1         = Mat_<float>::zeros(1, 4);
    //Mat_<float> disp2         = Mat_<float>::zeros(1, 4);
    Mat_<float> posedisp1     = Mat_<float>::zeros(1, 4);
    Mat_<float> posedisp2     = Mat_<float>::zeros(1, 4);

    // just for displacement
    Mat_<float> translation, disptranslation1, disptranslation2;
    float scale = 1.0f, dispscale1 = 1.0f, dispscale2 = 1.0f;
    vector<float> theta(1), dispangles1(1), dispangles2(1);
    Mat img;
    Mat tempImage1, tempImage2, resImage1, resImage2;
    

    // for each training example in the training set
    for(unsigned int i = 0; i < this->m_iNbOfSamples; i++)
    {
        if(this->m_iNbOfChannels == 1)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 0 );
        else if (this->m_iNbOfChannels == 3)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 1 );
        else
            cerr << "We can't deal with image channels not equal to 1 or 3!" << endl;

        if(recordIntermediateImgs)
        {
            img.copyTo(tempImage1);
            img.copyTo(tempImage2);
        }

        for(unsigned int j = 0; j < this->m_vvPoseDisps.size(); j = j+2)    // 4 -- number of displacements for each of the 4 pose parameters
        {
            for (unsigned int k = 0; k < this->m_vvPoseDisps[0].cols; k++)  // 4 -- number of pose parameters
            {
                posedisp1 = Mat_<float>::zeros(posedisp1.size());
                posedisp2 = Mat_<float>::zeros(posedisp2.size());
                posedisp1(0,k) = this->m_vvPoseDisps[j](0,k);
                posedisp2(0,k) = this->m_vvPoseDisps[j+1](0,k);
                VO_Shape::GlobalShapeNormalization2SimilarityTrans(posedisp1, dispscale1, dispangles1, disptranslation1 );
                VO_Shape::GlobalShapeNormalization2SimilarityTrans(posedisp2, dispscale2, dispangles2, disptranslation2 );

                // do displacement measures
                currentConcatenatedParameters = this->m_MatAppearanceProject2Truncated.row(i);

                // According to Cootes' "Comparing Variations on the Active Appearance Model Algorithm" - Equation (3)

                // Build the shape instance from the combined model
//                cv::gemm(currentConcatenatedParameters, this->m_MatQs, 1, this->m_PCAAlignedShape.mean, 1, currentShape, GEMM_2_T );
                currentShape = currentConcatenatedParameters * this->m_MatQs.t() + this->m_PCAAlignedShape.mean;

                // Build the texture instance from the combined model
//                cv::gemm(currentConcatenatedParameters, this->m_MatQg, 1, this->m_PCANormalizedTexture.mean, 1,currentTexture, GEMM_2_T);
                currentTexture = currentConcatenatedParameters * this->m_MatQg.t() + this->m_PCANormalizedTexture.mean;

                // Calculate the align transformation
                currentShapeInstance.SetTheShape(currentShape, 2);
                currentShapeInstance.AlignTransformation(this->m_vShapes[i], scale, theta, translation);

                currentShapeInstance1 = currentShapeInstance;
                currentShapeInstance2 = currentShapeInstance;

                vector<float> tempTheta(1, 0.0f);
                Mat_<float> tempTranslate = Mat_<float>::zeros(translation.size());
                tempTheta[0] = dispangles1[0] + theta[0];
                tempTranslate(0,0) = disptranslation1(0,0) + translation(0,0);
                tempTranslate(1,0) = disptranslation1(1,0) + translation(1,0);
                currentShapeInstance1.GlobalShapeNormalization2D(dispscale1 * scale, tempTheta, tempTranslate);
                tempTheta[0] = dispangles2[0] + theta[0];
                tempTranslate(0,0) = disptranslation2(0,0) + translation(0,0);
                tempTranslate(1,0) = disptranslation2(1,0) + translation(1,0);
                currentShapeInstance2.GlobalShapeNormalization2D(dispscale2 * scale, tempTheta, tempTranslate);

                if(!VO_TextureModel::VO_LoadOneTextureFromShape(currentShapeInstance1, img, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, delta_g1))
                    continue;
                if(!VO_TextureModel::VO_LoadOneTextureFromShape(currentShapeInstance2, img, this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo, delta_g2))
                    continue;

                currentTextureInstance.SetTheTexture(currentTexture, delta_g1.GetNbOfTextureRepresentation());

                //// The following codes are just for intermediate display
                if(recordIntermediateImgs)
                {
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(delta_g1, this->m_vTemplateTriangle2D, tempImage1);
                    VO_TextureModel::VO_PutOneTextureToTemplateShape(delta_g2, this->m_vTemplateTriangle2D, tempImage2);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstance1, this->m_vTemplateTriangle2D, tempImage1, resImage1);
                    VO_TextureModel::VO_WarpFromOneShapeToAnother(this->m_VOReferenceShape, currentShapeInstance2, this->m_vTemplateTriangle2D, tempImage2, resImage2);

                    stringstream ssi, ssj, ssj1, ssk;
                    string stri, strj, strj1, strk;
                    ssi << i;
                    ssj << j;
                    ssj1 << (j + 1);
                    ssk << k;
                    ssi >> stri;
                    ssj >> strj;
                    ssj1 >> strj1;
                    ssk >> strk;

                    string temp1Str = "poseDisplaceLoadedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string temp2Str = "poseDisplaceLoadedImage" + stri + "_" + strj1 + "_" + strk + ".jpg";
                    string res1Str     = "poseDisplaceLoadedWarpedImage" + stri + "_" + strj + "_" + strk + ".jpg";
                    string res2Str     = "poseDisplaceLoadedWarpedImage" + stri + "_" + strj1 + "_" + strk + ".jpg";

                    imwrite(temp1Str.c_str(), tempImage1 );
                    imwrite(temp2Str.c_str(), tempImage2 );
                    imwrite(res1Str.c_str(), resImage1 );
                    imwrite(res2Str.c_str(), resImage2 );

                    ssi.clear();
                    ssj.clear();
                    ssj1.clear();
                    ssk.clear();
                }


                // Normalize the extracted(loaded) textures
                delta_g1.Normalize();
                delta_g2.Normalize();
                delta_g2 -= currentTextureInstance;
                delta_g1 -= currentTextureInstance;

                // form central difference
                cDiff = (delta_g2-delta_g1)/(this->m_vvPoseDisps[j+1](0,k) - this->m_vvPoseDisps[j](0,k));
                for (unsigned int n = 0; n < this->m_iNbOfTextures; n++)
                {
                    oPoseGM(n, k) += cDiff.GetATexture(n);
                }

                nExperiment++;
            }
        }
    }

    // normalize
    // this->m_MatCParamGradientMatrix is a summation for 240 pictures (size of this->m_iNbOfSamples)
    // 4 -- refer to 4 possible shifts for just one parameter (16 parameters altogether)
    // 2 -- every pair of shifts, we got only one cDiff, which is actually stored in this->m_MatCParamGradientMatrix
//    oPoseGM /= (this->m_iNbOfSamples * 4 / 2);
    // nExperiment should be equal to this->m_iNbOfSamples * 4 / 2, if every texture can be successfully loaded.
    oPoseGM /= nExperiment;
}


/**
 * @author      JIA Pei
 * @version     2010-02-07
 * @brief       appearance parameters constrain
 * @param       ioC         Input and Output - appearance parameters
 * @param       nSigma      Input - number of sigmas
 * @return      void
*/
void VO_AAMBasic::VO_AppearanceParameterConstraint(Mat_<float>& ioC, float nSigma)
{
    for (unsigned int i = 0; i < ioC.cols; ++i)
    {
        float ct = nSigma * sqrt(this->m_PCAAppearance.eigenvalues.at<float>(i,0) );
        if ( ioC(0, i) > ct )
        {
            ioC(0, i) = ct;
        }
        else if ( ioC(0, i) < -ct )
        {
            ioC(0, i) = -ct;
        }
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       Shape and texture project to shape parameters and texture parameters, and then concatenated
 * @param       iShape      Input   - input shape
 * @param       iTexture    Input   - input texture
 * @param       b           Output  - projected concatenated parameters
 * @return      void
*/
void VO_AAMBasic::VO_ShapeTexture2Appearance( VO_Shape iShape, VO_Texture iTexture, Mat_<float>& app ) const
{
    Mat_<float> b_s = this->m_PCAAlignedShape.project( iShape.GetTheShapeInARow() ) 
                        * this->m_MatWeightsScaleShape2Texture;                             // 1*116->1*15
//    cv::gemm(b_s_temp, this->m_MatWeightsScaleShape2Texture, 1.0, Mat(), 0.0, b_s);       // 1*21->1*21, just for rescaling
    Mat_<float> b_t = this->m_PCANormalizedTexture.project(iTexture.GetTheTextureInARow()); // 1*80259->1*36
    app = Mat_<float>::zeros(1, b_s.cols + b_t.cols);
    
    Mat_<float> roib_s     = app(Range(0, 1), Range(0, b_s.cols) );
    Mat_<float> roib_t     = app(Range(0, 1), Range(b_s.cols, b_s.cols + b_t.cols) );
    b_s.copyTo(roib_s);
    b_t.copyTo(roib_t);
}


/**
* @author     JIA Pei
* @version    2010-04-05
* @brief      appearance project to appearance parameters
* @param      app     Input     - input appearance, a row vector
* @param      outC    Output     - projected appearance parameters, a row vector
* @return     void
*/
void VO_AAMBasic::VO_AppearanceProjectToCParam( const Mat_<float>& app,
                                                Mat_<float>& outC) const
{
    this->m_PCAAppearance.project(app, outC);
}


/**
* @author     JIA Pei
* @version    2010-04-05
* @brief      Shape parameters and texture parameters projected to concatenated parameters
* @param      inS       Input - shape model parameters, a row vector
* @param      inT       Input - texture model parameters, a row vector
* @param      outC      Output - output concatenated parameters, a row vector
* @return     void
*/
void VO_AAMBasic::VO_SParamTParamProjectToCParam(   const Mat_<float>& inS,
                                                    const Mat_<float>& inT,
                                                    Mat_<float>& outC) const
{
    Mat_<float> tempConcatenated = Mat_<float>::zeros(1, this->m_iNbOfShapeEigens + this->m_iNbOfTextureEigens);

    Mat_<float> tempShape = tempConcatenated( Range(0,1), Range(0, this->m_iNbOfShapeEigens) );
    Mat_<float> tempTexture = tempConcatenated( Range(0,1), Range(this->m_iNbOfShapeEigens, this->m_iNbOfAppearance) );
    inS.copyTo(tempShape);
    tempShape *= this->m_MatWeightsScaleShape2Texture;
    inT.copyTo(tempTexture);
    outC = this->m_PCAAppearance.project(tempConcatenated);
}


/**
* @author     JIA Pei
* @version    2010-04-05
* @brief      Appearance parameters back project to appearance
* @param      inC       Input - input appearance parameters
* @param      app       Output - the appearance
* @return     void
*/
void VO_AAMBasic::VO_CParamBackProjectToAppearance(const Mat_<float>& inC, Mat_<float>& app) const
{
    this->m_PCAAppearance.backProject(inC, app);
}


/**
* @author     JIA Pei
* @version    2010-02-05
* @brief      Concatenated parameters back project to shape and texture parameters
* @param      inC             Input - input concatenated parameters
* @param      outS            Output - back projected shape model parameters
* @param      outT            Output - back projected texture model parameters
* @return     void
*/
void VO_AAMBasic::VO_CParamBackProjectToSParamTParam(const Mat_<float>& inC, Mat_<float>& outS, Mat_<float>& outT) const
{
    Mat_<float> tempConcatenated = this->m_PCAAppearance.backProject(inC);

    outS = tempConcatenated( Range(0,1), Range(0, this->m_iNbOfShapeEigens) );
    outT = tempConcatenated( Range(0,1), Range(this->m_iNbOfShapeEigens, this->m_iNbOfAppearance) );
    outS *= this->m_MatWeightsScaleShape2Texture.inv();
}


/**
* @author     JIA Pei
* @version    2010-02-05
* @brief      Concatenated parameters back project to aligned shape
* @param      inC         Input     - input concatenated parameters, a row vector
* @param      oShape      Output     - the back projected shape
* @return     void
*/
void VO_AAMBasic::VO_CParamBackProjectToAlignedShape(const Mat_<float>& inC, VO_Shape& oShape, int dim) const
{
    // c back to shape
//    cv::gemm(inC, this->m_MatQs, 1, this->m_PCAAlignedShape.mean, 1, oShape, GEMM_1_T + GEMM_2_T);
    oShape.SetTheShape(inC * this->m_MatQs.t() + this->m_PCAAlignedShape.mean, dim);
}


/**
* @author     JIA Pei
* @version    2010-02-05
* @brief      Concatenated parameters back project to aligned shape
* @param      inC         Input - input concatenated parameters
* @param      oTexture    Output - the back projected shape
* @return     void
*/
void VO_AAMBasic::VO_CParamBackProjectToNormalizedTexture(const Mat_<float>& inC, VO_Texture& oTexture, int tr) const
{
    // c back to texture
//    cv::gemm(inC, this->m_MatQg, 1, this->m_PCANormalizedTexture.mean, 1, oTexture, GEMM_1_T + GEMM_2_T);
    oTexture.SetTheTexture(inC * this->m_MatQg.t() + this->m_PCANormalizedTexture.mean, tr);
}


/**
* @author     JIA Pei
* @version    2010-02-05
* @brief      generate displacement vectors for later numerical analysis
* @note       Explained by Stegmann: sets must *always* be in anti-symmetric pairs with the largest displacements first, 
                e.g. [ -.5, .5, -.25, .25 ]
                According to (7.7) from Stegmann's AAM-API, there will be n=m(4k+24)=240*(4*12+24)=21120 displacements.
                However, here, for the pose displacements, the amount is not 24 but 16 instead. n=m(4k+16)=240*(4*12+16)=21120
 * @ref         1) Cootes' "Comparing Variations on the Active Appearance Model Algorithm"
                2) Cootes' "Statistical Models of Appearance for Computer Vision" page 56
                3) Stegmann's" AAM-API, Basic Active Appearance Models". 
*/
void VO_AAMBasic::VO_CreateDisplacementSets()
{
    // generate c and pose displacement sets
    vector<float> vStdDisp;
    vector<float> vXYDisp;
    vector<float> vScaleDisp;
    vector<float> vRotDisp;

    // Mikkel B. Stegmann's master's thesis displacement set
    vStdDisp.push_back(-.25f);
    vStdDisp.push_back( .25f);
    vStdDisp.push_back(-.5f);
    vStdDisp.push_back( .5f);

    // relative displacement set
    vScaleDisp.push_back( .95f);
    vScaleDisp.push_back(1.05f);
    vScaleDisp.push_back( .90f);
    vScaleDisp.push_back(1.10f);

    // in radian measure
    vRotDisp.push_back( (float)(-5.0 / 180.0 * CV_PI) );
    vRotDisp.push_back( (float)( 5.0 / 180.0 * CV_PI) );
    vRotDisp.push_back( (float)(-10.0 / 180.0 * CV_PI) );
    vRotDisp.push_back( (float)( 10.0 / 180.0 * CV_PI) );

    vXYDisp.push_back(-.05f);
    vXYDisp.push_back( .05f);
    vXYDisp.push_back(-.10f);
    vXYDisp.push_back( .10f);

    // Pre-process for X, Y pose parameters
    vector<float> vXDisp, vYDisp;
    vXDisp.resize (vXYDisp.size());
    vYDisp.resize (vXYDisp.size());
    for (unsigned int i = 0; i < vXYDisp.size(); i++)
    {
        vXDisp[i] = vXYDisp[i] * this->m_ImageTemplateFace.cols;
        vYDisp[i] = vXYDisp[i] * this->m_ImageTemplateFace.rows;
    }

    this->m_vvCDisps = VO_AAMBasic::VO_CalcCParamDisplacementVectors( vStdDisp, this->m_PCAAppearance.eigenvalues );
    this->m_vvPoseDisps = VO_AAMBasic::VO_CalcPoseDisplacementVectors( vScaleDisp, vRotDisp, vXDisp, vYDisp );
}


/**
 * @author      Stegmann, JIA Pei
 * @version     2010-02-05
 * @brief       Generates a set of combined model parameter displacement vectors
                where each parameter is displaced at a time according to the values in vStdDisp.
 * @param       vStdDisp                    Input - A row vector of parameter displacements in standard deviations of the corresponding parameter.
 * @param       ConcatenatedSD                Input - Concatenated standard deviation, a column vector
 * @return      A vector of displacement vectors.
*/
vector< Mat_<float> > VO_AAMBasic::VO_CalcCParamDisplacementVectors(const vector<float>& vStdDisp, const Mat_<float>& ConcatenatedSD)
{
    vector< Mat_<float> >    cDisplacements;
    cDisplacements.resize( vStdDisp.size() );
    for (unsigned int i = 0; i < cDisplacements.size(); i++)
    {
        cDisplacements[i] = Mat_<float>::zeros(1, ConcatenatedSD.rows );
    }

    // calculate displacements, for each parameter
    for (unsigned int i = 0; i < cDisplacements.size(); i++)
    {
        for(unsigned int j = 0; j < cDisplacements[0].cols; j++)
        {
            // for each displacement
            cDisplacements[i](0,j) = vStdDisp[i] * ConcatenatedSD(j, 0);
        }
    }

    return cDisplacements;
}


/**
* @author     JIA Pei
* @version    2010-02-05
* @brief      Build displacement sets for Pose parameters
* @param      vScaleDisp      Input - all input shapes
* @param      vRotDisp        Input - all input images
* @param      vXDisp          Input - 1 or 3
* @param      vYDisp          Input - texture build method
* @param      method          Input - truncated percentage for shape model
* @return     vector<vector<float> > - all pose displacement vectors
*/
vector< Mat_<float> >   VO_AAMBasic::VO_CalcPoseDisplacementVectors(const vector<float>& vScaleDisp, 
                                                                    const vector<float>& vRotDisp,
                                                                    const vector<float>& vXDisp,
                                                                    const vector<float>& vYDisp)
{
    vector< Mat_<float> > poseDisplacements;
    poseDisplacements.resize( vXDisp.size() );
    //for (unsigned int i = 0; i < poseDisplacements.size(); i++)
    //{
    //    poseDisplacements[i] = Mat_<float>::zeros(1, 4);    // 4 refers to X, Y, Scale, and Rotate
    //}

    //Mat_<float> singlePose = Mat_<float>::zeros(1, 4);
    float scale = 1.0f;
    vector<float> angles(1);
    Mat_<float> translation = Mat_<float>::zeros(2, 1);

    for(unsigned int i = 0; i < poseDisplacements.size(); i++)
    {
        //singlePose(0,0) = vScaleDisp[i];
        //singlePose(0,1) = vRotDisp[i];
        //singlePose(0,2) = vXDisp[i];
        //singlePose(0,3) = vYDisp[i];
        scale                = vScaleDisp[i];
        angles[0]            = vRotDisp[i];
        translation(0,0)    = vXDisp[i];
        translation(1,0)    = vYDisp[i];

        VO_Shape::SimilarityTrans2GlobalShapeNormalization(scale, angles, translation, poseDisplacements[i]);
    }

    return poseDisplacements;
}


/**
 * @author      JIA Pei
 * @version     2010-04-03
 * @brief       Build Appearance Model, Basic AAM
 * @param       allLandmarkFiles4Training   Input - all training landmark files
 * @param       allImgFiles4Training        Input - all training image files
 * @param       shapeinfoFileName           Input - shape info file
 * @param       database                    Input - which database is it?
 * @param       channels                    Input - How many channels are to be used?
 * @param       levels                      Input - multiscale levels
 * @param       trm                         Input - texture representation method
 * @param       TPShape                     Input - truncated percentage for shape model
 * @param       TPTexture                   Input - truncated percentage for texture model
 * @param       TPConcatenated              Input - truncated percentage for appearance model
 * @param       useKnownTriangles           Input - use known triangle structures??
 * @return      void
 * @note        Using "float* oProf" is much much faster than using "VO_Profile& oProf" or "vector<float>"
 */
void VO_AAMBasic::VO_BuildAppearanceModel(  const vector<string>& allLandmarkFiles4Training,
                                            const vector<string>& allImgFiles4Training,
                                            const string& shapeinfoFileName, 
                                            unsigned int database,
                                            unsigned int channels,
                                            unsigned int levels,
                                            int trm, 
                                            float TPShape, 
                                            float TPTexture,
                                            float TPConcatenated, 
                                            bool useKnownTriangles)
{
    this->VO_BuildTextureModel( allLandmarkFiles4Training,
                                allImgFiles4Training,
                                shapeinfoFileName,
                                database,
                                channels,
                                trm,
                                TPShape,
                                TPTexture,
                                useKnownTriangles);
    this->m_iNbOfPyramidLevels                  = levels;
    this->m_iNbOfAppearance                     = this->m_iNbOfShapeEigens + this->m_iNbOfTextureEigens;
    this->m_iNbOfEigenAppearanceAtMost          = MIN(this->m_iNbOfSamples, this->m_iNbOfAppearance);
    this->m_fTruncatedPercent_Appearance        = TPConcatenated;

    this->m_MatWeightsScaleShape2Texture        = Mat_<float>::zeros( this->m_iNbOfShapeEigens, this->m_iNbOfShapeEigens);
    float SumOfEigenValues_Shape                = cv::sum( this->m_PCAAlignedShape.eigenvalues ).val[0];
    float SumOfEigenValues_Texture              = cv::sum( this->m_PCANormalizedTexture.eigenvalues ).val[0];
    float val = (float)(SumOfEigenValues_Texture / SumOfEigenValues_Shape);
    cv::setIdentity(this->m_MatWeightsScaleShape2Texture, val);

    ////////////////////////////////////////////////////////////////////////////
    // Assign concatenated
    ////////////////////////////////////////////////////////////////////////////
    Mat_<float> matAlignedShapes                = Mat_<float>::zeros(this->m_iNbOfSamples, this->m_iNbOfShapes);
    Mat_<float> matNormalizedTextures           = Mat_<float>::zeros(this->m_iNbOfSamples, this->m_iNbOfTextures);
    for(unsigned int i = 0; i < this->m_iNbOfSamples; ++i)
    {
        Mat tmpRowShape     = matAlignedShapes.row(i);
        this->m_vAlignedShapes[i].GetTheShapeInARow().copyTo(tmpRowShape);
        Mat tmpRowTexture   = matNormalizedTextures.row(i);
        this->m_vNormalizedTextures[i].GetTheTextureInARow().copyTo(tmpRowTexture);
    }
    Mat_<float> AlignedShapesProject2Truncated          = this->m_PCAAlignedShape.project(matAlignedShapes);
    Mat_<float> NormalizedTexturesProject2Truncated     = this->m_PCANormalizedTexture.project(matNormalizedTextures);
    Mat_<float> WeightedAlignedShapesProject2Truncated  = AlignedShapesProject2Truncated * this->m_MatWeightsScaleShape2Texture;
//    cv::gemm(AlignedShapesProject2Truncated, this->m_MatWeightsScaleShape2Texture, 1.0, Mat(), 0.0, WeightedAlignedShapesProject2Truncated);

    Mat_<float> matConcatenated                 = Mat_<float>::zeros( this->m_iNbOfSamples, this->m_iNbOfAppearance);
    Mat_<float> matMeanConcatenated             = Mat_<float>::zeros(1, this->m_iNbOfAppearance);        // Obviously, matMeanConcatenated should be all zeros

    Mat roiShape    = matConcatenated(Range(0, this->m_iNbOfSamples), Range(0, this->m_iNbOfShapeEigens) );
    Mat roiTexture  = matConcatenated(Range(0, this->m_iNbOfSamples), Range(this->m_iNbOfShapeEigens, this->m_iNbOfAppearance) );
    WeightedAlignedShapesProject2Truncated.copyTo(roiShape);
    NormalizedTexturesProject2Truncated.copyTo(roiTexture);

    ////////////////////////////////////////////////////////////////////////////

    this->m_PCAAppearance(matConcatenated, matMeanConcatenated, CV_PCA_DATA_AS_ROW, this->m_iNbOfEigenAppearanceAtMost );
    // to decide how many components to be selected
    this->m_iNbOfAppearanceEigens = 0;

    double SumOfEigenValues = cv::sum( this->m_PCAAppearance.eigenvalues ).val[0];
    double ps = 0.0f;

    for(unsigned int i = 0; i < this->m_iNbOfEigenAppearanceAtMost; i++)
    {
        ps += this->m_PCAAppearance.eigenvalues.at<float>(i, 0 );
        ++this->m_iNbOfAppearanceEigens;
        if( ps/SumOfEigenValues >= this->m_fTruncatedPercent_Appearance) break;
    }
    // m_iNbOfAppearanceEigens decided. For simplicity, we carry out PCA once again.
    this->m_PCAAppearance(matConcatenated, matMeanConcatenated, CV_PCA_DATA_AS_ROW, this->m_iNbOfAppearanceEigens );
    this->m_MatAppearanceProject2Truncated = this->m_PCAAppearance.project(matConcatenated);

    // extract the shape part of the combined eigen vectors
    this->m_MatPcs = this->m_PCAAppearance.eigenvectors (Range(0, this->m_iNbOfAppearanceEigens), Range(0, this->m_iNbOfShapeEigens));

    // extract the texture part of the combined eigen vectors
    this->m_MatPcg = this->m_PCAAppearance.eigenvectors (Range(0, this->m_iNbOfAppearanceEigens), Range(this->m_iNbOfShapeEigens, this->m_iNbOfAppearance));

    // calculate m_MatQs = m_PCAAlignedShape.eigenvectors * m_MatWeightsScaleShape2Texture^{-1} * m_MatPcs
    this->m_MatQs = this->m_PCAAlignedShape.eigenvectors.t() * this->m_MatWeightsScaleShape2Texture.inv() * this->m_MatPcs.t();

    // calculate m_MatQg = m_PCANormalizedTexture.eigenvectors * m_MatPcg
    this->m_MatQg = this->m_PCANormalizedTexture.eigenvectors.t() * this->m_MatPcg.t();

    this->VO_CreateDisplacementSets( );

//    this->VO_CalcRegressionMatrices();
    this->VO_CalcGradientMatrices();
}


/**
 * @author      JIA Pei
 * @version     2010-02-11
 * @brief       Save Appearance Model to a specified folder
 * @param       fn      Input - the folder that AAMBasic to be saved to
 * @return      void
*/
void VO_AAMBasic::VO_Save(const string& fd)
{
    VO_AXM::VO_Save(fd);

    string fn = fd+"/AppearanceModel";
    if (!boost::filesystem::is_directory(fn) )
        boost::filesystem::create_directory( fn );

    fstream fp;
    string tempfn;

    // AppearanceModel
    tempfn = fn + "/AppearanceModel" + ".txt";
    fp.open(tempfn.c_str (), ios::out);

    fp << "m_iNbOfAppearance" << endl << this->m_iNbOfAppearance << endl;                           // m_iNbOfAppearance
    fp << "m_iNbOfEigenAppearanceAtMost" << endl << this->m_iNbOfEigenAppearanceAtMost << endl;     // m_iNbOfEigenAppearanceAtMost
    fp << "m_iNbOfAppearanceEigens" << endl << this->m_iNbOfAppearanceEigens << endl;               // m_iNbOfAppearanceEigens
    fp << "m_fTruncatedPercent_Appearance" << endl << this->m_fTruncatedPercent_Appearance << endl; // m_fTruncatedPercent_Appearance

    fp.close();fp.clear();

    // m_MatWeightsScaleShape2Texture
    tempfn = fn + "/m_MatWeightsScaleShape2Texture" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatWeightsScaleShape2Texture" << endl;
    fp << this->m_MatWeightsScaleShape2Texture;
    fp.close();fp.clear();
    
    // m_PCAAppearanceMean
    tempfn = fn + "/m_PCAAppearanceMean" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_PCAAppearanceMean" << endl;
    fp << Mat_<float>(this->m_PCAAppearance.mean);
    fp.close();fp.clear();

    // m_PCAAppearanceEigenValues
    tempfn = fn + "/m_PCAAppearanceEigenValues" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_PCAAppearanceEigenValues" << endl;
    fp << Mat_<float>(this->m_PCAAppearance.eigenvalues);
    fp.close();fp.clear();

    // m_PCAAppearanceEigenVectors
    tempfn = fn + "/m_PCAAppearanceEigenVectors" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_PCAAppearanceEigenVectors" << endl;
    fp << Mat_<float>(this->m_PCAAppearance.eigenvectors);
    fp.close();fp.clear();

    // m_MatPcs    
    tempfn = fn + "/m_MatPcs" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatPcs" << endl;
    fp << this->m_MatPcs;
    fp.close();fp.clear();

    // m_MatPcg
    tempfn = fn + "/m_MatPcg" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatPcg" << endl;
    fp << this->m_MatPcg;
    fp.close();fp.clear();

    // m_MatQs
    tempfn = fn + "/m_MatQs" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatQs" << endl;
    fp << this->m_MatQs;
    fp.close();fp.clear();

    // m_MatQg
    tempfn = fn + "/m_MatQg" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatQg" << endl;
    fp << this->m_MatQg;
    fp.close();fp.clear();

    // m_MatRc
    tempfn = fn + "/m_MatRc" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatRc" << endl;
    fp << this->m_MatRc;
    fp.close();fp.clear();

    // m_MatRt
    tempfn = fn + "/m_MatRt" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatRt" << endl;
    fp << this->m_MatRt;
    fp.close();fp.clear();

    // m_MatCParamGradientMatrix
    tempfn = fn + "/m_MatCParamGradientMatrix" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatCParamGradientMatrix" << endl;
    fp << this->m_MatCParamGradientMatrix;
    fp.close();fp.clear();

    // m_MatPoseGradientMatrix, we may ignore this first
    tempfn = fn + "/m_MatPoseGradientMatrix" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_MatPoseGradientMatrix" << endl;
    fp << this->m_MatPoseGradientMatrix;
    fp.close();fp.clear();

}


/**
 * @author      JIA Pei
 * @version     2010-02-11
 * @brief       Load all Appearance Model data from a specified folder
 * @param       fd  Input - the folder that AppearanceModel to be loaded from
 * @return      void
*/
void VO_AAMBasic ::VO_Load(const string& fd)
{
    VO_AXM::VO_Load(fd);

    string fn = fd+"/AppearanceModel";
    if (!boost::filesystem::is_directory(fn) )
    {
        cout << "AppearanceModel subfolder is not existing. " << endl;
        exit(EXIT_FAILURE);
    }

    //    ifstream fp;
    //    string tempfn;
    //    string temp;
    //    
    //    // m_MatAppearanceProject2Truncated
    //    tempfn = fn + "/m_MatAppearanceProject2Truncated" + ".txt";
    //    fp.open(tempfn.c_str (), ios::in);
    //    fp >> temp;
    //    fp >> this->m_MatAppearanceProject2Truncated;
    //    fp.close();fp.clear();
    
    // m_vvCDisps
    
    // m_vvPoseDisps

}


/**
 * @author      JIA Pei
 * @version     2010-02-11
 * @brief       Load all AAM data from a specified folder for later fitting
 * @param       fd      Input - the folder that AAM to be loaded from
 * @return      void
*/
void VO_AAMBasic::VO_LoadParameters4Fitting(const string& fd)
{
    VO_AXM::VO_LoadParameters4Fitting(fd);

    string fn = fd+"/AppearanceModel";
    if (!boost::filesystem::is_directory(fn) )
    {
        cout << "AppearanceModel subfolder is not existing. " << endl;
        exit(EXIT_FAILURE);
    }

    ifstream fp;
    string tempfn;
    string temp;

    // AppearanceModel
    tempfn = fn + "/AppearanceModel" + ".txt";
    fp.open(tempfn.c_str (), ios::in);

    fp >> temp >> this->m_iNbOfAppearance;
    fp >> temp >> this->m_iNbOfEigenAppearanceAtMost;
    fp >> temp >> this->m_iNbOfAppearanceEigens;
    fp >> temp >> this->m_fTruncatedPercent_Appearance;
    fp.close();fp.clear();
    
    this->m_PCAAppearance = cv::PCA();

    // m_PCAAppearanceMean
    this->m_PCAAppearance.mean = Mat_<float>::zeros(1, this->m_iNbOfAppearance);
    tempfn = fn + "/m_PCAAppearanceMean" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_PCAAppearance.mean;
    fp.close();fp.clear();
    
    // m_PCAAppearanceEigenValues
    this->m_PCAAppearance.eigenvalues = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, 1);
    tempfn = fn + "/m_PCAAppearanceEigenValues" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_PCAAppearance.eigenvalues;
    fp.close();fp.clear();
        
    // m_PCAAppearanceEigenVectors
    this->m_PCAAppearance.eigenvectors = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, this->m_iNbOfAppearance);
    tempfn = fn + "/m_PCAAppearanceEigenVectors" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_PCAAppearance.eigenvectors;
    fp.close();fp.clear();
    
    // m_MatWeightsScaleShape2Texture
    this->m_MatWeightsScaleShape2Texture = Mat_<float>::zeros(this->m_iNbOfShapeEigens, this->m_iNbOfShapeEigens);
    tempfn = fn + "/m_MatWeightsScaleShape2Texture" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatWeightsScaleShape2Texture;
    fp.close();fp.clear();

    // m_MatPcs
    this->m_MatPcs = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, this->m_iNbOfShapeEigens);
    tempfn = fn + "/m_MatPcs" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatPcs;
    fp.close();fp.clear();

    // m_MatPcg
    this->m_MatPcg = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, this->m_iNbOfTextureEigens);
    tempfn = fn + "/m_MatPcg" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatPcg;
    fp.close();fp.clear();

    // m_MatQs
    this->m_MatQs = Mat_<float>::zeros(this->m_iNbOfShapes, this->m_iNbOfAppearanceEigens);
    tempfn = fn + "/m_MatQs" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatQs;
    fp.close();fp.clear();

    // m_MatQg
    this->m_MatQg = Mat_<float>::zeros(this->m_iNbOfTextures, this->m_iNbOfAppearanceEigens);
    tempfn = fn + "/m_MatQg" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatQg;
    fp.close();fp.clear();

    // m_MatRc
    this->m_MatRc = Mat_<float>::zeros(this->m_iNbOfAppearanceEigens, this->m_iNbOfTextures);
    tempfn = fn + "/m_MatRc" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatRc;
    fp.close();fp.clear();

    // m_MatRt
    this->m_MatRt = Mat_<float>::zeros(4, this->m_iNbOfTextures);
    tempfn = fn + "/m_MatRt" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatRt;
    fp.close();fp.clear();

    // m_MatCParamGradientMatrix
    this->m_MatCParamGradientMatrix = Mat_<float>::zeros(this->m_iNbOfTextures, this->m_iNbOfAppearanceEigens);
    tempfn = fn + "/m_MatCParamGradientMatrix" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatCParamGradientMatrix;
    fp.close();fp.clear();

    // m_MatPoseGradientMatrix, we may ignore this first
    this->m_MatPoseGradientMatrix = Mat_<float>::zeros(this->m_iNbOfTextures, 4);
    tempfn = fn + "/m_MatPoseGradientMatrix" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_MatPoseGradientMatrix;
    fp.close();fp.clear();

}

