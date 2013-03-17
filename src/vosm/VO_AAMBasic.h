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
* Create Date:      2010-04-03                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/



#ifndef __VO_AAMBASIC_H__
#define __VO_AAMBASIC_H__


#include <vector>

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_AXM.h"


using namespace std;
using namespace cv;


/**
 * @author    JIA Pei
 * @brief    Concatenated appearance Model -- 
 *              a direct combination of shape model and texture model.
 * @ref                    http://www2.imm.dtu.dk/~aam/main/
 */
class VO_AAMBasic : public VO_AXM
{
friend class VO_Fitting2DSM;
friend class VO_FittingAAMBasic;
friend class VO_FittingAAMForwardIA;
friend class VO_FittingAAMInverseIA;
friend class VO_FittingASMLTCs;
friend class VO_FittingASMNDProfiles;
friend class VO_FittingAFM;
protected:
    /** PCA transform for appearance, including eigenvectors, eigenvalues, and mean */
    PCA                     m_PCAAppearance;

    /** Shape weights, for scaling to texture scale. 15*15 */
    Mat_<float>             m_MatWeightsScaleShape2Texture;

    /** Original appearance project to truncated space. For IMM, 60*12 */
    Mat_<float>             m_MatAppearanceProject2Truncated;

    /** The shape part of the appearance model eigenvectors, 12*15, refer to equation (5.7)
     * Cootes "Statistical Model of Appearance for Computer Vision" */
    Mat_<float>             m_MatPcs;

    /** The texture part of the appearance model eigenvectors, 12*36, refer to equation (5.7)
     * Cootes "Statistical Model of Appearance for Computer Vision" */
    Mat_<float>             m_MatPcg;

    /** For shape, 116*12, refer to equation (5.9)
     * Cootes "Statistical Model of Appearance for Computer Vision" */
    Mat_<float>             m_MatQs;

    /** For texture, 80259*12, refer to equation (5.9)
     * Cootes "Statistical Model of Appearance for Computer Vision" */
    Mat_<float>             m_MatQg;

    /** In face, m_MatRc and m_MatRt are just the Hessian Matrics!!! */
    /** For shape, 12*80259, Multivariate Linear Regression Matrix,
        refer to Stegmann's AAM-API Equation (7.4) */
    Mat_<float>             m_MatRc;

    /** For shape, 4*80259, Multivariate Linear Regression Matrix,
        refer to Stegmann's AAM-API Equation (7.4) */
    /** JIA Pei declare that this variable is absolutely useless, 
        and if using it, it makes everything unreasonable and illogical */
    Mat_<float>             m_MatRt;

    /** Totally, n=m(4(k+4))=60*(4*(12+4))=3840 displacements. 4*12 */
    vector< Mat_<float> >   m_vvCDisps;

    /** Totally, n=m(4(k+4))=60*(4*(12+4))=3840 displacements. 4*4, 
        refer to AAM-API page 3 of 10 */
    vector< Mat_<float> >   m_vvPoseDisps;

    /** Stegmann: Gradient Matrix 80259*12 */
    Mat_<float>             m_MatCParamGradientMatrix;

    /** Stegmann: Pose Gradient Matrix 80259*4 we may ignore this */
    Mat_<float>             m_MatPoseGradientMatrix;

    /** Number of Appearance m_iNbOfAppearance = m_iNbOfShapeEigens +
        m_iNbOfTextureEigens. For IMM: 15+36=51 */
    unsigned int            m_iNbOfAppearance;

    /** Most possible appearance model eigens before PCA. For IMM: min (51, 60) = 51 */
    unsigned int            m_iNbOfEigenAppearanceAtMost;

    /** Number of appearance model eigens. For IMM: 12 */
    unsigned int            m_iNbOfAppearanceEigens;

    /** Truncate Percentage for appearance PCA. Normally, 0.95 */
    float                   m_fTruncatedPercent_Appearance;

    /** Initialization */
    void                    init()
    {
        this->m_iMethod                         = VO_AXM::AAM_BASIC;    // AAM_DIRECT
        this->m_iNbOfAppearance                 = 0;
        this->m_iNbOfEigenAppearanceAtMost      = 0;
        this->m_iNbOfAppearanceEigens           = 0;
        this->m_fTruncatedPercent_Appearance    = 0.95f;
    }

public:
    /** Default constructor to create a VO_AAMBasic object */
    VO_AAMBasic()
    {
        this->init();
    }

    /** Destructor */
    ~VO_AAMBasic()
    {
        this->m_vvCDisps.clear();
        this->m_vvPoseDisps.clear();
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////Regression//////////////////////////////
    /** Unfinished yet.... */
    /** Off-line build prediction matrix for fitting. */
    /** Please refer to http://www2.imm.dtu.dk/~aam/main/node16.html; revised from AAM-API */
    void            VO_CalcRegressionMatrices();

    /** Carry out C displacement experiments */
    void            VO_DoCParamExperiments();

    /** Carry out pose displacement experiments */
    void            VO_DoPoseExperiments();

    /** Carry out multi variate linear regression experiments */
    void            VO_DoRegression();
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////////////////////////
    //////Gradient -- This is what's happening in Stegmann's code///////////////
    /** Build gradient matrices */
    void            VO_CalcGradientMatrices();

    /** Build gradient matrices in terms of C parameters */
    void            VO_EstCParamGradientMatrix(Mat_<float>& oCParamGM);

    /** Build gradient matrices in terms of pose */
    void            VO_EstPoseGradientMatrix(Mat_<float>& oPoseGM);
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    /** Appearance parameters constraints */
    void            VO_AppearanceParameterConstraint(Mat_<float>& ioC, 
                                                            float nSigma = 4.0f);

    /** Shape and texture project to shape parameters and texture parameters, and then concatenated */
    void            VO_ShapeTexture2Appearance( VO_Shape iShape, 
                                                VO_Texture iTexture, 
                                                Mat_<float>& app ) const;
    
    /** Appearance projected to appearance parameters */
    void            VO_AppearanceProjectToCParam(   const Mat_<float>& app, 
                                                    Mat_<float>& outC) const;

    /** Shape parameters and texture parameters projected to concatenated parameters */
    void            VO_SParamTParamProjectToCParam( const Mat_<float>& inS, 
                                                    const Mat_<float>& inT, 
                                                    Mat_<float>& outC) const;

    /** Appearance parameters back projected to appearance */
    void            VO_CParamBackProjectToAppearance(   const Mat_<float>& inC, 
                                                        Mat_<float>& app) const;
    
    /** Concatenated parameters back projected to shape parameters and texture parameters */
    void            VO_CParamBackProjectToSParamTParam( const Mat_<float>& inC, 
                                                        Mat_<float>& outS, 
                                                        Mat_<float>& outT) const;

    /** Concatenated parameters back projected to aligned shape */
    void            VO_CParamBackProjectToAlignedShape( const Mat_<float>& inC, 
                                                        VO_Shape& oShape, 
                                                        int dim = 2) const;

    /** Concatenated parameters back projected to normalized texture */
    void            VO_CParamBackProjectToNormalizedTexture(const Mat_<float>& inC, 
                                                            VO_Texture& oTexture, 
                                                            int tr = 3) const;

    /** Build displacement sets */
    void            VO_CreateDisplacementSets();

    /** Build displacement sets for C parameters */
    static vector< Mat_<float> >    VO_CalcCParamDisplacementVectors(   const vector<float>& vStdDisp, 
                                                                        const Mat_<float>& cVectors);

    /** Build displacement sets for Pose parameters */
    static vector< Mat_<float> >    VO_CalcPoseDisplacementVectors( const vector<float> &vScaleDisp, 
                                                                    const vector<float>& vRotDisp, 
                                                                    const vector<float>& vXDisp, 
                                                                    const vector<float>& vYDisp);

    /** Build Appearance model */
    void            VO_BuildAppearanceModel(const vector<string>& allLandmarkFiles4Training,
                                            const vector<string>& allImgFiles4Training,
                                            const string& shapeinfoFileName,
                                            unsigned int database,
                                            unsigned int channels = 3,
                                            unsigned int levels = 1,
                                            int trm = VO_Features::DIRECT,
                                            float TPShape = 0.95f,
                                            float TPTexture = 0.95f,
                                            float TPConcatenated = 0.95f,
                                            bool useKnownTriangles = false);

    /** Save Appearance Model, to a specified folder */
    void            VO_Save(const string& fd);

    /** Load all parameters */
    void            VO_Load(const string& fd);

    /** Load Parameters for fitting */
    void            VO_LoadParameters4Fitting(const string& fd);

    /** Gets and Sets */
    Mat_<float>             GetAppearanceMean() const {return this->m_PCAAppearance.mean;}
    Mat_<float>             GetAppearanceEigenValues() const {return this->m_PCAAppearance.eigenvalues;}
    Mat_<float>             GetAppearanceEigenVectors() const {return this->m_PCAAppearance.eigenvectors;}
    Mat_<float>             GetWeightsScaleShape2Texture() const {return this->m_MatWeightsScaleShape2Texture;}
    Mat_<float>             GetAppearanceProject2Truncated() const {return this->m_MatAppearanceProject2Truncated;}
    Mat_<float>             GetPcs() const {return this->m_MatPcs;}
    Mat_<float>             GetPcg() const {return this->m_MatPcg;}
    Mat_<float>             GetQs() const {return this->m_MatQs;}
    Mat_<float>             GetQg() const {return this->m_MatQg;}
    Mat_<float>             GetRc() const {return this->m_MatRc;}
    Mat_<float>             GetRt() const {return this->m_MatRt;}
    vector< Mat_<float> >   GetCDisps() const {return this->m_vvCDisps;}
    vector< Mat_<float> >   GetPoseDisps() const {return this->m_vvPoseDisps;}
    Mat_<float>             GetCParamGradientMatrix() const {return this->m_MatCParamGradientMatrix;}
    Mat_<float>             GetPoseGradientMatrix() const {return this->m_MatPoseGradientMatrix;}
    unsigned int            GetNbOfAppearance() const {return this->m_iNbOfAppearance;}
    unsigned int            GetNbOfEigenAppearanceAtMost() const {return this->m_iNbOfEigenAppearanceAtMost;}
    unsigned int            GetNbOfAppearanceEigens() const {return this->m_iNbOfAppearanceEigens;}
    float                   GetTruncatedPercent_Concatenated() const {return this->m_fTruncatedPercent_Appearance;}

    //    void                    SetWeightsScaleShape2Texture(const Mat_<float>& inWeightsScaleShape2Texture) 
    //                            {inWeightsScaleShape2Texture.copyTo(this->m_MatWeightsScaleShape2Texture);}
    //    void                    SetPcs(const Mat_<float>& inPcs) {inPcs.copyTo(this->m_MatPcs);}
    //    void                    SetPcg(const Mat_<float>& inPcg) {inPcg.copyTo(this->m_MatPcg);}
    //    void                    SetQs(const Mat_<float>& inQs) {inQs.copyTo(this->m_MatQs);}
    //    void                    SetQg(const Mat_<float>& inQg) {inQg.copyTo(this->m_MatQg);}
    //    void                    SetRc(const Mat_<float>& inRc) {inRc.copyTo(this->m_MatRc);}
    //    void                    SetRt(const Mat_<float>& inRt) {inRt.copyTo(this->m_MatRt);}
    //    void                    SetCDisps(const vector<vector<float> >& inCDisps) {this->m_vvCDisps = inCDisps;}
    //    void                    SetPoseDisps(const vector<vector<float> >& inPoseDisps) {this->m_vvPoseDisps = inPoseDisps;}
    //    void                    SetCParamGradientMatrix(const Mat_<float>& inCParamGradientMatrix) 
    //                            {inCParamGradientMatrix.copyTo(this->m_MatCParamGradientMatrix);}
    //    void                    SetPoseParamGradientMatrix(const Mat_<float>& inPoseGradientMatrix) 
    //                            {inPoseGradientMatrix.copyTo(this->m_MatPoseGradientMatrix);}
    //    void                    SetNbOfConcatenated(unsigned int inNbOfConcatenated) {this->m_iNbOfAppearance = inNbOfConcatenated;}
    //    void                    SetTruncatedPercent_Concatenated(float inTruncatedPercent_Concatenated) 
    //                            {this->m_fTruncatedPercent_Appearance = inTruncatedPercent_Concatenated;}
 };


#endif // __VO_AAMBASIC_H__
