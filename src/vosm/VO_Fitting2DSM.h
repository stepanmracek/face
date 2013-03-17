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
* Create Date:      2010-11-04                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/


#ifndef     __VO_2DSMFitting__
#define     __VO_2DSMFitting__

#include "opencv/cv.h"
#include "opencv/highgui.h"


#include "VO_Shape.h"
#include "VO_Texture.h"
#include "VO_Shape2DInfo.h"
#include "VO_WarpingPoint.h"
#include "VO_AXM.h"

using namespace std;
using namespace cv;

/** 
 * @author        JIA Pei
 * @brief        Generalized class for 2D statistical model fitting algorithms.
 */
class VO_Fitting2DSM
{
public:
    /** Maximum iteration times */
    static const int                EPOCH = 10;

    /** pClose */
    static float                    pClose;

protected:
    /** number of pyramid levels */
    unsigned int                    m_iNbOfPyramidLevels;

    /** which fitting method is now undertaken */
    unsigned int                    m_iFittingMethod;

    /** number of iterations */
    unsigned int                    m_iIteration;
    
    /** fitting time */
    float                           m_fFittingTime;

    // The following 3 parameters are just for first estimation from the aligned shape to the real size shape instance.
    // If we take pose into our consideration, (C - only deal with non-rigid transform and pose - global shape normalizatioin)
    // the following 3 parameters are of course will not change during the fitting iterations.
    /** scaling in Procrustes Analysis */
    float                           m_fScale;

    /** rotation in Procrustes Analysis */
    vector<float>                   m_vRotateAngles;
//    float                            m_fRot;

    /** center of gravity for translation, in Procrustes Analysis */
    Mat_<float>                     m_MatCenterOfGravity;

    /** original input image */
    Mat                             m_ImageInput;

    /** image to be processed, the image under processing might not be the same channel as original input image */
    Mat                             m_ImageProcessing;

    /** output image, the final image of the fitting process*/
    Mat                             m_ImageOutput;

    /** Shape parameter */
    Mat_<float>                     m_MatModelAlignedShapeParam;

    /** Texture parameter */
    Mat_<float>                     m_MatModelNormalizedTextureParam;

    /** current fitting shape instance */
    VO_Shape                        m_VOTemplateAlignedShape;
    VO_Shape                        m_VOModelAlignedShape;
    VO_Shape                        m_VOFittingShape;
    VO_Shape                        m_VOEstimatedShape;

    /** current fitting texture instance */
    VO_Texture                      m_VOTemplateNormalizedTexture;
    VO_Texture                      m_VOModelNormalizedTexture;
    VO_Texture                      m_VOFittingTexture;
    VO_Texture                      m_VOTextureError;
    VO_Texture                      m_VOEstimatedTextureError;

    /** current fitting normalized texture instance extracted from original image, but with shape parameters */
    Mat_<float>                     m_MatNormalizedTextureInstanceExtractedFromImage;

    /** triangle structure */
    vector<VO_Triangle2DStructure>  m_vTriangle2D;

    /** shape information */
    vector<VO_Shape2DInfo>          m_vShape2DInfo;

    /** face parts information, seems to have the duplicate information as m_vShape2DInfo, but not! */
    VO_FaceParts                    m_FaceParts;

    /** point warp information of all pixels */
    vector<VO_WarpingPoint>         m_vPointWarpInfo;

    /** Initialization */
    void                            init();

public:
    enum {USEGLOBALSHAPENORMALIZATION = 1, USESIMILARITYTRANSFORM = 2};

    /** constructor */
    VO_Fitting2DSM();

    /** destructor */
    virtual ~VO_Fitting2DSM();

    /** Calculate texture difference between model texture built from C and the texture built from image */
    float                           VO_CalcErrorImage(  const Mat& iImg,
                                                        const VO_Shape& iShape,
                                                        const VO_Texture& iTexture,
                                                        VO_Texture& textureDiff);

    /** Fitting the object until convergence for the input image */
    float                           VO_StartFitting(const Mat& iImage,
                                                    vector<Mat>& oImages,
                                                    int fittingMethod,
                                                    const Point2f& ptLeftEyeCenter,
                                                    const Point2f& ptRightEyeCenter,
                                                    const Point2f& ptMouthCenter,
                                                    unsigned int epoch = EPOCH,
                                                    unsigned int pyramidlevel = 4,
                                                    bool record = true);

    /** Draw mesh on the input image and save to the output image */
    static void                     VO_DrawMesh(const VO_Shape& iShape,
                                                const VO_AXM* iAXMModel,
                                                Mat& oImg);

    /** Draw a line on the fitted image and save to the output image */
    static void                     VO_DrawAline(   const VO_Shape& iShape,
                                                    const VO_Shape& theSubshape,
                                                    const vector<float>& iLine,
                                                    Mat& oImg,
                                                    unsigned int dir = HORIZONTAL,
                                                    bool ws = false,
                                                    unsigned int offset = 0,
                                                    unsigned int ci = 0);

    /** Draw a point on the fitted image and save to the output image */
    static void                     VO_DrawAPoint(const Point2f& pt, Mat& ioImg);

    /** Gets and Sets */
    Mat                             GetFittedImage() const {return this->m_ImageOutput; }
    void                            SetInputImage(const Mat& iImg)
    {
                                    iImg.copyTo(this->m_ImageInput);
    }

    void                            SetProcessingImage(const Mat& iImg, const VO_AXM* aammodel)
    {
                                    unsigned int NbOfChannels = aammodel->GetNbOfChannels();

                                    if (iImg.channels() == NbOfChannels)
                                    {
                                        iImg.copyTo(this->m_ImageProcessing );
                                    }
                                    else if(iImg.channels() == GRAYCHANNELS && NbOfChannels == COLORCHANNELS )
                                    {
                                        cv::cvtColor(iImg, this->m_ImageProcessing, CV_GRAY2BGR);
                                    }
                                    else if(iImg.channels() == COLORCHANNELS && NbOfChannels == GRAYCHANNELS )
                                    {
                                        cv::cvtColor(iImg, this->m_ImageProcessing, CV_BGR2GRAY);
                                    }
                                    else
                                    {
                                        cerr << " Can't handle channel issue in Built Statistcal Model ! " << endl;
                                        exit(EXIT_FAILURE);
                                    }
    }

    float                           GetFittingTime() const { return this->m_fFittingTime; }
    unsigned int                    GetNbOfIterations() const { return this->m_iIteration; }
    VO_Shape                        VO_GetFittedShape() const { return this->m_VOFittingShape; }
    VO_Texture                      VO_GetFittedTexture()
    {
                                    VO_TextureModel::VO_LoadOneTextureFromShape(this->m_VOFittingShape,
                                                                                this->m_ImageInput,
                                                                                this->m_vTriangle2D,
                                                                                this->m_vPointWarpInfo,
                                                                                this->m_VOFittingTexture,
                                                                                VO_Features::DIRECT);
                                    return this->m_VOFittingTexture;
    }

    /** N Points mapping from aligned shape to real size shape */
    static Mat_<float>              VO_FirstEstimationBySingleWarp(const VO_FaceParts& iFaceParts,
                                                                    const VO_Shape& iShape,
                                                                    const Point2f& ptLeftEyeCenter,
                                                                    const Point2f& ptRightEyeCenter,
                                                                    const Point2f& ptMouthCenter );

    /** */
    static VO_Shape                 VO_FirstEstimationByScaling(const VO_Shape& iShape,
                                                                const Rect& rect);

};

#endif  // __VO_2DSMFitting__

