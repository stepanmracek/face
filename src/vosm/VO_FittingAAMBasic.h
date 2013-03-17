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


#ifndef __VO_FittingAAMBasic__
#define __VO_FittingAAMBasic__

#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "VO_CVCommon.h"
#include "VO_AAMBasic.h"
#include "VO_Fitting2DSM.h"

using namespace std;
using namespace cv;


/** 
 * @author      JIA Pei
 * @brief       Basic AAM fitting algorithm.
 */
class VO_FittingAAMBasic : public VO_Fitting2DSM
{
public:
    /** dampen coefficients */
    static vector<float>        k_values;

private:
    /** For non-rigid transform */
    Mat_<float>                 m_MatDeltaC;
    Mat_<float>                 m_MatEstimatedC;
    Mat_<float>                 m_MatCurrentC;

    /** For rigid transform */
    Mat_<float>                 m_MatDeltaT;
    Mat_<float>                 m_MatEstimatedT;
    Mat_<float>                 m_MatCurrentT;

    /** errors */
    float                       m_E;
    float                       m_E_previous;

    /** Initialization */
    void                        init();

    /** calculate the real-size modeled shape, from the trained appearance model */
    void                        VO_CParamTParam2FittingShape(   const Mat_<float>& c,
                                                                const Mat_<float>& t,
                                                                VO_Texture& modelNormalizedTexture,
                                                                VO_Shape& oShape,
                                                                float& scale,
                                                                vector<float>& rotateAngles,
                                                                Mat_<float>& matCOG,
                                                                unsigned int mtd = VO_Fitting2DSM::USESIMILARITYTRANSFORM);

public:
    VO_AAMBasic*                m_VOAAMBasic;

    /** Constructor */
    VO_FittingAAMBasic();

    /** Destructor */
    ~VO_FittingAAMBasic();

    /** Load Basic AAM fitting training results */
    void                        VO_LoadParameters4Fitting(const string& fd);

    /** Start Basic AAM fitting, for static images, recording all iterations of every single image */
    float                       VO_BasicAAMFitting(const Mat& iImg, vector<Mat>& oImages, unsigned int epoch = EPOCH, bool record = false);

    /** Start Basic AAM fitting, for dynamic image sequence */
    float                       VO_BasicAAMFitting(const Mat& iImg, VO_Shape& ioShape, Mat& oImg, unsigned int epoch = EPOCH);

    /** Start Direct AAM fitting, for static images, recording all iterations of every single image */
    float                       VO_DirectAAMFitting(const Mat& iImg, vector<Mat>& oImages, unsigned int epoch = EPOCH, bool record = false);

    /** Start Direct AAM fitting, for dynamic image sequence */
    float                       VO_DirectAAMFitting(const Mat& iImg, VO_Shape& ioShape, Mat& oImg,  unsigned int epoch = EPOCH);
};

#endif

