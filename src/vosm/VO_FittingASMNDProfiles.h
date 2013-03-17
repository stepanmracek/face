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


#ifndef __VO_FittingASMNDProfiles__
#define __VO_FittingASMNDProfiles__

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "VO_CVCommon.h"
#include "VO_ASMNDProfiles.h"
#include "VO_Fitting2DSM.h"

using namespace std;
using namespace cv;


/** 
 * @author      JIA Pei
 * @brief       1D and 2D Profile ASM fitting algorithms.
 */
class VO_FittingASMNDProfiles : public VO_Fitting2DSM
{
private:
    /** scale between original input image and search image */
    float                   m_fScale2;

    /** Initialization */
    void                    init();

public:
    VO_ASMNDProfiles*       m_VOASMNDProfile;

    /** constructor */
    VO_FittingASMNDProfiles();

    /** destructor */
    ~VO_FittingASMNDProfiles();

    static int              VO_FindBestMatchingProfile1D(   const Mat& iImg,
                                                            const Point2f& ThisPoint,
                                                            const Mat_<float>& iMean,
                                                            const Mat_<float>& iCovInverse,
                                                            unsigned int ProfileLength,
                                                            unsigned int offSetTolerance,
                                                            float& DeltaX,
                                                            float& DeltaY);

    static int              UpdateShape(    const VO_ASMNDProfiles* asmmodel,
                                            const Mat& iImg,
                                            VO_Shape& ioShape,
                                            const vector<VO_Shape2DInfo>& iShapeInfo,
                                            const vector< VO_Profile >& iMean,
                                            const vector< vector< Mat_<float> > >& iCovInverse,
                                            unsigned int offSetTolerance,
                                            unsigned int profdim = 2);

    void                    PyramidFit( VO_Shape& ioShape,
                                        const Mat& iImg,
                                        vector<Mat>& oImages,
                                        unsigned int iLev,
                                        float PClose = 0.90f,
                                        unsigned int epoch = VO_Fitting2DSM::EPOCH,
                                        unsigned int profdim = 2,
                                        bool record = false);

    void                    PyramidFit( VO_Shape& ioShape,
                                        const Mat& iImg,
                                        unsigned int iLev,
                                        float PClose = 0.90f,
                                        unsigned int epoch = VO_Fitting2DSM::EPOCH,
                                        unsigned int profdim = 2);

    /** Load ASM fitting training results */
    void                    VO_LoadParameters4Fitting(const string& fd);

    /** Start ASM ND Profile fitting, for static images, recording all iterations of every single image */
    float                   VO_ASMNDProfileFitting( const Mat& iImg,
                                                    vector<Mat>& oImages,
                                                    unsigned int epoch = VO_Fitting2DSM::EPOCH,
                                                    unsigned int pyramidlevel = 4,
                                                    unsigned int profdim = 2,
                                                    bool record = false);

    /** Start ASM ND Profile fitting, for dynamic image sequence */
    float                   VO_ASMNDProfileFitting( const Mat& iImg,
                                                    VO_Shape& ioShape,
                                                    Mat& oImg,
                                                    unsigned int epoch = 4,
                                                    unsigned int pyramidlevel = 4,
                                                    unsigned int profdim = 2);
};

#endif  // __VO_FittingASMNDProfiles__

