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


#ifndef __VO_HUMANDETECTIONALGS_H__
#define __VO_HUMANDETECTIONALGS_H__

#include <cstring>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_CVCommon.h"
#include "VO_FaceCompPos.h"
#include "VO_DetectionAlgs.h"

/** 
 * @author  JIA Pei
 * @brief   Human detection algorithms.
 */
class CHumanDetectionAlgs : public CDetectionAlgs
{
private:
    /** Original detected human rectangle */
    vector<Rect>    m_vDetectedHumanRects;
    
    /** Whether .... is detected */
    bool            m_bHumanDetected;
    
    /** HOG detector */
    HOGDescriptor   m_hog;
    
    /** Initialization */
    void            init(const string& str, unsigned int mtd)
    {
        CDetectionAlgs::init(str, mtd);
        this->m_bHumanDetected  = false;
        this->m_vDetectedHumanRects.clear();
        this->m_hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    }

public:
    /** Constructor */
    CHumanDetectionAlgs(const string& str="",
                        unsigned int mtd=
                        VO_AdditiveStrongerClassifier::BOOSTING)
    {
                    this->init(str, mtd);
    }

    /** Destructor */
    ~CHumanDetectionAlgs() {this->m_vDetectedHumanRects.clear();}
    
    /** only two main methods are adopted here, random forest 
    or boosting cascade. Every detected face will be recorded */
    double          HumanDetection( const Mat& img,
                                    const Rect* confinedArea = NULL,
                                    const double scale = 1.0,
                                    Size sSize = Size(8, 8),
                                    Size bSize = Size(32, 32));

    /** Draw the detected human */
    void            VO_DrawDetection(   Mat& ioImg, 
                                        Scalar color = colors[0]);

    /** Return detected face parts rectangles */
    vector<Rect>    GetDetectedHumanRects() const
    {
                    return this->m_vDetectedHumanRects;
    }
    Rect            GetDetectedHumanWindow() const
    {
                    return this->m_vDetectedHumanRects[0];
    }
    
    /** Is human body detected */
    bool            IsHumanDetected() const
    {
                    return this->m_bHumanDetected;
    }

};

#endif    // __VO_HUMANDETECTIONALGS_H__
