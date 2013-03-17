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


#ifndef __VO_LOCALIZATIONALGS_H__
#define __VO_LOCALIZATIONALGS_H__

#include <cstring>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_CVCommon.h"

#include "VO_DetectionAlgs.h"
#include "VO_TrackingAlgs.h"


using namespace std;
using namespace cv;


/** 
* @author   JIA Pei
* @brief    Object localization algorithms, currently, 
*           can deal with only a single concerned object.
*/
class CLocalizationAlgs
{
protected:
    /** Original localized face rectangle */
    Rect                m_CVLocalizedObjectRect;

    /** Whether the objects is localized */
    bool                m_bObjectLocalized;

    /** The detection algorithm */
    CDetectionAlgs      m_detectionAlgs;

    /** The tracking algorithm */
    CTrackingAlgs       m_trackingAlgs;

    /** Initialization */
    void                init(   const string& str,
                                unsigned int detectionMtd,
                                unsigned int trackingMtd)
    {
        this->m_detectionAlgs.SetConfiguration(str, detectionMtd);
        this->m_trackingAlgs.SetConfiguration(trackingMtd);
        this->m_bObjectLocalized    = false;
    }

public:
    enum {DETECTIONONLY = 0, DETECTIONTRACKING = 1};

    CLocalizationAlgs(  const string& str="",
                        unsigned int detectionMtd
                        =VO_AdditiveStrongerClassifier::BOOSTING,
                        unsigned int trackingMtd
                        =CTrackingAlgs::CAMSHIFT)
    {
        this->init(str, detectionMtd, trackingMtd);
    }
    ~CLocalizationAlgs() {}

    void                SetConfiguration(   const string& str,
                                            unsigned int detectionMtd,
                                            unsigned int trackingMtd)
    {
        this->m_detectionAlgs.SetConfiguration(str, detectionMtd);
        this->m_trackingAlgs.SetConfiguration(trackingMtd);
    }

    double              Localization(   const Mat& img,
                                Size sSize = 
                                Size(FACESMALLESTSIZE, FACESMALLESTSIZE),
                                Size bSize = 
                                Size(FACEBIGGESTSIZE, FACEBIGGESTSIZE) );

    static double       Localization(   const Mat& img,
                                        CDetectionAlgs& detectAlg,
                                        CTrackingAlgs& trackAlg,
                                        bool& isTracked,
                                        Rect& objPos,
                                        const Size& size1,
                                        const Size& size2);

    /** Draw all detected objects on the image */
    void                VO_DrawLocalization(Mat& ioImg,
                                            Scalar color = colors[6]);

    /** Is object detected? */
    bool                IsObjectLocalized() const
    {
        return this->m_bObjectLocalized;
    }

    /** Return localized object */
    Rect                GetLocalizedObjectRect () const
    {
        return this->m_CVLocalizedObjectRect;
    }
};

#endif    // __VO_LOCALIZATIONALGS_H__
