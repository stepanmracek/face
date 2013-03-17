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


#ifndef __VO_TRACKINGALGS_H__
#define __VO_TRACKINGALGS_H__


#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_CVCommon.h"

using namespace std;
using namespace cv;


/** 
* @author   JIA Pei
* @brief    Object tracking algorithms, unlike CDetectionAlgs,
*           CTrackingAlgs can deal with only one single concerned object.
*/
class CTrackingAlgs
{
    friend class CLocalizationAlgs;
protected:
    /** Tracked object rectangle */
    Rect            m_CVTrackedObjectRect;

    /** Tracking Method */
    unsigned int    m_iTrackingMethod;

    /** Tracker Method */
    unsigned int    m_iTrackerMethod;

    /** Whether the tracker has been initialized */
    bool            m_bTrackerInitialized;

    /** Whether the objects is tracked */
    bool            m_bObjectTracked;

    /** Initialization */
    void            init(unsigned int trackingmtd, unsigned int trackermtd)
    {
        this->SetConfiguration(trackingmtd, trackermtd);
        this->m_bTrackerInitialized = false;
        this->m_bObjectTracked      = false;
    }

public:
    enum { PROBABILITYIMAGE = 1};

    enum {
        NONE = 0,
        CAMSHIFT = 1,
        KALMANFILTER = 2,
        PARTICLEFILTER = 3,
        ASMAAM = 4
    };

    CTrackingAlgs(  unsigned int trackingmtd
                    = CTrackingAlgs::CAMSHIFT,
                    unsigned int trackermtd
                    = CTrackingAlgs::PROBABILITYIMAGE)
    {
        this->init(trackingmtd, trackermtd);
    }

    ~CTrackingAlgs() {}

    void            SetConfiguration(   unsigned int trackingmtd
                                        = CTrackingAlgs::CAMSHIFT,
                                        unsigned int trackermtd
                                        = CTrackingAlgs::PROBABILITYIMAGE)
    {
        this->m_iTrackingMethod = trackingmtd;
        this->m_iTrackerMethod  = trackermtd;
    }

    void            UpdateTracker(const Mat& img, const Rect& obj);

    double          Tracking(   Rect& obj,
                                const Mat& img,
                                Size smallSize
                                = Size(FACESMALLESTSIZE, FACESMALLESTSIZE),
                                Size bigSize
                                = Size(FACEBIGGESTSIZE, FACEBIGGESTSIZE));

    static bool     CamshiftUpdateTracker(  const Mat& img,
                                            const Rect& obj,
                                            MatND& hist);

    static double   CamshiftTracking(   Rect& obj,
                                        const Mat& img,
                                        MatND& hist,
                                        bool& isTracked,
                                        Size smallSize,
                                        Size bigSize);

    static double   KalmanTracking( Rect& obj,
                                    const Mat& img,
                                    bool& isTracked,
                                    Size smallSize,
                                    Size bigSize);

    static double   ParticleFilterTracking( Rect& obj,
                                            const Mat& img,
                                            bool& isTracked,
                                            Size smallSize,
                                            Size bigSize);

    static double   ASMAAMTracking( Rect& obj,
                                    const Mat& img,
                                    bool& isTracked,
                                    Size smallSize,
                                    Size bigSize);

    /** Draw all detected objects on the image */
    void            VO_DrawTracking(Mat& ioImg,
                                    Scalar color = colors[6]);

    /** Are objects tracked? */
    bool            IsObjectTracked() const
    {
        return this->m_bObjectTracked;
    }

    /** Return tracked objects rectangles */
    Rect            GetTrackedObjectRect() const
    {
        return this->m_CVTrackedObjectRect;
    }

    static const int    vmin = 10;
    static const int    vmax = 256;
    static const int    smin = 30;
    static const int    hbins = 32; // quantize the hue to 32 levels
    static const int    sbins = 32; // quantize the saturation to 32 levels
    static int          histSize[] ;
    static float        hranges[];
    static float        sranges[];
    static const float* ranges[];
    static int          channels[];

    MatND               m_hist;
};

#endif    // __VO_TRACKINGALGS_H__
