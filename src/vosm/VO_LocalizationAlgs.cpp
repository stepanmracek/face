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


#include "VO_LocalizationAlgs.h"

using namespace cv;


/** 
* @author   JIA Pei
* @version  2009-10-04
* @brief    Object Localization
* @param    img     Input - image to be searched in
* @param    sSize   Input - localized obj must be bigger than sSize
* @param    bSize   Input - localized object must be smaller than bSize
* @return   localization time cost
*/
double CLocalizationAlgs::Localization( const Mat& img,
                                        Size sSize,
                                        Size bSize)
{
    return ( CLocalizationAlgs::Localization(
        img,
        this->m_detectionAlgs,
        this->m_trackingAlgs,
        this->m_bObjectLocalized,
        this->m_CVLocalizedObjectRect,
        sSize,
        bSize) );
}


/** 
* @author   JIA Pei
* @version  2009-10-04
* @brief    Object Detection
* @param    img             Input - image to be searched within
* @param    detectAlg       Input - detection algorithm
* @param    trackAlg        Input - tracking algorithm
* @param    isLocalized     Output - is the object localized?
* @param    objPos          Output - objects' positions
* @param    sSize   Input - detected object should be bigger than sSize
* @param    bSize   Input - detected object should be smaller than bSize
* @return   localization time cost
*/
double CLocalizationAlgs::Localization( const Mat& img,
                                        CDetectionAlgs& detectAlg,
                                        CTrackingAlgs& trackAlg,
                                        bool& isLocalized,
                                        Rect& objPos,
                                        const Size& sSize,
                                        const Size& bSize)
{
    double res = (double)cvGetTickCount();
    double scale = 1.0;

    // only detection is used
    if ( trackAlg.m_iTrackingMethod == CTrackingAlgs::NONE )
    {
        detectAlg.Detection(img,
                            NULL,
                            scale,
                            sSize,
                            bSize);
        if( detectAlg.IsObjectDetected() )
        {
            isLocalized = true;
            objPos = detectAlg.GetDetectedObjectRects()[0];
        }
    }
    // tracking is used
    else
    {
        // not localized yet
        if( !isLocalized ||
            (objPos.x < 0 &&
            objPos.y < 0 &&
            objPos.width < 0 &&
            objPos.height < 0) )
        {
            detectAlg.Detection(img,
                                NULL,
                                scale,
                                sSize,
                                bSize);
            if( detectAlg.IsObjectDetected() )
            {
                isLocalized = true;
                objPos = detectAlg.GetDetectedObjectRects()[0];
                trackAlg.UpdateTracker(img, objPos);
            }
        }
        // keep tracking now
        else
        {
            trackAlg.Tracking(  objPos,
                                img,
                                sSize,
                                bSize);
            isLocalized = trackAlg.IsObjectTracked();
        }
    }

    res = ((double)cvGetTickCount() - res)
        / ((double)cvGetTickFrequency()*1000.);
    return res;
}


void CLocalizationAlgs::VO_DrawLocalization(Mat& ioImg, Scalar color)
{
    Rect curRect;
    Point lefttop, rightbottom;

    if ( this->m_bObjectLocalized )
    {
        curRect = this->m_CVLocalizedObjectRect;
        lefttop.x = cvRound(curRect.x);
        lefttop.y = cvRound(curRect.y);
        rightbottom.x = cvRound((curRect.x+curRect.width));
        rightbottom.y = cvRound((curRect.y+curRect.height));
        cv::rectangle(ioImg, lefttop, rightbottom, color, 2, 8, 0);
    }
}
