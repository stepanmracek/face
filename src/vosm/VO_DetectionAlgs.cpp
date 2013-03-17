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


#include <iostream>
#include <cstdio>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_DetectionAlgs.h"



/************************************************************************/
/*@author   JIA Pei                                                     */
/*@version  2009-10-04                                                  */
/*@brief    Object Detection                                            */
/*@param    iImg    Input - image to be searched in                     */
/*@param    confinedArea    Input - only detect the object in this      */
/*                          confined area                               */
/*@param    scale   Input - scalar for img scaling                      */
/*@param    sSize   Input - detected obj must be bigger than sSize      */
/*@param    bSize   Input - detected object must be smaller than bSize  */
/*@return   detection time cost                                         */
/************************************************************************/
double CDetectionAlgs::Detection(   const Mat& iImg,
                                    const Rect* confinedArea,
                                    const double scale,
                                    Size sSize,
                                    Size bSize)
{
    double res = (double)cvGetTickCount();

    switch(this->m_iDetectionMethod)
    {
    case VO_AdditiveStrongerClassifier::BAGGING:
        CDetectionAlgs::BaggingDetection(   this->m_vDetectedObjectRects,
                                            this->m_rtreeClassifier,
                                            iImg,
                                            confinedArea,
                                            scale,
                                            sSize,
                                            bSize);
        break;
    case VO_AdditiveStrongerClassifier::BOOSTING:
    default:
        CDetectionAlgs::BoostingDetection(  this->m_vDetectedObjectRects,
                                            this->m_cascadeClassifier,
                                            iImg,
                                            confinedArea,
                                            scale,
                                            sSize,
                                            bSize);
        break;
    }

    if(this->m_vDetectedObjectRects.size() >= 1)
        this->m_bObjectDetected = true;
    else
        this->m_bObjectDetected = false;

    res = ((double)cvGetTickCount() - res)
        / ((double)cvGetTickFrequency()*1000.);
    return res;
}


/************************************************************************/
/*@author   JIA Pei                                                     */
/*@version  2009-10-04                                                  */
/*@brief    Boosting based Object Detection                             */
/*@param    objs    Output - detected objects                           */
/*@param    cascade Input - cascade classifier to be used               */
/*@param    iImg    Input - image to be searched in                     */
/*@param    confinedArea    Input - only detect the object in this      */
/*                          confined area                               */
/*@param    scale   Input - scalar for img scaling                      */
/*@param    sSize   Input - detected obj must be bigger than sSize      */
/*@param    bSize   Input - detected object must be smaller than bSize  */
/*@return   detection time cost                                         */
/************************************************************************/
double CDetectionAlgs::BoostingDetection( vector<Rect>& objs,
                                          const CascadeClassifier& cascade,
                                          const Mat& img,
                                          const Rect* confinedArea,
                                          const double scale,
                                          Size sSize,
                                          Size bSize)
{
    double res = (double)cvGetTickCount();
    objs.clear();

    Mat confinedImg;
    if(confinedArea)
        confinedImg = img(*confinedArea);
    else
        confinedImg = img;

    Mat gray, smallImg( cvRound (confinedImg.rows/scale),
                        cvRound(confinedImg.cols/scale),
                        CV_8UC1 );

    if(confinedImg.channels() == 3)
        cvtColor( confinedImg, gray, CV_BGR2GRAY );
    else
        gray = confinedImg;
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    /////////////////detection/////////////////////////////////////////
    //t = (double)cvGetTickCount();
    const_cast<CascadeClassifier&>(cascade).detectMultiScale(
        smallImg,
        objs,
        1.1,
        2,
        0,
        //|CascadeClassifier::DO_CANNY_PRUNING
        //|CascadeClassifier::FIND_BIGGEST_OBJECT
        //|CascadeClassifier::DO_ROUGH_SEARCH
        //|CascadeClassifier::SCALE_IMAGE,
        sSize,
        bSize);

    ///////////////////////sort///////////////////////////////////////
    if (objs.size() > 0)
    {
        qsort( (void *)&(objs[0]), objs.size(), sizeof(Size), cvSizeCompare );
        // re-position
        if (confinedArea)
        {
            for (int i = 0; i < objs.size(); ++i)
            {
                objs[i].x = objs[i].x*scale+confinedArea->x;
                objs[i].y = objs[i].y*scale+confinedArea->y;
            }
        }
        else
        {
            for (int i = 0; i < objs.size(); ++i)
            {
                objs[i].x = objs[i].x*scale;
                objs[i].y = objs[i].y*scale;
            }
        }

        //scale back
        for ( int i = 0; i < objs.size(); ++i)
        {
            objs[i].width *= scale;
            objs[i].height *= scale;
        }
    }

    res = ((double)cvGetTickCount() - res)
        / ((double)cvGetTickFrequency()*1000.);
    return res;
}


/************************************************************************/
/*@author   JIA Pei                                                     */
/*@version  2009-10-04                                                  */
/*@brief    Bagging based Object Detection                              */
/*@param    objs    Output - detected objects                           */
/*@param    rtree   Input - random tree classifier to be used           */
/*@param    iImg    Input - image to be searched in                     */
/*@param    confinedArea    Input - only detect the object in this      */
/*                          confined area                               */
/*@param    scale   Input - scalar for img scaling                      */
/*@param    sSize   Input - detected obj must be bigger than sSize      */
/*@param    bSize   Input - detected object must be smaller than bSize  */
/*@return   detection time cost                                         */
/************************************************************************/
double CDetectionAlgs::BaggingDetection(vector<Rect>& objs,
                                        const RTreeClassifier& rtree,
                                        const Mat& img,
                                        const Rect* confinedArea,
                                        const double scale,
                                        Size sSize,
                                        Size bSize)
{
    double res = (double)cvGetTickCount();

    // To be finished....

    res = ((double)cvGetTickCount() - res)
        / ((double)cvGetTickFrequency()*1000.);
    return res;
}


void CDetectionAlgs::VO_DrawDetection(Mat& ioImg, Scalar color)
{
    unsigned int NbOfDetectedObjs = this->m_vDetectedObjectRects.size();

    Rect curRect;
    Point lefttop, rightbottom;

    if ( NbOfDetectedObjs > 0 )
    {
        for(unsigned int i = 0; i < NbOfDetectedObjs; ++i)
        {
            curRect = this->m_vDetectedObjectRects[i];
            lefttop.x = cvRound(curRect.x);
            lefttop.y = cvRound(curRect.y);
            rightbottom.x = cvRound((curRect.x+curRect.width));
            rightbottom.y = cvRound((curRect.y+curRect.height));
            cv::rectangle(ioImg, lefttop, rightbottom, color, 2, 8, 0);
        }
    }
}

