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


#include "VO_HumanDetectionAlgs.h"


/************************************************************************/
/*@author   JIA Pei                                                     */
/*@version  2011-02-07                                                  */
/*@brief    Human Detection -- refer to CDetectionAlgs::Detection       */
/*@param    iImg    Input - image to be searched in                     */
/*@param    confinedArea    Input - only detect human in this           */
/*                          confined area                               */
/*@param    scale   Input - scalar for img scaling                      */
/*@param    sSize   Input - detected human must be bigger than sSize    */
/*@param    bSize   Input - detected human must be smaller than bSize   */
/*@return   detection time cost                                         */
/************************************************************************/
double CHumanDetectionAlgs::HumanDetection( const Mat& iImg,
                                            const Rect* confinedArea,
                                            const double scale,
                                            Size sSize,
                                            Size bSize)
{
    double res = (double)getTickCount();

    vector<Rect> found;
    this->m_hog.detectMultiScale(iImg, found, 0, sSize, bSize, 1.05, 2);

    size_t i, j;
    for( i = 0; i < found.size(); i++ )
    {
        Rect r = found[i];
        for( j = 0; j < found.size(); j++ )
            if( j != i && (r & found[j]) == r)
                break;
        if( j == found.size() )
            this->m_vDetectedHumanRects.push_back(r);
    }

    if ( this->m_vDetectedHumanRects.size() <= 0 )
        this->m_bHumanDetected = false;
    else
    {
        this->m_bHumanDetected = true;

        for( i = 0; i < this->m_vDetectedHumanRects.size(); i++ )
        {
            Rect r = this->m_vDetectedHumanRects[i];
            // the HOG detector returns slightly larger rectangles than the real objects.
            // so we slightly shrink the rectangles to get a nicer output.
            r.x += cvRound(r.width*0.1);
            r.width = cvRound(r.width*0.8);
            r.y += cvRound(r.height*0.07);
            r.height = cvRound(r.height*0.8);
        }
    }

    res = ((double)cvGetTickCount() - res) / ((double)cvGetTickFrequency()*1000.);
    return res;
}


/************************************************************************/
/*@author   JIA Pei                                                     */
/*@version  2011-02-07                                                  */
/*@brief    Draw human detection results                                */
/*@param    ioImg   Input & Output - the input image, in which          */
/*@param            the face detection will be carried out              */
/*@param    color   Input - line color                                  */
/*@return   void                                                        */
/************************************************************************/
void CHumanDetectionAlgs::VO_DrawDetection (Mat& ioImg, Scalar color)
{
    Point pt1, pt2;
    unsigned int NbOfDetectedHumans = this->m_vDetectedHumanRects.size();

    for(unsigned int i = 0; i < NbOfDetectedHumans; i++)
    {
        pt1.x = this->m_vDetectedHumanRects[i].x;
        pt1.y = this->m_vDetectedHumanRects[i].y;
        pt2.x = this->m_vDetectedHumanRects[i].x
                + this->m_vDetectedHumanRects[i].width;
        pt2.y = this->m_vDetectedHumanRects[i].y
                + this->m_vDetectedHumanRects[i].height;
        cv::rectangle( ioImg, pt1, pt2, color, 2, 8, 0 );
    }
}
