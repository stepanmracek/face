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
* Create Date:      2010-03-05                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/

#include "VO_WindowFunc.h"


VO_WindowFunc::VO_WindowFunc(Size windowSize, unsigned int windowFunc)
{
    this->init();
    this->m_iWindowSize         = windowSize;
    this->m_iWindowFunc         = windowFunc;
    this->m_MatWindowedKernel   = this->VO_GenerateWindowFunc(this->m_iWindowSize, this->m_iWindowFunc);
}


void VO_WindowFunc::init()
{
    this->m_iWindowSize         = Size(0, 0);
    this->m_iWindowFunc         = 0;
    this->m_MatWindowedKernel.release();
}


VO_WindowFunc::~VO_WindowFunc()
{
    this->m_MatWindowedKernel.release();
}


/**
 * @author      JIA Pei
 * @version     2010-02-22
 * @brief       Generate all types of window functions
 * @param       windowSize      Input    -- the window size
 * @param       windowFunc      Output    -- the window function
 * @return      Mat_<float>     return 
 */
Mat_<float> VO_WindowFunc::VO_GenerateWindowFunc(Size windowSize, unsigned int windowFunc)
{
    Mat_<float> resMat = Mat_<float>::ones(windowSize);
    Mat_<float> rowVec = Mat_<float>::ones(1, windowSize.width);
    Mat_<float> colVec = Mat_<float>::ones(windowSize.height, 1);

    switch(windowFunc)
    {
        case HAMMING:
        {
            double a = 2.0 * CV_PI / (windowSize.width - 1.0);
            for(unsigned int i = 0; i < windowSize.width; ++i)
                rowVec(0, i)    = 0.53836 - 0.46164 * cos(a*i);
            double b = 2.0 * CV_PI / (windowSize.height - 1.0);
            for(unsigned int i = 0; i < windowSize.height; ++i)
                colVec(i, 0)    = 0.53836 - 0.46164 * cos(b*i);
            resMat = colVec*rowVec;
        }
        break;
        case HANN:
        {
            double a = 2.0 * CV_PI / (windowSize.width - 1.0);
            for(unsigned int i = 0; i < windowSize.width; ++i)
                rowVec(0, i)    = 0.5 * (1.0 - cos(a*i));
            double b = 2.0 * CV_PI / (windowSize.height - 1.0);
            for(unsigned int i = 0; i < windowSize.height; ++i)
                colVec(i, 0)    = 0.5 * (1.0 - cos(b*i));
            resMat = colVec*rowVec;
        }
        break;
        case TUKEY:
        break;        
        case COSINE:
        {
            double a = CV_PI / (windowSize.width - 1.0);
            for(unsigned int i = 0; i < windowSize.width; ++i)
                rowVec(0, i)    = sin(a*i);
            double b = CV_PI / (windowSize.height - 1.0);
            for(unsigned int i = 0; i < windowSize.height; ++i)
                colVec(i, 0)    = sin(b*i);
            resMat = colVec*rowVec;
        }
        break;
        case LANCZOS:
        break;
        case BARTLETT:
        break;
        case TRIANGULAR:
        break;
        case GAUSS:
        {
        }
        break;
        case BARTLETTHANN:
        break;
        case BLACKMAN:
        break;
        case KAISER:
        break;        
        case NUTTALL:
        break;
        case BLACKMANHARRIS:
        break;
        case BLACKMANNUTTALL:
        break;
        case FLATTOP:
        break;
        case BESSEL:
        break;        
        case DOLPHCHEBYSHEV:
        break;
        case EXPONENTIAL:
        break;
        case RIFEVINCENT:
        break;
        case GABOR:
        break;
        case RECTANGULAR:
        default:
        {
            for(unsigned int i = 0; i < windowSize.width; ++i)
                for(unsigned int j = 0; j < windowSize.height; ++j)
                    resMat(i, j) = 1.0f;
        }
        break;
    }
    
    return resMat;
}


/**
 * @author      JIA Pei
 * @version     2010-03-05
 * @brief       Display the Window Function Kernel using STRETCH
 * @param       fn  Input    -- file name to store an Gabor Kernel image
 * @return      void
 */
void VO_WindowFunc::VO_DisplayWindowFuncKernel(const string& fn)
{
    Mat img = Mat::ones(Size(this->m_MatWindowedKernel.rows, this->m_MatWindowedKernel.cols), CV_8UC1);
    
    double minVal = 0.0;
    double maxVal = 0.0;
    cv::minMaxLoc(this->m_MatWindowedKernel, &minVal, &maxVal);
    double stretchLen = maxVal - minVal;

    if(fabs(stretchLen) < FLT_EPSILON)
    {
        img = img*(unsigned char) stretchLen;
    }
    else
    {
        for(unsigned int i = 0; i < this->m_MatWindowedKernel.rows; i++)
        {
            for(unsigned int j = 0; j < this->m_MatWindowedKernel.cols; j++)
            {
                img.at<uchar>(i, j) = (unsigned char) ( (this->m_MatWindowedKernel(i,j) - minVal) / (maxVal - minVal) * 255.0 );
            }
        }
    }
    imwrite(fn, img);
}


