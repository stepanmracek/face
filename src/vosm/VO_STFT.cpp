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

#include "VO_STFT.h"


/**
 * @author      JIA Pei
 * @version     2010-03-05
 * @param       kernel        -- Input, not used at all.
 * @brief       Prepare a STFT Kernel
 * @return      void
 */
void VO_STFT::VO_PrepareKernel(unsigned int kernel)
{

}


/**
 * @brief       Window Function is just an element-wise product, a type of weighted staff. 
 *              Absolutely not a convolution !!!
 * @param       iImg    Input   -- the original signal
 * @param       oImg    Output  -- the transformed signal
 */
void VO_STFT::VO_ForwardTransform(  const Mat_<float>& iImg,
                                    Mat_<float>& oImg) const
{
    if( (iImg.cols != this->m_VOWindowFunc->m_MatWindowedKernel.cols) ||
        (iImg.rows != this->m_VOWindowFunc->m_MatWindowedKernel.rows) )
    {
        cerr << "STFT: signal should have the same size as Window kernel" << endl;
        exit(1);
    }
    // Explained by JIA Pei. for STFT, it's element-wise weighting, rather than convolution!!
    oImg = iImg.mul(this->m_VOWindowFunc->m_MatWindowedKernel);
    cv::dft(oImg, oImg);
}


/**
* @brief   Window Function is just an element-wise product, a type of weighted staff. 
*          Absolutely not a convolution !!!
* @param   iImg    Input    -- the transformed signal (image)
* @param   oImg    Output    -- the original signal
 */
void VO_STFT::VO_BackwardTransform(const Mat_<float>& iImg, Mat_<float>& oImg) const
{
    cv::idft(iImg, oImg, DFT_SCALE);
    // Explained by JIA Pei. for STFT, it's element-wise weighting, rather than convolution!!
    oImg /= this->m_VOWindowFunc->m_MatWindowedKernel;
}


/**
* @brief   Window Function is just an element-wise product, a type of weighted staff. 
*          Absolutely not a convolution !!!
* @param   iImg    Input    -- the original signal (image)
* @param   pt      Input    -- the concern point, at the center of the rectangle
* @param   oImg    Output    -- the transformed signal
 */
void VO_STFT::VO_ForwardTransform(const Mat_<float>& iImg, Point pt, Mat_<float>& oImg) const
{
    Mat_<float> tmpImg = Mat_<float>::zeros( this->m_VOWindowFunc->m_MatWindowedKernel.size() );
    unsigned int y, x, yy, xx;

    if (pt.y-tmpImg.rows/2 >= 0 )
    {
        y     = pt.y-tmpImg.rows/2;
        yy    = 0;
    }
    else
    {
        y    = 0;
        yy    = abs(pt.y-tmpImg.rows/2);
    }

    for( ;
        y <= (pt.y+tmpImg.rows/2 < oImg.rows ? pt.y+tmpImg.rows/2 : oImg.rows-1);
        y++, yy++ )
    {
        if (pt.x-tmpImg.cols/2 >= 0 )
        {
            x     = pt.x-tmpImg.cols/2;
            xx    = 0;
        }
        else
        {
            x    = 0;
            xx    = abs(pt.x-tmpImg.cols/2);
        }
        for( ;
            x <= (pt.x+tmpImg.cols/2 < oImg.cols ? pt.x+tmpImg.cols/2 : oImg.cols-1);
            x++, xx++ )
        {
            tmpImg(yy, xx) = iImg(y, x);
        }
    }

    this->VO_ForwardTransform(tmpImg, oImg);
}

