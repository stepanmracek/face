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
* Create Date:      2008-04-03                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/

#ifndef __VO_SHAPE2DINFO_H__
#define __VO_SHAPE2DINFO_H__

#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "VO_FaceParts.h"

using namespace std;
using namespace cv;


/** 
* @author   JIA Pei
* @brief    Shape information, refer to "ShapeInfo.txt".
*/
class VO_Shape2DInfo
{
friend class VO_ShapeModel;
friend class VO_TextureModel;
friend ostream& operator<<(ostream& os, const VO_Shape2DInfo& shapeinfo);
friend istream& operator>>(istream& is, VO_Shape2DInfo& shapeinfo);

protected:
    /** Which path is this point in, refer to IMM face database */
    unsigned int        m_iPath;

    /** Which type does this point belong to - closed or open */
    unsigned int        m_iType;

    /** Index of this point, in the model vertex sequence */
    unsigned int        m_iIndex;

    /** In the path, which point does this point connected from? */
    unsigned int        m_iFrom;

    /** In the path, which point does this point connected to? */
    unsigned int        m_iTo;

    void                CopyData(const VO_Shape2DInfo& iShapeInfo)
    {
                        this->m_iPath   = iShapeInfo.GetPath();
                        this->m_iType   = iShapeInfo.GetType();
                        this->m_iIndex  = iShapeInfo.GetIndex();
                        this->m_iFrom   = iShapeInfo.GetFrom();
                        this->m_iTo     = iShapeInfo.GetTo();
    }

public:
    /** Default constructor to create a VO_Shape2DInfo object */
    VO_Shape2DInfo()
    {
                        this->m_iPath   = 0;
                        this->m_iType   = 0;
                        this->m_iIndex  = 0;
                        this->m_iFrom   = 0;
                        this->m_iTo     = 0;
    }

    /** Copy constructor */
    VO_Shape2DInfo( const VO_Shape2DInfo &iShapeInfo )
    {
                        this->CopyData(iShapeInfo);
    }

    /** Destructor */
    virtual ~VO_Shape2DInfo() {}

    /** operator= overloading, similar to copy constructor */
    VO_Shape2DInfo&     operator=(const VO_Shape2DInfo &iShapeInfo)
    {
                        this->CopyData(iShapeInfo);
                        return *this;
    }

    /** Get path which this point is on */
    unsigned int        GetPath() const { return this->m_iPath;}

    /** Get point type */
    unsigned int        GetType() const { return this->m_iType;}

    /** Get point index, in the AAM model vertex sequence */
    unsigned int        GetIndex() const { return this->m_iIndex;}

    /** Get the point index in the path, which this point connects from */
    unsigned int        GetFrom() const { return this->m_iFrom;}

    /** Get the point index in the path, which this point connects to */
    unsigned int        GetTo() const { return this->m_iTo;}

    /** Set path the point go through */
    void                SetPath(unsigned int iPath) { this->m_iPath = iPath;}

    /** Set point type */
    void                SetType(unsigned int iType) { this->m_iType = iType;}

    /** Set point index, in the AAM model vertex sequence  */
    void                SetIndex(unsigned int iIndex) { this->m_iIndex = iIndex;}

    /** Set the point index, which this point connected from in path */
    void                SetFrom(unsigned int iFrom) { this->m_iFrom = iFrom;}

    /** Set the point index, which this point connected to  in path */
    void                SetTo(unsigned int iTo) { this->m_iTo = iTo;}

    // read VO_Shape2DInfo
    static void         ReadShape2DInfo(const string& filename, vector<VO_Shape2DInfo>& oShapeInfo, VO_FaceParts& faceparts );
};

#endif      // __VO_SHAPE2DINFO_H__

