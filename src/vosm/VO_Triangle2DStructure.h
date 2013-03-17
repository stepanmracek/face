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



#ifndef __VO_TRIANGLE2DSTRUCTURE_H__
#define __VO_TRIANGLE2DSTRUCTURE_H__

#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_Triangle2D.h"


using namespace std;
using namespace cv;


/** 
* @author       JIA Pei
* @brief        A triangle specifically used to build statistical shape model.
*/
class VO_Triangle2DStructure : public VO_Triangle2D
{
friend class VO_ShapeModel;
friend class VO_TextureModel;
friend class VO_WarpingPoint;
friend ostream& operator<<(ostream& os, const VO_Triangle2DStructure& aamtriangle2d);
friend istream& operator>>(istream& is, VO_Triangle2DStructure& aamtriangle2d);
protected:
    /** Indexes of 3 triangle vertexes in the built AAM model vertex sequence */
    vector<unsigned int>            m_vVertexIndexes;

    /** Indexes of this triangle in the built AAM model triangle sequence */
    unsigned int                    m_iTriangleIndex;

public:
    /** Default constructor to create a VO_Triangle2DStructure object */
    VO_Triangle2DStructure() {this->m_vVertexIndexes.resize(3);}

    /** Constructor to create a VO_Triangle2DStructure object with three coordinate vertexes in vector format */
    VO_Triangle2DStructure(const vector<Point2f>& iVertexes):VO_Triangle2D(iVertexes)
    {
        this->m_vVertexIndexes.resize(3);
    }

    /** Constructor to create a VO_Triangle2DStructure object with three coordinate vertexes in a row in Mat_<float> format */
    VO_Triangle2DStructure(const Mat_<float>& iVertexes):VO_Triangle2D(iVertexes)
    {
        this->m_vVertexIndexes.resize(3);
    }

    /** Constructor to create a VO_Triangle2DStructure object with three coordinate vertexes and 
        three corresponding vertex indexes in vector format */
    VO_Triangle2DStructure( const vector<Point2f>& iVertexes, const vector<unsigned int>& iVertexIndexes )
        :VO_Triangle2D(iVertexes)
    {
        assert (iVertexIndexes.size () == 3);
        this->m_vVertexIndexes = iVertexIndexes;
    }

    /** Constructor to create a VO_Triangle2DStructure object with three coordinate vertexes
        and three corresponding vertex indexes in Mat_<float> format */
    VO_Triangle2DStructure( const Mat_<float>& iVertexes, const vector<unsigned int>& iVertexIndexes )
        :VO_Triangle2D(iVertexes)
    {
        assert (iVertexIndexes.size () == 3);
        this->m_vVertexIndexes = iVertexIndexes;
    }

    /** Destructor */
    virtual ~VO_Triangle2DStructure()   {this->m_vVertexIndexes.clear ();}

    /** operator= overloading, similar to copy constructor */
    VO_Triangle2DStructure&         operator=(const VO_Triangle2DStructure& s)
    {
        this->m_vVertexIndexes = s.m_vVertexIndexes;
        this->m_iTriangleIndex = s.m_iTriangleIndex;
        this->m_MatShape = s.m_MatShape;
        this->SetdD( s.GetdD() );
        return (*this);
    }

    /** Judge whether this triangle has a vertex of index "iIndex" */
    bool                            HasNode(unsigned int iIndex) const
    {
        if( (this->m_vVertexIndexes[0] == iIndex) 
            || (this->m_vVertexIndexes[1] == iIndex)
            || (this->m_vVertexIndexes[2] == iIndex) )
            return true;
        else
            return false;
    }

    /** VO_Triangle2DStructure to Mat_<float> */
    static VO_Shape                 Triangle2D2Shape(const vector <VO_Triangle2DStructure>& triangles);

    /** Adjust vertex sequence, to COUNTER_CLOCKWISE or CLOCKWISE */
    void                            AdjustVertexSequence();

    /** Is point in triangles? If so, which triangle is it in? */
    static int                      IsPointInTriangles(const Point2f& pt, const vector<VO_Triangle2DStructure>& triangles);

    /** Get three indexes of three vertexes as a vector */
    vector<unsigned int>            GetVertexIndexes() const { return this->m_vVertexIndexes;}

    /** Get one index of one vertex, in the built AAM model vertex sequence. Apparently, ptIdx could only be 0,1,2 */
    unsigned int                    GetVertexIndex(unsigned int ptIdx) const { return this->m_vVertexIndexes[ptIdx];}

    /** Get the index of this triangle, in the built AAM model triangle sequence */
    unsigned int                    GetTriangleIndex() const { return this->m_iTriangleIndex;}

    /** Set the indexes of three vertexes of this triangle */
    void                            SetVertexIndexes(const vector<unsigned int>& iVertexes) { this->m_vVertexIndexes = iVertexes;}

    /** Set the index of this triangle, in the built AAM model triangle sequence */
    void                            SetTriangleIndex(unsigned int iTriangleIndex) { this->m_iTriangleIndex = iTriangleIndex;}
};

#endif  // __VO_TRIANGLE2DSTRUCTURE_H__

