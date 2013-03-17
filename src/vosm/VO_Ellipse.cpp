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


#include "VO_Ellipse.h"


ostream& operator<<(ostream& os, const VO_Ellipse& ellipse)
{
    os  << ellipse.m_fPhimin << " "
        << ellipse.m_fPhimax << " "         
        << ellipse.m_fAxisXHalfLen << " " 
        << ellipse.m_fAxisYHalfLen << " "
        << ellipse.m_fTheta << " " 
        << ellipse.m_Center.x << " "
        << ellipse.m_Center.y;

    return os;
}


istream& operator>>(istream& is, VO_Ellipse& ellipse)
{
    is  >> ellipse.m_fPhimin
        >> ellipse.m_fPhimax        
        >> ellipse.m_fAxisXHalfLen
        >> ellipse.m_fAxisYHalfLen
        >> ellipse.m_fTheta
        >> ellipse.m_Center.x
        >> ellipse.m_Center.y;    

    return is;
}


VO_Ellipse& VO_Ellipse::operator=(const VO_Ellipse& ellipse)
{
    this->CopyData(ellipse);
    return (*this);
}


VO_Ellipse VO_Ellipse::operator*(float value)
{
    VO_Ellipse res(*this);
    res.m_fAxisXHalfLen *= value;
    res.m_fAxisYHalfLen *= value;

    return res;
}


VO_Ellipse& VO_Ellipse::operator*=(float value)
{
    this->m_fAxisXHalfLen *= value;
    this->m_fAxisYHalfLen *= value;

    return *this;
}


void VO_Ellipse::Translate( const Mat_<float>& translation )
{
    this->m_Center.x += translation(0, 0);
    this->m_Center.y += translation(1, 0);
}


void VO_Ellipse::ScaleCenter(float value)
{
    this->m_Center.x *= value;
    this->m_Center.y *= value;
}


/** 
 * @brief   Find the nearest point on the boundary to the input point 
 * @note    to be done, tough. needs optimization */
Point2f    VO_Ellipse::FindNearestPointOnBoundary(const Point2f& pt)
{
    Point2f res;

    return res;
}
    
/**
 * @brief   Find the intersect point on the boundary connecting from the COG and the input point  
 * @ref     http://my.opera.com/Vorlath/blog/show.dml/476448 
 *     x = a*b*xx/sqrt((b*xx)2 + (a*yy)2)
 *     y = a*b*yy/sqrt((b*xx)2 + (a*yy)2)
*/
Point2f VO_Ellipse::FindIntersectPointOnBoundary(const Point2f& pt)
{
    if(pt == this->m_Center)
    {
        cerr << "The input point shouldn't be the same as Ellipse COG!" << endl;
        exit(1);
    }
    Point2f res;
    float xx = pt.x*cos(this->m_fTheta) - pt.y*sin(this->m_fTheta) - this->m_Center.x;
    float yy = pt.x*sin(this->m_fTheta) + pt.y*cos(this->m_fTheta) - this->m_Center.y;
    float denominator = sqrt(    pow(this->m_fAxisYHalfLen*xx, 2.0f) +
                                pow(this->m_fAxisXHalfLen*yy, 2.0f) );
    float x = this->m_fAxisXHalfLen*this->m_fAxisYHalfLen*xx/denominator + this->m_Center.x;
    float y = this->m_fAxisXHalfLen*this->m_fAxisYHalfLen*yy/denominator + this->m_Center.y;
    res.x     = x*cos(this->m_fTheta) + y*sin(this->m_fTheta);
    res.y    = -x*sin(this->m_fTheta) + y*cos(this->m_fTheta);

    return res;
}


/** 
 * @brief   Obtain point list within the ellipse according to its range
 * @param   ellipse     - Input ellipse
 */
vector<Point> VO_Ellipse::VO_PointsInEllipse( const VO_Ellipse& ellipse)
{
    vector<Point> res;

    Rect brect = this->CalcBoundingRect();
    int startx = cvFloor(brect.x);
    int starty = cvFloor(brect.y);
    int finishx = cvCeil(brect.x+brect.width); 
    int finishy = cvCeil(brect.y+brect.height);
    for(unsigned int i = startx; i++; i <= finishx )
    {
        for(unsigned int j = starty; j++; j <= finishy)
        {
            if( this->IsPointWithinEllipse( Point2f(i, j) ) )
                res.push_back( Point(i,j) );
        }
    }
    
    return res;
}


/** 
 * @brief   Find bounding rectangle for multiple ellipses, this rectangle should cover all ellipses
 * @param   ellipses     - Input ellipses
 * @return  Rect
 */
Rect VO_Ellipse::VO_CalcBoundingRect4MultipleEllipses(const vector<VO_Ellipse>& ellipses)
{
    unsigned int NbOfEllipses = ellipses.size();
    vector<Point> allVertexes;
    
    for(unsigned int i = 0; i < NbOfEllipses; i++)
    {
        Rect rect = ellipses[i].CalcBoundingRect();
        allVertexes.push_back( Point(rect.x, rect.y) );
        allVertexes.push_back( Point(rect.x+rect.width, rect.y+rect.height) );
    }
    Mat pointsMat(allVertexes);
    return cv::boundingRect( pointsMat );
}

