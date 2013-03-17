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


#ifndef __VO_SHAPE_H__
#define __VO_SHAPE_H__


#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include "opencv/cv.h"
#include "opencv/highgui.h"

//#include "VO_Triangle2DStructure.h"    // always bear in mind, this is absolutely wrong!!
class VO_Triangle2DStructure;

using namespace std;
using namespace cv;


/** 
* @author        JIA Pei
* @brief        Generalized class for shape.
*/
class VO_Shape
{
friend class VO_ShapeModel;
friend class VO_TextureModel;
friend class VO_AAMBasic;
friend class VO_AAMForwardIA;
friend class VO_AAMInverseIA;
friend class VO_AFM;
friend class VO_ASMLTCs;
friend class VO_ASMNDProfiles;
friend class VO_AXM;
friend ostream& operator<<(ostream& os, const VO_Shape& shape);
friend istream& operator>>(istream& is, VO_Shape& shape);
protected:
    /** Every VO_Shape is corresponding to an annotation file */
    string                      m_sAnnotationFileName;
     
    /** the shape model, NbOfDim * NbOfAnnotatedPoints */
    Mat_<float>                 m_MatShape;

    /** whether this point in 3D coordinates is able to be seen on 2D screen - dealing with occlusion. NbOfAnnotatedPoints */
    vector<bool>                m_MatSeeable;

    void                        CopyData(const VO_Shape& iShape)
    {
        // copy Mat_ data (i.e. the point coordinates)
        iShape.m_MatShape.copyTo(this->m_MatShape);
        if(iShape.m_MatSeeable.size() > 0)
            this->m_MatSeeable = iShape.m_MatSeeable;
    }

public:
    enum {
        LEFTMOST    = -1,
        RIGHTMOST    = -2,
        TOPMOST = -3,
        BOTTOMMOST = -4,
        CENTER = -5,
        OUTERMOST = -6,
        INNERMOST = -7};

    /** Default constructor */
    VO_Shape(unsigned int dim = 0, unsigned int pts = 0) {this->m_MatShape.zeros(dim, pts);}
    
    /** Copy constructor */
    VO_Shape(const VO_Shape& iShape) {this->CopyData(iShape);}

    /** Constructor to create a VO_Shape object with a Mat_ of float values */
    VO_Shape(const Mat_<float>& iShape) {iShape.copyTo(this->m_MatShape);}
    VO_Shape(const Mat_<float>& iShape, unsigned int dim) {this->SetTheShape(iShape, dim);}
    VO_Shape(const vector<Point2f>& iShape)
    {
        unsigned int NbOfPoints     = iShape.size();
        this->m_MatShape            = Mat_<float>::zeros(2, NbOfPoints);
        for(unsigned int j = 0; j < NbOfPoints; ++j)
        {
            this->m_MatShape(0,j)   = iShape[j].x;
            this->m_MatShape(1,j)   = iShape[j].y;
        }
    }
    VO_Shape(const vector<Point3f>& iShape)
    {
        unsigned int NbOfPoints     = iShape.size();
        this->m_MatShape            = Mat_<float>::zeros(3, NbOfPoints);
        for(unsigned int j = 0; j < NbOfPoints; ++j)
        {
            
            this->m_MatShape(0,j)   = iShape[j].x;
            this->m_MatShape(1,j)   = iShape[j].y;
            this->m_MatShape(2,j)   = iShape[j].z;
        }
    }

    /** Destructor */
    virtual ~VO_Shape()             {this->m_MatShape.release(); this->m_MatSeeable.clear();}
    
    /** Clone */
    void                            clone(const VO_Shape& iShape) {this->CopyData(iShape);}

    // operators
    VO_Shape&                       operator=(const VO_Shape& iShape);
    VO_Shape&                       operator=(const Mat_<float>& iShape);
    VO_Shape&                       operator=(float value);
    VO_Shape                        operator+(float value);
    VO_Shape&                       operator+=(float value);
    VO_Shape                        operator+(const VO_Shape& iShape);
    VO_Shape&                       operator+=(const VO_Shape& iShape);
    VO_Shape                        operator-(float value);
    VO_Shape&                       operator-=(float value);
    VO_Shape                        operator-(const VO_Shape& iShape);
    VO_Shape&                       operator-=(const VO_Shape& iShape);
    VO_Shape                        operator*(float value);
    VO_Shape&                       operator*=(float value);
    VO_Shape                        operator*(const VO_Shape& iShape);
    VO_Shape&                       operator*=(const VO_Shape& iShape);
    VO_Shape                        operator/(float value);
    VO_Shape&                       operator/=(float value);
    VO_Shape                        operator/(const VO_Shape& iShape);
    VO_Shape&                       operator/=(const VO_Shape& iShape);
    Mat_<float>                     operator[](unsigned int idx) { return this->m_MatShape.col(idx); }
    float&                          operator() (unsigned row, unsigned col);
    float                           operator() (unsigned row, unsigned col) const;
    float                           dot(const VO_Shape& iShape);

    void                            Resize(unsigned int rows, unsigned int cols);
    Mat_<float>                     CenterOfGravity() const;
    // Transformations
    void                            Centralize();
    void                            Translate( const Mat_<float>& translation );
    void                            Scale( float s);
    void                            Scale( const Mat_<float>& svec);
    void                            ScaleX( float sX);
    void                            ScaleY( float sY);
    void                            ScaleZ( float sZ);
    void                            Rotate( const vector<float>& angles);
    void                            Normalize();
    void                            Transform(const Mat_<float>& t);
    void                            Transform(float scale, vector<float> rotateAngles, Mat_<float> translation);
    float                           GetCentralizedShapeSize() const;
    float                           GetShapeNorm() const;
    /** This function is still to be evaluated !! */
    vector<float>                   GetRotation( const VO_Shape& ref ) const;

    void                            ConstrainShapeInSize(const Size& isize);
    void                            ConstrainShapeInImage(const Mat& iImg);

    // Align the shapes with respect to position, scale and orientation.
    void                            AlignTo( const VO_Shape& ref, float* scale = NULL, vector<float>* angles = NULL, Mat_<float>* translation = NULL);
    void                            Affine2D(const Mat_<float>& affineMat);
    void                            AlignTransformation( const VO_Shape& ref, float& scale, vector<float>& angles, Mat_<float>& translation ) const;
    void                            ProcrustesAnalysis( const VO_Shape& ref, float& norm, vector<float>& angles, Mat_<float>& translation );
    void                            InverseProcrustesAnalysis( const float& norm, const vector<float>& angles, const Mat_<float>& translation );
    static void                     GlobalShapeNormalization2D(const VO_Shape& iShape, VO_Shape& oShape, const Mat_<float>& q);
    void                            GlobalShapeNormalization2D(const Mat_<float>& q);
    static void                     GlobalShapeNormalization2D(const VO_Shape& iShape, VO_Shape& oShape, float scale, const vector<float>& angles, const Mat_<float>& translation);
    void                            GlobalShapeNormalization2D(float scale, const vector<float>& angles, const Mat_<float>& translation);
    static void                     SimilarityTrans2GlobalShapeNormalization(float scale, const vector<float>& angles, const Mat_<float>& translation, Mat_<float>& q);
    static void                     GlobalShapeNormalization2SimilarityTrans(const Mat_<float>& q, float& scale, vector<float>& angles, Mat_<float>& translation );

    Mat_<float>                     Min() const;
    float                           MinX() const;
    float                           MinY() const;
    float                           MinZ() const;
    Mat_<float>                     Max() const;
    float                           MaxX() const;
    float                           MaxY() const;
    float                           MaxZ() const;
    Mat_<float>                     Mean() const;
    float                           MeanX() const;
    float                           MeanY() const;
    float                           MeanZ() const;
    void                            MinMaxX(double* minX, double* maxX) const;
    void                            MinMaxY(double* minY, double* maxY) const;
    void                            MinMaxZ(double* minZ, double* maxZ) const;
    Point                           GetLeftTop() const {Point res; res.x = MinX(); res.y = MinY(); return res;}
    Point                           GetRightBottom() const {Point res; res.x = MaxX(); res.y = MaxY(); return res;}
    Point                           GetLeftBottom() const {Point res; res.x = MinX(); res.y = MaxY(); return res;}
    Point                           GetRightTop() const {Point res; res.x = MaxX(); res.y = MinY(); return res;}
//    Rect_<float>                    GetShapeRect() const {Rect_<float> res; res.x = MinX(); res.y = MinY(); res.width = GetWidth(); res.height = GetHeight(); return res;}
    Rect_<float>                    GetShapeRect() const 
    {
                                    double minX, maxX, minY, maxY;
                                    cv::minMaxLoc(this->m_MatShape.row(0), &minX, &maxX, 0, 0);
                                    cv::minMaxLoc(this->m_MatShape.row(1), &minY, &maxY, 0, 0);
                                    Rect_<float> res;
                                    res.x = (float)minX;
                                    res.y = (float)minY;
                                    res.width = (float)(maxX - minX); 
                                    res.height = (float)(maxY - minY); 
                                    return res;
    }
    Rect                            GetShapeBoundRect() const
    {                                
                                    double minX, maxX, minY, maxY;
                                    cv::minMaxLoc(this->m_MatShape.row(0), &minX, &maxX, 0, 0);
                                    cv::minMaxLoc(this->m_MatShape.row(1), &minY, &maxY, 0, 0);
                                    Rect res; 
                                    res.x = floor(minX); 
                                    res.y = floor(minY); 
                                    res.width = ceil(maxX) - res.x ; 
                                    res.height = ceil(maxY) - res.y;
                                    return res;
    }
    Mat                             ToPointList() const
    {
                                    Mat res(1, m_MatShape.cols, CV_32FC2);
                                    for(unsigned int i = 0; i < m_MatShape.cols; ++i)
                                    {
                                        res.at<Point2f>(0, i)     = Point2f(m_MatShape(0, i), m_MatShape(1, i));
                                    }
                                    return res;
    }
    
    double                          HausDorffDist(const VO_Shape& iShape);

    /** Is the current point "pt" in current shape? */
    int                             IsPointInShape(const Point2f& pt, const vector<VO_Triangle2DStructure>& triangles) const;
    
    /** Get VO_Triangle2DStructure specific for this VO_Shape */
    vector<VO_Triangle2DStructure>  GetTriangle2DStructure(const vector<VO_Triangle2DStructure> triangles) const;

    /** Get shape width */
    float                           GetWidth() const { return this->MaxX() - this->MinX(); }

    /** Get shape height */
    float                           GetHeight() const { return this->MaxY() - this->MinY(); }

    /** Get shape height */
    float                           GetDepth() const { return this->MaxZ() - this->MinZ(); }

    /** Get area (number of pixels) inside the shape */
    unsigned int                    GetArea() const;

    /** Get shape dimension */
    unsigned int                    GetNbOfDim() const { return this->m_MatShape.rows; }

    /** Get Number of points */
    unsigned int                    GetNbOfPoints() const { return this->m_MatShape.cols; }

    /** Get the shape Mat_ */
    Mat_<float>                     GetTheShape() const { return this->m_MatShape; }
    
    /** Get a shape */
    float                           GetAShape(unsigned int idx) const 
    {
        unsigned int row = idx / this->m_MatShape.cols;
        unsigned int col = idx % this->m_MatShape.cols;
        return this->m_MatShape(row, col);
    }
    float                           GetAShape(unsigned int row, unsigned int col) const 
    {
        return this->m_MatShape(row, col);
    }
    
    /** Get the shape Mat_ in a row, x1x2x3...y1y2y3...z1z2z3... */
    Mat_<float>                     GetTheShapeInARow() const 
    {
        return this->m_MatShape.reshape(0, 1);
    }

    /** Get a single col in Mat_<float> */
    Mat_<float>                     GetACol(unsigned int idx) const {return this->m_MatShape.col(idx);}
    
    /** Get a 2D point */
    Point2f                         GetA2DPoint(unsigned int idx) const;
    
    /** Get a 3D point */
    Point3f                         GetA3DPoint(unsigned int idx) const;
    
    /** Seeable point list */
    vector<bool>                    GetSeeable() const { return this->m_MatSeeable; }

    /** Get a sub shape from shape */
    VO_Shape                        GetSubShape(const vector<unsigned int>& iPtIdx) const;
    
    /** Combine two shapes */
    static VO_Shape                 Combine2Shapes(const VO_Shape& shape1, const VO_Shape& shape2);
    
    /** Set a 2D point */
    void                            SetA2DPoint(const Point2f& pt, unsigned int idx)
    {
        assert (m_MatShape.rows == 2);
        this->m_MatShape(0, idx) = pt.x;
        this->m_MatShape(1, idx) = pt.y;
    }

    /** Set a 3D point */
    void                            SetA3DPoint(const Point3f& pt, unsigned int idx)
    {
        assert (m_MatShape.rows == 3);
        this->m_MatShape(0, idx) = pt.x;
        this->m_MatShape(1, idx) = pt.y;
        this->m_MatShape(2, idx) = pt.z;
    }

    /** Set The shape */
    void                            SetTheShape(const Mat_<float>& iShape) { iShape.copyTo(this->m_MatShape); }

    /** Set The shape in ND. iShape is of size 1*cols */
    void                            SetTheShape(const Mat_<float>& iShape, unsigned int dim)
    {
                                    assert (iShape.rows == 1 && iShape.cols%dim == 0);
                                    this->m_MatShape = iShape.reshape(0, dim);
    }
    /** Set a specific shape at some position */
    void                            SetAShape(float iS, unsigned int row, unsigned int col)
    {
                                    this->m_MatShape(row, col) = iS;
    }

    /** Set seeable */
    void                            SetSeeable(const vector<bool>& iSeeable) { this->m_MatSeeable = iSeeable; }
    
    /** Set Annotation file name */
    void                            SetAnnotationFileName(const string& fn) { this->m_sAnnotationFileName = fn; }
    
    /** Export shape coordinates sequentially */
    static void                     ExportShape(const string& fn, const VO_Shape& shape);
    static void                     ExportShape(const string& fn, const Mat_<float>& shapeMat);
};

#endif  // __VO_SHAPE_H__

