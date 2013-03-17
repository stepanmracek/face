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

#ifndef __VO_TEXTURE_H__
#define __VO_TEXTURE_H__


#include <vector>
#include <string>
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;
using namespace cv;


/** 
* @author       JIA Pei
* @brief        Generalized class for texture.
*/
class VO_Texture
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
friend ostream& operator<<(ostream& os, const VO_Texture& texture);
friend istream& operator>>(istream& is, VO_Texture& texture);
private:
    /** Every VO_Texture is corresponding to an image file */
    string                  m_sImageFileName;

    /** Generalized texture representation, the texture model. NbOfTextureRepresentation * NbOfPixels */
    Mat_<float>             m_MatTexture;

    void                    CopyData( const VO_Texture& iTexture )
    {
                            iTexture.m_MatTexture.copyTo(this->m_MatTexture);
    }

public:
    /** Default constructor to create a VO_Texture object */
    VO_Texture(unsigned int chan = 0, unsigned int pxs = 0) {this->m_MatTexture.zeros(chan, pxs);}

    /** Copy constructor */
    VO_Texture( const VO_Texture& iTexture )    {this->CopyData(iTexture); }

    /** Constructor to create a VO_Texture object with a vector of float values */
    VO_Texture( const Mat_<float>& iTexture )    {iTexture.copyTo(this->m_MatTexture); }

    /** Destructor */
    virtual ~VO_Texture()   {this->m_MatTexture.release(); }

    /** Clone */
    void                    clone(const VO_Texture& iTexture) {this->CopyData(iTexture);}

    // operators
    VO_Texture&             operator=(const VO_Texture& iTexture);
    VO_Texture&             operator=(const Mat_<float>& iTexture);
    VO_Texture&             operator=(float value);
    VO_Texture              operator+(float value);
    VO_Texture&             operator+=(float value);
    VO_Texture              operator+(const VO_Texture& iTexture);
    VO_Texture&             operator+=(const VO_Texture& iTexture);
    VO_Texture              operator-(float value);
    VO_Texture&             operator-=(float value);
    VO_Texture              operator-(const VO_Texture& iTexture);
    VO_Texture&             operator-=(const VO_Texture& iTexture);
    VO_Texture              operator*(float value);
    VO_Texture&             operator*=(float value);
    VO_Texture              operator*(const VO_Texture& iTexture);
    VO_Texture&             operator*=(const VO_Texture& iTexture);
    VO_Texture              operator/(float value);
    VO_Texture&             operator/=(float value);
    VO_Texture              operator/(const VO_Texture& iShape);
    VO_Texture&             operator/=(const VO_Texture& iShape);
    Mat_<float>             operator[](unsigned int idx) { return this->m_MatTexture.col(idx); }
    float&                  operator() (unsigned row, unsigned col);
    float                   operator() (unsigned row, unsigned col) const;
    float                   dot(const VO_Texture& iTexture);

    void                    Resize(unsigned int rows, unsigned int cols);
    void                    Centralize();
    void                    Scale( float s);
    void                    Shift( float s);
    void                    Normalize();
    float                   GetStandardizedTextureNorm() const;
    float                   GetTextureNorm() const;

    /** Clamp the texture, constrain between infimum and supremum */
    void                    Clamp(float inf, float sup);

    /** Get texture size */
    unsigned int            GetNbOfTextureRepresentation() const { return this->m_MatTexture.rows; }

    /** Get Number of pixels */
    unsigned int            GetNbOfPixels() const { return this->m_MatTexture.cols; }

    /** Get the texture vector */
    Mat_<float>             GetTheTexture() const { return this->m_MatTexture;}

    /** Get a texture */
    float                   GetATexture(unsigned int idx) const 
    {
        unsigned int row = idx / this->m_MatTexture.cols;
        unsigned int col = idx % this->m_MatTexture.cols;
        return this->m_MatTexture(row, col);
    }
    float                   GetATexture(unsigned int row, unsigned int col) const 
    {
        return this->m_MatTexture(row, col);
    }

    /** Get the texture Mat_ in a row, b1b2b3...g1g2g3...r1r2r3... */
    Mat_<float>             GetTheTextureInARow() const 
    {
        return this->m_MatTexture.reshape(0, 1);
    }

    /** Get the texture value at texture vector position i */
    Mat_<float>             GetACol(unsigned int idx) const    { return this->m_MatTexture.col(idx); }

    /** Get the texture value at texture vector position i */
    Mat_<float>             GetAPixel(unsigned int idx) const    { return this->m_MatTexture.col(idx); }

    /** Set the texture. Not a hardcopy, but copy the header and data position */
    void                    SetTheTexture(const Mat_<float>& iTexture) { this->m_MatTexture = iTexture;}

    /** Set The texture in ND. iTexture is of size 1*cols. Not a hardcopy, but copy the header and data position */
    void                    SetTheTexture(const Mat_<float>& iTexture, unsigned int textureRep)
    {
        assert (iTexture.rows == 1 && iTexture.cols%textureRep == 0);
        this->m_MatTexture = iTexture.reshape(0, textureRep);
    }
    
    void                    SetAPixel(const Mat_<float>& iCol, int idx)
    {
        assert (iCol.rows == this->m_MatTexture.rows && iCol.cols == 1);
        Mat tmpCol = this->m_MatTexture.col(idx);
        iCol.copyTo(tmpCol);
    }

};

#endif  // __VO_TEXTURE_H__

