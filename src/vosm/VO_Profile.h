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

#ifndef __VO_PROFILE_H__
#define __VO_PROFILE_H__

#include <vector>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "VO_Shape.h"
#include "VO_Shape2DInfo.h"
#include "VO_TextureModel.h"


using namespace std;
using namespace cv;


/** 
* @author        JIA Pei
* @brief        Profile used to build ND profile model or for ND profile model fitting
*/
class VO_Profile
{
friend class VO_ASMNDProfiles;
friend class VO_FittingASMNDProfiles;
friend ostream& operator<<(ostream& os, const VO_Profile& profile);
friend istream& operator>>(istream& is, VO_Profile& profile);
private:
    /** the profile features, every single profile vector is a column. Multiple columns means multi-dim profiles */
    Mat_<float>             m_MatProf;

    void                    CopyData(const VO_Profile& iProfile) {iProfile.m_MatProf.copyTo(this->m_MatProf);}
public:
    // Constructors and Destructor
    VO_Profile()  { this->m_MatProf.release(); }
    VO_Profile(unsigned int length, unsigned int dim = 1) {this->m_MatProf = Mat_<float>::zeros(length, dim);}
    VO_Profile(const VO_Profile& iProf) {this->CopyData (iProf);}
    VO_Profile(const Mat_<float>& iProf) {iProf.copyTo(this->m_MatProf);}
    virtual ~VO_Profile() { this->m_MatProf.release(); }
    const float operator()(unsigned int i) const { return this->m_MatProf(i, 0); }

    // operators
    VO_Profile&             operator=(const VO_Profile& iProfile);
    VO_Profile&             operator=(const Mat_<float>& iProfile);
    VO_Profile&             operator=(float value);
    VO_Profile              operator+(float value);
    VO_Profile&             operator+=(float value);
    VO_Profile              operator+(const VO_Profile& iProfile);
    VO_Profile&             operator+=(const VO_Profile& iProfile);
    VO_Profile              operator-(float value);
    VO_Profile&             operator-=(float value);
    VO_Profile              operator-(const VO_Profile& iProfile);
    VO_Profile&             operator-=(const VO_Profile& iProfile);
    VO_Profile              operator*(float value);
    VO_Profile&             operator*=(float value);
    VO_Profile              operator*(const VO_Profile& iProfile);
    VO_Profile&             operator*=(const VO_Profile& iProfile);
    VO_Profile              operator/(float value);
    VO_Profile&             operator/=(float value);
    VO_Profile              operator/(const VO_Profile& iProfile);
    VO_Profile&             operator/=(const VO_Profile& iProfile);
    float                   operator[](unsigned int idx) { return this->m_MatProf(idx, 0); }
    float&                  operator() (unsigned row, unsigned col);
    float                   operator() (unsigned row, unsigned col) const;
    float                   dot(const VO_Profile& iProfile);

    void                    Resize(unsigned int rows, unsigned int cols);

    /** Key function to extract ND profile vector around one landmark */
    static void             VO_Get1DProfileInMat4OneLandmark (  const Mat& iImg,
                                                                const Point2f& ThisPoint,
                                                                VO_Profile& oProf, 
                                                                float normX, 
                                                                float normY,
                                                                unsigned int NbOfProfilesPerPixel = 17);
    /** Key function to extract ND profile vector around one landmark */
    static void             VO_GetNDProfiles4OneLandmark (  const Mat& iImg,
                                                            const VO_Shape& iShape,
                                                            const vector<VO_Shape2DInfo>& iShapeInfo,
                                                            unsigned int ptIdx,
                                                            VO_Profile& oProf,
                                                            unsigned int dim = 1,
                                                            unsigned int NbOfProfilesPerPixel = 17,
                                                            float* pDeltaX = NULL,
                                                            float* pDeltaY = NULL);
    /* Normalization for every dim */
    void                    Normalize();

    unsigned int            GetProfileLength() const { return this->m_MatProf.rows; }
    unsigned int            GetProfileDim() const { return this->m_MatProf.cols; }
    Mat_<float>             GetTheProfile() const { return this->m_MatProf; }
    Mat_<float>             GetSubProfile(int start, unsigned int length, unsigned int dimIdx) const;
    Mat_<float>             Get1DimProfile(unsigned int dimIdx) const {return this->m_MatProf.col(dimIdx);}
    void                    SetProfile(const VO_Profile& iProf ) { iProf.m_MatProf.copyTo(this->m_MatProf); }
    void                    SetProfile(const Mat_<float>& iProf ) { iProf.copyTo(this->m_MatProf); }
    void                    Set1DimProfile(const Mat_<float>& iOneDimProf, unsigned int dimIdx );
    void                    Set1DimProfile(const VO_Profile& iOneDimProf, unsigned int dimIdx );
};

#endif  // __VO_PROFILE_H__

