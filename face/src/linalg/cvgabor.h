/***************************************************************************
 *   Copyright (C) 2006 by Mian Zhou   *
 *   M.Zhou@reading.ac.uk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef CVGABOR_H
#define CVGABOR_H

#include <iostream>


#include <opencv2/opencv.hpp>
#include "linalg/common.h"

#define PI 3.14159265
#define CV_GABOR_REAL 1
#define CV_GABOR_IMAG 2
#define CV_GABOR_MAG  3
#define CV_GABOR_PHASE 4

/**
@author Mian Zhou

openCV gabor filtration

*/
class CvGabor{
public:
    CvGabor();
    CvGabor(int iMu, int iNu);
    CvGabor(int iMu, int iNu, double dSigma);
    CvGabor(int iMu, int iNu, double dSigma, double dF);
    CvGabor(double dPhi, int iNu);
    CvGabor(double dPhi, int iNu, double dSigma);
    CvGabor(double dPhi, int iNu, double dSigma, double dF);
    bool IsInit();
    long mask_width();
    //IplImage* get_image(int Type);
    bool IsKernelCreate();
    long get_mask_width();
    void Init(int iMu, int iNu, double dSigma, double dF);
    void Init(double dPhi, int iNu, double dSigma, double dF);
    //CvMat* get_matrix(int Type);
    //void conv_img(IplImage *src, IplImage *dst, int Type);

    //void normalize( const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask );
    //void conv_img_a(IplImage *src, IplImage *dst, int Type);
    //Honza
    //void setPhaseShift(double pShift);

    Matrix Imag;
    Matrix Real;
protected:
    double Sigma;
    double F;
    double Kmax;
    double K;
    double Phi;
    double PhaseShift;
    bool bInitialised;
    bool bKernel;
    long Width;

private:
    void creat_kernel();
    //void creat_my_kernel();


};

#endif
