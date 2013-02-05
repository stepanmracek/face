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
#include "cvgabor.h"

/*!
    \fn CvGabor::CvGabor(int iMu, int iNu, double dSigma)
Construct a gabor

Parameters:
        iMu		The orientation iMu*PI/8,
        iNu 		The scale,
    dSigma 		The sigma value of Gabor,

Returns:
    None

Create a gabor with a orientation iMu*PI/8, a scale iNu, and a sigma value dSigma. The spatial frequence (F) is set to sqrt(2) defaultly. It calls Init() to generate parameters and kernels.
 */
CvGabor::CvGabor(int iMu, int iNu, double dSigma)
{
    F = sqrt(2.0);
    PhaseShift = 0;
    Init(iMu, iNu, dSigma, F);
}


/*!
    \fn CvGabor::CvGabor(int iMu, int iNu, double dSigma, double dF)
Construct a gabor

Parameters:
        iMu		The orientation iMu*PI/8
        iNu 		The scale
    dSigma 		The sigma value of Gabor
    dF		The spatial frequency

Returns:
    None

Create a gabor with a orientation iMu*PI/8, a scale iNu, a sigma value dSigma, and a spatial frequence dF. It calls Init() to generate parameters and kernels.
 */
CvGabor::CvGabor(int iMu, int iNu, double dSigma, double dF)
{
    PhaseShift = 0;
    Init(iMu, iNu, dSigma, dF);

}


/*!
    \fn CvGabor::CvGabor(double dPhi, int iNu)
Construct a gabor

Parameters:
        dPhi		The orientation in arc
        iNu 		The scale

Returns:
    None

Create a gabor with a orientation dPhi, and with a scale iNu. The sigma (Sigma) and the spatial frequence (F) are set to 2*PI and sqrt(2) defaultly. It calls Init() to generate parameters and kernels.
 */
CvGabor::CvGabor(double dPhi, int iNu)
{
    PhaseShift = 0;
    Sigma = 2*PI;
    F = sqrt(2.0);
    Init(dPhi, iNu, Sigma, F);
}


/*!
    \fn CvGabor::CvGabor(double dPhi, int iNu, double dSigma)
Construct a gabor

Parameters:
        dPhi		The orientation in arc
        iNu 		The scale
    dSigma		The sigma value of Gabor

Returns:
    None

Create a gabor with a orientation dPhi, a scale iNu, and a sigma value dSigma. The spatial frequence (F) is set to sqrt(2) defaultly. It calls Init() to generate parameters and kernels.
 */
CvGabor::CvGabor(double dPhi, int iNu, double dSigma)
{
    PhaseShift = 0;
    F = sqrt(2);
    Init(dPhi, iNu, dSigma, F);
}


/*!
    \fn CvGabor::CvGabor(double dPhi, int iNu, double dSigma, double dF)
Construct a gabor

Parameters:
        dPhi		The orientation in arc
        iNu 		The scale
    dSigma 		The sigma value of Gabor
    dF		The spatial frequency

Returns:
    None

Create a gabor with a orientation dPhi, a scale iNu, a sigma value dSigma, and a spatial frequence dF. It calls Init() to generate parameters and kernels.
 */
CvGabor::CvGabor(double dPhi, int iNu, double dSigma, double dF)
{
    PhaseShift = 0;
    Init(dPhi, iNu, dSigma,dF);
}

/*!
    \fn CvGabor::IsInit()
Determine the gabor is initilised or not

Parameters:
    None

Returns:
    a boolean value, TRUE is initilised or FALSE is non-initilised.

Determine whether the gabor has been initlized - variables F, K, Kmax, Phi, Sigma are filled.
 */
bool CvGabor::IsInit()
{

    return bInitialised;
}

/*!
    \fn CvGabor::mask_width()
Give out the width of the mask

Parameters:
    None

Returns:
    The long type show the width.

Return the width of mask (should be NxN) by the value of Sigma and iNu.
 */
long CvGabor::mask_width()
{

    long lWidth;
    if (IsInit() == false)  {
        return 0;
    }
    else {
        //determine the width of Mask
        double dModSigma = Sigma/K;
        double dWidth = round(dModSigma*6 + 1);
        //test whether dWidth is an odd.
        if (fmod(dWidth, 2.0)==0.0) dWidth++;
        lWidth = (long)dWidth;

        return lWidth;
    }
}


/*!
    \fn CvGabor::creat_kernel()
Create gabor kernel

Parameters:
    None

Returns:
    None

Create 2 gabor kernels - REAL and IMAG, with an orientation and a scale
 */
/*
void CvGabor::creat_my_kernel(){
    if (IsInit() == false) {perror("Error: The Object has not been initilised in creat_kernel()!\n");}
    else {
      CvMat *mReal, *mImag;
      mReal = cvCreateMat( Width, Width, CV_32FC1);
      mImag = cvCreateMat( Width, Width, CV_32FC1);


      int x, y;
      double dReal;
      double dImag;
      double dTemp1, dTemp2, dTemp3;

      double gama = 0.5;
      double lambda = 5.0;

      for (int i = 0; i < Width; i++){
          for (int j = 0; j < Width; j++){
              x = i-(Width-1)/2;
              y = j-(Width-1)/2;

              double xdot =  x*cos(Phi) + y*sin(Phi);
              double ydot = -x*sin(Phi) + y*cos(Phi);

              double expPart = exp(
                          -
                          (pow(xdot, 2) + pow(gama, 2)*pow(ydot, 2))
                          /
                          (2*pow(Sigma,2))
                    );
              double cosPart = (2*M_PI*xdot/lambda + PhaseShift);

              cvSetReal2D((CvMat*)mReal, i, j, expPart * cos(cosPart));
              cvSetReal2D((CvMat*)mImag, i, j, expPart * sin(cosPart));
          }
      }
      bKernel = true;
     cvCopy(mReal, Real, NULL);
     cvCopy(mImag, Imag, NULL);
    //printf("A %d x %d Gabor kernel with %f PI in arc is created.\n", Width, Width, Phi/PI);
     cvReleaseMat( &mReal );
     cvReleaseMat( &mImag );
    }
}*/
void CvGabor::creat_kernel()
{
    if (IsInit() == false) {}
    else {
        Real = Matrix::zeros(Width, Width);
        Imag = Matrix::zeros(Width, Width);

        /**************************** Gabor Function ****************************/
        int x, y;
        double dReal;
        double dImag;
        double dTemp1, dTemp2, dTemp3;

        for (int i = 0; i < Width; i++)
        {
            for (int j = 0; j < Width; j++)
            {
                x = i-(Width-1)/2;
                y = j-(Width-1)/2;
                dTemp1 = (pow(K,2)/pow(Sigma,2))*exp(-(pow((double)x,2)+pow((double)y,2))*pow(K,2)/(2*pow(Sigma,2)));
                dTemp2 = cos(K*cos(Phi)*x + K*sin(Phi)*y + K*PhaseShift) - exp(-(pow(Sigma,2)/2));
                dTemp3 = sin(K*cos(Phi)*x + K*sin(Phi)*y + K*PhaseShift);
                dReal = dTemp1*dTemp2;
                dImag = dTemp1*dTemp3;

                Real(i,j) = dReal;
                Imag(i,j) = dImag;
            }
        }
        /**************************** Gabor Function ****************************/
        bKernel = true;
    }
}

/*!
    \fn CvGabor::IsKernelCreate()
Determine the gabor kernel is created or not

Parameters:
    None

Returns:
    a boolean value, TRUE is created or FALSE is non-created.

Determine whether a gabor kernel is created.
 */
bool CvGabor::IsKernelCreate()
{

    return bKernel;
}


/*!
    \fn CvGabor::get_mask_width()
Reads the width of Mask

Parameters:
    None

Returns:
    Pointer to long type width of mask.
 */
long CvGabor::get_mask_width()
{
    return Width;
}


/*!
    \fn CvGabor::Init(int iMu, int iNu, double dSigma, double dF)
Initilize the.gabor

Parameters:
        iMu 	The orientations which is iMu*PI.8
        iNu 	The scale can be from -5 to infinit
        dSigma 	The Sigma value of gabor, Normally set to 2*PI
        dF 	The spatial frequence , normally is sqrt(2)

Returns:

Initilize the.gabor with the orientation iMu, the scale iNu, the sigma dSigma, the frequency dF, it will call the function creat_kernel(); So a gabor is created.
 */
void CvGabor::Init(int iMu, int iNu, double dSigma, double dF)
{
    //Initilise the parameters
    bInitialised = false;
    bKernel = false;

    Sigma = dSigma;
    F = dF;

    Kmax = PI/2;
    // Absolute value of K
    K = Kmax / pow(F, (double)iNu);
    Phi = PI*iMu/8;
    bInitialised = true;
    Width = mask_width();
    Real = cvCreateMat( Width, Width, CV_32FC1);
    Imag = cvCreateMat( Width, Width, CV_32FC1);
    creat_kernel();
}

/*!
    \fn CvGabor::Init(double dPhi, int iNu, double dSigma, double dF)
Initilize the.gabor

Parameters:
        dPhi 	The orientations
        iNu 	The scale can be from -5 to infinit
        dSigma 	The Sigma value of gabor, Normally set to 2*PI
        dF 	The spatial frequence , normally is sqrt(2)

Returns:
    None

Initilize the.gabor with the orientation dPhi, the scale iNu, the sigma dSigma, the frequency dF, it will call the function creat_kernel(); So a gabor is created.filename 	The name of the image file
        file_format 	The format of the file, e.g. GAN_PNG_FORMAT
        image 	The image structure to be written to the file
        octrlstr 	Format-dependent control structure

 */
void CvGabor::Init(double dPhi, int iNu, double dSigma, double dF)
{


    bInitialised = false;
    bKernel = false;
    Sigma = dSigma;
    F = dF;

    Kmax = PI/2;

    // Absolute value of K
    K = Kmax / pow(F, (double)iNu);
    Phi = dPhi;
    bInitialised = true;
    Width = mask_width();
    Real = cvCreateMat( Width, Width, CV_32FC1);
    Imag = cvCreateMat( Width, Width, CV_32FC1);
    creat_kernel();
}


/*!
    \fn CvGabor::CvGabor(int iMu, int iNu)
 */
CvGabor::CvGabor(int iMu, int iNu)
{
    double dSigma = 2*PI;
    F = sqrt(2.0);
    Init(iMu, iNu, dSigma, F);
}

