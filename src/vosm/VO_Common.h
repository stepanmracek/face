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
* Create Date:      2008-03-04                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/


#ifndef __VO_COMMON_H__
#define __VO_COMMON_H__


#include <climits>
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>
#include <cmath>
#include <float.h>
#include <time.h>

//#include <gsl/gsl_matrix.h>
//#include <gsl/gsl_linalg.h>

using namespace std;



#define INF                             DBL_MAX

#define ROW                             1
#define COL                             2


//////////////////////////////////////////////////////////////////////////
#define AVNONE                          0
#define AVAUDIO                         1
#define AVVIDEO                         2
#define AVAUDIOVIDEO                    3

#define SYNCEVEN                        1
#define SYNCTIMESTAMP                   2

#define INTER_NONSYNC                   1
#define INTER_SYNCAUDIO                 2
#define INTER_SYNCVIDEO                 3

#define AUDIO_MFCC                      1
#define AUDIO_RASTA                     2
#define AUDIO_PLP                       3
#define AUDIO_TECC                      4       // Teager Energy


#define SR_LINEAR                       1

#define MFCC_FRAMEOVERLAPCUT            1
#define MFCC_FRAMEOVERLAPACCUMULATE     2

#define VAD_FRAMEENERGYFREQUENCY        1
#define VAD_SPECTRALENTROPYWITHENERGY   2
#define VAD_AUTOCORRELATION             3
#define VAD_WAVELET                     4
#define VAD_SPEEX                       10

#define DTW_FULL                        1
#define DTW_GC_SakoeChibaBand           1
#define DTW_GC_ItakuraParallelogram     2

#define LB_KEOGH                        1
#define LB_YI                           2
#define LB_KIM                          3

#define CLAMP                           1
#define STRETCH                         2

// For detected face or tracking face constrain
#define FACESMALLESTSIZE                80      // for both detection and tracking
#define FACEBIGGESTSIZE                 240     // for both detection and tracking
#define FACEPARTSMALLESTSIZE            16
#define FACEPARTBIGGESTSIZE             128
#define FRAMEEDGE                       5       // if too close the image boundary, look on as lost

#define AAMADABOOSTDOWNPERCENTAGE       0.2

#define FACEHEIGHT2WIDTH                1.1

#define SHAPE                           1
#define TEXTURE                         2

// a) Single image b) image sequence c) video d) webcam
#define NOTHINGLOADED                   0
#define SINGLEIMAGE                     1
#define IMAGESEQUENCE                   2
#define CAM                             3
#define AVI                             4


#define HORIZONTAL                      1
#define VERTICAL                        2
#define ANY                             3

#define GRAYCHANNELS                    1
#define COLORCHANNELS                   3       // R G B 3 channels

#define DIRECTMAP                       1
#define LINEARIZEMAP                    2


/** video show type */
#define ORIGINAL                        0
#define DETECTED                        1
#define FIT                             2

#define MEAN0NORM1                      1
#define VARY01                          2


#define STRING_MAXSIZE                  1024

enum {
    Audio_LChannel,
    Audio_RChannel,
    Audio_BChannel
};



/** local (static) compare function for the qsort() call */
static int str_compare( const void *arg1, const void *arg2 )
{
    return strcmp ( ( * ( std::string* ) arg1 ).c_str (), 
                    ( * ( std::string* ) arg2 ).c_str () );
}


/** simple clamp function */
template <class T1, class T2>
static T2 clamp ( T1 input, T2 low, T2 up )
{
    return ( input < low ? low : ( input > up ? up : ( T2 ) input ) );
}


/** vector element-wise summation */
template <class T, class T1>
static std::vector<T> operator+ (   const std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    std::vector<T> res;
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] + vect2[i];
    return res;
}


/** every element of a vector add the same value */
template <class T, class T1>
static std::vector<T> operator+ ( const std::vector<T>& vect1, T1 value )
{
    std::vector<T> res;
    unsigned int size = vect1.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] + value;
    return res;
}


/** vector element-wise summation */
template <class T, class T1>
static std::vector<T> operator += ( std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] += vect2[i];
    return vect1;
}


/** every element of a vector add the same value */
template <class T, class T1>
static std::vector<T> operator += ( std::vector<T>& vect1, T1 value )
{
    unsigned int size = vect1.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] += value;
    return vect1;
}


/** reverse sign */
template <class T>
static std::vector<T> operator- ( const std::vector<T>& vect )
{
    std::vector<T> res;
    unsigned int size = vect.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = -vect[i];
    return res;
}


/** vector element-wise subtraction */
template <class T, class T1>
static std::vector<T> operator- (   const std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    std::vector<T> res;
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] - vect2[i];
    return res;
}


/** every element of a vector subtract the same value */
template <class T, class T1>
static std::vector<T> operator- ( const std::vector<T>& vect1, T1 value )
{
    std::vector<T> res;
    unsigned int size = vect1.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] - value;
    return res;
}


/** vector element-wise subtraction */
template <class T, class T1>
static std::vector<T> operator -= ( std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] -= vect2[i];
    return vect1;
}


/** every element of a vector subtract the same value */
template <class T, class T1>
static std::vector<T> operator -= ( std::vector<T>& vect1, T1 value )
{
    unsigned int size = vect1.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] -= value;
    return vect1;
}


/** vector element-wise multiplication */
template <class T, class T1>
static std::vector<T> operator* (   const std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    std::vector<T> res;
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] * vect2[i];
    return res;
}


/** every element of a vector multiply the same value */
template <class T, class T1>
static std::vector<T> operator* ( const std::vector<T>& vect1, T1 value )
{
    std::vector<T> res;
    unsigned int size = vect1.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] * value;
    return res;
}


/** vector element-wise multiplication */
template <class T, class T1>
static std::vector<T> operator *= ( std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] *= vect2[i];
    return vect1;
}


/** every element of a vector multiply the same value */
template <class T, class T1>
static std::vector<T> operator *= ( std::vector<T>& vect1, T1 value )
{
    unsigned int size = vect1.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] *= value;
    return vect1;
}


/** vector element-wise division */
template <class T, class T1>
static std::vector<T> operator/ (   const std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    std::vector<T> res;
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
    {
        if (fabs((float)vect2[i]) < FLT_MIN)
        {
            cerr << "divide by 0 ! " << endl;
            exit(EXIT_FAILURE);
        }
        res[i] = vect1[i] / vect2[i];
    }
    return res;
}


/** every element of a vector multiply the same value */
template <class T, class T1>
static std::vector<T> operator/ ( const std::vector<T>& vect1, T1 value )
{
    if (fabs((float)value) < FLT_MIN)
    {
        cerr << "divide by 0 ! " << endl;
        exit(EXIT_FAILURE);
    }
    std::vector<T> res;
    unsigned int size = vect1.size();
    res.resize ( size );
    for ( unsigned int i = 0; i < size; ++i )
        res[i] = vect1[i] / value;
    return res;
}


/** vector element-wise division */
template <class T, class T1>
static std::vector<T> operator /= ( std::vector<T>& vect1, 
                                    const std::vector<T1>& vect2 )
{
    unsigned int size = vect1.size() < vect2.size() ? vect1.size() : vect2.size();
    for ( unsigned int i = 0; i < size; ++i )
    {
        if (fabs((float)vect2[i]) < FLT_MIN)
        {
            cerr << "divide by 0 ! " << endl;
            exit(EXIT_FAILURE);
        }
        vect1[i] /= vect2[i];
    }
    return vect1;
}


/** every element of a vector multiply the same value */
template <class T, class T1>
static std::vector<T> operator /= ( std::vector<T>& vect1, T1 value )
{
    if (fabs((float)value) < FLT_MIN)
    {
        cerr << "divide by 0 ! " << endl;
        exit(EXIT_FAILURE);
    }
    unsigned int size = vect1.size();
    for ( unsigned int i = 0; i < size; ++i )
        vect1[i] /= value;
    return vect1;
}


///** output gsl_matrix */
//static std::ostream& operator<< (std::ostream &os, const gsl_matrix* gslm)
//{
//    for (unsigned int i = 0; i < gslm->size1; ++i)
//    {
//        for(unsigned int j = 0; j < gslm->size2; ++j)
//        {
//            os << gsl_matrix_get (gslm, i, j) << " ";
//        }
//        os << std::endl;
//    }
//    return os;
//}
//
//
///** input gsl_matrix */
//static std::istream& operator>> ( std::istream &is, gsl_matrix* gslm)
//{
//    for ( unsigned int i = 0; i < gslm->size1; ++i )
//    {
//        for ( unsigned int j = 0; j < gslm->size2; ++j )
//        {
//            is >> gslm->data[i * gslm->tda + j];
//        }
//    }
//    return is;
//}


template <class T>
static std::ostream& operator<< ( std::ostream &os, const std::vector<T>& vec )
{
    unsigned int size = vec.size();
    for ( unsigned int i = 0; i < size; i++ )
    {
        os << vec[i] << std::endl;
    }
    return os;
}


template <class T>
static std::istream& operator>> ( std::istream &is, std::vector<T>& vec )
{
    unsigned int size = vec.size();
    for ( unsigned int i = 0; i < size; i++ )
    {
        is >> vec[i];
    }
    return is;
}


template <class T>
static std::ostream& operator<< (   std::ostream &os, 
                                    const std::vector< std::vector<T> >& vec )
{
    unsigned int row = vec.size();
    if(row !=0)
    {
        unsigned int col = vec[0].size();
        for ( unsigned int i = 0; i < row; i++ )
        {
            for ( unsigned int j = 0; j < col; j++ )
            {
                os << vec[i][j] << " ";
            }
            os << std::endl;
        }
    }
    return os;
}


template <class T>
static std::istream& operator>> (   std::istream &is, 
                                    std::vector< std::vector<T> >& vec )
{
    unsigned int row = vec.size();
    if(row !=0)
    {
        unsigned int col = vec[0].size();
        for ( unsigned int i = 0; i < row; i++ )
        {
            for ( unsigned int j = 0; j < col; j++ )
            {
                is >> vec[i][j];
            }
        }
    }
    return is;
}


/** static template function to judge whether the vector contains a value */
template <class T>
static bool IsContaining ( std::vector<T> v, T t )
{
    unsigned int size = v.size();
    for ( unsigned int i = 0; i < size; i++ )
    {
        if ( v[i] == t )
            return true;
    }

    return false;
}


/** static template function to judge whether 
        the vector contains another vector */
template <class T>
static bool IsContaining ( std::vector<T> v, std::vector<T> t )
{
    unsigned int size = t.size();
    for ( unsigned int i = 0; i < size; i++ )
    {
        if ( !IsContaining ( v, t[i] ) )
            return false;
    }

    return true;
}


static int sign(double in)
{
    if (fabs(in) <= DBL_MIN ) return 0;
    else if (in > DBL_MIN ) return 1;
    else return -1;
}


static string StandardizeFN(unsigned int idx, 
                            unsigned int totalNumber = 100000, 
                            const string& prefix="", 
                            const string& suffix=".jpg")
{
    string res = prefix;
    stringstream ssidx, sstotal;
    string stridx, strtotal;
    ssidx << idx;
    ssidx >> stridx;
    sstotal << totalNumber;
    sstotal >> strtotal;

    for(unsigned int i = 0; i < strtotal.length() - stridx.length(); ++i )
    {
        res += "0";
    }

    res+=stridx;
    res+=suffix;

    ssidx.clear();
    sstotal.clear();

    return res;
}


static string getSystemTimeInString(bool year = true, 
                                    bool mon = true, 
                                    bool mday = true, 
                                    bool hour = false,
                                    bool min = false,
                                    bool sec =  false)
{
    string res = "";
    stringstream ss;
    string s_year, s_mon, s_mday, s_hour, s_min, s_sec;

    time_t rawtime;
    struct tm * timeinfo = NULL;
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    if(year)
    {
        ss << (timeinfo->tm_year+1900);
        ss >> s_year; res += s_year;
        ss.clear();
    }
    if(mon)
    {
        ss << (timeinfo->tm_mon+1); ss >> s_mon;
        if(timeinfo->tm_mon < 10)
            res += "0" + s_mon;
        else 
            res += s_mon;
        ss.clear();
    }
    if(mday)
    {
        ss << timeinfo->tm_mday; ss >> s_mday;
        if(timeinfo->tm_mday < 10)
            res += "0" + s_mday;
        else 
            res += s_mday;
        ss.clear();
    }
    if(hour)
    {
        ss << timeinfo->tm_hour;
        ss >> s_hour;
        res += s_hour;
        ss.clear();
    }
    if(min)
    {
        ss << timeinfo->tm_min;
        ss >> s_min;
        res += s_min;
        ss.clear();
    }
    if(sec)
    {
        ss << timeinfo->tm_sec;
        ss >> s_sec;
        res += s_sec;
        ss.clear();
    }

    return res;
}


static void SaveData(   const double * d,
                        unsigned int datasize,
                        const string& fn)
{
    ofstream fp;
    fp.open(fn.c_str ());
    fp << "# Coordinates" << endl;
    for(unsigned int i = 0; i < datasize; ++i)
    {
        fp << i << "   " << d[i] << endl;
    }
    fp.close();
}


static void SaveData(   const vector<double>& d, 
                        const string& fn)
{
    ofstream fp;
    fp.open(fn.c_str ());
    fp << "# Coordinates" << endl;
    for(unsigned int i = 0; i < d.size(); ++i)
    {
        fp << i << "   " << d[i] << endl;
    }
    fp.close();
}

#endif  // __VO_COMMON_H__

