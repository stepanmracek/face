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


#include <sstream>
#include <cstring>
#include <iostream>
#include <fstream>

#include "VO_ScanFilesInDir.h"



/**
 * @author      JIA Pei
 * @version     2007-08-08
 * @brief       Scan a directory and sort all files with a specified extension
 * @param       dir_path            input parameter, path to read from
 * @param       file_extension      input parameter, file extension to search for
 * @param       files               output parameter, file names found without any path
 * @return      bool                fail to scan or not
 * @note        http://www.boost.org/libs/filesystem/doc/index.htm
*/
bool VO_IO::ScanDirectory(  const path &dir_path,
                            const string &file_extension,
                            vector<string>& files)
{
    if ( !exists( dir_path ) )
    {
        return false;
    }

    directory_iterator end_itr;

    for ( directory_iterator itr(dir_path); itr != end_itr; ++itr )
    {
        if ( is_directory(itr->status()) )
        {
            VO_IO::ScanDirectory(itr->path(), file_extension, files);
        }
        else if ( extension( itr->path() ) == file_extension )
        {
            files.push_back( itr->path().string() );
        }
    }

    return true;
}


/**
 * @author      JIA Pei
 * @version     2007-08-08
 * @brief       Scan a directory and sorts all file in format of .pts or .asf
 * @param       dir_path            input parameter, path to read from
 * @return      vector<string>      A vector of all scanned file names
*/
vector<string> VO_IO::ScanNSortAnnotationInDirectory(const string &dir_path)
{
    vector<string> annotationALL;
    vector< vector<string> > annotations;
    annotations.resize(4);
    for(unsigned int i = 0; i < annotations.size(); i++)
    {
        annotations[i].resize(0);
    }

    path tempp = path(dir_path);

    // 3 types, pts, asf, obj, wrl
    // Here, look on Lightwave .obj and VRML .wrl as annotations
    VO_IO::ScanDirectory( tempp, ".pts", annotations[0]);
    VO_IO::ScanDirectory( tempp, ".asf", annotations[1]);
    VO_IO::ScanDirectory( tempp, ".obj", annotations[2]);
    VO_IO::ScanDirectory( tempp, ".wrl", annotations[3]);

    for(unsigned int i = 0; i < annotations.size(); i++)
    {
        for(unsigned int j = 0; j < annotations[i].size(); j++)
        {
            annotationALL.push_back(annotations[i][j]);
        }
    }

    // sort the filenames
    if(annotationALL.size() > 0)
    {
        qsort( (void *)&(annotationALL[0]), 
            (size_t)annotationALL.size(),
            sizeof(string),
            str_compare );
    }

    return annotationALL;
}


/**
 * @author      JIA Pei
 * @version     2007-08-08
 * @brief       Scan a directory and sort all files in format of .jpg .bmp .ppm .pgm .png .gif .tif
 * @param       dir_path            input parameter, path to read from
 * @return      vector<string>      A vector of all scanned file names
*/
vector<string> VO_IO::ScanNSortImagesInDirectory(const string &dir_path)
{
    vector<string> imgALL;
    vector< vector<string> > imgs;
    imgs.resize(7);
    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        imgs[i].resize(0);
    }

    path tempp = path(dir_path);

    VO_IO::ScanDirectory( tempp, ".jpg", imgs[0]);
    VO_IO::ScanDirectory( tempp, ".bmp", imgs[1]);
    VO_IO::ScanDirectory( tempp, ".ppm", imgs[2]);
    VO_IO::ScanDirectory( tempp, ".pgm", imgs[3]);
    VO_IO::ScanDirectory( tempp, ".png", imgs[4]);
    VO_IO::ScanDirectory( tempp, ".gif", imgs[5]);
    VO_IO::ScanDirectory( tempp, ".tif", imgs[6]);

    for(unsigned int i = 0; i < imgs.size(); i++)
    {
        for(unsigned int j = 0; j < imgs[i].size(); j++)
        {
            imgALL.push_back(imgs[i][j]);
        }
    }

    // sort the filenames
    if(imgALL.size() > 0)
    {
        qsort( (void *)&(imgALL[0]),
            (size_t)imgALL.size(),
            sizeof(string),
            str_compare );
    }

    return imgALL;
}


/**
 * @author      JIA Pei
 * @version     2010-03-13
 * @brief       Scan a directory and sort all files in format of .xml, .yml, .yaml
 * @param       dir_path            input parameter, path to read from
 * @return      vector<string>      A vector of all scanned file names
*/
vector<string> VO_IO::ScanNSortXMLYMLsInDirectory(const string& dir_path)
{
    vector<string> mlALL;
    vector< vector<string> > mls;
    mls.resize(3);
    for(unsigned int i = 0; i < mls.size(); i++)
    {
        mls[i].resize(0);
    }

    path tempp = path(dir_path);

    VO_IO::ScanDirectory( tempp, ".xml", mls[0]);
    VO_IO::ScanDirectory( tempp, ".yml", mls[1]);
    VO_IO::ScanDirectory( tempp, ".yaml", mls[2]);

    for(unsigned int i = 0; i < mls.size(); i++)
    {
        for(unsigned int j = 0; j < mls[i].size(); j++)
        {
            mlALL.push_back(mls[i][j]);
        }
    }

    // sort the filenames
    if(mlALL.size() > 0)
    {
        qsort( (void *)&(mlALL[0]),
            (size_t)mlALL.size(),
            sizeof(string),
            str_compare );
    }

    return mlALL;
}


/**
 * @author      JIA Pei
 * @version     2010-03-13
 * @brief       Scan a directory and sort all files in format of .txt
 * @param       dir_path            input parameter, path to read from
 * @return      vector<string>      A vector of all scanned file names
*/
vector<string> VO_IO::ScanNSortTextFilesInDirectory(const string& dir_path)
{
    vector<string> textALL;
    vector< vector<string> > texts;
    texts.resize(3);
    for(unsigned int i = 0; i < texts.size(); i++)
    {
        texts[i].resize(0);
    }

    path tempp = path(dir_path);

    VO_IO::ScanDirectory( tempp, ".txt", texts[0]);

    for(unsigned int i = 0; i < texts.size(); i++)
    {
        for(unsigned int j = 0; j < texts[i].size(); j++)
        {
            textALL.push_back(texts[i][j]);
        }
    }

    // sort the filenames
    if(textALL.size() > 0)
    {
        qsort( (void *)&(textALL[0]),
            (size_t)textALL.size(),
            sizeof(string),
            str_compare );
    }

    return textALL;
}

