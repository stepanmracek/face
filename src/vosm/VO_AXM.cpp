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


#include <fstream>
#include <sstream>
#include <string>

#include <boost/filesystem.hpp>

#include "VO_AXM.h"


/**
 * @author         JIA Pei
 * @version        2010-02-13
 * @brief          Save ASM to a specified folder
 * @param          fd             Input - the folder that ASM to be saved to
 * @return        void
*/
void VO_AXM::VO_Save ( const string& fd )
{
    switch(this->m_iMethod)
    {
        case AAM_BASIC:
        case AAM_DIRECT:
        case AAM_FAIA:
        case AAM_CMUICIA:
        case AAM_IAIA:
        VO_TextureModel::VO_Save(fd);
        break;
        case ASM_PROFILEND:
        case ASM_LTC:
        VO_ShapeModel::VO_Save(fd);
        break;
        case CLM:
        case AFM:
        break;
    }
    
    // create AXM subfolder for just AXM model data
    string fn = fd+"/AXM";
    if (!boost::filesystem::is_directory(fn) )
        boost::filesystem::create_directory( fn );

    ofstream fp;
    string tempfn;

    // AXM
    tempfn = fn + "/AXM" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_iNbOfPyramidLevels" << endl << this->m_iNbOfPyramidLevels << endl;
    fp.close();fp.clear();
}


/**
 * @author      JIA Pei
 * @version     2010-02-13
 * @brief       Load all trained data
 * @param       fd      Input - the folder that ASM to be saved to
 * @return      void
*/
void VO_AXM::VO_Load ( const string& fd )
{
    switch(this->m_iMethod)
    {
        case AAM_BASIC:
        case AAM_DIRECT:
        case AAM_FAIA:
        case AAM_CMUICIA:
        case AAM_IAIA:
        VO_TextureModel::VO_Load(fd);
        break;
        case ASM_PROFILEND:
        case ASM_LTC:
        VO_ShapeModel::VO_Load(fd);
        break;
        case CLM:
        case AFM:
        break;
    }
    
    string fn = fd+"/AXM";
    if (!boost::filesystem::is_directory(fn) )
    {
        cout << "AXM subfolder is not existing. " << endl;
        exit(EXIT_FAILURE);
    }

    ifstream fp;
    string tempfn;
    string temp;

    // AXM
    tempfn = fn + "/AXM" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp >> this->m_iNbOfPyramidLevels;   // m_iNbOfPyramidLevels
    fp.close();fp.clear();
}


/**
 * @author      JIA Pei
 * @version     2010-02-13
 * @brief       Load all trained data for fitting
 * @param       fd      Input - the folder that ASM to be saved to
 * @return      void
*/
void VO_AXM::VO_LoadParameters4Fitting ( const string& fd )
{
    switch(this->m_iMethod)
    {
        case AAM_BASIC:
        case AAM_DIRECT:
        case AAM_FAIA:
        case AAM_CMUICIA:
        case AAM_IAIA:
        VO_TextureModel::VO_LoadParameters4Fitting(fd);
        break;
        case ASM_PROFILEND:
        case ASM_LTC:
        VO_ShapeModel::VO_LoadParameters4Fitting(fd);
        break;
        case CLM:
        case AFM:
        break;
    }
    
    string fn = fd+"/AXM";
    if (!boost::filesystem::is_directory(fn) )
    {
        cout << "AXM subfolder is not existing. " << endl;
        exit(EXIT_FAILURE);
    }

    ifstream fp;
    string tempfn;
    string temp;

    // AXM
    tempfn = fn + "/AXM" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp >> this->m_iNbOfPyramidLevels;   // m_iNbOfPyramidLevels
    fp.close();fp.clear();
}

