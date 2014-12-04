/* defs.h: definitions for feature-preserving mesh denoising program.
 * Copyright (C) 2007 Cardiff University, UK
 *
 * Version: 1.0
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 59 Temple Place - Su
 *
 * Author: Xianfang Sun
 */

#ifndef MESHDENOISE_DEFS_H
#define MESHDENOISE_DEFS_H

namespace Face {
namespace FaceData {
namespace MDenoise {
//Constants

// New type definitions
typedef float FVECTOR3[3];
typedef int NVECTOR3[3];
typedef int NVECTOR2[2];

// Macros Definitions
#define MD_FMAX(x,y) ((x)>(y) ? (x) : (y))
#define MD_VEC3_ZERO(vec) { (vec)[0]=(vec)[1]=(vec)[2]=0; }
#define MD_VEC3_EQ(a,b) (((a)[0]==(b)[0]) && ((a)[1]==(b)[1]) && ((a)[2]==(b)[2]))
#define MD_VEC3_V_OP_S(a,b,op,c) {  (a)[0] = (b)[0] op (c);  \
    (a)[1] = (b)[1] op (c);  \
    (a)[2] = (b)[2] op (c);  }
#define MD_VEC3_V_OP_V(a,b,op,c) { (a)[0] = (b)[0] op (c)[0]; \
    (a)[1] = (b)[1] op (c)[1]; \
    (a)[2] = (b)[2] op (c)[2]; \
}
#define MD_VEC3_V_OP_V_OP_S(a,b,op1,c,op2,d) \
{ (a)[0] = (b)[0] op1 (c)[0] op2 (d); \
    (a)[1] = (b)[1] op1 (c)[1] op2 (d); \
    (a)[2] = (b)[2] op1 (c)[2] op2 (d); }
#define MD_VEC3_VOPV_OP_S(a,b,op1,c,op2,d)  \
{ (a)[0] = ((b)[0] op1 (c)[0]) op2 (d); \
    (a)[1] = ((b)[1] op1 (c)[1]) op2 (d); \
    (a)[2] = ((b)[2] op1 (c)[2]) op2 (d); }
#define MD_VEC3_V_OP_V_OP_V(a,b,op1,c,op2,d)  \
{ (a)[0] = (b)[0] op1 (c)[0] op2 (d)[0]; \
    (a)[1] = (b)[1] op1 (c)[1] op2 (d)[1]; \
    (a)[2] = (b)[2] op1 (c)[2] op2 (d)[2]; }
#define MD_VEC3_ASN_OP(a,op,b)      {a[0] op b[0]; a[1] op b[1]; a[2] op b[2];}
#define MD_DOTPROD3(a, b)		 ((a)[0]*(b)[0] + (a)[1]*(b)[1] + (a)[2]*(b)[2])
#define MD_CROSSPROD3(a,b,c)       {(a)[0]=(b)[1]*(c)[2]-(b)[2]*(c)[1]; \
    (a)[1]=(b)[2]*(c)[0]-(b)[0]*(c)[2]; \
    (a)[2]=(b)[0]*(c)[1]-(b)[1]*(c)[0];}

}
}
}

#endif // MESHDENOISE_DEFS_H
