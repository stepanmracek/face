#include "faceCommon/facedata/mdenoise/mdenoise.h"

#include <string>
#include <cstdio>
#include <iostream>

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/linalg/common.h"

using namespace Face::FaceData::MDenoise;

//functions deal with memory allocation errors.
void *MyMalloc(size_t size) {
    void *memptr;

    memptr = (void *) malloc(size);
    if (memptr == (void *) NULL) {
        fprintf(stderr,"\nError malloc:  Out of memory.\n");
        fprintf(stderr,"The model data is too big.\n");
        exit(1);
    }
    return(memptr);
}

void *MyRealloc(void *memblock, size_t size) {
    void *oldbuffer;
    oldbuffer = memblock;
    if((memblock = realloc(memblock, size))==NULL)
    {
        free(oldbuffer);
        fprintf(stderr,"\nError realloc:  Out of memory.\n");
        fprintf(stderr,"The model data is too big.\n");
        exit(1);
    }
    return(memblock);
}

MDenoise::MDenoise() {
    //std::cout << "MDenoise::MDenoise()" << std::endl;

    // Original Mesh
    m_nNumVertex = 0;
    m_nNumFace = 0;
    m_pf3Vertex = NULL;
    m_pn3Face = NULL;
    m_pf3FaceNormal = NULL;
    m_pf3VertexNormal = NULL;
    m_ppnVRing1V = NULL; //1-Ring neighbouring vertices of each vertex
    m_ppnVRing1T = NULL; //1-Ring neighbouring triangles of each vertex
    m_ppnTRing1TCV = NULL; //1-Ring neighbouring triangles with common vertex of each triangle
    m_ppnTRing1TCE = NULL; //1-Ring neighbouring triangles with common edge of each triangle

    //Scale parameter
    m_fScale = 1.0f;

    // Produced Mesh
    m_nNumVertexP = 0;
    m_nNumFaceP = 0;
    m_pf3VertexP = NULL;
    m_pn3FaceP = NULL;
    m_pf3FaceNormalP = NULL;
    m_pf3VertexNormalP = NULL;

    //Operation Parameters
    m_bNeighbourCV = true;
    m_fSigma = 0.4f;
    m_nIterations = 20;
    m_nVIterations = 50;

    //Add vertices in triangulation
    m_bAddVertices = true;
    //Only z-direction position is updated
    m_bZOnly = true;
}

MDenoise::~MDenoise() {
    clear();
    //std::cout << "MDenoise::~MDenoise()" << std::endl;
}

void MDenoise::clear() {
    // Original Mesh
    delete [] m_pf3Vertex;
    m_pf3Vertex = NULL;
    delete [] m_pn3Face;
    m_pn3Face = NULL;
    delete [] m_pf3FaceNormal;
    m_pf3FaceNormal = NULL;
    delete [] m_pf3VertexNormal;
    m_pf3VertexNormal = NULL;
    if (m_ppnVRing1V != NULL) {
        for (int i=0;i<m_nNumVertex;i++) {
            free(m_ppnVRing1V[i]);
        }
        free(m_ppnVRing1V);
    }
    m_ppnVRing1V = NULL; //1-Ring neighbouring vertices of each vertex
    if (m_ppnVRing1T != NULL) {
        for (int i=0;i<m_nNumVertex;i++) {
            free(m_ppnVRing1T[i]);
        }
        free(m_ppnVRing1T);
    }
    m_ppnVRing1T = NULL; //1-Ring neighbouring triangles of each vertex
    if (m_ppnTRing1TCV != NULL) {
        for (int i=0;i<m_nNumFace;i++) {
            free(m_ppnTRing1TCV[i]);
        }
        free(m_ppnTRing1TCV);
    }
    m_ppnTRing1TCV = NULL; //1-Ring neighbouring triangles with common vertex of each triangle
    if (m_ppnTRing1TCE != NULL) {
        for (int i=0;i<m_nNumFace;i++) {
            free(m_ppnTRing1TCE[i]);
        }
        free(m_ppnTRing1TCE);
    }
    m_ppnTRing1TCE = NULL; //1-Ring neighbouring triangles with common edge of each triangle
    m_nNumVertex = 0;
    m_nNumFace = 0;

    //Scale parameter
    m_fScale = 1.0f;

    // Produced Mesh
    m_nNumVertexP = 0;
    m_nNumFaceP = 0;
    delete [] m_pf3VertexP;
    m_pf3VertexP = NULL;
    delete [] m_pn3FaceP;
    m_pn3FaceP = NULL;
    delete [] m_pf3FaceNormalP;
    m_pf3FaceNormalP = NULL;
    delete [] m_pf3VertexNormalP;
    m_pf3VertexNormalP = NULL;
}

void MDenoise::loadModel(const std::string& fname) {
    clear();

    if (!fname.empty() && fname.find(".obj") != std::string::npos) {
        FILE *fp = fopen(fname.c_str(), "r");
        if (fp == NULL) {
            throw FACELIB_EXCEPTION("File could not be opened");
        }

        int i, j;
        int nTmp,nTmp1;
        char sTmp[200], sTmp1[200];
        FVECTOR3 *vVertex;
        NVECTOR3 * tTriangle;

        vVertex = (FVECTOR3 *)MyMalloc(10000* sizeof(FVECTOR3));
        tTriangle = (NVECTOR3 *)MyMalloc(10002* sizeof(NVECTOR3));

        m_nNumVertex = m_nNumFace = 0;
        while (!feof(fp))
        {
            fgets(sTmp, 200, fp);
            if(sTmp[0]=='v')
            {
                if((sTmp[1]=='t')||(sTmp[1]=='n'))
                {
                    printf("This OBJ file is not supported!\n");
                    m_nNumVertex = m_nNumFace = 0;
                    free(vVertex);
                    free(tTriangle);
                    fclose(fp);
                    return;
                }
                else
                {
                    sscanf(sTmp,"%s%f%f%f", sTmp1, &(vVertex[m_nNumVertex][0]), \
                            &(vVertex[m_nNumVertex][1]),&(vVertex[m_nNumVertex][2]));
                    m_nNumVertex++;
                    if (!(m_nNumVertex % 10000))
                        vVertex = (FVECTOR3 *)MyRealloc(vVertex, (m_nNumVertex+10000)* sizeof(FVECTOR3));
                }
            }
            else if(sTmp[0]=='f')
            {
                j = sscanf(sTmp,"%s%d%d%d%d%d", sTmp1, &(tTriangle[m_nNumFace][0]), \
                        &(tTriangle[m_nNumFace][1]),&(tTriangle[m_nNumFace][2]), &nTmp, &nTmp1);
                if (j==4)
                {
                    tTriangle[m_nNumFace][0]--;
                    tTriangle[m_nNumFace][1]--;
                    tTriangle[m_nNumFace][2]--;
                }
                else if(j==5)
                {
                    tTriangle[m_nNumFace][0]--;
                    tTriangle[m_nNumFace][1]--;
                    tTriangle[m_nNumFace][2]--;
                    m_nNumFace++;
                    if (!(m_nNumFace % 10000))
                        tTriangle = (NVECTOR3 *)MyRealloc(tTriangle, (m_nNumFace+10002)* sizeof(NVECTOR3));
                    tTriangle[m_nNumFace][0] = tTriangle[m_nNumFace-1][2];
                    tTriangle[m_nNumFace][1] = (--nTmp);
                    tTriangle[m_nNumFace][2] = tTriangle[m_nNumFace-1][0];
                }
                else
                {
                    printf("This OBJ file is not supported!\n");
                    m_nNumVertex = m_nNumFace = 0;
                    free(vVertex);
                    free(tTriangle);
                    return;
                }
                m_nNumFace++;
                if (!(m_nNumFace % 10000))
                    tTriangle = (NVECTOR3 *)MyRealloc(tTriangle, (m_nNumFace+10002)* sizeof(NVECTOR3));
            }
        }
        fclose(fp);

        m_pf3Vertex = new FVECTOR3[m_nNumVertex];
        for(i=0; i<m_nNumVertex; i++)
        {
            MD_VEC3_ASN_OP(m_pf3Vertex[i], =, vVertex[i]);
        }
        free(vVertex);

        m_pn3Face = new NVECTOR3[m_nNumFace];
        for(i=0; i<m_nNumFace; i++)
        {
            MD_VEC3_ASN_OP(m_pn3Face[i], =, tTriangle[i]);
        }
        free(tTriangle);
    }

    ScalingBox(); // scale to a box
    ComputeNormal(false);

    m_nNumVertexP = m_nNumVertex;
    m_nNumFaceP = m_nNumFace;
    m_pf3VertexP = new FVECTOR3[m_nNumVertexP];
    m_pn3FaceP = new NVECTOR3[m_nNumFaceP];
    m_pf3VertexNormalP = new FVECTOR3[m_nNumVertexP];
    m_pf3FaceNormalP = new FVECTOR3[m_nNumFaceP];

    for (int i=0;i<m_nNumVertex;i++)
    {
        MD_VEC3_ASN_OP(m_pf3VertexP[i],=,m_pf3Vertex[i]);
        MD_VEC3_ASN_OP(m_pf3VertexNormalP[i],=,m_pf3VertexNormal[i]);
    }
    for (int i=0;i<m_nNumFace;i++)
    {
        MD_VEC3_ASN_OP(m_pn3FaceP[i],=,m_pn3Face[i]);
        MD_VEC3_ASN_OP(m_pf3FaceNormalP[i],=,m_pf3FaceNormal[i]);
    }
}

void MDenoise::importModel(const Face::FaceData::Mesh& mesh) {
    clear();

    m_nNumVertex = m_nNumVertexP = mesh.pointsMat.rows;
    m_nNumFace = m_nNumFaceP = mesh.triangles.size();

    m_pf3Vertex = new FVECTOR3[m_nNumVertexP];
    m_pn3Face = new NVECTOR3[m_nNumFaceP];
    for (int i=0;i<m_nNumVertexP;i++) {
        m_pf3Vertex[i][0] = mesh.pointsMat(i, 0);
        m_pf3Vertex[i][1] = mesh.pointsMat(i, 1);
        m_pf3Vertex[i][2] = mesh.pointsMat(i, 2);
    }
    for (int i=0;i<m_nNumFaceP;i++) {
        MD_VEC3_ASN_OP(m_pn3Face[i], =, mesh.triangles[i]);
    }

    ScalingBox(); // scale to a box
    ComputeNormal(false);

    m_pf3VertexP = new FVECTOR3[m_nNumVertexP];
    m_pn3FaceP = new NVECTOR3[m_nNumFaceP];
    m_pf3VertexNormalP = new FVECTOR3[m_nNumVertexP];
    m_pf3FaceNormalP = new FVECTOR3[m_nNumFaceP];

    for (int i=0;i<m_nNumVertex;i++)
    {
        MD_VEC3_ASN_OP(m_pf3VertexP[i],=,m_pf3Vertex[i]);
        MD_VEC3_ASN_OP(m_pf3VertexNormalP[i],=,m_pf3VertexNormal[i]);
    }
    for (int i=0;i<m_nNumFace;i++)
    {
        MD_VEC3_ASN_OP(m_pn3FaceP[i],=,m_pn3Face[i]);
        MD_VEC3_ASN_OP(m_pf3FaceNormalP[i],=,m_pf3FaceNormal[i]);
    }

    //std::cout << "loadModel end." << std::endl;
}

void MDenoise::saveModel(const std::string& ofname) {
    for (int i=0;i<m_nNumVertexP;i++) {
        MD_VEC3_V_OP_V_OP_S(m_pf3VertexP[i],m_f3Centre,+, m_pf3VertexP[i],*, m_fScale);
    }

    FILE* fp = fopen(ofname.c_str(), "w");
    int i;

    fprintf(fp,"# The denoised result.\n");

    for (i=0;i<m_nNumVertexP;i++)
    {
        fprintf(fp,"v %f %f %f\n", m_pf3VertexP[i][0], m_pf3VertexP[i][1], m_pf3VertexP[i][2]);
    }
    for (i=0;i<m_nNumFaceP;i++)
    {
        fprintf(fp,"f %d %d %d\n", m_pn3FaceP[i][0]+1, m_pn3FaceP[i][1]+1, m_pn3FaceP[i][2]+1);
    }

    fclose(fp);
}

void MDenoise::exportVertices(Face::FaceData::Mesh& mesh) {
    if (m_nNumVertexP != mesh.pointsMat.rows) {
        throw FACELIB_EXCEPTION("MDenoise::exportVertices: Vertices count mismatch");
    }

    for (int i=0;i<m_nNumVertexP;i++) {
        MD_VEC3_V_OP_V_OP_S(m_pf3VertexP[i],m_f3Centre,+, m_pf3VertexP[i],*, m_fScale);
    }

    for (int i=0;i<m_nNumVertexP;i++) {
        mesh.pointsMat(i, 0) = m_pf3VertexP[i][0];
        mesh.pointsMat(i, 1) = m_pf3VertexP[i][1];
        mesh.pointsMat(i, 2) = m_pf3VertexP[i][2];
    }
}

void MDenoise::denoise(bool bNeighbourCV, float fSigma, int nIterations, int nVIterations) {
    int **ttRing; //store the list of triangle neighbours of a triangle

    FVECTOR3 *Vertex;
    FVECTOR3 *TNormal;

    int i,k,m;
    float tmp3;

    if (m_nNumFace == 0)
        return;

    delete []m_pf3VertexP;
    delete []m_pf3VertexNormalP;
    delete []m_pf3FaceNormalP;
    ComputeVRing1V(); //find the neighbouring vertices of each vertex
    ComputeVRing1T();     //find the neighbouring triangles of each vertex

    //find out the neighbouring triangles of each triangle
    if (bNeighbourCV)
    {
        ComputeTRing1TCV();
        ttRing = m_ppnTRing1TCV;
        for (k=0; k<m_nNumFace; k++)
        {
            ttRing[k] = m_ppnTRing1TCV[k];
        }
    }
    else
    {
        ComputeTRing1TCE();
        ttRing = m_ppnTRing1TCE;
        for (k=0; k<m_nNumFace; k++)
        {
            ttRing[k] = m_ppnTRing1TCE[k];
        }
    }

    //begin filter
    m_nNumVertexP = m_nNumVertex;
    m_nNumFaceP = m_nNumFace;
    m_pf3VertexP = new FVECTOR3[m_nNumVertexP];
    m_pf3FaceNormalP = new FVECTOR3[m_nNumFaceP];
    m_pf3VertexNormalP = new FVECTOR3[m_nNumVertexP];
    Vertex = new FVECTOR3[m_nNumVertexP];
    TNormal = new FVECTOR3[m_nNumFace];
    for(i=0; i<m_nNumFace; i++)
    {
        MD_VEC3_ASN_OP(m_pf3FaceNormalP[i], =, m_pf3FaceNormal[i]);
    }
    for(i=0; i<m_nNumVertex; i++)
    {
        MD_VEC3_ASN_OP(m_pf3VertexP[i], =, m_pf3Vertex[i]);
    }

    for(i=0; i<m_nNumVertex; i++)
    {
        MD_VEC3_ASN_OP(Vertex[i], =, m_pf3VertexP[i]);
    }

    for(m=0; m<nIterations; m++)
    {
        //initialization
        for(i=0; i<m_nNumFace; i++)
        {
            MD_VEC3_ASN_OP(TNormal[i], =, m_pf3FaceNormalP[i]);
        }

        //modify triangle normal
        for(k=0; k<m_nNumFace; k++)
        {
            MD_VEC3_ZERO(m_pf3FaceNormalP[k]);
            for(i=1; i<ttRing[k][0]+1; i++)
            {
                tmp3 = MD_DOTPROD3(TNormal[ttRing[k][i]],TNormal[k])-fSigma;
                if( tmp3 > 0.0)
                {
                    MD_VEC3_V_OP_V_OP_S(m_pf3FaceNormalP[k],m_pf3FaceNormalP[k], +, TNormal[ttRing[k][i]], *, tmp3*tmp3);
                }
            }
            V3Normalize(m_pf3FaceNormalP[k]);
        }
        for(k=0; k<m_nNumFace; k++)
        {
            MD_VEC3_ASN_OP(TNormal[k], =, m_pf3FaceNormalP[k]);
        }
    }

    //modify vertex coordinates
    VertexUpdate(m_ppnVRing1T, nVIterations);
    //m_L2Error = L2Error();

    delete []Vertex;
    delete []TNormal;

    return;
}

void MDenoise::ScalingBox(void) {
    int i,j;
    float box[2][3];

    box[0][0] = box[0][1] = box[0][2] = FLT_MAX;
    box[1][0] = box[1][1] = box[1][2] = -FLT_MAX;
    for (i=0;i<m_nNumVertex;i++)
    {
        for (j=0;j<3;j++)
        {
            if (box[0][j]>m_pf3Vertex[i][j])
                box[0][j] = m_pf3Vertex[i][j];
            if (box[1][j]<m_pf3Vertex[i][j])
                box[1][j] = m_pf3Vertex[i][j];
        }
    }
    m_f3Centre[0] = (box[0][0]+box[1][0])/2.0f;
    m_f3Centre[1] = (box[0][1]+box[1][1])/2.0f;
    m_f3Centre[2] = (box[0][2]+box[1][2])/2.0f;

    m_fScale = MD_FMAX(box[1][0]-box[0][0],MD_FMAX(box[1][1]-box[0][1],box[1][2]-box[0][2]));
    m_fScale /=2.0;
    for (i=0;i<m_nNumVertex;i++)
    {
        MD_VEC3_VOPV_OP_S(m_pf3Vertex[i],m_pf3Vertex[i],-,m_f3Centre,/,m_fScale);
    }
}

void MDenoise::V3Normalize(FVECTOR3 v) {
    float len;
    len=sqrt(MD_DOTPROD3(v,v));
    if (len!=0.0)
    {
        v[0]=v[0]/len;
        v[1]=v[1]/len;
        v[2]=v[2]/len;
    }
}

void MDenoise::ComputeNormal(bool bProduced) {
    int i, j;
    FVECTOR3 vect[3];
    float fArea;

    if(bProduced)
    {
        if(m_pf3VertexNormalP != NULL)
            delete []m_pf3VertexNormalP;
        if(m_pf3FaceNormalP != NULL)
            delete []m_pf3FaceNormalP;

        m_pf3VertexNormalP = new FVECTOR3[m_nNumVertexP];
        m_pf3FaceNormalP = new FVECTOR3[m_nNumFaceP];

        for (i=0;i<m_nNumVertexP;i++)
        {
            MD_VEC3_ZERO(m_pf3VertexNormalP[i]);
        }
        for (i=0;i<m_nNumFaceP;i++) // compute each triangle normal and vertex normal
        {
            MD_VEC3_V_OP_V(vect[0],m_pf3VertexP[m_pn3FaceP[i][1]],-,m_pf3VertexP[m_pn3FaceP[i][0]]);
            MD_VEC3_V_OP_V(vect[1],m_pf3VertexP[m_pn3FaceP[i][2]],-,m_pf3VertexP[m_pn3FaceP[i][0]]);
            MD_CROSSPROD3(vect[2],vect[0],vect[1]);
            fArea = sqrt(MD_DOTPROD3(vect[2], vect[2]))/2.0f; // Area of the face
            V3Normalize(vect[2]);
            MD_VEC3_ASN_OP(m_pf3FaceNormalP[i],=,vect[2]);
            for (j=0;j<3;j++)
            {
                MD_VEC3_V_OP_V_OP_S(m_pf3VertexNormalP[m_pn3FaceP[i][j]], \
                        m_pf3VertexNormalP[m_pn3FaceP[i][j]], +, vect[2], *, fArea);
            }
        }
        for (i=0;i<m_nNumVertexP;i++)
            V3Normalize(m_pf3VertexNormalP[i]);
    }
    else
    {
        if(m_pf3VertexNormal != NULL)
            delete []m_pf3VertexNormal;
        if(m_pf3FaceNormal != NULL)
            delete []m_pf3FaceNormal;

        m_pf3VertexNormal = new FVECTOR3[m_nNumVertex];
        m_pf3FaceNormal = new FVECTOR3[m_nNumFace];

        for (i=0;i<m_nNumVertex;i++)
        {
            MD_VEC3_ZERO(m_pf3VertexNormal[i]);
        }
        for (i=0;i<m_nNumFace;i++) // compute each triangle normal and vertex normal
        {
            MD_VEC3_V_OP_V(vect[0],m_pf3Vertex[m_pn3Face[i][1]],-,m_pf3Vertex[m_pn3Face[i][0]]);
            MD_VEC3_V_OP_V(vect[1],m_pf3Vertex[m_pn3Face[i][2]],-,m_pf3Vertex[m_pn3Face[i][0]]);
            MD_CROSSPROD3(vect[2],vect[0],vect[1]);
            fArea = sqrt(MD_DOTPROD3(vect[2], vect[2]))/2.0f; // Area of the face
            V3Normalize(vect[2]);
            MD_VEC3_ASN_OP(m_pf3FaceNormal[i],=,vect[2]);
            for (j=0;j<3;j++)
            {
                MD_VEC3_V_OP_V_OP_S(m_pf3VertexNormal[m_pn3Face[i][j]], \
                        m_pf3VertexNormal[m_pn3Face[i][j]], +, vect[2], *, fArea);
            }
        }
        for (i=0;i<m_nNumVertex;i++)
            V3Normalize(m_pf3VertexNormal[i]);
    }
}


void MDenoise::ComputeVRing1V(void) {
    int i,j,k;
    int tmp0, tmp1, tmp2;

    if(m_ppnVRing1V != NULL)
        return;

    m_ppnVRing1V=(int **)MyMalloc(m_nNumVertex*sizeof(int *));

    for (i=0;i<m_nNumVertex;i++) {
        m_ppnVRing1V[i]=(int *)MyMalloc(6*sizeof(int));
        m_ppnVRing1V[i][0]=0; // m_ppnVRing1V[i][0] stores the number of vertex neighbours
    }

    for (k=0; k<m_nNumFace; k++)
    {
        for (i=0;i<3;i++)
        {
            tmp0=m_pn3Face[k][i];
            tmp2=m_pn3Face[k][(i+2)%3];
            for (j=1;j<m_ppnVRing1V[tmp0][0]+1;j++)
                if (m_ppnVRing1V[tmp0][j] == tmp2)
                    break;
            if (j==m_ppnVRing1V[tmp0][0]+1)
            {
                m_ppnVRing1V[tmp0][j]=tmp2;
                m_ppnVRing1V[tmp0][0] += 1;
                if (!(m_ppnVRing1V[tmp0][0]%5))
                    m_ppnVRing1V[tmp0] = (int *)MyRealloc(m_ppnVRing1V[tmp0],(m_ppnVRing1V[tmp0][0]+6)*sizeof(int));
            }
            tmp1=m_pn3Face[k][(i+1)%3];
            for (j=1;j<m_ppnVRing1V[tmp0][0]+1;j++)
                if (m_ppnVRing1V[tmp0][j] == tmp1)
                    break;
            if (j==m_ppnVRing1V[tmp0][0]+1)
            {
                m_ppnVRing1V[tmp0][j]=tmp1;
                m_ppnVRing1V[tmp0][0] += 1;
                if (!(m_ppnVRing1V[tmp0][0]%5))
                    m_ppnVRing1V[tmp0] = (int *)MyRealloc(m_ppnVRing1V[tmp0],(m_ppnVRing1V[tmp0][0]+6)*sizeof(int));
            }
        }
    }
    for (i=0;i<m_nNumVertex;i++) {
        m_ppnVRing1V[i]=(int *)MyRealloc(m_ppnVRing1V[i],(m_ppnVRing1V[i][0]+1)*sizeof(int));
    }
}

void MDenoise::ComputeVRing1T(void) {
    int i,k;
    int tmp;

    if(m_ppnVRing1T != NULL)
        return;

    m_ppnVRing1T=(int **)MyMalloc(m_nNumVertex*sizeof(int *));
    for (i=0;i<m_nNumVertex;i++) {
        m_ppnVRing1T[i]=(int *)MyMalloc(6*sizeof(int));
        m_ppnVRing1T[i][0]=0; // m_ppnVRing1T[i][0] stores the number of triangle neighbours
    }

    for (k=0; k<m_nNumFace; k++)
    {
        for (i=0;i<3;i++)
        {
            tmp = m_pn3Face[k][i]; //the vertex incident to the k-th triangle
            m_ppnVRing1T[tmp][0] += 1;
            m_ppnVRing1T[tmp][m_ppnVRing1T[tmp][0]]=k;
            if (!(m_ppnVRing1T[tmp][0]%5))
            {
                m_ppnVRing1T[tmp] = (int *)MyRealloc(m_ppnVRing1T[tmp],(m_ppnVRing1T[tmp][0]+6)*sizeof(int));
            }
        }
    }
    for (i=0;i<m_nNumVertex;i++) {
        m_ppnVRing1T[i]=(int *)MyRealloc(m_ppnVRing1T[i],(m_ppnVRing1T[i][0]+1)*sizeof(int));
    }
}

void MDenoise::ComputeTRing1TCV(void) {
    int i,j,k;
    int tmp,tmp0,tmp1,tmp2;

    if(m_ppnTRing1TCV != NULL)
        return;

    m_ppnTRing1TCV=(int **)MyMalloc(m_nNumFace*sizeof(int *));
    for (k=0; k<m_nNumFace; k++)
    {
        tmp0 = m_pn3Face[k][0];
        tmp1 = m_pn3Face[k][1];
        tmp2 = m_pn3Face[k][2];
        tmp = m_ppnVRing1T[tmp0][0] + m_ppnVRing1T[tmp1][0] +  m_ppnVRing1T[tmp2][0];
        m_ppnTRing1TCV[k] =(int *)MyMalloc(tmp*sizeof(int));

        m_ppnTRing1TCV[k][0] = m_ppnVRing1T[tmp0][0];
        for (i=1; i<m_ppnVRing1T[tmp0][0]+1; i++)
        {
            m_ppnTRing1TCV[k][i] = m_ppnVRing1T[tmp0][i];
        }

        for (i=1; i<m_ppnVRing1T[tmp1][0]+1; i++)
        {
            if((m_pn3Face[m_ppnVRing1T[tmp1][i]][0] != tmp0) && (m_pn3Face[m_ppnVRing1T[tmp1][i]][1] != tmp0)\
                    && (m_pn3Face[m_ppnVRing1T[tmp1][i]][2] != tmp0))
            {
                ++m_ppnTRing1TCV[k][0];
                m_ppnTRing1TCV[k][m_ppnTRing1TCV[k][0]]= m_ppnVRing1T[tmp1][i];
            }
            else
            {
                for(j=1; j<m_ppnTRing1TCV[k][0]+1; j++)
                {
                    if(m_ppnTRing1TCV[k][j]==m_ppnVRing1T[tmp1][i])
                    {
                        break;
                    }
                }
            }
        }

        for (i=1; i<m_ppnVRing1T[tmp2][0]+1; i++)
        {
            if((m_pn3Face[m_ppnVRing1T[tmp2][i]][0] != tmp0) && (m_pn3Face[m_ppnVRing1T[tmp2][i]][1] != tmp0)\
                    && (m_pn3Face[m_ppnVRing1T[tmp2][i]][2] != tmp0) && (m_pn3Face[m_ppnVRing1T[tmp2][i]][0] != tmp1)\
                    && (m_pn3Face[m_ppnVRing1T[tmp2][i]][1] != tmp1) && (m_pn3Face[m_ppnVRing1T[tmp2][i]][2] != tmp1))
            {
                ++m_ppnTRing1TCV[k][0];
                m_ppnTRing1TCV[k][m_ppnTRing1TCV[k][0]]= m_ppnVRing1T[tmp2][i];
            }
            else
            {
                for(j=1; j<m_ppnTRing1TCV[k][0]+1; j++)
                {
                    if(m_ppnTRing1TCV[k][j]==m_ppnVRing1T[tmp2][i])
                    {
                        break;
                    }
                }
            }
        }
    }
    for (i=0;i<m_nNumFace;i++) {
        m_ppnTRing1TCV[i]=(int *)MyRealloc(m_ppnTRing1TCV[i],(m_ppnTRing1TCV[i][0]+1)*sizeof(int));
    }
}

void MDenoise::ComputeTRing1TCE(void) {
    int i,k;
    int tmp,tmp0,tmp1,tmp2;

    if(m_ppnTRing1TCE != NULL)
        return;

    m_ppnTRing1TCE=(int **)MyMalloc(m_nNumFace*sizeof(int *));
    for (k=0; k<m_nNumFace; k++)
    {
        tmp0 = m_pn3Face[k][0];
        tmp1 = m_pn3Face[k][1];
        tmp2 = m_pn3Face[k][2];

        m_ppnTRing1TCE[k] = (int *)MyMalloc(5*sizeof(int));

        tmp = 0;
        for (i=1; i<m_ppnVRing1T[tmp0][0]+1; i++)
        {
            if ((m_pn3Face[m_ppnVRing1T[tmp0][i]][0] == tmp1)||(m_pn3Face[m_ppnVRing1T[tmp0][i]][0] == tmp2)||\
                    (m_pn3Face[m_ppnVRing1T[tmp0][i]][1] == tmp1)||(m_pn3Face[m_ppnVRing1T[tmp0][i]][1] == tmp2)||\
                    (m_pn3Face[m_ppnVRing1T[tmp0][i]][2] == tmp1)||(m_pn3Face[m_ppnVRing1T[tmp0][i]][2] == tmp2))
            {
                tmp++;
                if (tmp>4)
                {
                    tmp--;
                    break;
                }
                m_ppnTRing1TCE[k][tmp] = m_ppnVRing1T[tmp0][i];
            }
        }

        for (i=1; i<m_ppnVRing1T[tmp1][0]+1; i++)
        {
            if ((m_pn3Face[m_ppnVRing1T[tmp1][i]][0] == tmp1)&&\
                    ((m_pn3Face[m_ppnVRing1T[tmp1][i]][1] == tmp2)||(m_pn3Face[m_ppnVRing1T[tmp1][i]][2] == tmp2)))
            {
                tmp++;
                if (tmp>4)
                {
                    tmp--;
                    break;
                }
                m_ppnTRing1TCE[k][tmp] = m_ppnVRing1T[tmp1][i];
                break;
            }
            else if((m_pn3Face[m_ppnVRing1T[tmp1][i]][0] == tmp2)&&\
                    ((m_pn3Face[m_ppnVRing1T[tmp1][i]][1] == tmp1)||(m_pn3Face[m_ppnVRing1T[tmp1][i]][2] == tmp1)))
            {
                tmp++;
                if (tmp>4)
                {
                    tmp--;
                    break;
                }
                m_ppnTRing1TCE[k][tmp] = m_ppnVRing1T[tmp1][i];
                break;
            }
            else if((m_pn3Face[m_ppnVRing1T[tmp1][i]][1] == tmp2)&&(m_pn3Face[m_ppnVRing1T[tmp1][i]][2] == tmp1))
            {
                tmp++;
                if (tmp>4)
                {
                    tmp--;
                    break;
                }
                m_ppnTRing1TCE[k][tmp] = m_ppnVRing1T[tmp1][i];
                break;
            }
            else if((m_pn3Face[m_ppnVRing1T[tmp1][i]][1] == tmp1)&&\
                    (m_pn3Face[m_ppnVRing1T[tmp1][i]][2] == tmp2)&&(m_pn3Face[m_ppnVRing1T[tmp1][i]][0] != tmp0))
            {
                tmp++;
                if (tmp>4)
                {
                    tmp--;
                    break;
                }
                m_ppnTRing1TCE[k][tmp] = m_ppnVRing1T[tmp1][i];
                break;
            }
        }
        m_ppnTRing1TCE[k][0] = tmp;
    }
}

void MDenoise::VertexUpdate(int** tRing, int nVIterations) {
    int i, j, m;
    int nTmp0, nTmp1, nTmp2;
    float fTmp1;

    FVECTOR3 vect[3];

    for(m=0; m<nVIterations; m++)
    {

        for(i=0; i<m_nNumVertex; i++)
        {
            MD_VEC3_ZERO(vect[1]);
            for(j=1; j<tRing[i][0]+1; j++)
            {
                nTmp0 = m_pn3Face[tRing[i][j]][0]; // the vertex number of triangle tRing[i][j]
                nTmp1 = m_pn3Face[tRing[i][j]][1];
                nTmp2 = m_pn3Face[tRing[i][j]][2];
                MD_VEC3_V_OP_V_OP_V(vect[0], m_pf3VertexP[nTmp0],+, m_pf3VertexP[nTmp1],+, m_pf3VertexP[nTmp2]);
                MD_VEC3_V_OP_S(vect[0], vect[0], /, 3.0f); //vect[0] is the centr of the triangle.
                MD_VEC3_V_OP_V(vect[0], vect[0], -, m_pf3VertexP[i]); //vect[0] is now vector PC.
                fTmp1 = MD_DOTPROD3(vect[0], m_pf3FaceNormalP[tRing[i][j]]);
                if(m_bZOnly)
                    vect[1][2] = vect[1][2] + m_pf3FaceNormalP[tRing[i][j]][2] * fTmp1;
                else
                    MD_VEC3_V_OP_V_OP_S(vect[1], vect[1], +, m_pf3FaceNormalP[tRing[i][j]],*, fTmp1);
            }
            if (tRing[i][0]!=0)
            {
                if(m_bZOnly)
                    m_pf3VertexP[i][2] = m_pf3VertexP[i][2] + vect[1][2]/tRing[i][0];
                else
                    MD_VEC3_V_OP_V_OP_S(m_pf3VertexP[i], m_pf3VertexP[i],+, vect[1], /, tRing[i][0]);
            }
        }
    }
    ComputeNormal(true);
}

