#ifndef MDENOISE_H
#define MDENOISE_H

#include <string>

#include "defs.h"
#include "faceCommon/facedata/mesh.h"

namespace Face {
namespace FaceData {
namespace MDenoise {

class FACECOMMON_EXPORTS MDenoise {
public:
    MDenoise();
    virtual ~MDenoise();

    void loadModel(const std::string& fname);
    void saveModel(const std::string& ofname);

    void importModel(const Mesh& mesh);
    void exportVertices(Mesh& mesh);

    void denoise(bool bNeighbourCV, float fSigma, int nIterations, int nVIterations);

private:
    void clear();
    // Preprocessing Operations
    void ScalingBox(void);
    void V3Normalize(FVECTOR3 v);
    void ComputeNormal(bool bProduced);
    void ComputeVRing1V(void);
    void ComputeVRing1T(void);
    void ComputeTRing1TCV(void);
    void ComputeTRing1TCE(void);
    void VertexUpdate(int** tRing, int nVIterations);

private:
    // Original Mesh
    int			m_nNumVertex;
    int			m_nNumFace;
    FVECTOR3*	m_pf3Vertex;
    NVECTOR3*	m_pn3Face;
    FVECTOR3*	m_pf3FaceNormal;
    FVECTOR3*	m_pf3VertexNormal;
    int**		m_ppnVRing1V; //1-Ring neighbouring vertices of each vertex
    int**		m_ppnVRing1T; //1-Ring neighbouring triangles of each vertex
    int**		m_ppnTRing1TCV; //1-Ring neighbouring triangles with common vertex of each triangle
    int**		m_ppnTRing1TCE; //1-Ring neighbouring triangles with common edge of each triangle

    //Scale parameter
    float		m_fScale;
    float		m_f3Centre[3];

    // Produced Mesh
    int			m_nNumVertexP;
    int			m_nNumFaceP;
    FVECTOR3*	m_pf3VertexP;
    NVECTOR3*	m_pn3FaceP;
    FVECTOR3*	m_pf3FaceNormalP;
    FVECTOR3*	m_pf3VertexNormalP;

    //Operation Parameters
    bool m_bNeighbourCV;
    float m_fSigma;
    int m_nIterations;
    int m_nVIterations;

    //Add vertices in triangulation
    bool m_bAddVertices;
    //Only z-direction position is updated
    bool m_bZOnly;
};

}
}
}

#endif // MDENOISE_H
