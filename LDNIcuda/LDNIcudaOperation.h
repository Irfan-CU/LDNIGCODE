/*
 *  Copyright (C) 2014, Geometric Design and Manufacturing Lab in THE CHINESE UNIVERSITY OF HONG KONG
 *  All rights reserved.
 *   
 *		 http://ldnibasedsolidmodeling.sourceforge.net/
 *  
 *   
 *  Redistribution and use in source and binary forms, with or without modification, 
 *  are permitted provided that the following conditions are met:
 *  
 *  1. Redistributions of source code must retain the above copyright notice, 
 *     this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, 
 *     this list of conditions and the following disclaimer in the documentation 
 *	   and/or other materials provided with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 *   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 *   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 *   IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 *   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 *   OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 *   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 *   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 *   OF SUCH DAMAGE.
 */


#ifndef	_CCL_LDNI_CUDA_OPERATION
#define	_CCL_LDNI_CUDA_OPERATION

#define BUFFER_SIZE		1024*1024*16	// this is the space of 192MB since total size if BUFFER_SIZE*3*4 (i.e., three float for each sample)
//#define BISECTION_INTERVAL_SEARCH	true
#define INTERSECTIONFREE_CONTOURING	true

#include <iostream>
#include <vector_types.h>
#include<driver_types.h>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>



class LDNIcpuSolid;
class LDNIcudaSolid;
class LDMIProcessor;
class QuadTrglMesh;
class ContourMesh;
class GLKObList;
class LDMIContourMesh;


class LDMIcudaOperation {
public:

	void MallocMemory(int imageSize,int mat_combination) {
		
		m_lDMIContourMesh = NULL;
		totalMaterialRegions = mat_combination;
		stickMaterial = (int *)malloc(mat_combination * sizeof(int));
		stickIDcheck = (int**)(malloc((mat_combination) * sizeof(int*)));
		prevStickIDcheck = (int**)(malloc(mat_combination * sizeof(int*)));
		stickDircheck = (int2**)(malloc(mat_combination * sizeof(int2*)));
		stickStartcheck = (float2**)(malloc(mat_combination * sizeof(float2*)));
		stickEndcheck = (float2**)(malloc(mat_combination * sizeof(float2*)));

	};	

	

	void freeMemory()
	{
		if (m_lDMIContourMesh != NULL) {
			m_lDMIContourMesh = NULL;
			free(m_lDMIContourMesh);
		};
		if (stickIDcheck != NULL)
		{
			stickIDcheck = NULL;
			free(stickIDcheck);
		}
		if (stickMaterial != NULL)
		{
			stickMaterial = NULL;
			free(stickMaterial);
		}
		if (prevStickIDcheck != NULL)
		{
			prevStickIDcheck = NULL;
			free(prevStickIDcheck);
		}
		if (stickDircheck != NULL)
		{
			stickDircheck = NULL;
			free(stickDircheck);
		}
		if (stickStartcheck != NULL)
		{
			stickStartcheck = NULL;
			free(stickStartcheck);
		}
		if (stickEndcheck != NULL)
		{
			stickEndcheck = NULL;
			free(stickEndcheck);
		}
		totalMaterialRegions = 0;


	}
	


	int **GetstickIDcheck() { return stickIDcheck; };
	int *GetStickMaterial() { return stickMaterial; };
	int **GetPrevStickIDcheck() { return prevStickIDcheck; };
	int2 **GetstickDircheck() { return stickDircheck; };
	float2 **GetstickStartcheck() { return stickStartcheck; };
	float2 **GetstickEndcheck() { return stickEndcheck; };
	int gettotalMaterialRegions() { return totalMaterialRegions; };
	
	ContourMesh *m_contourMesh;
	LDMIContourMesh *m_lDMIContourMesh;

	


private:

	//std::vector<int*>stickIDcheckreg;
	
	int * stickMaterial; //stick material will be same as a material for whole contour
	int **stickIDcheck;
	int2 **stickDircheck;
	float2 **stickStartcheck;
	float2 **stickEndcheck;
	int** prevStickIDcheck;
	int totalMaterialRegions;
	std::vector<std::string>materialData;


	

};





class LDNIcudaOperation	
{
public:
	LDNIcudaOperation(void) {};
	~LDNIcudaOperation(void) {};

	

	static bool BooleanOperation(LDNIcudaSolid* &inputSolid, QuadTrglMesh *meshB, short nOperationType);
	static bool BooleanOperation(LDNIcudaSolid* &inputSolid, QuadTrglMesh *meshB, short nOperationType, float boundingBox[]);
	static bool BooleanOperation(QuadTrglMesh *meshA, QuadTrglMesh *meshB, int res, short nOperationType, LDNIcudaSolid* &solid);
	static bool BooleanOperation(QuadTrglMesh *meshA, QuadTrglMesh *meshB, int res, short nOperationType, LDNIcudaSolid* &solid, LDNIcudaSolid* &Savedsolid);
	//static bool BooleanOperation(LDNIcudaSolid* &solidA, LDNIcudaSolid* &solidB, short nOperationType);
	static bool BRepToLDNISampling(QuadTrglMesh *mesh, LDNIcudaSolid* &solid, float boundingBox[], int res);
	static bool BRepToLDMISampling(QuadTrglMesh *mesh, LDNIcudaSolid* &solid, LDMIProcessor *&ldmiProcessor, float boundingBox[], int res);
	static void LDNIToBRepReconstruction(LDNIcudaSolid* solid, QuadTrglMesh* &mesh, int nMeshRes, bool bWithIntersectionPrevention);

	static void SolidOffsetting(LDNIcudaSolid* inputSolid, LDNIcudaSolid* &newSolid, float offset);
	static void SolidOffsettingBySpatialHashing(LDNIcudaSolid* inputSolid, LDNIcudaSolid* &newSolid, float offset, bool bWithRayPacking);
	static void SolidQuickSuccessiveOffsettingBySpatialHashing(LDNIcudaSolid* inputSolid, LDNIcudaSolid* &newSolid, float offset, bool bWithNormal);


	static bool InstancedBRepToLDNISampling(QuadTrglMesh *mesh, LDNIcudaSolid* &solid, float boundingBox[], int res, float UnitOff[], int UnitNum[], float UnitWidth[], int UnitFlip[], bool bsingleRow, bool bsingleCol);
	static bool ScaffoldBooleanOperation(LDNIcudaSolid* &outputSolid,QuadTrglMesh *UnitMesh, int UnitNum[], float UnitOff[], int UnitFlip[], int nRes, LDNIcudaSolid* savedSolid);
	static bool SuperUnionOperation(LDNIcudaSolid* &solid, GLKObList* meshlist, float boundingBox[], int res);
	static bool MultiObjectSamplingInOneSolid(LDNIcudaSolid* &solid, GLKObList* meshlist, float boundingBox[], int res);
	static bool _UnionMultiObjects(LDNIcudaSolid* &inputSolid, int res);


	static float LDNIFDMContouring_CompRotationBoundingBox(LDNIcudaSolid* solid, double rotBoundingBox[], double clipPlanNm[]);
	static void LDNIFDMContouring_BinarySamlping(LDNIcudaSolid* solid,LDMIProcessor *ldmiProcessor , LDMIcudaOperation *&ldmiCudaOperation, ContourMesh *c_mesh, double rotBoundingBox[], int imageSize[],
												float angle, float thickness, double clipPlanNm[], float nSampleWidth, bool *&gridNodes,
												float2 *&stickStart, float2 *&stickEnd, unsigned int *&stickIndex, 
												int *&stickID ,int *&prevStickID,  int2 *&stickDir, 
												unsigned int *&material_index,	QuadTrglMesh *qmesh, unsigned int*&material_status);
	//static void LDNIFDMContouring_ConstrainedSmoothing(LDNIcudaSolid* solid, ContourMesh *c_mesh, double rotBoundingBox[], int imageSize[],
	//												float angle, float thickness, double clipPlanNm[], float nSampleWidth, float2 *stickStart, 
	//												float2 *stickEnd, int *stickID);
	/*static void LDNIFDMContouring_Infill(int imageSize[]);  */


	

	static char* Materialdecoder(int encodedMaterial);
	
	


	static void LDNIFDMContouring_ConstrainedSmoothing(LDNIcudaSolid* solid, ContourMesh *c_mesh, LDMIcudaOperation *ldmiCudaOperation, double rotBoundingBox[], int imageSize[],
													  float nSampleWidth, float2 *stickStart, float2 *stickEnd, int *stickID, int2 *stickDir, unsigned int *&material_index1, bool bOutPutSGM = false);
													
	static void LDNIFDMContouring_SupportContourGeneration(LDNIcudaSolid* solid, ContourMesh *c_mesh, bool *gridNodes, double rotBoundingBox[], double clipPlaneNm[], double thickness, double nSampleWidth, double distortRatio, int imageSize[], bool bOutPutSGM = false);
	static void LDNIFDMContouring_BuildDistanceMap(bool *gridNodes, int layerID, int imageSize[], short2 *&disMapA, short2 *&disMapB, int disTexSize);
	static void LDNIFDMContouring_GrowthAndSwallow(bool *gridNodes, int layerID, int imageSize[], double t, double nSampleWidth, bool *&suptRegion, bool *&solidRegion, short2 *disTextureA, short2 *disTextureB, int disTexSize, bool *test1, bool *test2);
	static void LDNIFDMContouring_Closing(int imageSize[], double t, double nSampleWidth, bool *&inputNodes, int i, bool *test1, bool *test2);
	static void LDNIFDMContouring_SupportFindAllStick(LDNIcudaSolid* solid, ContourMesh *c_mesh, double rotBoundingBox[], int imageSize[], float thickness, 
							float nSampleWidth, bool *suptNodes, float2 *&stickStart, float2 *&stickEnd, unsigned int *&stickIndex, int *&stickID, int *&prevStickID,  int2 *&stickDir);
	
	
	static void LDMIFDMContouring_BuildSearchStickIndex(LDMIcudaOperation *ldmiCudaOperation , int *stickNum);
	
	static void LDNIFDMContouring_BuildSearchStickIndex(int *&stickID, int *prevStickId, int stickNum);
	static void LDNIFDMContouring_Generation(LDNIcudaSolid* solid, LDMIProcessor *ldmiprocessor, ContourMesh *c_mesh, QuadTrglMesh *mesh, float nSampleWidth);
	
	static void LDNIFDMContouring_GenerationwithSupporting(LDNIcudaSolid* solid, ContourMesh *c_mesh, ContourMesh *supt_mesh, float nSampleWidth);


	static void LDNISLAContouring_GenerationwithSupporting(LDNIcudaSolid* solid, ContourMesh *c_mesh, ContourMesh *supt_mesh, float nSampleWidth, bool bImgDisplay, float thickness, float anchorR, float threshold, float cylinderRadius, float patternThickness);
	static void LDNISLAContouring_SupportImageGeneration(LDNIcudaSolid* solid, ContourMesh *c_mesh, bool *gridNodes, double rotBoundingBox[], double clipPlaneNm[], double thickness, double nSampleWidth, double distortRatio, int imageSize[], float anchorR, float threshold, float cylinderRadius, float patternThickness);
	static void LDNISLAContouring_SupportImageGeneration(LDNIcudaSolid* solid, ContourMesh *c_mesh, bool *&suptNodes, bool *gridNodes,
														double rotBoundingBox[], double clipPlaneNm[], double thickness, 
														double nSampleWidth, double distortRatio, int imageSize[], float anchorR, float threshold, float cylinderRadius, float patternThickness);
	static void LDNISLAContouring_ThirdClassCylinder(double threshold, bool *&targetImg, bool *&assistImg, bool *&tempImg, int2 imgRes, double nSampleWidth, int i, short2 *disTextureA, short2 *disTextureB, int disTexSize);
	static void LDNISLAContouring_AssistForImageGrouping(bool *&targetImg, bool *&tempImg, double increaseDist, int2 imgRes, int i, double nSampleWidth);
	static void LDNISLAContouring_GrowthAndSwallow(double t, bool *&suptNodes, bool *&seedNodes, int i, int imageSize[],  double nSampleWidth, short2 *disTextureA, short2 *disTextureB, int disTexSize);
	static void LDNISLAContouring_BuildDistanceMap(bool *seedNodes, int i, int imageSize[], short2 *&disMapA, short2 *&disMapB, int disTexSize);

	static void LDNISLAContouring_Generation(LDNIcudaSolid* solid, ContourMesh *c_mesh, int dx, int dy, float thickness);
	static void LDNISLAContouring_BinarySampling(LDNIcudaSolid* solid, ContourMesh *c_mesh, double rotBoundingBox[], int imageSize[], float thickness
												,double clipPlanNm[], double pixelWidth, double imageRange[], float angle);
	static void LDNISLAContouring_GenerateConnectionforCylinders(unsigned int *linkLayerC, short *linkLayerD,
												short2 *linkID, bool *gridNodes, bool *&suptNodes, int imageSize[], int linkThreshold, int lengthofLayer, 
												int furtherStepLength, int linkNum, double nSampleWidth);
	static void LDNIFDMContouring_BinarySamlping(LDNIcudaSolid* solid, ContourMesh *c_mesh, double rotBoundingBox[], int imageSize[],
												float angle, float thickness, double clipPlanNm[], float nSampleWidth, bool *&gridNodes);

	static unsigned int LDNIFlooding_Color2DFlooding(unsigned int *&index, bool *&key, unsigned int *&value, unsigned int arrsize, unsigned int init, int2 imgRes);

	//-----------------------------------------------------------------------------------------------------
	static void DistanceFieldGeneration(QuadTrglMesh *mesh, GLuint *vbo, unsigned int &vbosize, int res, int offdist, float boundingBox[]);
	static bool LDNIDistanceField_SitesGeneration(QuadTrglMesh *mesh, GLuint *vbo, int &vbosize, int res, float boundingBox[], unsigned int *&site_index, unsigned short *&sites);
	static int LDNIDistanceField_Read3DTextureToVBO(cudaGraphicsResource *resource, GLuint* vbo, int res, float width, float origin[3]);
	static int LDNIDistanceField_ReadArrayToVBO(cudaGraphicsResource *resource, GLuint* vbo, unsigned int *m_3dArray, int res, float width, float origin[3]);
	//-----------------------------------------------------------------------------------------------------
	


	//-----------------------------------------------------------------------------------------------------
	//	The following offsetting function does not consider about normal vector in the computation
	//		but still generally use spatial hashing + ray packing (optional)
	static void SolidOffsettingWithoutNormal(LDNIcudaSolid* inputSolid, LDNIcudaSolid* &newSolid, float offset, bool bWithRayPacking);

	static void ParallelProcessingNormalVector(LDNIcudaSolid *solid, unsigned int nSupportSize=3, float normalPara=0.15f);	// by Bilateral Filtering
	static void OrientedNormalReconstruction(LDNIcudaSolid *solid, unsigned int nSupportSize, bool bWithOrientationVoting);

	static void SolidRegularization(LDNIcudaSolid *solid);  // Removing samples that are nearly tangentially contacted

	static void CopyCPUSolidToCUDASolid(LDNIcpuSolid *cpuSolid, LDNIcudaSolid* &cudaSolid);
	static void CopyCUDASolidToCPUSolid(LDNIcudaSolid *cudaSolid, LDNIcpuSolid* &cpuSolid);
	
	static void GetCudaDeviceProperty();



private:
	static void _expansionLDNIcudaSolidByNewBoundingBox(LDNIcudaSolid *cudaSolid, float boundingBox[]);
	static void _switchSolid(LDNIcudaSolid* solidA, LDNIcudaSolid* solidB);

	//-----------------------------------------------------------------------------------------------------
	//	Functions for sorting based offsetting

	//-----------------------------------------------------------------------------------------------------
	//	Functions for building hashing table for samples perpendicular
	static void _buildHashingTableForSamplesOnPerpendicularRays(LDNIcudaSolid* inputSolid, short nCurrentDir, 
							int &hashTableElementNum, unsigned int *&hashTableIndexArray, float *&hashElementsData);
	static void _sortingRaysByPossibleMergingGivenBySpatialHashingSamples(float *hashElementsData, unsigned int *hashTableIndexArray,
							int *devSHBoxToBeSearched, int devNumOfSearchedSHBox, short nAxis, int res, float 
		, float offset, 
							unsigned int *&orderOfRayProcessing, unsigned int &nonzeroStIndex);

	//-----------------------------------------------------------------------------------------------------
	//	Functions for Boolean operations
	static bool _booleanOperation(LDNIcudaSolid* solidA, LDNIcudaSolid* solidB, short nOperationType);
	static void _compBoundingCube(QuadTrglMesh *meshA, QuadTrglMesh *meshB, float boundingBox[], int res);

	//-----------------------------------------------------------------------------------------------------
	//	Functions for sampling a B-rep into LNDIcpuSolid
	static void _decomposeLDNIByFBOPBO(LDNIcudaSolid *solid,LDMIProcessor *ldmiProcessor, int displayListIndex);
	static void _decomposeLDNIByFBOPBO(LDNIcudaSolid *solid, GLuint vbo, GLuint vboI, int instanceCount, int indexCount);
	static void _decomposeLDNIByFBOPBO(LDNIcudaSolid *solid, GLuint* vbo, GLuint* vboI, int mesh_count, float Cent[], unsigned int g_programObj, int indexCount[]);
	static unsigned char* _readShaderFile( const char *fileName );
	static void _texCalProduct(int in, int &outx, int &outy);

	//-----------------------------------------------------------------------------------------------------
	//	Functions for service
	static float _distanceToBoundBoxBoundary(LDNIcudaSolid* inputSolid);   
	// LDNI sampling arranged into layer heights arranged 

	/*Data Set for LDMI Program -- Developed by Irfan---------*/
	thrust::host_vector<int>cpuStickMaterial1;
	

	
	
	/*
	std::vector<float>cpuStickStart1;
	std::vector<float>cpuStickStart2;
	std::vector<float>cpuStickStart3;
	std::vector<float>cpuStickEnd1;
	std::vector<float>cpuStickEnd2;
	std::vector<float>cpuStickEnd3;
	std::vector<int>cpuStickMaterial1;
	std::vector<int>cpuStickMaterial2;
	std::vector<int>cpuStickMaterial3;
	 */

};




class  Layers
{
public:
	int max_no_of_layer;
	int max_height;
	float layer_thickness;




	float allowable_layer_height = 1;	 
	void SetlayernodePosition(int iteration, float pos1[]);
	void GetlayerNode_Position(int nodeIndex, float pos1[]);

	void MallocMemory(int nodeNum);
	//void ClearAll();
	float layer_height;
	float z_postion;

	//void calculate_layer_heights();
	

private:
	int m_nodeNum1, m_faceNum;
	float *m_layer_positions;
	bool bUpdate;

};

#endif
