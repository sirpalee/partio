/* partio4Maya  3/12/2012, John Cassella  http://luma-pictures.com and  http://redpawfx.com
PARTIO Instancer
Copyright 2012 (c)  All rights reserved

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

Disclaimer: THIS SOFTWARE IS PROVIDED BY  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE ARE DISCLAIMED.
IN NO EVENT SHALL  THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND BASED ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
*/

#ifndef Partio4MayaInstancer_H
#define Partio4MayaInstancer_H

#define _USE_MGL_FT_

#include <math.h>
#include <stdlib.h>
#include <vector>
#include <set>

#include <maya/MBoundingBox.h>
#include <maya/MColor.h>
#include <maya/MDagPath.h>
#include <maya/MDrawData.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MDoubleArray.h>
#include <maya/MDistance.h>
#include <maya/MFloatArray.h>
#include <maya/MFloatVector.h>
#include <maya/MGLFunctionTable.h>
#include <maya/MGlobal.h>
#include <maya/MIntArray.h>
#include <maya/MIOStream.h>
#include <maya/MMatrix.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MPointArray.h>
#include <maya/MStringArray.h>
#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MSceneMessage.h>
#include <maya/MSelectionList.h>
#include <maya/MTypeId.h>
#include <maya/MTypes.h>
#include <maya/MTime.h>
#include <maya/MVectorArray.h>
#include <maya/MVector.h>
#include <maya/M3dView.h>

#include <maya/MFnUnitAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnStringData.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnArrayAttrsData.h>
#include <maya/MFnNumericAttribute.h>

#include <maya/MPxSurfaceShape.h>
#include <maya/MPxNode.h>
#include <maya/MPxSurfaceShape.h>
#include <maya/MPxSurfaceShapeUI.h>

#include <Partio.h>
#include <PartioAttribute.h>
#include <PartioIterator.h>

#include "partio4MayaShared.h"
#include "iconArrays.h"

class partioInstReaderCache
{
public:
    partioInstReaderCache();
    MBoundingBox bbox;
    int dList;
    Partio::ParticlesDataMutable* particles;
    Partio::ParticleAttribute positionAttr;
    Partio::ParticleAttribute idAttr;
    Partio::ParticleAttribute velocityAttr;
    Partio::ParticleAttribute rotationAttr;
	Partio::ParticleAttribute aimDirAttr;
	Partio::ParticleAttribute aimPosAttr;
	Partio::ParticleAttribute aimAxisAttr;
	Partio::ParticleAttribute aimUpAttr;
	Partio::ParticleAttribute aimWorldUpAttr;
	Partio::ParticleAttribute lastRotationAttr;
    Partio::ParticleAttribute scaleAttr;
	Partio::ParticleAttribute lastScaleAttr;
	Partio::ParticleAttribute lastAimDirAttr;
	Partio::ParticleAttribute lastAimPosAttr;
    Partio::ParticleAttribute indexAttr;
//    Partio::ParticleAttribute shaderIndexAttr;
    float* flipPos;
    MFnArrayAttrsData instanceData;
    MObject instanceDataObj;


};


class partioInstancerUI : public MPxSurfaceShapeUI
{
public:

    partioInstancerUI();
    virtual ~partioInstancerUI();
    virtual void draw(const MDrawRequest & request, M3dView & view) const;
    virtual void getDrawRequests(const MDrawInfo & info, bool objectAndActiveOnly, MDrawRequestQueue & requests);
    void 	drawBoundingBox() const;
    void 	drawPartio(partioInstReaderCache* pvCache, int drawStyle, M3dView &view) const;
    static void * creator();
    virtual bool	select( MSelectInfo &selectInfo,
                         MSelectionList &selectionList,
                         MPointArray &worldSpaceSelectPts ) const;

};

class partioInstancer : public MPxSurfaceShape
{
public:
    partioInstancer();
    virtual ~partioInstancer();

    virtual MStatus   		compute( const MPlug& plug, MDataBlock& block );
    virtual bool            isBounded() const;
    virtual MBoundingBox    boundingBox() const;
    static  void *          creator();
    static  MStatus         initialize();
    static void 			reInit(void *data);
    void 					initCallback();
    virtual void 			postConstructor();

    bool GetPlugData();
    void addParticleAttr(int attrIndex, MString attrName );
    partioInstReaderCache* updateParticleCache();

	void updateInstanceDataVector ( partioInstReaderCache &pvCache, MVectorArray &arrayToCheck, MString arrayChannel );
	void updateInstanceDataDouble ( partioInstReaderCache &pvCache, MDoubleArray &arrayToCheck, MString arrayChannel );


    MCallbackId partioInstancerOpenCallback;
    MCallbackId partioInstancerImportCallback;
    MCallbackId partioInstancerReferenceCallback;


/// ATTRS
    static MObject  time;
	static MObject  aByFrame;
    static MObject  aSize;         // The size of the logo
    static MObject  aFlipYZ;
	static MObject  aDrawStyle;
	static MObject  aPointSize;

/// Cache file related stuff
    static MObject 	aUpdateCache;
    static MObject 	aCacheDir;
    static MObject 	aCacheFile;
	static MObject 	aCacheActive;
    static MObject 	aCacheOffset;
    static MObject  aCacheStatic;
    static MObject 	aCacheFormat;
	static MObject  aForceReload;
	static MObject  aRenderCachePath;

/// point position / velocity
	static MObject  aComputeVeloPos;
	static MObject  aVeloMult;

/// attributes
    static MObject 	aPartioAttributes;
	static MObject	aScaleFrom;
	static MObject  aRotationType;

    static MObject	aRotationFrom;
	static MObject  aAimDirectionFrom;
	static MObject  aAimPositionFrom;
	static MObject  aAimAxisFrom;
	static MObject  aAimUpAxisFrom;
	static MObject  aAimWorldUpFrom;

	static MObject	aLastScaleFrom;
	static MObject	aLastRotationFrom;
	static MObject  aLastAimDirectionFrom;
	static MObject  aLastAimPositionFrom;

    static MObject	aIndexFrom;

/// not implemented yet
//	static MObject 	aJitterPos;
//  static MObject 	aJitterFreq;
//	static MObject	aShaderIndexFrom;
//	static MObject	aInMeshInstances;
//	static MObject	aOutMesh;

//  output data to instancer
    static MObject	aInstanceData;


    static	MTypeId			id;
    float 					multiplier;
    bool 					cacheChanged;
    partioInstReaderCache  	pvCache;
	int  drawError;


private:

    MString mLastFileLoaded;
    MString mLastPath;
    MString mLastFile;
    MString mLastExt;
    bool mLastFlipStatus;
    bool mFlipped;
    bool  frameChanged;
    MStringArray attributeList;
	int mLastRotationTypeIndex;
    int mLastRotationFromIndex;
	int mLastLastRotationFromIndex;
	int mLastAimDirectionFromIndex;
	int mLastLastAimDirecitonFromIndex;
	int mLastAimPositionFromIndex;
	int mLastLastAimPositionFromIndex;
	int mLastAimAxisFromIndex;
	int mLastAimUpAxisFromIndex;
	int mLastAimWorldUpFromIndex;
    int mLastScaleFromIndex;
	int mLastLastScaleFromIndex;
    int mLastIndexFromIndex;
    bool canMotionBlur;
	//    int mLastShaderIndexFromIndex;

protected:

    int dUpdate;
    GLuint dList;

};

#endif
