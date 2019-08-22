/****************************************************************************** 
Copyright (c) 2008-2017, Teledyne DALSA Inc.
All rights reserved.

File: gev_legacy_api.h
	Public API for GigEVision C library (legacy definitions)

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions 
are met:
	-Redistributions of source code must retain the above copyright 
	notice, this list of conditions and the following disclaimer. 
	-Redistributions in binary form must reproduce the above 
	copyright notice, this list of conditions and the following 
	disclaimer in the documentation and/or other materials provided 
	with the distribution. 
	-Neither the name of Teledyne DALSA nor the names of its contributors 
	may be used to endorse or promote products derived 
	from this software without specific prior written permission. 

===============================================================================
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

/*! \file gev_legacy_api.h
\brief GEV API Legacy definitions.

*/

#ifndef _GEV_LEGACY_API_H_
#define _GEV_LEGACY_API_H_
	
#ifdef __cplusplus
extern "C" {
#endif

//=================================================================================================
// GEV_REGISTER definitions 
// (static camera register definitions for tables in cameraregdata.c)
//
//
//====================================================================
// GEV API Camera identification information (internal)
//====================================================================
typedef enum
{
	cameraUnknown     = 0,
	cameraGenieMono   = 1,
	cameraGenieColor  = 2,
	cameraGenieHM     = 3,
	cameraDracoBased  = 4,
	cameraSpyder3SG114K = 5,
	cameraSpyder3SG111K = 6,
	cameraSpyder3SG144K = 7,
	cameraSpyder3SG324K = 8,
	cameraSpyder3SG344K = 9,
	cameraGenieTS = 10,
	cameraXMLBased = 11,
} cameraType;

//=============================================================================
// Camera basic "feature" access. 
// (Complex features use the full feature access methods (not implemented yet)).
//=============================================================================

#define FEATURE_NAME_MAX_SIZE 64
#define NOREF_ADDR 				0	// No reference address - feature has no backing register.

typedef void* PGENICAM_FEATURE;	// Place holder until feature methods are complete.

typedef enum
{
	RO = 1,
	WO = 2,
	RW = 3
} RegAccess;

typedef enum
{
	stringReg,
	floatReg,		/* Big-endian register - float 			*/
	integerReg,		/* Big-endian register - integer 		*/
	bitReg,			/* Big-endian register - bit 				*/
	fixedVal,		/* Reg has one value - access is a trigger    */
	intVal,			/* Locally stored value - no backing register */
	floatVal,
	dataArea,		/* Application must handle endian-ness of a data area */
	floatRegLE,		/* Little-endian register - float 		*/
	integerRegLE,	/* Little-endian register - integer 	*/
	bitRegLE		/* Little-endian register - bit 			*/
} RegType;

typedef struct
{
	char   name[FEATURE_NAME_MAX_SIZE];	
	UINT32 value;
} ENUM_ENTRY, *PENUM_ENTRY;

typedef union
{
		UINT32			bitIndex;	// Bit number if a Bit
		UINT32			intValue;
		float				floatValue;
} GENIREG_VALUE;

//typedef UINT32 BOOL32;

typedef struct
{
	char					featureName[FEATURE_NAME_MAX_SIZE];   // String name of feature for this register.
	UINT32				address;		 // Address for accessing feature in camera (NOREF_ADDR if not in camera).
	RegAccess			accessMode;	 // RO, WO, RW access allowed.
	BOOL32				available;	 // True if feature is available (in camera or not) - False is not available.
	RegType				type;			 // String, Float, Integer, Enum, Bit
	UINT32				regSize;		 // Size of storage for register (or register set / area).
	UINT32				regStride;   // Increment between register items accessed via selector
	UINT32				minSelector; // Minimum value for selector (corresponds to base address).
	UINT32				maxSelector; // Maximum value for selector.
	GENIREG_VALUE		value;       // Current value (storage for features not backed by a register).
	GENIREG_VALUE		minValue;    // Minimum allowable value.
	GENIREG_VALUE		maxValue;    // Maximum allowable value.
	UINT32				readMask;	 // AND Mask for read (integers only)
	UINT32				writeMask;	 // AND Mask for write (integers only)
	PGENICAM_FEATURE	feature;		 // Pointer to feature in feature table (future).
	char					selectorName[FEATURE_NAME_MAX_SIZE];  // String name of selector for feature.
	char					indexName[FEATURE_NAME_MAX_SIZE];     // String name of index (second selector)for feature.
} GEV_REGISTER, *PGEV_REGISTER;



//=======================================================================================
// Registers for GigE Cameras (includes DALSA-specific registers and GeniCam standard registers).
// Only actual registers are here - not generic "features".
//=======================================================================================

typedef struct
{
	GEV_REGISTER DeviceVendorName;
	GEV_REGISTER DeviceModelName; 
	GEV_REGISTER DeviceVersion;
	GEV_REGISTER DeviceFirmwareVersion;
	GEV_REGISTER DeviceID;
	GEV_REGISTER DeviceUserID;
	GEV_REGISTER DeviceScanType;
	GEV_REGISTER DeviceMaxThroughput; 
	GEV_REGISTER DeviceRegistersStreamingStart; 
	GEV_REGISTER DeviceRegistersStreamingEnd; 
	GEV_REGISTER DeviceRegistersCheck; 
	GEV_REGISTER DeviceRegistersValid;

	GEV_REGISTER SensorWidth;	
	GEV_REGISTER SensorHeight;	
	GEV_REGISTER WidthMax;	
	GEV_REGISTER HeightMax;	 
	GEV_REGISTER Width;	 
	GEV_REGISTER Height;	 
	GEV_REGISTER OffsetX;	
	GEV_REGISTER OffsetY; 
	GEV_REGISTER LinePitch;	 
	GEV_REGISTER BinningHorizontal;	 
	GEV_REGISTER BinningVertical;	
	GEV_REGISTER DecimationHorizontal;
	GEV_REGISTER DecimationVertical;
	GEV_REGISTER ReverseX;	 
	GEV_REGISTER ReverseY;	 
	GEV_REGISTER PixelColorFilter;	 
	GEV_REGISTER PixelCoding;	 
	GEV_REGISTER PixelSize;	 
	GEV_REGISTER PixelFormat;	 
	GEV_REGISTER PixelDynamicRangeMin;	
	GEV_REGISTER PixelDynamicRangeMax;	
	GEV_REGISTER TestImageSelector;
	
	GEV_REGISTER AcquisitionMode;	
	GEV_REGISTER AcquisitionStart;	
	GEV_REGISTER AcquisitionStop;	 
	GEV_REGISTER AcquisitionAbort;	
	GEV_REGISTER AcquisitionArm;
	GEV_REGISTER AcquisitionFrameCount;	
	GEV_REGISTER AcquisitionFrameRateMax;
	GEV_REGISTER AcquisitionFrameRateMin;
	GEV_REGISTER AcquisitionFrameRateRaw;	
	GEV_REGISTER AcquisitionFrameRateAbs;	 
	GEV_REGISTER AcquisitionLineRateRaw;
	GEV_REGISTER AcquisitionLineRateAbs;	 
	GEV_REGISTER AcquisitionStatusSelector;
	GEV_REGISTER AcquisitionStatus;	 
	GEV_REGISTER TriggerSelector;	
	GEV_REGISTER TriggerMode;	
	GEV_REGISTER TriggerSoftware;	
	GEV_REGISTER TriggerSource;	
	GEV_REGISTER TriggerActivation;	 
	GEV_REGISTER TriggerOverlap;
	GEV_REGISTER TriggerDelayAbs;	
	GEV_REGISTER TriggerDelayRaw;	
	GEV_REGISTER TriggerDivider;
	GEV_REGISTER TriggerMultiplier;
	GEV_REGISTER ExposureMode;	
	GEV_REGISTER ExposureAlignment;	// (*) in Genie but not in standard	
	GEV_REGISTER ExposureDelay;		// (*) in Genie but not in standard	
	GEV_REGISTER ExposureTimeRaw;	 
	GEV_REGISTER ExposureTimeAbs;
	GEV_REGISTER ExposureAuto;	
	GEV_REGISTER ExposureTimeMin;	  // (*) in Genie but not in standard	
	GEV_REGISTER ExposureTimeMax;	  // (*) in Genie but not in standard		

	GEV_REGISTER LineSelector;
	GEV_REGISTER LineMode;
	GEV_REGISTER LineInverter;
	GEV_REGISTER LineStatus;
	GEV_REGISTER LineStatusAll;
	GEV_REGISTER LineSource;
	GEV_REGISTER OutputLineEventSource;			// Genie version of "LineSource" 
	GEV_REGISTER LineFormat;
	GEV_REGISTER UserOutputValue;
	GEV_REGISTER OutputLineValue;					// Genie Version of "UserOutputValue"
	GEV_REGISTER UserOutputSelector;
	GEV_REGISTER UserOutputValueAll;
	GEV_REGISTER UserOutputValueAllMask;

	GEV_REGISTER InputLinePolarity;				// (*) in Genie but not in standard	
	GEV_REGISTER InputLineDebouncingPeriod;	// (*) in Genie but not in standard	
	GEV_REGISTER OutputLinePulsePolarity;		// (*) in Genie but not in standard	
	GEV_REGISTER OutputLineMode;					// (*) in Genie but not in standard	
	GEV_REGISTER OutputLinePulseDelay;			// (*) in Genie but not in standard		 
	GEV_REGISTER OutputLinePulseDuration;		// (*) in Genie but not in standard	

	GEV_REGISTER CounterSelector;	
	GEV_REGISTER CounterEventSource;	
	GEV_REGISTER CounterLineSource;			// (*) in Genie but not in standard	
	GEV_REGISTER CounterReset;
	GEV_REGISTER CounterValue;					
	GEV_REGISTER CounterValueAtReset;		
	GEV_REGISTER CounterDuration;				
	GEV_REGISTER CounterStatus;				
	GEV_REGISTER CounterTriggerSource;		
	GEV_REGISTER CounterTriggerActivation;	
	GEV_REGISTER TimerSelector;				
	GEV_REGISTER TimerDurationAbs;			
	GEV_REGISTER TimerDurationRaw;			
	GEV_REGISTER TimerDelayAbs;				
	GEV_REGISTER TimerDelayRaw;				
	GEV_REGISTER TimerValueAbs;				
	GEV_REGISTER TimerValueRaw;				
	GEV_REGISTER TimerStatus;					
	GEV_REGISTER TimerTriggerSource;			
	GEV_REGISTER TimerTriggerActivation;	

	GEV_REGISTER EventSelector; 
	GEV_REGISTER EventNotification; 

	GEV_REGISTER GainSelector;	
	GEV_REGISTER GainRaw;	 
	GEV_REGISTER GainAbs;					
	GEV_REGISTER GainAuto;					
	GEV_REGISTER GainAutoBalance;			
	GEV_REGISTER BlackLevelSelector;	 
	GEV_REGISTER BlackLevelRaw;	 
	GEV_REGISTER BlackLevelAbs;			
	GEV_REGISTER BlackLevelAuto;			
	GEV_REGISTER BlackLevelAutoBalance;	
	GEV_REGISTER WhiteClipSelector;		
	GEV_REGISTER WhiteClipRaw;				
	GEV_REGISTER WhiteClipAbs;				
	GEV_REGISTER BalanceRatioSelector;	
	GEV_REGISTER BalanceRatioAbs;			
	GEV_REGISTER BalanceWhiteAuto;		
	GEV_REGISTER Gamma;						

	GEV_REGISTER LUTSelector;
	GEV_REGISTER LUTEnable;
	GEV_REGISTER LUTIndex;
	GEV_REGISTER LUTValue;
	GEV_REGISTER LUTValueAll;			

	GEV_REGISTER UserSetDefaultSelector; 
	GEV_REGISTER UserSetSelector;
	GEV_REGISTER UserSetLoad; 
	GEV_REGISTER UserSetSave; 

	//===================================================
	// Gev transport layer registers.
	//
	GEV_REGISTER PayloadSize; 
/*==================================================
	GEV_REGISTER GevVersionMajor; 
	GEV_REGISTER GevVersionMinor; 
	GEV_REGISTER GevDeviceModeIsBigEndian; 
	GEV_REGISTER GevDeviceModeCharacterSet; 
	GEV_REGISTER GevInterfaceSelector; 
	GEV_REGISTER GevMACAddress; 
==================================================*/
	GEV_REGISTER GevSupportedIPConfigurationLLA; 
	GEV_REGISTER GevSupportedIPConfigurationDHCP; 
	GEV_REGISTER GevSupportedIPConfigurationPersistentIP; 


	GEV_REGISTER GevCurrentIPConfigurationLLA;				// (**)not in Genie
	GEV_REGISTER GevCurrentIPConfigurationDHCP;				// (**)not in Genie 
	GEV_REGISTER GevCurrentIPConfigurationPersistentIP;	// (**)not in Genie 

	GEV_REGISTER GevCurrentIPConfiguration; 
	GEV_REGISTER GevCurrentIPAddress; 
	GEV_REGISTER GevCurrentSubnetMask; 
	GEV_REGISTER GevCurrentDefaultGateway; 
	GEV_REGISTER GevPersistentIPAddress; 
	GEV_REGISTER GevPersistentSubnetMask; 
	GEV_REGISTER GevPersistentDefaultGateway; 
	GEV_REGISTER GevFirstURL; 
	GEV_REGISTER GevSecondURL; 
	GEV_REGISTER GevNumberOfInterfaces;
/*================================================== 
	GEV_REGISTER GevMessageChannelCount; 
	GEV_REGISTER GevStreamChannelCount;
	GEV_REGISTER GevSupportedOptionalCommandsUserDefinedName; 
	GEV_REGISTER GevSupportedOptionalCommandsSerialNumber; 
	GEV_REGISTER GevSupportedOptionalCommandsEVENTDATA; 
	GEV_REGISTER GevSupportedOptionalCommandsEVENT; 
	GEV_REGISTER GevSupportedOptionalCommandsPACKETRESEND; 
	GEV_REGISTER GevSupportedOptionalCommandsWRITEMEM; 
	GEV_REGISTER GevSupportedOptionalCommandsConcatenation; 
	GEV_REGISTER GevHeartbeatTimeout; 
	GEV_REGISTER GevTimestampTickFrequency; 
	GEV_REGISTER GevTimestampControlLatch; 
	GEV_REGISTER GevTimestampControlReset; 
	GEV_REGISTER GevTimestampValue;

	GEV_REGISTER GevCCP;	
	GEV_REGISTER GevMCPHostPort;
	GEV_REGISTER GevMCDA;		
	GEV_REGISTER GevMCTT;	
	GEV_REGISTER GevMCRC;	

	GEV_REGISTER GevStreamChannelSelector; 
	GEV_REGISTER GevSCPInterfaceIndex;

	GEV_REGISTER GevSCPHostPort;	
	GEV_REGISTER GevSCPFireTestPacket;
	GEV_REGISTER GevSCPDoNotFragment;
	GEV_REGISTER GevSCPSBigEndian;	

	GEV_REGISTER GevSCPSPacketSize;
	GEV_REGISTER GevSCPD; 

	GEV_REGISTER GevSCDA;			
====================================*/

	GEV_REGISTER GevLinkSpeed;	
	GEV_REGISTER GevIPConfigurationStatus;

	//============================================
	//	Chunk data support (not in Genie)
	//
	GEV_REGISTER ChunkModeActive;			
	GEV_REGISTER ChunkSelector;			
	GEV_REGISTER ChunkEnable;				
	GEV_REGISTER ChunkOffsetX;				
	GEV_REGISTER ChunkOffsetY;				
	GEV_REGISTER ChunkWidth;				
	GEV_REGISTER ChunkHeight;				
	GEV_REGISTER ChunkPixelFormat;		
	GEV_REGISTER ChunkDynamicRangeMax;
	GEV_REGISTER ChunkDynamicRangeMin;	
	GEV_REGISTER ChunkTimestamp;			
	GEV_REGISTER ChunkLineStatusAll;		
	GEV_REGISTER ChunkCounterSelector;
	GEV_REGISTER ChunkCounter;				
	GEV_REGISTER ChunkTimerSelector;		
	GEV_REGISTER ChunkTimer;				

	//============================================
	// File Access support (not in Genie)
	//
	GEV_REGISTER FileSelector;				
	GEV_REGISTER FileOperationSelector;	
	GEV_REGISTER FileOperationExecute;	
	GEV_REGISTER FileOpenModeSelector;	
	GEV_REGISTER FileAccessOffset;		
	GEV_REGISTER FileAccessLength;		
	GEV_REGISTER FileAccessBuffer;		
	GEV_REGISTER FileOperationStatus;	
	GEV_REGISTER FileOperationResult;	
	GEV_REGISTER FileSize;

} DALSA_GENICAM_GIGE_REGS;


//=================================================================================================
// GEV_REGISTER API : Camera register access. (Standard features implemented as simple static register structures)
// (Extensible / camera-device specific).
//
// Obtaining camera register structures.

GEV_STATUS GevGetCameraRegisters( GEV_CAMERA_HANDLE handle, DALSA_GENICAM_GIGE_REGS *camera_registers, int size);
GEV_STATUS GevSetCameraRegInfo( GEV_CAMERA_HANDLE handle, cameraType type, BOOL fSupportedDalsaCamera, 
							DALSA_GENICAM_GIGE_REGS *camera_registers, int size);
GEV_STATUS GevInitCameraRegisters( GEV_CAMERA_HANDLE handle);

GEV_STATUS GevGetNumberOfRegisters(GEV_CAMERA_HANDLE handle, UINT32 *pNumReg);
GEV_STATUS GevGetRegisterNameByIndex(GEV_CAMERA_HANDLE handle, UINT32 index, int size, char *name);
GEV_STATUS GevGetRegisterByName(GEV_CAMERA_HANDLE handle, char *name, GEV_REGISTER *pReg);
GEV_STATUS GevGetRegisterPtrByName(GEV_CAMERA_HANDLE handle, char *name, GEV_REGISTER **pReg);
GEV_STATUS GevGetRegisterByIndex(GEV_CAMERA_HANDLE handle, UINT32 index, GEV_REGISTER *pReg);
GEV_STATUS GevGetRegisterPtrByIndex(GEV_CAMERA_HANDLE handle, UINT32 index, GEV_REGISTER **pReg);

GEV_STATUS GevReadRegisterByName( GEV_CAMERA_HANDLE handle, char *name, int selector, UINT32 size, void *value);
GEV_STATUS GevWriteRegisterByName( GEV_CAMERA_HANDLE handle, char *name, int selector, UINT32 size, void *value);

GEV_STATUS GevRegisterRead(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 size, void *data);
GEV_STATUS GevRegisterWrite(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 size, void *data);
GEV_STATUS GevRegisterWriteNoWait(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 size, void *data);

GEV_STATUS GevRegisterWriteArray(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 array_offset, UINT32 num_entries, void *data);
GEV_STATUS GevRegisterReadArray(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 array_offset, UINT32 num_entries, void *data);

GEV_STATUS GevRegisterWriteInt(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 value);
GEV_STATUS GevRegisterReadInt(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, UINT32 *value);
GEV_STATUS GevRegisterWriteFloat(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, float value);
GEV_STATUS GevRegisterReadFloat(GEV_CAMERA_HANDLE handle, GEV_REGISTER *pReg, int selector, float *value);

#ifdef __cplusplus
}
#endif



#endif
