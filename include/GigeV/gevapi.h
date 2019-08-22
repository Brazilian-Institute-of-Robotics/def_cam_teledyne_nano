/****************************************************************************** 
Copyright (c) 2008-2017, Teledyne DALSA Inc.
All rights reserved.

File: gevapi.h
	Public API for GigEVision C library.

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

/*! \file gevapi.h
\brief GEV API definitions.

*/

#ifndef _GEVAPI_H_
#define _GEVAPI_H_
			
// Environment variable for expected GenICam version.
// This gets updated with the targetted GenICam version supported.
#define GENICAM_TARGET_ROOT_VERSION 	"GENICAM_ROOT_V3_0"
#define GIGEV_XML_DOWNLOAD  "GIGEV_XML_DOWNLOAD"

#ifdef __cplusplus
extern "C" {
#endif

//====================================================================
// INCLUDES
//====================================================================

#include "gevoslib.h"		   //!< OS-specific definitions
#include "dynaqueue.h"

#define GEVAPI_DEFS
//====================================================================
// CONSTANTS
//====================================================================

#define GEV_LOG_LEVEL_OFF			0
#define GEV_LOG_LEVEL_NORMAL		1
#define GEV_LOG_LEVEL_ERRORS		1
#define GEV_LOG_LEVEL_WARNINGS	2
#define GEV_LOG_LEVEL_DEBUG	3
#define GEV_LOG_LEVEL_TRACE	4

#define GEV_LOG_FATAL	0
#define GEV_LOG_ERROR	1
#define GEV_LOG_WARNING 2
#define GEV_LOG_INFO	3
#define GEV_LOG_DEBUG	3
#define GEV_LOG_TRACE	4

//=========================================================================
// GevLib error codes. 
//=========================================================================
#define GEVLIB_OK					   	         0
#define GEVLIB_SUCCESS					   	   GEVLIB_OK
#define GEVLIB_STATUS_SUCCESS			   	   GEVLIB_OK
#define GEVLIB_STATUS_ERROR			   	   -1

//standard api errors
#define GEVLIB_ERROR_GENERIC                 -1    // Generic Error. A catch-all for unexpected behaviour.
#define GEVLIB_ERROR_NULL_PTR                -2    // NULL pointer passed to function or the result of a cast operation
#define GEVLIB_ERROR_ARG_INVALID			   	-3		// Passed argument to a function is not valid            												   
#define GEVLIB_ERROR_INVALID_HANDLE	         -4		// Invalid Handle
#define GEVLIB_ERROR_NOT_SUPPORTED           -5    // This version of hardware/fpga does not support this feature
#define GEVLIB_ERROR_TIME_OUT                -6    // Timed out waiting for a resource
#define GEVLIB_ERROR_NOT_IMPLEMENTED         -10   // Function / feature is not implemented.
#define GEVLIB_ERROR_NO_CAMERA			      -11   // The action can't be execute because the camera is not connected.
#define GEVLIB_ERROR_INVALID_PIXEL_FORMAT    -12   // Pixel Format is invalid (not supported or not recognized)
#define GEVLIB_ERROR_PARAMETER_INVALID       -13   // Passed Parameter (could be inside a data structure) is invalid/out of range.
#define GEVLIB_ERROR_SOFTWARE                -14   // software error, unexpected result
#define GEVLIB_ERROR_API_NOT_INITIALIZED     -15   // API has not been initialized
#define GEVLIB_ERROR_DEVICE_NOT_FOUND	      -16   // Device/camera specified was not found.
#define GEVLIB_ERROR_ACCESS_DENIED		      -17   // API will not access the device/camera/feature in the specified manner.
#define GEVLIB_ERROR_NOT_AVAILABLE		      -18   // Feature / function is not available for access (but is implemented).
#define GEVLIB_ERROR_NO_SPACE    		      -19   // The data being written to a feature is too large for the feature to store.
#define GEVLIB_ERROR_XFER_NOT_INITIALIZED    -20   // Payload transfer is not initialized but is required to be for this function.
#define GEVLIB_ERROR_XFER_ACTIVE             -21   // Payload transfer is active but is required to be inactive for this function.
#define GEVLIB_ERROR_XFER_NOT_ACTIVE         -22   // Payload transfer is not active but is required to be active for this function.

// Resource errors.
#define GEVLIB_ERROR_SYSTEM_RESOURCE          -2001 // Error creating a system resource
#define GEVLIB_ERROR_INSUFFICIENT_MEMORY	    -2002 // error allocating memory
#define GEVLIB_ERROR_INSUFFICIENT_BANDWIDTH   -2003 // Not enough bandwidth to perform operation and/or acquisition
#define GEVLIB_ERROR_RESOURCE_NOT_ALLOCATED   -2004 // Resource is not currently allocated
#define GEVLIB_ERROR_RESOURCE_IN_USE          -2005 // resource is currently being used.
#define GEVLIB_ERROR_RESOURCE_NOT_ENABLED     -2006 // The resource(feature) is not enabled
#define GEVLIB_ERROR_RESOURCE_NOT_INITIALIZED -2007 // Resource has not been intialized.
#define GEVLIB_ERROR_RESOURCE_CORRUPTED       -2008 // Resource has been corrupted
#define GEVLIB_ERROR_RESOURCE_MISSING         -2009 // A resource (ie.DLL) needed could not located
#define GEVLIB_ERROR_RESOURCE_LACK            -2010 // Lack of resource to perform a request.
#define GEVLIB_ERROR_RESOURCE_ACCESS          -2011 // Unable to correctly access the resource.
#define GEVLIB_ERROR_RESOURCE_INVALID         -2012 // A specified resource does not exist.
#define GEVLIB_ERROR_RESOURCE_LOCK            -2013 // resource is currently lock
#define GEVLIB_ERROR_INSUFFICIENT_PRIVILEGE   -2014 // Need administrator privilege.
#define GEVLIB_ERROR_RESOURCE_WRITE_PROTECTED -2015 // No data can be written to the resource
#define GEVLIB_ERROR_RESOURCE_INCOHERENCY     -2016 // The required resources are not valid together

// Data errors
#define GEVLIB_ERROR_DATA_NO_MESSAGES	      -5001	// no more messages (in fifo, queue, input stream etc)
#define GEVLIB_ERROR_DATA_OVERFLOW           -5002 // data could not be added to fifo, queue, stream etc.
#define GEVLIB_ERROR_DATA_CHECKSUM           -5003 // checksum validation fail
#define GEVLIB_ERROR_DATA_NOT_AVAILABLE		-5004 // data requested isn't available yet
#define GEVLIB_ERROR_DATA_OVERRUN				-5005 // data requested has been overrun by newer data
#define GEVLIB_ERROR_DATA_XFER_ABORT			-5006 // transfer of requested data did not finish
#define GEVLIB_ERROR_DATA_INVALID_HEADER     -5007 // data header is invalid.
#define GEVLIB_ERROR_DATA_ALIGNMENT          -5008 // data is not correctly align.

// Ethernet errors
#define GEVLIB_ERROR_CONNECTION_DROPPED          -11000
#define GEVLIB_ERROR_ANSWER_TIMEOUT              -11001
#define GEVLIB_ERROR_SOCKET_INVALID              -11002
#define GEVLIB_ERROR_PORT_NOT_AVAILABLE          -11003
#define GEVLIB_ERROR_INVALID_IP                  -11004
#define GEVLIB_ERROR_INVALID_CAMERA_OPERATION    -11005
#define GEVLIB_ERROR_INVALID_PACKET              -11006
#define GEVLIB_ERROR_INVALID_CONNECTION_ATTEMPT  -11007
#define GEVLIB_ERROR_PROTOCOL                    -11008
#define GEVLIB_ERROR_WINDOWS_SOCKET_INIT         -11009
#define GEVLIB_ERROR_WINDOWS_SOCKET_CLOSE        -11010
#define GEVLIB_ERROR_SOCKET_CREATE               -11011
#define GEVLIB_ERROR_SOCKET_RELEASE              -11012
#define GEVLIB_ERROR_SOCKET_DATA_SEND            -11013
#define GEVLIB_ERROR_SOCKET_DATA_READ            -11014
#define GEVLIB_ERROR_SOCKET_WAIT_ACKNOWLEDGE     -11015
#define GEVLIB_ERROR_INVALID_INTERNAL_COMMAND    -11016
#define GEVLIB_ERROR_INVALID_ACKNOWLEDGE         -11017
#define GEVLIB_ERROR_PREVIOUS_ACKNOWLEDGE        -11018
#define GEVLIB_ERROR_INVALID_MESSAGE             -11019
#define GEVLIB_ERROR_GIGE_ERROR                  -11020


//===================================================
// Device Level Status Codes (From low-level library)

#define GEV_STATUS_SUCCESS					0x0000		//!< Requested operation was completed successfully.
#define GEV_STATUS_NOT_IMPLEMENTED		0x8001		//!< The request isn't supported by the device.
#define GEV_STATUS_INVALID_PARAMETER	0x8002		//!< At least one parameter provided in the command is invalid (or out of range) for the device
#define GEV_STATUS_INVALID_ADDRESS		0x8003		//!< An attempt was made to access a non existent address space location.
#define GEV_STATUS_WRITE_PROTECT			0x8004		//!< The addressed register must not be written.
#define GEV_STATUS_BAD_ALIGNMENT			0x8005		//!< A badly aligned address offset or data size was specified.
#define GEV_STATUS_ACCESS_DENIED			0x8006		//!< An attempt was made to access an address location which is currently/momentary not accessible.
#define GEV_STATUS_BUSY						0x8007		//!< A required resource to service the request isn't currently available. The request may be retried.
#define GEV_STATUS_LOCAL_PROBLEM			0x8008		//!< A internal problem in the device implementation occurred while processing the request. 
#define GEV_STATUS_MSG_MISMATCH			0x8009		//!< Message mismatch (request and acknowledge don't match)
#define GEV_STATUS_INVALID_PROTOCOL		0x800A		//!< This version of the GVCP protocol is not supported
#define GEV_STATUS_NO_MSG					0x800B		//!< No message received, timeout.
#define GEV_STATUS_PACKET_UNAVAILABLE	0x800C		//!< The request packet is not available anymore.
#define GEV_STATUS_DATA_OVERRUN			0x800D		//!< Internal memory of device overrun (typically for image acquisition)
#define GEV_STATUS_INVALID_HEADER		0x800E		//!< The message header is not valid. Some of its fields do not match the specificiation.

#define GEV_STATUS_ERROR					0x8FFF		//!< Generic error. 

//=======================================
// Public Pixel Format Value Definitions
//=======================================

typedef enum
{
	fmtMono8         = 0x01080001,	/* 8 Bit Monochrome Unsigned    */
	fmtMono8Signed   = 0x01080002,	/* 8 Bit Monochrome Signed      */
	fmtMono10        = 0x01100003,	/* 10 Bit Monochrome Unsigned   */
	fmtMono10Packed  = 0x010C0004,	/* 10 Bit Monochrome Packed     */
	fmtMono12        = 0x01100005,	/* 12 Bit Monochrome Unsigned   */
	fmtMono12Packed  = 0x010C0006,	/* 12 Bit Monochrome Packed     */
	fmtMono14        = 0x01100025,	/* 14 Bit Monochrome Unsigned   */
	fmtMono16        = 0x01100007,	/* 16 Bit Monochrome Unsigned   */
	fmtBayerGR8        = 0x01080008,	/*  8-bit Bayer GR        */
	fmtBayerRG8        = 0x01080009,	/*  8-bit Bayer RG        */
	fmtBayerGB8        = 0x0108000A,	/*  8-bit Bayer GB        */
	fmtBayerBG8        = 0x0108000B,	/*  8-bit Bayer BG        */
	fmtBayerGR10       = 0x0110000C,	/* 10-bit Bayer GR        */
	fmtBayerRG10       = 0x0110000D,	/* 10-bit Bayer RG        */
	fmtBayerGB10       = 0x0110000E,	/* 10-bit Bayer GB        */
	fmtBayerBG10       = 0x0110000F,	/* 10-bit Bayer BG        */
	fmtBayerGR10Packed = 0x010C0026,  /* 10-bit Bayer GR packed */
	fmtBayerRG10Packed = 0x010C0027,  /* 10-bit Bayer RG packed */
	fmtBayerGB10Packed = 0x010C0028,  /* 10-bit Bayer GB packed */
	fmtBayerBG10Packed = 0x010C0029,  /* 10-bit Bayer BG packed */
	fmtBayerGR12       = 0x01100010,	/* 12-bit Bayer GR        */
	fmtBayerRG12       = 0x01100011,	/* 12-bit Bayer RG        */
	fmtBayerGB12       = 0x01100012,	/* 12-bit Bayer GB        */
	fmtBayerBG12       = 0x01100013,	/* 12-bit Bayer BG        */
	fmtBayerGR12Packed = 0x010C002A,  /* 12-bit Bayer GR packed */
	fmtBayerRG12Packed = 0x010C002B,  /* 12-bit Bayer RG packed */
	fmtBayerGB12Packed = 0x010C002C,  /* 12-bit Bayer GB packed */
	fmtBayerBG12Packed = 0x010C002D,  /* 12-bit Bayer BG packed */
	fmtRGB8Packed      = 0x02180014,	/* 8 Bit RGB Unsigned in 24bits */
	fmtBGR8Packed      = 0x02180015,	/* 8 Bit BGR Unsigned in 24bits */
	fmtRGBA8Packed     = 0x02200016,	/* 8 Bit RGB Unsigned           */
	fmtBGRA8Packed     = 0x02200017,	/* 8 Bit BGR Unsigned           */
	fmtRGB10Packed     = 0x02300018,	/* 10 Bit RGB Unsigned          */
	fmtBGR10Packed     = 0x02300019,	/* 10 Bit BGR Unsigned          */
	fmtRGB12Packed     = 0x0230001A,	/* 12 Bit RGB Unsigned          */
	fmtBGR12Packed     = 0x0230001B,	/* 12 Bit BGR Unsigned          */
	fmtRGB14Packed     = 0x0230005E,	/* 14 Bit RGB Unsigned          */
	fmtBGR14Packed     = 0x0230004A,	/* 14 Bit BGR Unsigned          */
	fmtRGB16Packed     = 0x02300033,	/* 16 Bit RGB Unsigned          */
	fmtBGR16Packed     = 0x0230004B,	/* 16 Bit BGR Unsigned          */
	fmtRGBA16Packed    = 0x02400064,	/* 16 Bit RGBA Unsigned         */
	fmtBGRA16Packed    = 0x02400051,	/* 16 Bit BGRA Unsigned         */
	fmtRGB10V1Packed   = 0x0220001C,	/* 10 Bit RGB custom V1 (32bits)*/
	fmtRGB10V2Packed   = 0x0220001D,	/* 10 Bit RGB custom V2 (32bits)*/
	fmtYUV411packed    = 0x020C001E,	/* YUV411 (composite color) */
	fmtYUV422packed    = 0x0210001F,	/* YUV422 (composite color) */
	fmtYUV444packed    = 0x02180020,	/* YUV444 (composite color) */
	fmt_PFNC_YUV422_8  = 0x02100032,	/* YUV 4:2:2 8-bit */
	fmtRGB8Planar      = 0x02180021,	/* RGB8 Planar buffers      */
	fmtRGB10Planar     = 0x02300022,	/* RGB10 Planar buffers     */
	fmtRGB12Planar     = 0x02300023,	/* RGB12 Planar buffers     */
	fmtRGB16Planar     = 0x02300024,	/* RGB16 Planar buffers     */
	fmt_PFNC_BiColorBGRG8   = 0x021000A6, /* Bi-color Blue/Green - Red/Green 8-bit */
	fmt_PFNC_BiColorBGRG10  = 0x022000A9, /* Bi-color Blue/Green - Red/Green 10-bit unpacked */
	fmt_PFNC_BiColorBGRG10p = 0x021400AA, /* Bi-color Blue/Green - Red/Green 10-bit packed */
	fmt_PFNC_BiColorBGRG12  = 0x022000AD, /* Bi-color Blue/Green - Red/Green 12-bit unpacked */
	fmt_PFNC_BiColorBGRG12p = 0x021800AE, /* Bi-color Blue/Green - Red/Green 12-bit packed */
	fmt_PFNC_BiColorRGBG8   = 0x021000A5, /* Bi-color Red/Green - Blue/Green 8-bit */
	fmt_PFNC_BiColorRGBG10  = 0x022000A7, /* Bi-color Red/Green - Blue/Green 10-bit unpacked */
	fmt_PFNC_BiColorRGBG10p = 0x021400A8, /* Bi-color Red/Green - Blue/Green 10-bit packed */
	fmt_PFNC_BiColorRGBG12  = 0x022000AB, /* Bi-color Red/Green - Blue/Green 12-bit unpacked */
	fmt_PFNC_BiColorRGBG12p = 0x021800AC /* Bi-color Red/Green - Blue/Green 12-bit packed */
} enumGevPixelFormat;

#define GEV_PIXFORMAT_ISMONO		0x01000000
#define GEV_PIXFORMAT_ISCOLOR		0x02000000
#define GEV_PIXFORMAT_ISCUSTOM	0x80000000

#define GEV_PIXEL_FORMAT_MONO				0x0001		//!< Monochrome - each pixel aligned on a byte boundary.
#define GEV_PIXEL_FORMAT_MONO_PACKED	0x0002		//!< Monochrome - pixels packed end to end in memory.	
#define GEV_PIXEL_FORMAT_RGB				0x0004		//!< RGB - each pixel color aligned on a byte boundary.
#define GEV_PIXEL_FORMAT_RGB_PACKED		0x0008		//!< RGB - pixel colors packed end to end in memory.
#define GEV_PIXEL_FORMAT_BAYER			0x0010		//!< RGB - Bayer filter output.	
#define GEV_PIXEL_FORMAT_YUV				0x0020		//!< RGB - Packed YUV.	
#define GEV_PIXEL_FORMAT_RGB_PLANAR		0x0040		//!< RGB - Planar (each color plane in its own memory region).

#define GEV_PIXEL_ORDER_NONE		0x0000				//!< A "don't care" or "not applicable" value.
#define GEV_PIXEL_ORDER_RGB		0x0001				//!< Pixels are RGB (or RG for Bayer).
#define GEV_PIXEL_ORDER_BGR		0x0002				//!< Pixels are BGR (or BG for Bayer).
#define GEV_PIXEL_ORDER_GRB		0x0004				//!< Pixels are GR for Bayer.
#define GEV_PIXEL_ORDER_GBR		0x0008				//!< Pixels are GB for Bayer.
#define GEV_PIXEL_ORDER_RGB10V1	0xF000				//!< Custom format #1 for 10-bit RGB.
#define GEV_PIXEL_ORDER_RGB10V2	0xE000				//!< Custom format #2 for 10-bit RGB.

// Helper functions for Pixel Format Types.
BOOL GevIsPixelTypeMono( UINT32 pixelType);
BOOL GevIsPixelTypeRGB( UINT32 pixelType);
BOOL GevIsPixelTypeBayer (UINT32 pixelType);
BOOL GevIsPixelTypeCustom( UINT32 pixelType);
BOOL GevIsPixelTypePacked( UINT32 pixelType);
UINT32 GevGetPixelSizeInBytes( UINT32 pixelType);
UINT32 GevGetPixelDepthInBits( UINT32 pixelType);
UINT32 GevGetRGBPixelOrder( UINT32 pixelType);
UINT32 GevGetUnpackedPixelType(UINT32 pixelType);
UINT32 GevGetBayerAsRGBPixelType(UINT32 pixelType);
UINT32 GevGetPixelComponentCount(UINT32 pixelType);
UINT32 GevGetConvertedPixelType(int convertBayer, UINT32 pixelType);

GEVLIB_STATUS GevTranslateRawPixelFormat( UINT32 rawFormat, PUINT32 translatedFormat, PUINT32 bitDepth, PUINT32 order);
const char *GevGetFormatString( UINT32 format);
//=======================================
// Public Access Mode Value Definitions
//=======================================
typedef enum
{
	GevMonitorMode = 0,
	GevControlMode = 2,
	GevExclusiveMode = 4
} GevAccessMode;


//====================================================================
// Public Data Structures
//====================================================================

typedef struct
{
	UINT32 version;
	UINT32 logLevel;
	UINT32 numRetries;
	UINT32 command_timeout_ms;
	UINT32 discovery_timeout_ms;
	UINT32 enumeration_port;
	UINT32 gvcp_port_range_start;
	UINT32 gvcp_port_range_end;
	UINT32 manual_xml_handling;
} GEVLIB_CONFIG_OPTIONS, *PGEVLIB_CONFIG_OPTIONS;

typedef struct
{
	UINT32 numRetries;
	UINT32 command_timeout_ms;
	UINT32 heartbeat_timeout_ms;
	UINT32 streamPktSize;				// GVSP max packet size ( less than or equal to MTU size).
	UINT32 streamPktDelay;				// Delay between packets (microseconds) - to tune packet pacing out of NIC.
	UINT32 streamNumFramesBuffered;	// # of frames to buffer (min 2)
	UINT32 streamMemoryLimitMax;		// Maximum amount of memory to use (puts an upper limit on the # of frames to buffer).
	UINT32 streamMaxPacketResends;	// Maximum number of packet resends to allow for a frame (defaults to 1000).
	UINT32 streamFrame_timeout_ms;	// Frame timeout (msec) after leader received.
	INT32  streamThreadAffinity;		// CPU affinity for streaming thread (marshall/unpack/write to user buffer) - default handling is "-1" 
	INT32  serverThreadAffinity;		// CPU affinity for packet server thread (recv/dispatch) - default handling is "-1"
	UINT32 msgChannel_timeout_ms;
	UINT32 enable_passthru_mode;		// Zero (default) to enable automatic conversion of packed pixel formats to unpacked pixel format. 
												// Non-zero for passthru mode.  
												// (Use Unpacked pixels for processing, Use Packed pixels for archiving to save space) 
} GEV_CAMERA_OPTIONS, *PGEV_CAMERA_OPTIONS;


typedef struct
{
	BOOL fIPv6;				// GEV is only IPv4 for now.
	UINT32 ipAddr;
	UINT32 ipAddrLow;
	UINT32 ipAddrHigh;
	UINT32 ifIndex;		// Index of network interface (set by system - required for packet interface access).
} GEV_NETWORK_INTERFACE, *PGEV_NETWORK_INTERFACE;

#define MAX_GEVSTRING_LENGTH	64

typedef struct
{
	BOOL fIPv6;				// GEV is only IPv4 for now.
	UINT32 ipAddr;
	UINT32 ipAddrLow;
	UINT32 ipAddrHigh;
	UINT32 macLow;
	UINT32 macHigh;
	GEV_NETWORK_INTERFACE host;
	UINT32 mode;
	UINT32 capabilities;
	char   manufacturer[MAX_GEVSTRING_LENGTH+1];
	char   model[MAX_GEVSTRING_LENGTH+1];
	char   serial[MAX_GEVSTRING_LENGTH+1];
	char   version[MAX_GEVSTRING_LENGTH+1];
	char   username[MAX_GEVSTRING_LENGTH+1];
} GEV_DEVICE_INTERFACE, *PGEV_DEVICE_INTERFACE, GEV_CAMERA_INFO, *PGEV_CAMERA_INFO;

typedef void* GEV_CAMERA_HANDLE;

// Buffer object structure - returned 
typedef struct _tag_GEVBUF_ENTRY
{
	UINT32 payload_type;	// Type of payload received (???list them ???)
	UINT32 state;			// Full/empty state for payload buffer (tag used for buffer cycling)
	INT32  status;			// Frame Status (success, error types) 
	UINT32 timestamp_hi;	// Most 32 significant bit of the timestamp (for legacy code) .
	UINT32 timestamp_lo;	// Least 32 significant bit of the timestamp (for legacy code) .
	UINT64 timestamp; 	// 64bit version of timestamp for payload (device level timestamp using device level timebase).
	UINT64 recv_size;		// Received size of entire payload (including all appended "chunk" (metadata) information) . 
	UINT64 id;				// Block id for the payload (starts at 1, may wrap to 1 at 65535 - depending on the payload type).
								// Image specific payload entries.
	UINT32 h;				// Received height (lines) for an image payload. 
	UINT32 w;				// Received width (pixels) for an image payload.
	UINT32 x_offset;		// Received x offset for origin of ROI for an image payload_type.
	UINT32 y_offset;		// Received y offset for origin of ROI for an image payload_type.
	UINT32 x_padding;		// Received x padding bytes for an image payload_type (invalid data padding end of each line [horizontal invalid]). 
	UINT32 y_padding;		// Received y padding bytes for an image payload_type (invalid data padding end of image [vertical invalid]).
	UINT32 d;				// Received pixel depth (bytes per pixel) for an image payload with a Gige Vision defined pixel format.
	UINT32 format;			// Received Gige Vision pixel format for image or JPEG data types.
								// (If the format value is not a valid Gige Vision pixel type and the payload_type is JPEG, then the format value 
								//  is to be interpreted as a color space value (EnumCS value for JPEG) to be used by a JPEG decoder).
								//
	PUINT8 address;		// Address of the "payload_type" data, NULL if the payload has been sent to trash (no buffer available to receive it).
								//
								// New entries for non-image payload types
								//
	PUINT8 chunk_data;	// Address of "chunk" data (metadata) associated with the received payload (NULL if no "chunk" data (metadata) is available).
								// (The "chunk_data" address is provided here as a shortcut. It is the address immediatley following the end of "paylod_type" data)
	UINT32 chunk_size;	// The size of the chunk_data (uncompressed). Zero if no "chunk" data (metadata) is available.
								// (The "chunk_size" is provided as a helper for decoding raw TurboDrive compressed data in passthru mode).												
								//
   char  filename[256];	// Name of file for payload type "file" (0 terminated string, 255 characters maximum system limit in Linux).
} GEVBUF_ENTRY, *PGEVBUF_ENTRY, GEVBUF_HEADER, *PGEVBUF_HEADER, GEV_BUFFER_OBJECT, *PGEV_BUFFER_OBJECT;

#define GEV_FRAME_STATUS_RECVD		0	// Frame is complete.
#define GEV_FRAME_STATUS_PENDING		1	// Frame is not ready.
#define GEV_FRAME_STATUS_TIMEOUT		2	// Frame was not ready before timeout condition met.
#define GEV_FRAME_STATUS_OVERFLOW	3  // Frame was not complete before the max number of frames to buffer queue was full.
#define GEV_FRAME_STATUS_BANDWIDTH	4	// Frame had too many resend operations due to insufficient bandwidth.
#define GEV_FRAME_STATUS_LOST			5	// Frame had resend operations that failed.
#define GEV_FRAME_STATUS_RELEASED	-1 // (Internal) Frame has been released for re-use.

// Buffer cycling control definition
typedef enum
{
	Asynchronous = 0, 
	SynchronousNextEmpty = 1 
} GevBufferCyclingMode;

// For Future use
typedef void (*GEVTL_CBFUNCTION) (PGEV_BUFFER_OBJECT buffer_object, void *context);


// Buffer list for a transfer.
typedef struct _GEVBUF_QUEUE
{
	UINT32 type;		// Payload type expected for buffers in list (from camera)
	UINT64 size;		// Payload size allocated for buffers in list.
	UINT32 numBuffer;	// Number of buffers in the list.
	UINT32 height;		// Image height for allocated buffers in list (for image payload types).
	UINT32 width;		// Image width for allocated buffers in list (for image payload types).
	UINT32 depth;		// Image depth (bytes) for allocated buffers in list (for image payload types).
	INT32  lastBuffer; // Temp for debugging
	UINT32 nextBuffer; // Temp for debugging
	UINT32 trashCount; // Number of buffers that have been sent to trash (no available free buffer).
	GevBufferCyclingMode	cyclingMode; // Buffer queue operation mode (Asynchonous, SynchronousNextEmpty, SynchronousNextEmptyWithTrash)
	DQUEUE	*pEmptyBuffers;	// Queue of empty/unlocked GEV_BUFFER_OBJECT data structures.
	DQUEUE	*pFullBuffers;		// Queue of full/locked GEV_BUFFER_OBJECT data structures.
	GEVBUF_ENTRY trashBuf;		// Dummy entry for delivering trashed buffers.  //??????????????????????????????????????????????????????????????????????
	GEVBUF_ENTRY *pCurBuf;		// Pointer to the current (active) buffer object (NULL if cycling is dumping to trash).
	GEVBUF_ENTRY buffer[1];		// List of buffers available for storage.
} GEV_BUFFER_LIST;

// Asycnhronous event information structure - returned in callback.
typedef struct
{
   UINT16 eventNumber;           // Event number.
   UINT16 streamChannelIndex;    // Channel index associated with this event.
   UINT64 blockId;               // Block Id associated with this event.
   UINT64 timestamp;         		// Timestamp for this event (device level timestamp using device level timebase).
   UINT32 timeStampHigh;         // Most 32 significant bit of the timestamp (for legacy code) .
   UINT32 timeStampLow;          // Least 32 significant bit of the timestamp (for legacy code) .
} EVENT_MSG, *PEVENT_MSG;

typedef void (*GEVEVENT_CBFUNCTION) (PEVENT_MSG msg, PUINT8 data, UINT16 size, void *context);

//=========================================================================
// Utility functions and macros
//=========================================================================
UINT32 GevFormatCameraInfo( GEV_DEVICE_INTERFACE *pCamera, char *pBuf, UINT32 size);

int GevPrint( int level, const char *file, unsigned int line, const char *fmt, ...);
#define LogPrint(lvl,args...) GevPrint((lvl), __FILE__, __LINE__, ##args)


//======================================================================================
// GenApi feature access definitions.
// Exception handling catch macro (extensible - logging can also be modified / extended).
#define CATCH_GENAPI_ERROR(statusToReturn) \
				catch (InvalidArgumentException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: Invalid Argument Exception %s", E.what()); \
					statusToReturn = GEVLIB_ERROR_ARG_INVALID; \
				} \
				catch (PropertyException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: Property Exception %s", E.what()); \
				} \
				catch (LogicalErrorException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: Logical Error Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_SOFTWARE; \
				} \
				catch (OutOfRangeException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: OutOfRange Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_PARAMETER_INVALID; \
				} \
				catch (RuntimeException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: Runtime Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_GENERIC; \
				} \
				catch (AccessException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: Access Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_ACCESS_DENIED; \
				} \
				catch (TimeoutException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: Timeout Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_TIME_OUT ; \
				} \
				catch (DynamicCastException &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, E.GetSourceFileName(), E.GetSourceLine(), " GenApi: DynamiceCast Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_NULL_PTR; \
				} \
				catch (std::exception &E) \
				{ \
					GevPrint(GEV_LOG_DEBUG, __FILE__, __LINE__, " StdLib: Exception %s\n", E.what()); \
					statusToReturn = GEVLIB_ERROR_SOFTWARE; \
				} \
				catch (...) \
				{ \
					GevPrint(GEV_LOG_DEBUG, __FILE__, __LINE__, " GenApi: Unknown Error Exception (catchall)\n"); \
					statusToReturn = GEVLIB_ERROR_GENERIC; \
				} \

// Feature types returned (in C API).
#define GENAPI_UNUSED_TYPE		1
#define GENAPI_VALUE_TYPE		0
#define GENAPI_BASE_TYPE		1	
#define GENAPI_INTEGER_TYPE	2
#define GENAPI_BOOLEAN_TYPE	3
#define GENAPI_COMMAND_TYPE	4
#define GENAPI_FLOAT_TYPE		5
#define GENAPI_STRING_TYPE		6
#define GENAPI_REGISTER_TYPE	7
#define GENAPI_CATEGORY_TYPE	8
#define GENAPI_ENUM_TYPE		9
#define GENAPI_ENUMENTRY_TYPE	10

// Feature access mode (in C API)
#define	GENAPI_ACCESSMODE_NI		0	// Not implemented
#define	GENAPI_ACCESSMODE_NA		1	// Not available
#define	GENAPI_ACCESSMODE_WO		2	// Write-only
#define	GENAPI_ACCESSMODE_RO		3	// Read-only
#define	GENAPI_ACCESSMODE_RW		4	// Read-write
#define	GENAPI_ACCESSMODE_NONE	5	// Undefined

// Feature visibility (in C API)
#define	GENAPI_VISIBILITY_BEGINNER		0
#define	GENAPI_VISIBILITY_EXPERT		1
#define	GENAPI_VISIBILITY_GURU			2
#define	GENAPI_VISIBILITY_INVISIBLE	3
#define	GENAPI_VISIBILITY_UNDEFINED	99

// Feature increment type (in C API)
#define GENAPI_INCREMENT_NONE		0
#define GENAPI_INCREMENT_FIXED	1
#define GENAPI_INCREMENT_LIST		2



//====================================================================
// Public API
//====================================================================
// API Initialization
GEV_STATUS	GevApiInitialize(void);
GEV_STATUS	GevApiUninitialize(void);

//====================================================================
// API Configuratoin options
GEV_STATUS GevGetLibraryConfigOptions( GEVLIB_CONFIG_OPTIONS *options);
GEV_STATUS GevSetLibraryConfigOptions( GEVLIB_CONFIG_OPTIONS *options);

//=================================================================================================
// Camera automatic discovery
int GevDeviceCount(void);	// Get the number of Gev devices seen by the system.
GEV_STATUS GevGetCameraList( GEV_CAMERA_INFO *cameras, int maxCameras, int *numCameras); // Automatically detect and list cameras.

GEV_STATUS GevForceCameraIPAddress( UINT32 macHi, UINT32 macLo, UINT32 IPAddress, UINT32 subnetmask);
GEV_STATUS GevEnumerateNetworkInterfaces(GEV_NETWORK_INTERFACE *pIPAddr, UINT32 maxInterfaces, PUINT32 pNumInterfaces );

//=================================================================================================
// Utility function (external) for discovering camera devices.  
GEV_STATUS GevEnumerateGevDevices(GEV_NETWORK_INTERFACE *pIPAddr, UINT32 discoveryTimeout, GEV_DEVICE_INTERFACE *pDevice, UINT32 maxDevices, PUINT32 pNumDevices );

// Camera Manual discovery/setup 
GEV_STATUS GevSetCameraList( GEV_CAMERA_INFO *cameras, int numCameras); // Manually set camera list from data structure.

//=================================================================================================
// Gige Vision Camera Access
GEV_STATUS GevOpenCamera( GEV_CAMERA_INFO *device, GevAccessMode mode, GEV_CAMERA_HANDLE *handle);
GEV_STATUS GevOpenCameraByAddress( unsigned long ip_address, GevAccessMode mode, GEV_CAMERA_HANDLE *handle);
GEV_STATUS GevOpenCameraByName( char *name, GevAccessMode mode, GEV_CAMERA_HANDLE *handle);
GEV_STATUS GevOpenCameraBySN( char *sn, GevAccessMode mode, GEV_CAMERA_HANDLE *handle);

GEV_STATUS GevCloseCamera(GEV_CAMERA_HANDLE *handle);

GEV_STATUS Gev_Reconnect(GEV_CAMERA_HANDLE handle);

GEV_CAMERA_INFO *GevGetCameraInfo( GEV_CAMERA_HANDLE handle);

GEV_STATUS GevGetCameraInterfaceOptions( GEV_CAMERA_HANDLE handle, GEV_CAMERA_OPTIONS *options);
GEV_STATUS GevSetCameraInterfaceOptions( GEV_CAMERA_HANDLE handle, GEV_CAMERA_OPTIONS *options);

//=================================================================================================
// Manual GigeVision access to GenICam XML File
GEV_STATUS Gev_RetrieveXMLData( GEV_CAMERA_HANDLE handle, int size, char *xml_data, int *num_read, int *data_is_compressed );
GEV_STATUS Gev_RetrieveXMLFile( GEV_CAMERA_HANDLE handle, char *file_name, int size, BOOL force_download );

//=================================================================================================
// GenICam XML Feature Node Map manual registration/access functions (for use in C++ code).
GEV_STATUS GevConnectFeatures(  GEV_CAMERA_HANDLE handle,  void *featureNodeMap);
void * GevGetFeatureNodeMap(  GEV_CAMERA_HANDLE handle);

//=================================================================================================
// GenICam XML Feature access functions (C language compatible).
GEV_STATUS GevGetGenICamXML_FileName( GEV_CAMERA_HANDLE handle, int size, char *xmlFileName);
GEV_STATUS GevInitGenICamXMLFeatures( GEV_CAMERA_HANDLE handle, BOOL updateXMLFile);
GEV_STATUS GevInitGenICamXMLFeatures_FromFile( GEV_CAMERA_HANDLE handle, char *xmlFileName);
GEV_STATUS GevInitGenICamXMLFeatures_FromData( GEV_CAMERA_HANDLE handle, int size, void *pXmlData);

GEV_STATUS GevGetFeatureValue( GEV_CAMERA_HANDLE handle, const char *feature_name, int *feature_type, int value_size, void *value);
GEV_STATUS GevSetFeatureValue( GEV_CAMERA_HANDLE handle, const char *feature_name, int value_size, void *value);
GEV_STATUS GevGetFeatureValueAsString( GEV_CAMERA_HANDLE handle, const char *feature_name, int *feature_type, int value_string_size, char *value_string);
GEV_STATUS GevSetFeatureValueAsString( GEV_CAMERA_HANDLE handle, const char *feature_name, const char *value_string);

//=================================================================================================
// Camera data acquisition (payload frame based)

GEV_STATUS GevInitializeTransfer( GEV_CAMERA_HANDLE handle, GevBufferCyclingMode mode, UINT64 bufSize, UINT32 numBuffers, UINT8 **bufAddress);
GEV_STATUS GevFreeTransfer( GEV_CAMERA_HANDLE handle);
GEV_STATUS GevGetPayloadParameters(GEV_CAMERA_HANDLE handle, PUINT64 payload_size, PUINT32 data_format);

GEV_STATUS GevStartTransfer( GEV_CAMERA_HANDLE handle, UINT32 numFrames);
GEV_STATUS GevStopTransfer( GEV_CAMERA_HANDLE handle);
GEV_STATUS GevAbortTransfer( GEV_CAMERA_HANDLE handle);

GEV_STATUS GevQueryTransferStatus( GEV_CAMERA_HANDLE handle, PUINT32 pTotalBuffers, PUINT32 pNumUsed, PUINT32 pNumFree, PUINT32 pNumTrashed, GevBufferCyclingMode *pMode);
GEV_STATUS GevWaitForNextFrame( GEV_CAMERA_HANDLE handle, GEV_BUFFER_OBJECT **frame_object, UINT32 timeout);
GEV_STATUS GevGetNextFrame( GEV_CAMERA_HANDLE handle, GEV_BUFFER_OBJECT **frame_object, struct timeval *pTimeout);

GEV_STATUS GevReleaseFrame( GEV_CAMERA_HANDLE handle, GEV_BUFFER_OBJECT *frame_object_ptr);
GEV_STATUS GevReleaseFrameBuffer( GEV_CAMERA_HANDLE handle, void *frame_buffer_ptr);

//=================================================================================================
// Camera image acquisition (subset of data acquisition)
GEV_STATUS GevGetImageParameters(GEV_CAMERA_HANDLE handle,PUINT32 width, PUINT32 height, PUINT32 x_offset, PUINT32 y_offset, PUINT32 format);
GEV_STATUS GevSetImageParameters(GEV_CAMERA_HANDLE handle,UINT32 width, UINT32 height, UINT32 x_offset, UINT32 y_offset, UINT32 format);

GEV_STATUS GevInitImageTransfer( GEV_CAMERA_HANDLE handle, GevBufferCyclingMode mode, UINT32 numBuffers, UINT8 **bufAddress);
GEV_STATUS GevInitializeImageTransfer( GEV_CAMERA_HANDLE handle, UINT32 numBuffers, UINT8 **bufAddress);

// Some legacy definitions.
#define GevFreeImageTransfer        GevFreeTransfer
#define GevStartImageTransfer       GevStartTransfer
#define GevStopImageTransfer        GevStopTransfer
#define GevAbortImageTransfer       GevAbortTransfer
#define GevQueryImageTransferStatus GevQueryTransferStatus

int GetPixelSizeInBytes (UINT32 pixelType);

// Legacy (image based) functions.
GEV_STATUS GevGetImageBuffer( GEV_CAMERA_HANDLE handle, void **image_buffer);
GEV_STATUS GevGetImage( GEV_CAMERA_HANDLE handle, GEV_BUFFER_OBJECT **image_object);
GEV_STATUS GevWaitForNextImageBuffer( GEV_CAMERA_HANDLE handle, void **image_buffer, UINT32 timeout);

#define GevGetNextImage       GevGetNextFrame
#define GevWaitForNextImage   GevWaitForNextFrame
#define GevReleaseImage       GevReleaseFrame
#define GevReleaseImageBuffer GevReleaseFrameBuffer

//=================================================================================================
// Camera event handling
GEV_STATUS GevRegisterEventCallback(GEV_CAMERA_HANDLE handle,  UINT32 EventID, GEVEVENT_CBFUNCTION func, void *context);
GEV_STATUS GevRegisterApplicationEvent(GEV_CAMERA_HANDLE handle,  UINT32 EventID, _EVENT appEvent);
GEV_STATUS GevUnregisterEvent(GEV_CAMERA_HANDLE handle,  UINT32 EventID);


//=================================================================================================
// Data conversion functions (and definitions).
//
// Bayer
//
#define BAYER_ALIGN_GB_RG	0	
#define BAYER_ALIGN_BG_GR	1	
#define BAYER_ALIGN_RG_GB	2	
#define BAYER_ALIGN_GR_BG	3	

#define BAYER_CONVERSION_2X2 0
GEV_STATUS ConvertBayerToRGB( int convAlgorithm, UINT32 h, UINT32 w, UINT32 inFormat, void *inImage, UINT32 outFormat, void *outImage);

// Packed Pixel Unpacker
GEV_STATUS Gev_UnpackMonoPackedFrame( UINT32 infmt, UINT32 inSize, void *packed_data, UINT32 outSize, void *unpacked_data);

// Stand-alone TurboDrive Frame Decoder.
GEV_STATUS	Gev_DecodeTurboDriveFrame( UINT32 payload_type, UINT32 height, UINT32 width, UINT32 pixel_format, UINT32 chunk_size, \
											UINT64 inSize, void *inData, UINT64 outSize, void *outData, \
											UINT64 *total_decoded_size, UINT64 *chunk_offset);




//=================================================================================================
// Legacy definitions for the now deprecated, original, static register definition API.
// 
#include "gev_legacy_api.h"

#ifdef __cplusplus
}
#endif



#endif
