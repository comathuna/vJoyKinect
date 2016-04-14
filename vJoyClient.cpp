// vJoyClient.cpp : Simple feeder application with a FFB demo
//

#define _USE_MATH_DEFINES

// Monitor Force Feedback (FFB) vJoy device
#include "stdafx.h"
//#include "Devioctl.h"
#include "public.h"
#include <malloc.h>
#include <string.h>
#include <stdlib.h>
#include "vjoyinterface.h"
//#include "Math.h"
//#include <math.h>
#include <cmath>

// Windows Header Files
#include <windows.h>

#include <Shlobj.h>

// Direct2D Header Files
#include <d2d1.h>

// Kinect Header files
#include <Kinect.h>
#include <Kinect.Face.h>

// Default device ID (Used when ID not specified)
#define DEV_ID		1

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != nullptr)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = nullptr;
	}
}

// Prototypes
int Polar2Deg(BYTE Polar);
int Byte2Percent(BYTE InByte);
int TwosCompByte2Int(BYTE in);
void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll);
void ProcessFaces();
HRESULT UpdateBodyData(IBody** ppBodies);
HRESULT InitializeDefaultSensor();
HRESULT init_Kinect();

JOYSTICK_POSITION_V2 iReport; // The structure that holds the full position data

HRESULT hresult = S_OK, hResult = S_OK;

int pitch=0, yaw=0, roll=0;

// Initialize Kinect and get color, body and face readers
IColorFrameSource* pColorFrameSource = nullptr;
IBodyFrameSource* pBodyFrameSource = nullptr;
IBodyIndexFrameSource* pBodyIndexFrameSource = nullptr;

// Current Kinect
IKinectSensor*         m_pKinectSensor;

// Coordinate mapper
ICoordinateMapper*     m_pCoordinateMapper;

// Color source
IColorFrameSource*	   m_pColorFrameSource;

// Color reader
IColorFrameReader*     m_pColorFrameReader;

// Body source
IBodyFrameSource*	   m_pBodyFrameSource;

// Body reader
IBodyFrameReader*      m_pBodyFrameReader;

// Face sources
IFaceFrameSource*	   m_pFaceFrameSources[BODY_COUNT];

// Face readers
IFaceFrameReader*	   m_pFaceFrameReaders[BODY_COUNT];

// Frame description
IFrameDescription*     m_pDescription;

IFaceFrameResult* pFaceFrameResult = nullptr;
PointF facePoints[FacePointType::FacePointType_Count];
Vector4 faceRotation;
DetectionResult faceProperties[FaceProperty::FaceProperty_Count];
//D2D1_POINT_2F faceTextLayout;

// define the face frame features required to be computed by this application
static const DWORD c_FaceFrameFeatures =
FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
| FaceFrameFeatures::FaceFrameFeatures_Happy
| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
| FaceFrameFeatures::FaceFrameFeatures_LookingAway
| FaceFrameFeatures::FaceFrameFeatures_Glasses
| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;


int
__cdecl
_tmain(int argc, _TCHAR* argv[])
{
	int stat = 0;
	UINT DevID = DEV_ID;
	USHORT X = 0;
	USHORT Y = 0;
	USHORT Z = 0;
	LONG   Btns = 0;
	
	PVOID pPositionMessage;
	UINT	IoCode = LOAD_POSITIONS;
	UINT	IoSize = sizeof(JOYSTICK_POSITION);
	// HID_DEVICE_ATTRIBUTES attrib;
	BYTE id = 1;
	UINT iInterface = 1;

	// Set the target Joystick - get it from the command-line 
	if (argc>1)
		DevID = _tstoi(argv[1]);

	// Get the driver attributes (Vendor ID, Product ID, Version Number)
	if (!vJoyEnabled())
	{
		_tprintf("Function vJoyEnabled Failed - make sure that vJoy is installed and enabled\n");
		int dummy = getchar();
		stat = - 2;
		goto Exit;
	}
	else
	{
		//wprintf(L"Vendor: %s\nProduct :%s\nVersion Number:%hs\n", static_cast<TCHAR *> (GetvJoyManufacturerString()), static_cast<TCHAR *>(GetvJoyProductString()), static_cast<TCHAR *>(GetvJoySerialNumberString()));
		_tprintf("Function vJoyEnabled Succeeded - vJoy is installed and enabled\n");
	};

	// Get the status of the vJoy device before trying to acquire it
	VjdStat status = GetVJDStatus(DevID);

	switch (status)
	{
	case VJD_STAT_OWN:
		_tprintf("vJoy device %d is already owned by this feeder\n", DevID);
		break;
	case VJD_STAT_FREE:
		_tprintf("vJoy device %d is free\n", DevID);
		break;
	case VJD_STAT_BUSY:
		_tprintf("vJoy device %d is already owned by another feeder\nCannot continue\n", DevID);
		return -3;
	case VJD_STAT_MISS:
		_tprintf("vJoy device %d is not installed or disabled\nCannot continue\n", DevID);
		return -4;
	default:
		_tprintf("vJoy device %d general error\nCannot continue\n", DevID);
		return -1;
	};

	// Acquire the vJoy device
	if (!AcquireVJD(DevID))
	{
		_tprintf("Failed to acquire vJoy device number %d.\n", DevID);
		int dummy = getchar();
		stat = -1;
		goto Exit;
	}
	else
		_tprintf("Acquired device number %d - OK\n", DevID);

	

	//hresult = GetDefaultKinectSensor(&psensor);
	//if (FAILED(hresult))
	//{
	//	_tprintf("Failed to get Kinect Sensor");
	//	getchar();
	//	goto Exit;
	//}
	//else
	//	_tprintf("Acquired Kinect\n");

	//if (psensor)
	//{

	//	hresult = psensor->Open();

	//	if (SUCCEEDED(hresult))
	//	{
	//		hresult = psensor->get_CoordinateMapper(&m_pCoordinateMapper);
	//	}
	//	else
	//		_tprintf("Couldn't open Kinect\n");

	//	if (SUCCEEDED(hresult))
	//	{
	//		hresult = psensor->get_BodyFrameSource(&pBodyFrameSource);
	//	}
	//	else
	//		_tprintf("Couldn't get CoordinateMapper\n");

	//	if (SUCCEEDED(hresult))
	//	{
	//		hresult = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	//		_tprintf("Got BodyFrameSource\n");
	//	}
	//	else
	//		_tprintf("Couldn't get BodyFrameSource\n");

	//	if (SUCCEEDED(hresult))
	//	{
	//		_tprintf("Opened BodyFrameReader\n");
	//		// create a face frame source + reader to track each body in the fov
	//		for (int i = 0; i < BODY_COUNT; i++)
	//		{
	//			if (SUCCEEDED(hresult))
	//			{
	//				// create the face frame source by specifying the required face frame features
	//				hresult = CreateFaceFrameSource(psensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
	//			}
	//			else
	//				_tprintf("Wait, what?\n");
	//			if (SUCCEEDED(hresult))
	//			{
	//				// open the corresponding reader
	//				hresult = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
	//			}
	//			else
	//				_tprintf("Couldn't CreateFaceFrameSource\n");
	//			if (FAILED(hresult))
	//			{
	//				_tprintf("Couldn't open FaceFrameReaders\n");
	//			}
	//		}
	//	}
	//	else
	//		_tprintf("Couldn't open BodyFrameReader\n");

	//	//SafeRelease(pColorFrameSource);
	//	//SafeRelease(pBodyFrameSource);
	//}
	//else
	//	_tprintf("psensor isn't a thing\n");
	//	

	//InitializeDefaultSensor();
	init_Kinect();
	
	// Start endless loop
	// The loop injects position data to the vJoy device
	// If it fails it lets the user try again
	while (1)
	{
		//IBody* ppBodies[BODY_COUNT] = { 0 };
		////bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies));

		//// iterate through each face reader
		//for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
		//{
		//	// retrieve the latest face frame from this reader
		//	IFaceFrame* pFaceFrame = nullptr;
		//	hresult = (m_pFaceFrameReaders[iFace])->AcquireLatestFrame(&pFaceFrame);

		//	BOOLEAN bFaceTracked = false;
		//	if (SUCCEEDED(hresult) && nullptr != pFaceFrame)
		//	{
		//		// check if a valid face is tracked in this face frame
		//		hresult = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		//	}

		//	if (SUCCEEDED(hresult))
		//	{
		//		if (bFaceTracked)
		//		{

		//			hresult = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

		//			// need to verify if pFaceFrameResult contains data before trying to access it
		//			if (SUCCEEDED(hresult) && pFaceFrameResult != nullptr)
		//			{
		//				//hresult = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

		//				if (SUCCEEDED(hresult))
		//				{
		//					hresult = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
		//				}

		//				if (SUCCEEDED(hresult))
		//				{
		//					hresult = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
		//				}

		//				if (SUCCEEDED(hresult))
		//				{
		//					hresult = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
		//				}
		//			}

		//			//SafeRelease(pFaceFrameResult);
		//		}
		//	}
		//}

		// extract face rotation in degrees as Euler angles
		/*int pitch, yaw, roll;
		ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
		*/

		ProcessFaces();

		// Set destination vJoy device
		id = (BYTE)DevID;
		iReport.bDevice = id;

		// Set position data of 3 first axes
		if (Z>35000) Z=0;
		Z += 200;
		iReport.wAxisZ = Z;
		iReport.wAxisX = 32000-Z;
		iReport.wAxisY = Z/2+7000;

		// Set position data of first 8 buttons
		Btns = 1<<(Z/4000);
		iReport.lButtons = Btns;

		// Send position data to vJoy device
		pPositionMessage = (PVOID)(&iReport);
		if (!UpdateVJD(DevID, pPositionMessage))
		{
			printf("Feeding vJoy device number %d failed - try to enable device then press enter\n", DevID);
			getchar();
			AcquireVJD(DevID);
		}
		Sleep(2);
	}

Exit:
	RelinquishVJD(DevID);
	return 0;
}

// Polar values (0x00-0xFF) to Degrees (0-360)
int Polar2Deg(BYTE Polar)
{
	return ((UINT)Polar*360)/255;
}

// Convert range 0x00-0xFF to 0%-100%
int Byte2Percent(BYTE InByte)
{
	return ((UINT)InByte*100)/255;
}

// Convert One-Byte 2's complement input to integer
int TwosCompByte2Int(BYTE in)
{
	int tmp;
	BYTE inv = ~in;
	BOOL isNeg = in>>7;
	if (isNeg)
	{
		tmp = (int)(inv);
		tmp = -1*tmp;
		return tmp;
	}
	else
		return (int)in;
}

// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
inline void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees
	*pPitch = static_cast<int>(std::atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0f);
	*pYaw = static_cast<int>(std::asin(2 * (w * y - x * z)) / M_PI * 180.0f);
	*pRoll = static_cast<int>(std::atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0f);
}

/// <summary>
/// Processes new face frames
/// </summary>
void ProcessFaces()
{
	HRESULT hr;
	IBody* ppBodies[BODY_COUNT] = { 0 };
	bool bHaveBodyData = SUCCEEDED(UpdateBodyData(ppBodies));

	// iterate through each face reader
	for (int iFace = 0; iFace < BODY_COUNT; ++iFace)
	{
		// retrieve the latest face frame from this reader
		IFaceFrame* pFaceFrame = nullptr;
		hr = m_pFaceFrameReaders[iFace]->AcquireLatestFrame(&pFaceFrame);

		BOOLEAN bFaceTracked = false;
		if (SUCCEEDED(hr) && nullptr != pFaceFrame)
		{
			// check if a valid face is tracked in this face frame
			hr = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
		}

		if (SUCCEEDED(hr))
		{
			if (bFaceTracked)
			{
				IFaceFrameResult* pFaceFrameResult = nullptr;
				PointF facePoints[FacePointType::FacePointType_Count];
				Vector4 faceRotation;
				DetectionResult faceProperties[FaceProperty::FaceProperty_Count];

				hr = pFaceFrame->get_FaceFrameResult(&pFaceFrameResult);

				// need to verify if pFaceFrameResult contains data before trying to access it
				if (SUCCEEDED(hr) && pFaceFrameResult != nullptr)
				{
					/*hr = pFaceFrameResult->get_FaceBoundingBoxInColorSpace(&faceBox);

					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoints);
					}
*/
					if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->get_FaceRotationQuaternion(&faceRotation);
					}

					if (SUCCEEDED(hr))
					{
						ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
					}

					/*if (SUCCEEDED(hr))
					{
						hr = pFaceFrameResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperties);
					}*/

					/*if (SUCCEEDED(hr))
					{
						hr = GetFaceTextPositionInColorSpace(ppBodies[iFace], &faceTextLayout);
					}*/

					//if (SUCCEEDED(hr))
					//{
					//	// draw face frame results
					//	m_pDrawDataStreams->DrawFaceFrameResults(iFace, &faceBox, facePoints, &faceRotation, faceProperties, &faceTextLayout);
					//}
				}

				SafeRelease(pFaceFrameResult);
			}
			else
			{
				// face tracking is not valid - attempt to fix the issue
				// a valid body is required to perform this step
				if (bHaveBodyData)
				{
					// check if the corresponding body is tracked 
					// if this is true then update the face frame source to track this body
					IBody* pBody = ppBodies[iFace];
					if (pBody != nullptr)
					{
						BOOLEAN bTracked = false;
						hr = pBody->get_IsTracked(&bTracked);

						UINT64 bodyTId;
						if (SUCCEEDED(hr) && bTracked)
						{
							// get the tracking ID of this body
							hr = pBody->get_TrackingId(&bodyTId);
							if (SUCCEEDED(hr))
							{
								// update the face frame source with the tracking ID
								m_pFaceFrameSources[iFace]->put_TrackingId(bodyTId);
							}
						}
					}
				}
			}
		}

		SafeRelease(pFaceFrame);
	}

	if (bHaveBodyData)
	{
		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}
}

/// <summary>
/// Updates body data
/// </summary>
/// <param name="ppBodies">pointer to the body data storage</param>
/// <returns>indicates success or failure</returns>
HRESULT UpdateBodyData(IBody** ppBodies)
{
	HRESULT hr = E_FAIL;

	if (m_pBodyFrameReader != nullptr)
	{
		IBodyFrame* pBodyFrame = nullptr;
		hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, ppBodies);
		}
		SafeRelease(pBodyFrame);
	}

	return hr;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>S_OK on success else the failure code</returns>
HRESULT InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize Kinect and get color, body and face readers
		IColorFrameSource* pColorFrameSource = nullptr;
		IBodyFrameSource* pBodyFrameSource = nullptr;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		if (SUCCEEDED(hr))
		{
			// create a face frame source + reader to track each body in the fov
			for (int i = 0; i < BODY_COUNT; i++)
			{
				if (SUCCEEDED(hr))
				{
					// create the face frame source by specifying the required face frame features
					hr = CreateFaceFrameSource(m_pKinectSensor, 0, c_FaceFrameFeatures, &m_pFaceFrameSources[i]);
				}
				if (SUCCEEDED(hr))
				{
					// open the corresponding reader
					hr = m_pFaceFrameSources[i]->OpenReader(&m_pFaceFrameReaders[i]);
				}
			}
		}

		SafeRelease(pColorFrameSource);
		SafeRelease(pBodyFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		//SetStatusMessage(L"No ready Kinect found!", 10000, true);
		_tprintf("Couldn't get a Kinect\n");
		return E_FAIL;
	}

	return hr;
}

HRESULT init_Kinect()
{
	// Sensor
	//IKinectSensor* pSensor;
	//HRESULT hResult = S_OK;
	hResult = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hResult)) {
		//cerr << "Error : GetDefaultKinectSensor" << std::endl;
		_tprintf("Error : GetDefaultKinectSensor");
		return -1;
	}

	hResult = m_pKinectSensor->Open();
	if (FAILED(hResult)) {
		//std::cerr << "Error : IKinectSensor::Open()" << std::endl;
		_tprintf("Error : IKinectSensor::Open()");
		return -1;
	}

	// Color source
	//IColorFrameSource* pColorSource;
	hResult = m_pKinectSensor->get_ColorFrameSource(&m_pColorFrameSource);
	if (FAILED(hResult)) {
		//std::cerr << "Error : IKinectSensor::get_ColorFrameSource()" << std::endl;
		_tprintf("Error : IKinectSensor::get_ColorFrameSource()");
		return -1;
	}

	/*IBodyFrameSource* pBodyFrameSource;*/
	hResult = m_pKinectSensor->get_BodyFrameSource(&m_pBodyFrameSource);
	if (FAILED(hResult)) {
		//std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
		_tprintf("Error : IKinectSensor::get_BodyFrameSource()");
		return -1;
	}

	// Color reader
	//IColorFrameReader* pColorReader;
	hResult = m_pColorFrameSource->OpenReader(&m_pColorFrameReader);
	if (FAILED(hResult)) {
		//std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
		_tprintf("Error : IColorFrameSource::OpenReader()");
		return -1;
	}

	//IBodyFrameReader* pBodyReader;
	hResult = m_pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
	if (FAILED(hResult)) {
		//std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
		_tprintf("Error : IBodyFrameSource::OpenReader()");
		return -1;
	}

	// Description
	/*IFrameDescription* pDescription;*/
	hResult = m_pColorFrameSource->get_FrameDescription(&m_pDescription);
	if (FAILED(hResult)) {
		//std::cerr << "Error : IColorFrameSource::get_FrameDescription()" << std::endl;
		_tprintf("Error : IColorFrameSource::get_FrameDescription()");
		return -1;
	}

	// Coordinate Mapper
	//ICoordinateMapper* pCoordinateMapper;
	hResult = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
	if (FAILED(hResult)) {
		//std::cerr << "Error : IKinectSensor::get_CoordinateMapper()" << std::endl;
		_tprintf("Error : IKinectSensor::get_CoordinateMapper()");
		return -1;
	}

	//IFaceFrameSource* pFaceSource[BODY_COUNT];
	DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
		| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
		| FaceFrameFeatures::FaceFrameFeatures_Happy
		| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
		| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
		| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
		| FaceFrameFeatures::FaceFrameFeatures_LookingAway
		| FaceFrameFeatures::FaceFrameFeatures_Glasses
		| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;
	//IFaceFrameReader* pFaceReader[BODY_COUNT];
	for (int count = 0; count < BODY_COUNT; count++) {
		// Source
		hResult = CreateFaceFrameSource(m_pKinectSensor, 0, features, &m_pFaceFrameSources[count]);
		if (FAILED(hResult)) {
			//std::cerr << "Error : CreateFaceFrameSource" << std::endl;
			//_tprintf("Error : CreateFaceFrameSource" + count);
			printf("Error : CreateFaceFrameSource %d.\n", count);
			return -1;
		}

		// Reader
		hResult = m_pFaceFrameSources[count]->OpenReader(&m_pFaceFrameReaders[count]);
		if (FAILED(hResult)) {
			//std::cerr << "Error : IFaceFrameSource::OpenReader()" << std::endl;
			_tprintf("Error : IFaceFrameSource::OpenReader()");
			return -1;
		}
	}
}

//void update()
//{
//
//	int width = 0;
//	int height = 0;
//	m_pDescription->get_Width(&width); // 1920
//	m_pDescription->get_Height(&height); // 1080
//	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);
//
//
//	// Color Frame
//	IColorFrame* m_pColorFrame = nullptr;
//	hResult = m_pColorFrameReader->AcquireLatestFrame(&m_pColorFrame);
//	if (SUCCEEDED(hResult)) {
//		hResult = m_pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat::ColorImageFormat_Bgra);
//		/*if (SUCCEEDED(hResult)) {
//			cv::resize(bufferMat, faceMat, cv::Size(), 0.5, 0.5);
//		}*/
//	}
//	SafeRelease(m_pColorFrame);
//
//	// Body Frame
//	//cv::Point point[BODY_COUNT];
//	IBodyFrame* m_pBodyFrame = nullptr;
//	hResult = m_pBodyFrameReader->AcquireLatestFrame(&m_pBodyFrame);
//	if (SUCCEEDED(hResult)) {
//		IBody* m_pBody[BODY_COUNT] = { 0 };
//		hResult = m_pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, m_pBody);
//		if (SUCCEEDED(hResult)) {
//			for (int count = 0; count < BODY_COUNT; count++) {
//				BOOLEAN bTracked = false;
//				hResult = m_pBody[count]->get_IsTracked(&bTracked);
//				if (SUCCEEDED(hResult) && bTracked) {
//					// Joint
//					Joint joint[JointType::JointType_Count];
//					hResult = m_pBody[count]->GetJoints( JointType::JointType_Count, joint );
//					if( SUCCEEDED( hResult ) ){
//						for( int type = 0; type < JointType::JointType_Count; type++ ){
//							ColorSpacePoint colorSpacePoint = { 0 };
//							m_pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
//							int x = static_cast<int>( colorSpacePoint.X );
//							int y = static_cast<int>( colorSpacePoint.Y );
//							/*if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
//								cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA );
//							}*/
//						}
//					}
//
//					// Set TrackingID to Detect Face
//					UINT64 trackingId = _UI64_MAX;
//					hResult = m_pBody[count]->get_TrackingId(&trackingId);
//					if (SUCCEEDED(hResult)) {
//						m_pFaceFrameSources[count]->put_TrackingId(trackingId);
//					}
//				}
//			}
//		}
//		for (int count = 0; count < BODY_COUNT; count++) {
//			SafeRelease(m_pBody[count]);
//		}
//	}
//	SafeRelease(m_pBodyFrame);
//
//	// Face Frame
//	//std::system("cls");
//	for (int count = 0; count < BODY_COUNT; count++) {
//		IFaceFrame* m_pFaceFrame = nullptr;
//		hResult = m_pFaceFrameReader[count]->AcquireLatestFrame(&m_pFaceFrame);
//		if (SUCCEEDED(hResult) && pFaceFrame != nullptr) {
//			BOOLEAN bFaceTracked = false;
//			hResult = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
//			if (SUCCEEDED(hResult) && bFaceTracked) {
//				IFaceFrameResult* pFaceResult = nullptr;
//				hResult = pFaceFrame->get_FaceFrameResult(&pFaceResult);
//				if (SUCCEEDED(hResult) && pFaceResult != nullptr) {
//					std::vector<std::string> result;
//
//					// Face Point
//					PointF facePoint[FacePointType::FacePointType_Count];
//					hResult = pFaceResult->GetFacePointsInColorSpace(FacePointType::FacePointType_Count, facePoint);
//					if (SUCCEEDED(hResult)) {
//						cv::circle(bufferMat, cv::Point(static_cast<int>(facePoint[0].X), static_cast<int>(facePoint[0].Y)), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA); // Eye (Left)
//						cv::circle(bufferMat, cv::Point(static_cast<int>(facePoint[1].X), static_cast<int>(facePoint[1].Y)), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA); // Eye (Right)
//						cv::circle(bufferMat, cv::Point(static_cast<int>(facePoint[2].X), static_cast<int>(facePoint[2].Y)), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA); // Nose
//						cv::circle(bufferMat, cv::Point(static_cast<int>(facePoint[3].X), static_cast<int>(facePoint[3].Y)), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA); // Mouth (Left)
//						cv::circle(bufferMat, cv::Point(static_cast<int>(facePoint[4].X), static_cast<int>(facePoint[4].Y)), 5, static_cast<cv::Scalar>(color[count]), -1, CV_AA); // Mouth (Right)
//					}
//
//					// Face Bounding Box
//					RectI boundingBox;
//					hResult = pFaceResult->get_FaceBoundingBoxInColorSpace(&boundingBox);
//					if (SUCCEEDED(hResult)) {
//						cv::rectangle(bufferMat, cv::Rect(boundingBox.Left, boundingBox.Top, boundingBox.Right - boundingBox.Left, boundingBox.Bottom - boundingBox.Top), static_cast<cv::Scalar>(color[count]));
//					}
//
//					// Face Rotation
//					Vector4 faceRotation;
//					hResult = pFaceResult->get_FaceRotationQuaternion(&faceRotation);
//					if (SUCCEEDED(hResult)) {
//						int pitch, yaw, roll;
//						ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
//						result.push_back("Pitch, Yaw, Roll : " + std::to_string(pitch) + ", " + std::to_string(yaw) + ", " + std::to_string(roll));
//					}
//
//					// Face Property
//					DetectionResult faceProperty[FaceProperty::FaceProperty_Count];
//					hResult = pFaceResult->GetFaceProperties(FaceProperty::FaceProperty_Count, faceProperty);
//					if (SUCCEEDED(hResult)) {
//						for (int count = 0; count < FaceProperty::FaceProperty_Count; count++) {
//							switch (faceProperty[count]) {
//							case DetectionResult::DetectionResult_Unknown:
//								result.push_back(property[count] + " : Unknown");
//								break;
//							case DetectionResult::DetectionResult_Yes:
//								result.push_back(property[count] + " : Yes");
//								break;
//							case DetectionResult::DetectionResult_No:
//								result.push_back(property[count] + " : No");
//								break;
//							case DetectionResult::DetectionResult_Maybe:
//								result.push_back(property[count] + " : Mayby");
//								break;
//							default:
//								break;
//							}
//						}
//					}
//
//					if (boundingBox.Left && boundingBox.Bottom) {
//						int offset = 30;
//						for (std::vector<std::string>::iterator it = result.begin(); it != result.end(); it++, offset += 30) {
//							cv::putText(bufferMat, *it, cv::Point(boundingBox.Left, boundingBox.Bottom + offset), cv::FONT_HERSHEY_COMPLEX, 1.0f, static_cast<cv::Scalar>(color[count]), 2, CV_AA);
//						}
//					}
//				}
//				SafeRelease(pFaceResult);
//			}
//		}
//		SafeRelease(pFaceFrame);
//	}
//
//	cv::resize(bufferMat, faceMat, cv::Size(), 0.5, 0.5);
//	cv::imshow("Face", faceMat);
//
//	if (cv::waitKey(10) == VK_ESCAPE) {
//		break;
//	}
//}
//}
