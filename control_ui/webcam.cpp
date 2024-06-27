// This code is mostly based on:
//      https://elcharolin.wordpress.com/2017/08/28/webcam-capture-with-the-media-foundation-sdk/
// and: https://github.com/HowardsPlayPen/WebCamMicrosoftFoundation
//
#include "webcam.h"

#ifdef _MSC_VER
//media foundation headers
#define WINDOWS_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#include <mfidl.h> 
#include <Mfapi.h> 
#include <Mfreadwrite.h>
#include <Shlwapi.h>

#include <stdio.h>
#include <vector>
#include <iostream>
#include <assert.h>
#include <algorithm>

//include and lib dependencies for Media Foundation
#pragma comment(lib,"Mfplat.lib")
#pragma comment(lib,"Mf.lib")
#pragma comment(lib,"Mfreadwrite.lib")
#pragma comment(lib,"mfuuid.lib")
#pragma comment(lib,"shlwapi.lib")

const int My_MFVideoFormat_RGB24 = 1;
const int My_MFVideoFormat_YUY2  = 2;
const int My_MFVideoFormat_NV12  = 3;


struct RGBPixel
{
	BYTE r;
	BYTE g;
	BYTE b;
};

class Media : public IMFSourceReaderCallback
{
	CRITICAL_SECTION criticalSection;
	long referenceCount = 1;
	WCHAR* wSymbolicLink = NULL;
	UINT32 cchSymbolicLink = 0;
	std::vector<WebcamFrame> frames;

public:
	IMFSourceReader* sourceReader = NULL;
	LONG stride = 0;
	int video_format;
	UINT height = 0;
	UINT width = 0;
	float framerate = 0;
	char deviceNameString[2048] = "";
	int framerateCounter = 0;
	float framerate_estimate = 0;
	s64 lastFrameTime = 0;
	s64 last_capture_time = 0;

	HRESULT CreateCaptureDevice(const char* deviceName);
	HRESULT SetSourceReader(IMFActivate *device);
	HRESULT IsMediaTypeSupported(IMFMediaType* type);
	HRESULT GetDefaultStride(IMFMediaType *pType, LONG *plStride);
	void Close();
	Media();
	~Media();	
	std::vector<WebcamFrame> get_new_frames();

	// the class must implement the methods from IUnknown 
	STDMETHODIMP QueryInterface(REFIID iid, void** ppv);
	STDMETHODIMP_(ULONG) AddRef();
	STDMETHODIMP_(ULONG) Release();

	//  the class must implement the methods from IMFSourceReaderCallback 
	STDMETHODIMP OnReadSample(HRESULT status, DWORD streamIndex, DWORD streamFlags, LONGLONG timeStamp, IMFSample *sample);
	STDMETHODIMP OnEvent(DWORD, IMFMediaEvent *) { return S_OK; }
	STDMETHODIMP OnFlush(DWORD) { return S_OK; }
};

Media::Media()
{
	InitializeCriticalSection(&criticalSection);
}

Media::~Media()
{
	Close();
	// TODO:
	// If this is called while the capture is still active or has been closed a short time ago,
	// it's possible that OnReadSample will be called after sourceReader has been released
	// and the cs is destroyed. That would result in a crash in OnReadSample.
	for (WebcamFrame& frame : frames)
	{
		delete[] frame.image;
	}
	DeleteCriticalSection(&criticalSection);
}

void Media::Close()
{
	EnterCriticalSection(&criticalSection);

	if(sourceReader)
	{
		sourceReader->Release();
		sourceReader = NULL;
	}

	CoTaskMemFree(wSymbolicLink);
	wSymbolicLink = NULL;
	cchSymbolicLink = 0;

	LeaveCriticalSection(&criticalSection);
}

HRESULT Media::CreateCaptureDevice(const char* deviceName)
{
	HRESULT hr = S_OK;
	IMFAttributes *attributes = NULL;
	UINT32 count = 0;
	IMFActivate **devices = NULL;
	
	//this is important!!
	hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
	if (FAILED(hr)) goto fail;

	// Create an attribute store to specify enumeration parameters.
	hr = MFCreateAttributes(&attributes, 1);
	if (FAILED(hr)) goto fail;

	//The attribute to be requested is devices that can capture video
	hr = attributes->SetGUID(
		MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
		MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
	);
	if (FAILED(hr)) goto fail;
	
	//Enumerate the video capture devices
	hr = MFEnumDeviceSources(attributes, &devices, &count);
	
	if (FAILED(hr)) goto fail;
	//if there are any available devices

	for (int i = 0; i < (int)count; i++)
	{		
		// Get the human-friendly name of the device
		deviceNameString[0] = 0;
		WCHAR *nameString = NULL;
		UINT32 cchName;
		hr = devices[i]->GetAllocatedString(
			MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
			&nameString, &cchName);
		if (SUCCEEDED(hr))
		{
	        int r = WideCharToMultiByte(CP_UTF8, 0, nameString, cchName, deviceNameString, 2048, NULL, NULL);
			deviceNameString[std::min(r, 2048-1)] = 0;
		}
		CoTaskMemFree(nameString);
		if (strcmp(deviceNameString, deviceName) != 0)
			continue;
		hr = SetSourceReader(devices[i]);
		break;
	}

	fail:
	if (attributes)
	{
		attributes->Release();
		attributes = NULL;
	}
	for (DWORD i = 0; i < count; i++)
	{
		if (&devices[i])
		{
			devices[i]->Release();
			devices[i] = NULL;
		}
	}
	CoTaskMemFree(devices);
	return hr;
}

HRESULT Media::SetSourceReader(IMFActivate *device)
{
	HRESULT hr = S_OK;

	IMFMediaSource *source = NULL;
	IMFAttributes *attributes = NULL;
	IMFMediaType *mediaType = NULL;

	EnterCriticalSection(&criticalSection);

	// Note: I have found the following call to fail with the error: 		
	//       hr	0xc00d36b4 : The data specified for the media type is invalid, inconsistent, or not supported by this object.	HRESULT
	// So some vidcap devices are not happy with this combination.
	hr = device->ActivateObject(__uuidof(IMFMediaSource), (void**)&source);

	//get symbolic link for the device
	if(SUCCEEDED(hr))
		hr = device->GetAllocatedString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, &wSymbolicLink, &cchSymbolicLink);
	//Allocate attributes
	if (SUCCEEDED(hr))
		hr = MFCreateAttributes(&attributes, 2);
	//get attributes
	if (SUCCEEDED(hr))
		hr = attributes->SetUINT32(MF_READWRITE_DISABLE_CONVERTERS, TRUE);
	// Set the callback pointer.
	if (SUCCEEDED(hr))
		hr = attributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, this);
	//Create the source reader
	if (SUCCEEDED(hr))
		hr = MFCreateSourceReaderFromMediaSource(source,attributes,&sourceReader);
	// Try to find a suitable output type.
	if (SUCCEEDED(hr))
	{
		/// Update: This for loop was aimed at iterating across all supported (native) media types - but there were a couple of strange things which have been updated below
		int i = 26; // hack
		for (; ; i++)
		{
			hr = sourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM,i,&mediaType);

			/// Note: FAILED() here does not stop on S_FALSE - which is correct, as it should continue in this loop but might not sound logical
			/// but if you loop through ALL supported media types the GetNativeMediaType will eventually return a code to indicate there are no more types available
			if (FAILED(hr)) { break; }
			
			// Note: I moved this call to here as elsewhere in this code it uses stride without checking if it was zero
			//Get the stride for this format so we can calculate the number of bytes per pixel
			GetDefaultStride(mediaType, &stride);

			hr = IsMediaTypeSupported(mediaType);
			if (FAILED(hr)) { break; } // Note: this is not detecting S_FALSE
			// Get width and height
			MFGetAttributeSize(mediaType, MF_MT_FRAME_SIZE, &width, &height);

			UINT32 numerator   = 0;
			UINT32 denominator = 1;
			MFGetAttributeRatio(mediaType, MF_MT_FRAME_RATE, &numerator, &denominator);
			framerate = (float)numerator / denominator;

			if (S_OK == hr)
				hr = sourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, NULL, mediaType);

			if (mediaType) 
			{
				mediaType->Release();
				mediaType = NULL;
			}

			printf("%d  %dx%d  %f  %d\n", i, width, height, framerate, (int)(hr == S_OK));
			//if (width > 480) continue;
			//if (framerate < 28) continue;
			/// Note: this SUCCESS(hr) is WRONG - i.e. if 'hr' was S_FALSE (did not find media type) then this still goes ahead thinking it HAD found an output type
			if (S_OK == hr)// Found an output type - by explicityly checking for S_OK
				break;
		}
		printf("Done %d\n", i);
	}
	if (SUCCEEDED(hr))
	{
		// Ask for the first sample.
		hr = sourceReader->ReadSample((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM,	0, NULL, NULL,NULL,NULL);
	}

	if (FAILED(hr))
	{
		if (source)
		{
			source->Shutdown();	
		}
		Close();
	}
	if (source) { source->Release(); source = NULL; }
	if (attributes) { attributes->Release(); attributes = NULL; }
	if (mediaType) { mediaType->Release(); mediaType = NULL; }

	LeaveCriticalSection(&criticalSection);
	return hr;
}

HRESULT Media::IsMediaTypeSupported(IMFMediaType *pType)
{
	HRESULT hr = S_OK;

	GUID subtype = { 0 };

	hr = pType->GetGUID(MF_MT_SUBTYPE, &subtype);
	if (FAILED(hr))	{return hr;	}

	video_format = 0;

	if (subtype == MFVideoFormat_RGB24) { video_format = My_MFVideoFormat_RGB24; return S_OK; }
	if (subtype == MFVideoFormat_YUY2 ) { video_format = My_MFVideoFormat_YUY2 ; return S_OK; }
	if (subtype == MFVideoFormat_NV12 ) { video_format = My_MFVideoFormat_NV12 ; return S_OK; }

	/// Update: Returning S_FALSE here is unclear as SUCCESS(S_FALSE) is actually true 
	/// See https://docs.microsoft.com/en-us/windows/win32/learnwin32/error-handling-in-com
	/// So either return S_FALSE and check for if(S_FALSE == hr) specifically, but not if you use SUCCESS() or FAIL() as was done in this wider code
	return S_FALSE; 
}


//From IUnknown 
STDMETHODIMP Media::QueryInterface(REFIID riid, void** ppvObject)
{
	static const QITAB qit[] = {QITABENT(Media, IMFSourceReaderCallback),{ 0 },};
	return QISearch(this, qit, riid, ppvObject);
}
//From IUnknown
ULONG Media::Release()
{
	ULONG count = InterlockedDecrement(&referenceCount);
	if (count == 0)
		delete this;
	// For thread safety
	return count;
}
//From IUnknown
ULONG Media::AddRef()
{
	return InterlockedIncrement(&referenceCount);
}


// Calculates the default stride based on the format and size of the frames
HRESULT Media::GetDefaultStride(IMFMediaType *type, LONG *stride)
{
	// TODO: This method seems buggy on some resolutions
	LONG tempStride = 0;

	// Try to get the default stride from the media type.
	HRESULT hr = type->GetUINT32(MF_MT_DEFAULT_STRIDE, (UINT32*)&tempStride);
	if (FAILED(hr))
	{
		//Setting this atribute to NULL we can obtain the default stride
		GUID subtype = GUID_NULL;

		UINT32 width = 0;
		UINT32 height = 0;

		// Obtain the subtype
		hr = type->GetGUID(MF_MT_SUBTYPE, &subtype);
		//obtain the width and height
		if (SUCCEEDED(hr))
			hr = MFGetAttributeSize(type, MF_MT_FRAME_SIZE, &width, &height);
		//Calculate the stride based on the subtype and width
		if (SUCCEEDED(hr))
			hr = MFGetStrideForBitmapInfoHeader(subtype.Data1, width, &tempStride);
		// set the attribute so it can be read
		if (SUCCEEDED(hr))
			(void)type->SetUINT32(MF_MT_DEFAULT_STRIDE, UINT32(tempStride));
	}

	if (SUCCEEDED(hr))
		*stride = tempStride;
	return hr;
}

//Method from IMFSourceReaderCallback
HRESULT Media::OnReadSample(HRESULT status, DWORD streamIndex, DWORD streamFlags, LONGLONG timeStamp, IMFSample *sample)
{
	IMFMediaBuffer *mediaBuffer = nullptr;

	WebcamFrame frame;
	frame.image = nullptr;
	frame.cx = width;
	frame.cy = height;
	frame.stride = stride;
	frame.time = timeStamp;
	frame.video_format = video_format;

	HRESULT hr = status;
	if (SUCCEEDED(hr) && sample)
	{
		// Get the video frame buffer from the sample.
		hr = sample->GetBufferByIndex(0, &mediaBuffer);
		// Save the frame.
		if (SUCCEEDED(hr))
		{
			BYTE* data;
			mediaBuffer->Lock(&data, NULL, NULL);
			DWORD length = 0;
			mediaBuffer->GetCurrentLength(&length);
			//This is a good place to perform color conversion and drawing
			//Instead we're copying the data to a buffer
			frame.image = new BYTE[length];
			if (frame.image)
			{
				memcpy(frame.image, data, length);
				frame.size = length;
			}
		}
	}

	IMFSourceReader* my_source_reader = 0;
	
	if (frame.image)
	{
		EnterCriticalSection(&criticalSection);
		if (sourceReader)
		{
			frames.push_back(frame);
			framerateCounter++;
			if ((timeStamp-lastFrameTime)*webcam_time_to_seconds > 5.0f)
			{
				lastFrameTime = timeStamp;
				framerate_estimate = framerateCounter / 5.0f;
				framerateCounter = 0;
			}
			last_capture_time = timeStamp;

			my_source_reader = sourceReader;
			// It's possible that this is released by the main thread after we leave the cs
			// below, so we add a ref.
			my_source_reader->AddRef();
		}
		LeaveCriticalSection(&criticalSection);
	}
	else
	{
		// It's possible that we failed in getting the frame above.
		// We still want to request the next frame though. Otherwise
		// a single failure will stop the stream entirely.
		EnterCriticalSection(&criticalSection);
		my_source_reader = sourceReader;
		if (sourceReader) my_source_reader->AddRef();
		LeaveCriticalSection(&criticalSection);
	}

	// Request the next frame.
	if (my_source_reader)
	{
		hr = my_source_reader->ReadSample((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, NULL, NULL, NULL, NULL);
		my_source_reader->Release();
	}

	/*if (FAILED(hr))
	{
		//Notify there was an error
		printf("Error HRESULT = 0x%d", hr);
		//PostMessage(NULL, 1, (WPARAM)hr, 0L);
	}*/
	
	if (mediaBuffer) 
	{
		mediaBuffer->Unlock();
		mediaBuffer->Release(); 
		mediaBuffer = NULL;
	}

	return hr;
}

inline BYTE Clip(int clr)
{
	return (BYTE)(clr < 0 ? 0 : (clr > 255 ? 255 : clr));
}

RGBPixel ConvertYCrCbToRGB(
	int  	y,
	int  	cr,
	int  	cb)
{
	RGBPixel rgb;
	
	int c = y - 16;
	int d = cb - 128;
	int e = cr - 128;
	
	rgb.r = Clip((298 * c + 409 * e + 128) >> 8);
	rgb.g = Clip((298 * c - 100 * d - 208 * e + 128) >> 8);
	rgb.b = Clip((298 * c + 516 * d + 128) >> 8);
	
	return rgb;
}

/// Note: this is taken from https://software.intel.com/sites/landingpage/mmsf/documentation/preview_8cpp.html#af3d9de67be8e955b824d4b497bba4c96
/// In the online version (from Intel) the comment below on Byte order was incorrect - although the code was fine (!). Never trust documentation..(?)
void TransformImage_YUY2(RGBPixel* pDest,
	const BYTE* pSrc,
	LONG  	lSrcStride,
	DWORD  	dwWidthInPixels,
	DWORD  	dwHeightInPixels
)
{
	for (DWORD y = 0; y < dwHeightInPixels; y++)
	{
		RGBPixel* pDestPel = pDest;
		WORD * pSrcPel = (WORD*)pSrc;

		for (DWORD x = 0; x < dwWidthInPixels; x += 2)
		{
			// Byte order is Y0 U0 Y1 V0  /// NOTE: On the Intel site this comment was wrong.

			int y0 = (int)LOBYTE(pSrcPel[x]);
			int u0 = (int)HIBYTE(pSrcPel[x]);
			int y1 = (int)LOBYTE(pSrcPel[x + 1]);
			int v0 = (int)HIBYTE(pSrcPel[x + 1]);

			pDestPel[x] = ConvertYCrCbToRGB(y0, v0, u0);
			pDestPel[x + 1] = ConvertYCrCbToRGB(y1, v0, u0);
		}

		pSrc +=  lSrcStride;
		pDest += dwWidthInPixels;
	}
}

// source: https://www.programmersought.com/article/20801140501/
void NV12_To_RGB(RGBPixel* rgb,
		const BYTE* yuyv,
		int lSrcStride,
		int width,
		int height)
{
	// todo: lSrcStride isn't used yet
	const int nv_start = lSrcStride * height;
	int rgb_index = 0;
 
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			int nv_index = i / 2 * width + j - j % 2;
			BYTE y = yuyv[rgb_index];
 
			BYTE u = yuyv[nv_start + nv_index + 1]; 
			BYTE v = yuyv[nv_start + nv_index];
 
			BYTE r = Clip(y + (140 * (v - 128)) / 100);
			BYTE g = Clip(y - (34 * (u - 128)) / 100 - (71 * (v - 128)) / 100);
			BYTE b = Clip(y + (177 * (u - 128)) / 100);
 
			// don't know why this must be reversed here:
			rgb[rgb_index].r = b;
			rgb[rgb_index].g = g;
			rgb[rgb_index].b = r;
			rgb_index++;
		}
	}
}

std::vector<WebcamFrame> Media::get_new_frames()
{
	EnterCriticalSection(&criticalSection);
	std::vector<WebcamFrame> r = std::move(frames);
	frames.clear();
	LeaveCriticalSection(&criticalSection);
	return r;
}

Media* m;
char titleString[2048];

bool webcam_init()
{
	assert(!m);
	m = new Media();
	return true;
}

bool webcam_start_capture(const char* device_name)
{
	if (!m)
		return false;
	m->CreateCaptureDevice(device_name);
	if (!m->width || !m->height)
	{
		return false;
	}

	sprintf_s(titleString, "%s %dx%d %.2fHz", m->deviceNameString, m->width, m->height, m->framerate);
	return true;
}

void webcam_stop_capture()
{
	if (!m)
		return;
	m->Close();
}

void webcam_close()
{
	delete m;
	m = nullptr;
}

WebcamInfo webcam_get_info()
{
	WebcamInfo w = {0};
	w.title = titleString;
	if (m)
	{
		w.capturing = (m->sourceReader != nullptr);
		w.framerate = m->framerate;
		w.framerate_estimate = m->framerate_estimate;
		w.last_capture_time = m->last_capture_time;
	}
	return w;
}

std::vector<WebcamFrame> webcam_get_new_frames()
{
	return m->get_new_frames();
}

void webcam_free_frames(std::vector<WebcamFrame>& frames)
{
	for (int i = 0; i < frames.size(); i++)
	{
		delete[] frames[i].image;
	}
	frames.clear();
}

u8* webcam_get_frame(const WebcamFrame& frame)
{
	RGBPixel* pixelBuffer = new RGBPixel[frame.cx*frame.cy];
	if (frame.video_format == My_MFVideoFormat_RGB24)
	{ 
		memcpy(pixelBuffer, frame.image, frame.cx*frame.cy*3);
	}
	else if (frame.video_format == My_MFVideoFormat_YUY2)
	{
		TransformImage_YUY2(pixelBuffer, frame.image, frame.stride, frame.cx, frame.cy);
	}
	else if (frame.video_format == My_MFVideoFormat_NV12)
	{
		NV12_To_RGB(pixelBuffer, frame.image, frame.stride, frame.cx, frame.cy);
	}
	else
	{
		assert(0);
	}
	return (u8*)pixelBuffer;
}

void webcam_free_frame(u8* frame)
{
	delete[] (RGBPixel*)frame;
}

#else

bool webcam_init()
{
	return false;
}
void webcam_close()
{
}

bool webcam_start_capture(const char* device_name)
{
	return false;
}

void webcam_stop_capture()
{
}

WebcamInfo webcam_get_info()
{
	WebcamInfo w = {0};
	return w;
}

std::vector<WebcamFrame> webcam_get_new_frames()
{
	return {};
}

void webcam_free_frames(std::vector<WebcamFrame>& frames)
{
}

u8* webcam_get_frame(const WebcamFrame& frame)
{
	return nullptr;
}

void webcam_free_frame(u8* frame)
{
}

#endif