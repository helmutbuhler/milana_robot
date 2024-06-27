// Helper module that provides access to image data from a connected usb webcam.
// It also provides timestamp data and provides access to all frames that the webcam provides,
// even if you don't query this API fast enough.
// Right now it only works on Windows and doesn't yet allow you to enumerate the webcams or specify
// the wanted resolution/framerate (but that would be easy to add).

#pragma once

#include "../common/helper.h"
#include <vector>

struct WebcamInfo
{
	bool capturing;
	const char* title;
	float framerate;
	float framerate_estimate;
	s64 last_capture_time;
};

struct WebcamFrame
{
	int cx, cy;
	s64 time;

	// The following is internal data, use webcam_get_frame to get the image data.
	u8* image;
	int size;
	int stride;
	int video_format;
};

// Time is measured here in 100-nanosecond units.
// This is the factor to convert to seconds.
const float webcam_time_to_seconds = 0.0000001f;

bool webcam_init();
void webcam_close();
bool webcam_start_capture(const char* device_name);
void webcam_stop_capture();
WebcamInfo webcam_get_info();

// Get frames that were captured since last call
// You need to call webcam_free_frames on the result to free internal memory
std::vector<WebcamFrame> webcam_get_new_frames();
void webcam_free_frames(std::vector<WebcamFrame>& frames);

// This gets you the image data from a frame. The return value points to cx*cy*3 number of bytes,
// the RGB values of the image. Free it with webcam_free_frame.
u8* webcam_get_frame(const WebcamFrame& frame);
void webcam_free_frame(u8* frame);
