// Little helper module that stores a video with lossy compression in memory.
// It currently just uses jpeg to compress the frames and also applies a very simple
// delta compression if frames don't change too much over time.
// This is useful if you want to capture a longer video without saving to disk and without
// running out of memory too quickly. Because everything is in memory, it's easy to seek around
// and to change it.
// Note: This doesn't actually record anything, as the name might suggest. It just stores images
// in memory. I just couldn't figure out a better name.

#pragma once
#include "../common/common.h"
#include <vector>

using VideoRecorderTime = s64;

struct VideoRecorder;

struct VideoRecorderFrameInfo
{
	s32 compressed_size;
	s32 cx, cy;
	bool is_delta;
	VideoRecorderTime time;
};

VideoRecorder* video_recorder_create(int max_delta_frames);

// Adds a frame to the end. time must be equal or bigger than the time of the last frame.
void video_recorder_add_frame(VideoRecorder* recorder, u8* image, s32 cx, s32 cy, VideoRecorderTime time);

s64 video_recorder_get_num_frames(const VideoRecorder* recorder);
void video_recorder_get_frame_info(const VideoRecorder* recorder, s64 index, VideoRecorderFrameInfo* info);
u8* video_recorder_get_frame(VideoRecorder* recorder, s64 index, bool* fast);

// This assumes that all frames are sorted with respect to time and does a binary search
s64 video_recorder_get_frame_index_from_time(const VideoRecorder* recorder, VideoRecorderTime time);

// This serialization is done with a custom format. No official formats like avi are supported yet.
bool video_recorder_save_to_stream(const VideoRecorder* recorder, FILE* out);
bool video_recorder_save_to_file(const VideoRecorder* recorder, const char* filename);
bool video_recorder_load_from_stream(VideoRecorder* recorder, FILE* in);
bool video_recorder_load_from_file(VideoRecorder* recorder, const char* filename);

// This will delete some frames. frames_to_keep must be as big as the number of frames.
// All frames where frames_to_keep is false will be deleted.
void video_recorder_clear_frames(VideoRecorder* recorder, const std::vector<bool>& frames_to_keep);

// Delete all frames
void video_recorder_reset(VideoRecorder* recorder);

// Destroys the instance. Must be called for each created video recorder.
void video_recorder_destroy(VideoRecorder* recorder);
