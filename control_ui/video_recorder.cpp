#define _CRT_SECURE_NO_WARNINGS

#include "video_recorder.h"
#include <vector>
#include <cstring>

#include "../3rdparty/jpeg/jpge.h"
#include "../3rdparty/jpeg/jpgd.h"

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wunused-result"
#endif

struct VideoFrameHeader
{
	s32 magic = 0x12345678;
	s32 data_size;
	s32 cx, cy;
	s32 delta_no; // 0: no delta frame. >0: number of consecutive delta frames.
	VideoRecorderTime time;
};
struct VideoFrame : VideoFrameHeader
{
	u8* data;
};

struct VideoRecorder
{
	std::vector<VideoFrame> frames;

	int max_delta_frames;

	u8* last_added_frame = 0;

	u8* last_retrieved_frame = 0;
	s64 last_retrieved_frame_index = -1;
};

VideoRecorder* video_recorder_create(int max_delta_frames)
{
	VideoRecorder* v = new VideoRecorder();
	v->max_delta_frames = max_delta_frames;
	return v;
}

void video_recorder_add_frame(VideoRecorder* recorder, u8* image, s32 cx, s32 cy, VideoRecorderTime time)
{
	VideoFrame f;
	f.delta_no = 0;
	f.cx = cx;
	f.cy = cy;
	f.time = time;

	jpge::params params;
	params.m_quality = 45;
	u8 jpeg_buffer[200000];
	int jpeg_buffer_size = -1;
	if (recorder->last_added_frame &&
		recorder->frames.back().cx == cx && recorder->frames.back().cy == cy)
	{
		std::vector<u8> diff_image(cx*cy*3);
		bool fail = false;
		for (int i = 0; i < cx*cy*3; i++)
		{
			int diff = int(image[i]) - int(recorder->last_added_frame[i]) + 128;
			if (diff < 0 || diff >= 256)
			{
				fail = true;
				break;
			}
			diff_image[i] = diff;
		}
		if (!fail)
		{
			jpeg_buffer_size = sizeof(jpeg_buffer);
			bool r = jpge::compress_image_to_jpeg_file_in_memory(jpeg_buffer, jpeg_buffer_size, cx, cy, 3, diff_image.data(), params);
			if (!r)
			{
				printf("JPEG Error\n");
				return;
			}
			f.delta_no = recorder->frames.back().delta_no+1;
		}
	}

	if (jpeg_buffer_size == -1)
	{
		params.m_quality = 70;
		jpeg_buffer_size = sizeof(jpeg_buffer);
		bool r = jpge::compress_image_to_jpeg_file_in_memory(jpeg_buffer, jpeg_buffer_size, cx, cy, 3, image, params);
		if (!r)
		{
			printf("JPEG Error\n");
			return;
		}
	}

	f.data = new u8[jpeg_buffer_size];
	memcpy(f.data, jpeg_buffer, jpeg_buffer_size);
	f.data_size = jpeg_buffer_size;
	recorder->frames.push_back(f);
	
	int width = 0, height = 0, actual_comps = 0;
	u8* frame_data = jpgd::decompress_jpeg_image_from_memory(jpeg_buffer, jpeg_buffer_size,
			&width, &height, &actual_comps, 3);
	assert(actual_comps == 3);
	assert(width == cx);
	assert(height == cy);
	if (f.delta_no)
	{
		assert(recorder->last_added_frame);
		for (int i = 0; i < cx*cy*3; i++)
		{
			int new_value = int(recorder->last_added_frame[i]) + int(frame_data[i]) - 128;
			if (new_value < 0) new_value = 0;
			if (new_value >= 256) new_value = 255;
			frame_data[i] = new_value;
		}
	}
	if (recorder->last_added_frame)
		free(recorder->last_added_frame);
	recorder->last_added_frame = frame_data;
}

s64 video_recorder_get_num_frames(const VideoRecorder* recorder)
{
	return recorder->frames.size();
}

void video_recorder_get_frame_info(const VideoRecorder* recorder, s64 index, VideoRecorderFrameInfo* info)
{
	info->compressed_size = recorder->frames[index].data_size;
	info->cx = recorder->frames[index].cx;
	info->cy = recorder->frames[index].cy;
	info->is_delta = recorder->frames[index].delta_no != 0;
	info->time = recorder->frames[index].time;
}

u8* video_recorder_get_frame(VideoRecorder* recorder, s64 index, bool* fast)
{
	VideoFrame* frame = &recorder->frames[index];
	if (index == recorder->last_retrieved_frame_index)
	{
		if (fast) *fast = true;
		return recorder->last_retrieved_frame;
	}
	if (recorder->last_added_frame && index == (s64)recorder->frames.size()-1)
	{
		if (fast) *fast = true;
		return recorder->last_added_frame;
	}

	s64 index_playback = index;
	if (frame->delta_no != 0)
	{
		index_playback -= frame->delta_no;
		assert(index_playback >= 0);
		frame = &recorder->frames[index_playback];
		assert(frame->delta_no == 0);
	}

	u8* frame_data;
	if (recorder->last_retrieved_frame_index >= index_playback &&
		recorder->last_retrieved_frame_index < index)
	{
		frame_data = recorder->last_retrieved_frame;
		index_playback = recorder->last_retrieved_frame_index;
		frame = &recorder->frames[index_playback];
	}
	else
	{
		if (recorder->last_retrieved_frame)
		{
			free(recorder->last_retrieved_frame);
			recorder->last_retrieved_frame = nullptr;
			recorder->last_retrieved_frame_index = -1;
		}
		int width = 0, height = 0, actual_comps = 0;
		frame_data = jpgd::decompress_jpeg_image_from_memory(frame->data, frame->data_size,
				&width, &height, &actual_comps, 3);
		assert(width == frame->cx);
		assert(height == frame->cy);
		assert(actual_comps == 3);
	}

	while (index_playback != index)
	{
		index_playback++;
		frame = &recorder->frames[index_playback];
		assert(frame->delta_no != 0);

		int width = 0, height = 0, actual_comps = 0;
		u8* frame_data_diff = jpgd::decompress_jpeg_image_from_memory(frame->data, frame->data_size,
				&width, &height, &actual_comps, 3);
		assert(width == frame->cx);
		assert(height == frame->cy);
		assert(actual_comps == 3);
		for (int i = 0; i < width*height*3; i++)
		{
			int new_value = int(frame_data[i]) + int(frame_data_diff[i]) - 128;
			if (new_value < 0) new_value = 0;
			if (new_value >= 256) new_value = 255;
			frame_data[i] = (u8)new_value;
		}
		free(frame_data_diff);
	}

	recorder->last_retrieved_frame_index = index;
	recorder->last_retrieved_frame = frame_data;

	return frame_data;
}

s64 video_recorder_get_frame_index_from_time(const VideoRecorder* recorder, VideoRecorderTime time)
{
	s64 start = 0;
	s64 end = (s64)recorder->frames.size()-1;
	if (end < 0)
		return -1;
	while (start != end)
	{
		s64 middle = (start+end)/2;
		if (recorder->frames[middle].time < time)
			start = middle+1;
		else
			end = middle;
	}
	return start;
}

bool video_recorder_save_to_stream(const VideoRecorder* recorder, FILE* out)
{
	const s32 version = 1;
	fwrite(&version, sizeof(version), 1, out);
	s64 len = (s64)recorder->frames.size();
	fwrite(&len, sizeof(len), 1, out);
	for (s64 i = 0; i < len; i++)
	{
		fwrite(&recorder->frames[i], sizeof(VideoFrameHeader), 1, out);
		fwrite(recorder->frames[i].data, recorder->frames[i].data_size, 1, out);
	}
	return true;
}

bool video_recorder_save_to_file(const VideoRecorder* recorder, const char* filename)
{
	FILE* out = fopen(filename, "wb");
	if (!out)
		return false;
	bool r = video_recorder_save_to_stream(recorder, out);
	fclose(out);
	return r;
}

bool video_recorder_load_from_stream(VideoRecorder* recorder, FILE* in)
{
	video_recorder_reset(recorder);
	s32 version = 0;
	fread(&version, sizeof(version), 1, in);
	if (version != 1)
		return false;
	s64 len = -1;
	fread(&len, sizeof(len), 1, in);
	if (len < 0) return false;
	int counting_delta_no = 0;
	for (s64 i = 0; i < len; i++)
	{
		VideoFrame f;
		fread(&f, sizeof(VideoFrameHeader), 1, in);
		if (f.magic != 0x12345678) return false;
		if (f.cx < 0) return false;
		if (f.cy < 0) return false;
		if (f.data_size < 0) return false;
		if (f.delta_no < 0) return false;
		if (f.delta_no != 0 && f.delta_no != counting_delta_no) return false;
		counting_delta_no = f.delta_no+1;

		f.data = new u8[f.data_size];
		fread(f.data, f.data_size, 1, in);
		recorder->frames.push_back(f);
	}
	return true;
}

bool video_recorder_load_from_file(VideoRecorder* recorder, const char* filename)
{
	FILE* in = fopen(filename, "rb");
	if (!in)
		return false;
	bool r = video_recorder_load_from_stream(recorder, in);
	fclose(in);
	return r;
}

void video_recorder_clear_frames(VideoRecorder* recorder, const std::vector<bool>& frames_to_keep)
{
	assert(recorder->frames.size() == frames_to_keep.size());
	VideoRecorder new_recorder;
	new_recorder.max_delta_frames = recorder->max_delta_frames;
	bool steal_mode = true;
	for (s64 i = 0; i < (s64)recorder->frames.size(); i++)
	{
		if (!frames_to_keep[i])
		{
			steal_mode = false;
			if (new_recorder.last_added_frame)
			{
				free(new_recorder.last_added_frame);
				new_recorder.last_added_frame = nullptr;
			}
			recorder->frames[i].magic = 0;
			continue;
		}
		if (!steal_mode && recorder->frames[i].delta_no == 0)
			steal_mode = true;
		if (steal_mode)
		{
			new_recorder.frames.push_back(recorder->frames[i]);
			recorder->frames[i].magic = 1;
		}
		else
		{
			u8* data = video_recorder_get_frame(recorder, i, 0);
			video_recorder_add_frame(&new_recorder, data, recorder->frames[i].cx, recorder->frames[i].cy, recorder->frames[i].time);
			recorder->frames[i].magic = 0;
		}
	}
	if (new_recorder.last_added_frame)
	{
		free(new_recorder.last_added_frame);
		new_recorder.last_added_frame = nullptr;
	}

	for (s64 i = 0; i < (s64)recorder->frames.size(); i++)
	{
		if (recorder->frames[i].magic == 0)
			delete[] recorder->frames[i].data;
	}
	recorder->frames.clear();
	video_recorder_reset(recorder);
	*recorder = std::move(new_recorder);
}

void video_recorder_reset(VideoRecorder* recorder)
{
	for (s64 i = 0; i < (s64)recorder->frames.size(); i++)
		delete[] recorder->frames[i].data;
	recorder->frames.clear();
	if (recorder->last_added_frame)
	{
		free(recorder->last_added_frame);
		recorder->last_added_frame = nullptr;
	}
	if (recorder->last_retrieved_frame)
	{
		free(recorder->last_retrieved_frame);
		recorder->last_retrieved_frame = nullptr;
	}
	recorder->last_retrieved_frame_index = -1;
}

void video_recorder_destroy(VideoRecorder* recorder)
{
	video_recorder_reset(recorder);
	delete recorder;
}
