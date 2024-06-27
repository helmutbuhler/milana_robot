// Some basic math helper functions
// Also includes some wav serialization functions
#pragma once


#include <cmath>
#include <stdio.h>
#include <vector>
#include <cinttypes>

// This is important on gcc. Otherwise it will call the int math versions.
// I have now enabled warnings that check for this.
// See also: https://stackoverflow.com/questions/19159541/disable-math-h-crap-when-working-with-cmath
using std::tan;
using std::atan;
using std::atan2;
using std::abs;
using std::sin;
using std::cos;
using std::sqrt;
using std::pow;

using u64 = long long unsigned int; // uint64_t produces warnings in printf statements on Linux when used with %llu
using u32 = uint32_t;
using u16 = uint16_t;
using u8  = uint8_t;

using s64 = long long int;
using s32 = int32_t;
using s16 = int16_t;
using s8  = int8_t;
static_assert(sizeof(int) == 4, "");
static_assert(sizeof(u64) == 8, "");
static_assert(sizeof(s64) == 8, "");

#ifndef _MSC_VER
#define sprintf_s(buf, ...) snprintf((buf), sizeof(buf), __VA_ARGS__)
#define strcat_s strcat
#endif

template<typename T>
T sq(T x)
{
	return x*x;
}

inline float clamp(float val, float min, float max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

inline float clamp_and_map(float val, float min_from, float max_from, float min_to, float max_to)
{
	val = clamp(val, min_from, max_from);
	val = (val-min_from)/(max_from-min_from);
	return val*(max_to-min_to)+min_to;
}

inline float my_map(float val, float min_from, float max_from, float min_to, float max_to)
{
	val = (val-min_from)/(max_from-min_from);
	return val*(max_to-min_to)+min_to;
}

inline float rand_0_1()
{
	return rand()%1001 / 1000.0f;
}

inline float sign(float value)
{
	if (value > 0) return 1;
	if (value < 0) return -1;
	return 0;
}

inline float lerp(float a, float b, float t)
{
	return a + (b-a)*t;
}

inline float move_towards(float current, float target, float delta)
{
	if (current > target)
	{
		current -= delta;
		if (current < target)
			current = target;
	}
	else
	{
		current += delta;
		if (current > target)
			current = target;
	}
	return current;
}

inline float update_smooth(float old_val, float new_val, float smooth_factor, int target_delta_time_ms)
{
	// smooth_factor = 0: No smoothing. Value approaches new_val instantly.
	// smooth_factor > 0: Value approaches new_val in such a way that over
	// a time period, the distance to new_val decreases by a constant factor.
	return lerp(new_val, old_val, (float)pow(smooth_factor, (float)target_delta_time_ms));
}

const float pi = 3.141592653589f;
const float tau = pi*2.0f;

inline float to_degree(float rad)
{
	return rad*180/pi;
}
inline float to_rad(float degree)
{
	return degree*pi/180;
}

struct Quaternion
{
    float w;
    float x;
    float y;
    float z;
};

// from: https://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
inline void quaternion_rotate_vector(float& x, float& y, float& z, const Quaternion& rotation)
{
    float num12 = rotation.x + rotation.x;
    float num2 = rotation.y + rotation.y;
    float num = rotation.z + rotation.z;
    float num11 = rotation.w * num12;
    float num10 = rotation.w * num2;
    float num9 = rotation.w * num;
    float num8 = rotation.x * num12;
    float num7 = rotation.x * num2;
    float num6 = rotation.x * num;
    float num5 = rotation.y * num2;
    float num4 = rotation.y * num;
    float num3 = rotation.z * num;
    float num15 = ((x * ((1 - num5) - num3)) + (y * (num7 - num9)))       + (z * (num6 + num10));
    float num14 = ((x * (num7 + num9))       + (y * ((1 - num8) - num3))) + (z * (num4 - num11));
    float num13 = ((x * (num6 - num10))      + (y * (num4 + num11)))      + (z * ((1 - num8) - num5));
    x = num15;
    y = num14;
    z = num13;
}

bool write_wav_internal(const char* filename, void* samples, short bits_per_sample, int num_samples, int sample_rate);
template<typename T>
bool write_wav(const char* filename, T* samples, int num_samples, int sample_rate)
{
	return write_wav_internal(filename, samples, sizeof(T)*8, num_samples, sample_rate);
}

// WAVE file header format
struct WAV_HEADER
{
	unsigned char riff[4];						// RIFF string
	unsigned int overall_size;					// overall size of file in bytes
	unsigned char wave[4];						// WAVE string
	unsigned char fmt_chunk_marker[4];			// fmt string with trailing null char
	unsigned int length_of_fmt;					// length of the format data
	unsigned int format_type;					// format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law
	unsigned int channels;						// no.of channels
	unsigned int sample_rate;					// sampling rate (blocks per second)
	unsigned int byterate;						// SampleRate * NumChannels * BitsPerSample/8
	unsigned int block_align;					// NumChannels * BitsPerSample/8
	unsigned int bits_per_sample;				// bits per sample, 8- 8bits, 16- 16 bits etc
	unsigned char data_chunk_header[4];			// DATA string or FLLR string
	unsigned int data_size;						// NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read
};

template<typename SampleType>
bool read_wav(const char* filename, std::vector<SampleType>& samples, int expected_sample_rate)
{
	FILE* ptr = fopen(filename, "rb");
	if (ptr == NULL)
	{
		printf("Error opening file: %s\n", filename);
		return false;
	}

	WAV_HEADER header;
	size_t read = 0;

	// read header parts

	read = fread(header.riff, sizeof(header.riff), 1, ptr);
	//printf("(1-4): %s \n", header.riff);
	if (header.riff[0] != 'R' ||
		header.riff[1] != 'I' ||
		header.riff[2] != 'F' ||
		header.riff[3] != 'F')
	{
		printf("Error: Unexpected header.\n");
		fclose(ptr);
		return false;
	}

	unsigned char buffer4[4];
	unsigned char buffer2[2];
	read = fread(buffer4, sizeof(buffer4), 1, ptr);
	//printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

	// convert little endian to big endian 4 byte int
	header.overall_size = buffer4[0] |
		(buffer4[1] << 8) |
		(buffer4[2] << 16) |
		(buffer4[3] << 24);

	//printf("(5-8) Overall size: bytes:%u, Kb:%u \n", header.overall_size, header.overall_size / 1024);

	read = fread(header.wave, sizeof(header.wave), 1, ptr);
	//printf("(9-12) Wave marker: %s\n", header.wave);

	read = fread(header.fmt_chunk_marker, sizeof(header.fmt_chunk_marker), 1, ptr);
	//printf("(13-16) Fmt marker: %s\n", header.fmt_chunk_marker);

	read = fread(buffer4, sizeof(buffer4), 1, ptr);
	//printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

	// convert little endian to big endian 4 byte integer
	header.length_of_fmt = buffer4[0] |
		(buffer4[1] << 8) |
		(buffer4[2] << 16) |
		(buffer4[3] << 24);
	//printf("(17-20) Length of Fmt header: %u \n", header.length_of_fmt);

	read = fread(buffer2, sizeof(buffer2), 1, ptr);
	//printf("%u %u \n", buffer2[0], buffer2[1]);

	header.format_type = buffer2[0] | (buffer2[1] << 8);
	if (header.format_type != 1) // PCM
	{
		printf("Error: Unexpected format type.\n");
		fclose(ptr);
		return false;
	}

	read = fread(buffer2, sizeof(buffer2), 1, ptr);
	//printf("%u %u \n", buffer2[0], buffer2[1]);

	header.channels = buffer2[0] | (buffer2[1] << 8);
	//printf("(23-24) Channels: %u \n", header.channels);
	if (header.channels != 1)
	{
		printf("Error: Unexpected number of channels.\n");
		fclose(ptr);
		return false;
	}

	read = fread(buffer4, sizeof(buffer4), 1, ptr);
	//printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

	header.sample_rate = buffer4[0] |
		(buffer4[1] << 8) |
		(buffer4[2] << 16) |
		(buffer4[3] << 24);

	//printf("(25-28) Sample rate: %u\n", header.sample_rate);
	if (header.sample_rate != expected_sample_rate)
	{
		printf("Unexpected sample rate: %dHz. Expected %dHz.\n", header.sample_rate, expected_sample_rate);
		fclose(ptr);
		return false;
	}

	read = fread(buffer4, sizeof(buffer4), 1, ptr);
	//printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

	header.byterate = buffer4[0] |
		(buffer4[1] << 8) |
		(buffer4[2] << 16) |
		(buffer4[3] << 24);
	//printf("(29-32) Byte Rate: %u , Bit Rate:%u\n", header.byterate, header.byterate * 8);

	read = fread(buffer2, sizeof(buffer2), 1, ptr);
	//printf("%u %u \n", buffer2[0], buffer2[1]);

	header.block_align = buffer2[0] |
		(buffer2[1] << 8);
	//printf("(33-34) Block Alignment: %u \n", header.block_align);

	read = fread(buffer2, sizeof(buffer2), 1, ptr);
	//printf("%u %u n", buffer2[0], buffer2[1]);

	header.bits_per_sample = buffer2[0] |
		(buffer2[1] << 8);
	//printf("(35-36) Bits per sample: %u \n", header.bits_per_sample);

	read = fread(header.data_chunk_header, sizeof(header.data_chunk_header), 1, ptr);
	//printf("(37-40) Data Marker: %s \n", header.data_chunk_header);

	read = fread(buffer4, sizeof(buffer4), 1, ptr);
	//printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

	header.data_size = buffer4[0] |
		(buffer4[1] << 8) |
		(buffer4[2] << 16) |
		(buffer4[3] << 24);
	//printf("(41-44) Size of data chunk: %u \n", header.data_size);


	// calculate no.of samples
	long num_samples = (8 * header.data_size) / (header.channels * header.bits_per_sample);
	//printf("Number of samples:%lu \n", num_samples);

	long size_of_each_sample = (header.channels * header.bits_per_sample) / 8;
	//printf("Size of each sample:%ld bytes\n", size_of_each_sample);
	if (size_of_each_sample != sizeof(SampleType))
	{
		printf("Unexpected size sample: %ld bytes. Expected %d bytes.\n", size_of_each_sample, (int)sizeof(SampleType));
		fclose(ptr);
		return false;
	}

	// calculate duration of file
	float duration_in_seconds = (float)header.overall_size / header.byterate;
	//printf("Approx.Duration in seconds=%f\n", duration_in_seconds);

	long bytes_in_each_channel = (size_of_each_sample / header.channels);

	samples.resize(num_samples);
	for (int i = 0; i < num_samples; i++)
	{
		//printf("==========Sample %ld / %ld=============\n", i+1, num_samples);
		char data_buffer[sizeof(SampleType)];
		read = fread(data_buffer, sizeof(data_buffer), 1, ptr);
		if (read == 1)
		{
			// dump the data read
			int data_in_channel = 0;
			int offset = 0; // move the offset for every iteration in the loop below
			for (unsigned int xchannels = 0; xchannels < header.channels; xchannels++) {
				//printf("Channel#%d : ", (xchannels + 1));
				// convert data from little endian to big endian based on bytes in each channel sample
				if (bytes_in_each_channel == 4) {
					data_in_channel = (data_buffer[offset] & 0x00ff) |
						((data_buffer[offset + 1] & 0x00ff) << 8) |
						((data_buffer[offset + 2] & 0x00ff) << 16) |
						(data_buffer[offset + 3] << 24);
				}
				else if (bytes_in_each_channel == 2) {
					data_in_channel = (data_buffer[offset] & 0x00ff) |
						(data_buffer[offset + 1] << 8);
				}
				else if (bytes_in_each_channel == 1) {
					data_in_channel = data_buffer[offset] & 0x00ff;
					data_in_channel -= 128; //in wave, 8-bit are unsigned, so shifting to signed
				}

				offset += bytes_in_each_channel;
				//printf("%d ", data_in_channel);
				samples[i] = data_in_channel;

				//printf(" | ");
			}

			//printf("\n");
		}
		else
		{
			printf("Error reading file. %d bytes\n", (int)read);
			fclose(ptr);
			return false;
		}
	}
	
	fclose(ptr);
	return true;
}
