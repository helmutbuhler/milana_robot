#define _CRT_SECURE_NO_WARNINGS
#include "helper.h"
#include <assert.h>
#include <stdio.h>


/******************************
*  Magic file format strings. *
******************************/
const char fChunkID[]     = {'R', 'I', 'F', 'F'};
const char fFormat[]      = {'W', 'A', 'V', 'E'};
const char fSubchunk1ID[] = {'f', 'm', 't', ' '};
const char fSubchunk2ID[] = {'d', 'a', 't', 'a'};

const unsigned short N_CHANNELS = 1;
const unsigned short BITS_PER_BYTE = 8;

bool write_wav_internal(const char* filename, void* samples, short bits_per_sample, int num_samples, int sample_rate)
{
	const static unsigned int fSubchunk1Size = 16;
	const static unsigned short fAudioFormat = 1;

	unsigned int fByteRate = sample_rate * N_CHANNELS *
							 bits_per_sample / BITS_PER_BYTE;

	unsigned short fBlockAlign = N_CHANNELS * bits_per_sample / BITS_PER_BYTE;
	unsigned int fSubchunk2Size;
	unsigned int fChunkSize;

	FILE* fout = fopen(filename, "wb");
	if (!fout)
	{
		printf("Cannot open %s for writing!\n", filename);
		return false;
	}

	fSubchunk2Size = num_samples * N_CHANNELS * bits_per_sample / BITS_PER_BYTE;
	fChunkSize = 36 + fSubchunk2Size;

	// Writing the RIFF header:
	fwrite(&fChunkID, 1, sizeof(fChunkID),      fout);
	fwrite(&fChunkSize,  sizeof(fChunkSize), 1, fout);
	fwrite(&fFormat, 1,  sizeof(fFormat),       fout);

	// "fmt" chunk:
	fwrite(&fSubchunk1ID, 1, sizeof(fSubchunk1ID),      fout);
	fwrite(&fSubchunk1Size,  sizeof(fSubchunk1Size), 1, fout);
	fwrite(&fAudioFormat,    sizeof(fAudioFormat),   1, fout);
	fwrite(&N_CHANNELS,      sizeof(N_CHANNELS),     1, fout);
	fwrite(&sample_rate,     sizeof(sample_rate),    1, fout);
	fwrite(&fByteRate,       sizeof(fByteRate),      1, fout);
	fwrite(&fBlockAlign,     sizeof(fBlockAlign),    1, fout);
	fwrite(&bits_per_sample, sizeof(bits_per_sample), 1, fout);

	/* "data" chunk: */
	fwrite(&fSubchunk2ID, 1, sizeof(fSubchunk2ID),      fout);
	fwrite(&fSubchunk2Size,  sizeof(fSubchunk2Size), 1, fout);

	/* sound data: */
	fwrite(samples, bits_per_sample / BITS_PER_BYTE, num_samples * N_CHANNELS, fout);
	fclose(fout);
	return true;
}

