// Here we host a server that gives access to Automatic Speech Recognition (optionally with a LLM)
// The server basically does this:
// - Receives audio samples.
// - Passes those audio samples to whisper.cpp (the model from OpenAI) to translate it into text.
// - Optionally passes that text into a LLM to generate a text response.
// - Returns either the original text or the response back to the client.
// 
// Not everything what is heard in the input audio stream is processed. First a passphrase must be
// said ("OK, Milana"), then the following audio is further processed until silence is detected for
// a few seconds.
// 
// This passphrase (or prompt in the code) detection, and the silence detection, is done with a small
// whisper model (ggml-small.bin). Once a passphrase is detected, the audio after the passphrase is
// processed with a bigger model (ggml-large-v3.bin).
// The LLM stuff is done in chat.cpp.
//
// This file still has a lot of debugging code in it. You can see that I struggled a bit to make it work
// reliably.
//
// I ran this on a NVIDIA GeForce RTX 2080 Ti with 11GB. If you want to run it with a LLM, you need at least
// that much GPU memory. Sadly, the Jetson Nano isn't capable of running this.


// Optionally write the audio stream to a file.
#define WRITE_TO_FILE 1

// Optionally read audio from a file, instead of receiving it from the client.
#define DO_INPUT_WAV 0
//#define INPUT_WAV "dusty.wav"
//#define INPUT_WAV "pipe_rec_1.wav"
//#define INPUT_WAV "audiotest2.wav"
//#define INPUT_WAV "audiotest5.wav"
//#define INPUT_WAV "audiotest8.wav"
//#define INPUT_WAV "usb_jetson_2_rec.wav"
//#define INPUT_WAV "run_talking_motors_still.wav"
//#define INPUT_WAV "babble_5dB.wav"
#define INPUT_WAV "../asr/audiofiles/milana_test_2.wav"
//#define INPUT_WAV "../asr/audiofiles/transc_test_1.wav"
//#define INPUT_WAV "../asr/audiofiles/transc_test_2.wav"

// Print info about prompt search to console
#define PRINT_PROMT_SEARCH 1

// Set to 0 if audio should be recorded directly from a local microphone
#define DO_SERVER 1

// Should an LLM be used to generate responses (1) or just return the transcribed text (0)
#define DO_CHAT 1

// Test LLM
#define DO_CHAT_ONLY_TEST 0

// Used for debugging prompts
#define DO_PROMPT_SAVE 0
//#define DO_PROMPT_SAVE 1
//#define DO_PROMPT_SAVE 2

// Put the big whipser model on GPU?
#define DO_TRANSCRIPT_GPU 1

// Put the small whipser model on GPU?
#define DO_PROMPT_GPU 0

const unsigned short asr_port = 60125;

const unsigned int SAMPLE_RATE = 16000;

#define MODEL_PATH "../models/"

const int num_models = 1;
const char *const models[num_models]
{
	//"ggml-tiny.bin",
	//"ggml-tiny.en.bin",
	//"ggml-base.bin",
	//"ggml-base.en.bin",
	"ggml-small.bin",/////////////
	//"ggml-small.en.bin",
	//"ggml-medium.bin",
	//"ggml-medium.en.bin",
};

//const char* model_transcribe = MODEL_PATH "ggml-tiny.en.bin";
//const char* model_transcribe = MODEL_PATH "ggml-tiny.bin";
//const char* model_transcribe = MODEL_PATH "ggml-base.en.bin";
//const char* model_transcribe = MODEL_PATH "ggml-small.bin";
//const char* model_transcribe = MODEL_PATH "ggml-medium.bin";
//const char* model_transcribe = MODEL_PATH "ggml-large.bin";/////////
const char* model_transcribe = MODEL_PATH "ggml-large-v3.bin";

#define _CRT_SECURE_NO_WARNINGS

#include "../common/helper.h"
#include "../common/time_helper.h"
#include "../common/network.h"
#include "../llm/chat.h"
#include "../3rdparty/whisper.cpp/whisper.h"
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <algorithm>
#include <mutex>
#include <vector>
#include <cstring>
#include <string>
#include <sstream>
#include <cassert>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <fstream>
#include <algorithm>
#include <stdio.h> 

extern "C" int64_t ggml_time_ms(void);

#include "../3rdparty/portaudio/portaudio.h"
#define PA_SAMPLE_TYPE  paInt16

#if 0
#include "rnnoise.h"
void rnnoise_process_frame_helper(DenoiseState *st, float *out, const float *in, int count)
{
	const int frame_size = 480;
	while (count >= frame_size)
	{
		rnnoise_process_frame(st, out, in);
		out += frame_size;
		in += frame_size;
		count -= frame_size;
	}
	if (count == 0)
		return;
	//printf("do remaining\n");
	float out_remain[frame_size] = {0};
	float in_remain[frame_size] = {0};
	memcpy(in_remain, in, count*sizeof(float));
	rnnoise_process_frame(st, out_remain, in_remain);
	memcpy(out, out_remain, count*sizeof(float));
}
#endif


bool do_transcription(
		whisper_context* ctx, whisper_context* ctx_prompt,
		float initial_skip, float* prompt_errors, float* rt_factor, SOCKET s);

int point_vec_min_distance(const std::vector<int>& a, int b)
{
	int min_dist = 1000000;
	for (int i = 0; i < a.size(); i++)
		min_dist = std::min(min_dist, abs(a[i]-b));
	return min_dist;
}

int vec_vec_max_distance(const std::vector<int>& a, const std::vector<int>& b)
{
	int max_dist = 0;
	for (int i = 0; i < b.size(); i++)
		max_dist = std::max(max_dist, point_vec_min_distance(a, b[i]));
	for (int i = 0; i < a.size(); i++)
		max_dist = std::max(max_dist, point_vec_min_distance(b, a[i]));
	return max_dist;
}

int vec_vec_max_distance2(const std::vector<int>& a, const std::vector<int>& b)
{
	int errors = 0;
	for (int i = 0; i < b.size(); i++)
	{
		int dist = point_vec_min_distance(a, b[i]);
		if (dist > 1500)
		{
			printf("No prompt found at: %dms\n", b[i]);
			errors++;
		}
	}
	for (int i = 0; i < a.size(); i++)
	{
		int dist = point_vec_min_distance(b, a[i]);
		if (dist > 1500)
		{
			printf("Extra prompt at: %dms\n", a[i]);
			errors++;
		}
	}
	return errors;
}

char my_to_upper(char c)
{
	if (c >= 'a' && c <= 'z') return c+'A'-'a';
	return c;
}

bool starts_with(const char* s, const char* prefix)
{
	while (true)
	{
		if (*prefix == 0) return true;
		if (*s == 0) return false;
		if (my_to_upper(*s) != my_to_upper(*prefix)) return false;
		s++;
		prefix++;
	}
}

// compute similarity between two strings using Levenshtein distance
static float similarity(const std::string & s0, const std::string & s1)
{
	int len0 = (int)s0.size() + 1;
	int len1 = (int)s1.size() + 1;

	std::vector<int> col(len1, 0);
	std::vector<int> prevCol(len1, 0);

	for (int i = 0; i < len1; i++) {
		prevCol[i] = i;
	}

	for (int i = 0; i < len0; i++) {
		col[0] = i;
		for (int j = 1; j < len1; j++) {
			col[j] = std::min(std::min(1 + col[j - 1], 1 + prevCol[j]), prevCol[j - 1] + (i > 0 && my_to_upper(s0[i - 1]) == my_to_upper(s1[j - 1]) ? 0 : 1));
		}
		col.swap(prevCol);
	}

	const float dist = (float)prevCol[len1 - 1];

	return 1.0f - (dist / std::max(s0.size(), s1.size()));
}

static std::vector<std::string> get_words(const std::string &txt) {
	std::vector<std::string> words;

	std::istringstream iss(txt);
	std::string word;
	while (iss >> word) {
		words.push_back(word);
	}

	return words;
}

std::string combine_words(const std::vector<std::string>& words, int start, int count)
{
	std::string s;
	for (int i = start; i < start+count; i++)
	{
		if (i) s += " ";
		s += words[i];
	}
	return s;
}

bool text_contains_prompt(const std::vector<std::string>& text_words, const std::string& prompt, int* prompt_end_index_out)
{
	auto prompt_words = get_words(prompt);

	//float max_similarity = 0;
	//printf("text_contains_prompt %d %d\n", text_words.size(), prompt_words.size());

	for (int i = 0; i <= (int)text_words.size()-(int)prompt_words.size(); i++)
	{
		auto text_part = combine_words(text_words, i, (int)prompt_words.size());
		float sim = similarity(text_part, prompt);
		//printf("part: \'%s\' sim: %f\n", text_part.c_str(), sim);
		//max_similarity = std::max(max_similarity, sim);
		if (sim > 0.7f)
		{
			if (prompt_end_index_out) *prompt_end_index_out = i + (int)prompt_words.size();
			return true;
		}
	}
	return false;
}

bool text_contains_any_prompt(const std::vector<std::string>& text_words, const std::vector<std::string>& prompts, int* prompt_end_index_out)
{
	for (int i = 0; i < prompts.size(); i++)
	{
		bool r = text_contains_prompt(text_words, prompts[i], prompt_end_index_out);
		if (r)
		{
			return true;
		}
	}
	return false;
}

// #streamparam
const int n_samples_prompt         = int(2.0*WHISPER_SAMPLE_RATE);
//const int n_samples_prompt         = int(2.5*WHISPER_SAMPLE_RATE);
//const int n_samples_prompt         = int(3.0*WHISPER_SAMPLE_RATE);
const int n_samples_prompt_overlap = int(0.7*WHISPER_SAMPLE_RATE);

const int32_t n_threads  = std::min(4, (int32_t) std::thread::hardware_concurrency());
//const int32_t audio_ctx_prompt  = 192;
//const int32_t audio_ctx_prompt  = 384;
const int32_t audio_ctx_prompt  = 704;///////////////
//const int32_t audio_ctx_prompt  = 768;
//const int32_t audio_ctx_prompt  = 0;

const int32_t audio_ctx  = 768;
//const int32_t audio_ctx  = 0;
const int32_t audio_ctx_long  = 0;

bool translate              = true;
std::string language        = "en";
bool translate_prompt       = false;
std::string language_prompt = "en";

// The whisper model for prompt detection is fast, but also makes a lot of mistakes.
// Here are some versions of "OK, Milana" to improve detection accuracy.
const std::vector<std::string> prompts =
{
	"Okay Milana",
	"Ok Milana",
	"Okay, we got to",
	"Okay, we'll have",
	"Okay, we'll do this",
	"ok we have",
	"Okay, we're gonna",
	"Okay, we'll land",
	"Okay, we will",
	"Okay, me like",
	"Okay, we learn",
	"Okay, Mille",
	"Okay, meet",
	"Okay, meet",
	//"Okay, Lana",
	"I'm going to be Malana.",
	"Okay, Miriamo",
	"Okay, we done that.",
	"Okay, my la na",
	"Ok mamana",
	"Okay, Mamanu",
	"Okay, Madonna",
	"Okay, Malarma",
	"ok we can add",
	"ok miranabot",
	"Ok, we are done",
	"Okay, we got another",
	"Okemilana",
	"Okay, we don't know",
	"How can we learn?",
	"Milana.",
	"Okinigama",
	"Okay, Moana",
	"Ok, Megana",
};


bool running = true;

#ifdef _MSC_VER
#define NOMINMAX
#include <Windows.h>
static BOOL WINAPI consoleHandler(DWORD signal)
{
	if (signal == CTRL_C_EVENT)
	{
		printf("Ctrl-C handled\n");
		running = false;
	}
	return TRUE;
}
#else

#include <signal.h>
#include <arpa/inet.h>

void my_handler(int s)
{
	printf("Ctrl-C handled\n");
	running = false;
}
#endif

std::vector<short> pcmf32_buffer_16;
std::mutex buffer_mutex;
//double mic_average_amplitude_bias = 0;

static int stream_record_callback(const void *inputBuffer, void *outputBuffer,
						   unsigned long frameCount,
						   const PaStreamCallbackTimeInfo* timeInfo,
						   PaStreamCallbackFlags statusFlags,
						   void *userData )
{
	{
		std::lock_guard<std::mutex> lock(buffer_mutex);
		int buf_size = (int)pcmf32_buffer_16.size();

		pcmf32_buffer_16.resize(buf_size + frameCount);
		memcpy(pcmf32_buffer_16.data()+buf_size, inputBuffer, frameCount*sizeof(short));
#if 0
		if (SAMPLE_RATE_MIC != SAMPLE_RATE)
		{
			// Downsample
			for (s64 i = 0; i < (s64)(pcmf32_buffer_16.size()-buf_size)*SAMPLE_RATE/SAMPLE_RATE_MIC; i++)
				pcmf32_buffer_16[buf_size+i] = pcmf32_buffer_16[buf_size + i*SAMPLE_RATE_MIC/SAMPLE_RATE];
			pcmf32_buffer_16.resize(buf_size + (s64)(pcmf32_buffer_16.size()-buf_size)*SAMPLE_RATE/SAMPLE_RATE_MIC);
		}
#endif

		// Remove bias/low frequencys
		/*for (int i = buf_size; i < pcmf32_buffer_16.size(); i++)
		{
			double a = pcmf32_buffer_16[i];
			mic_average_amplitude_bias = mic_average_amplitude_bias*0.9999 + a*0.0001;
			pcmf32_buffer_16[i] -= (short)mic_average_amplitude_bias;
		}*/
	}
	//printf("stream_record_callback: %d\n", frameCount);
	return paContinue;
}

#if DO_SERVER
SOCKET accept_connection()
{
	SOCKET server = net_listen(asr_port, true, &running);
	if (server == INVALID_SOCKET)
		return INVALID_SOCKET;
	
	printf("Waiting for connection...\n");
	SOCKET client = net_accept_blocking(server, true, &running);
	if (client == INVALID_SOCKET)
	{
		printf("accept failed!\n");
		net_close_socket(server);
		return INVALID_SOCKET;
	}
	net_close_socket(server);
	return client;
}
#endif

int do_stream(int argc, char ** argv, float initial_skip, float* prompt_errors, float* rt_factor, const char* model_prompt)
{
	// print system information
	{
		printf("\n");
		printf("whisper system_info: n_threads = %d / %d | %s\n",
				n_threads, std::thread::hardware_concurrency(), whisper_print_system_info());
	}

	//if (whisper_params_parse(argc, argv, params) == false) {
	//	return 1;
	//}

	
#if !DO_INPUT_WAV && !DO_SERVER
	// init audio
	PaError err = Pa_Initialize();
	if( err != paNoError ) return 1;
	
	PaStream* stream = nullptr;
	int numDevices = Pa_GetDeviceCount();
	for (int i = 0; i < numDevices; i++)
	{
		printf("Device %d: %s\n", i, Pa_GetDeviceInfo(i)->name);
	}

	PaStreamParameters inputParameters;
	inputParameters.device = Pa_GetDefaultInputDevice(); /* default input device */
	if (inputParameters.device == paNoDevice) {
		fprintf(stderr,"Error: No default input device.\n");
		return 1;
	}
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
	printf("Default Device %d\n", Pa_GetDefaultInputDevice());
#ifdef JETSON
	inputParameters.device = 11;
#endif
	inputParameters.channelCount = 1;
	inputParameters.sampleFormat = PA_SAMPLE_TYPE;
	inputParameters.hostApiSpecificStreamInfo = NULL;

	/* Record some audio. -------------------------------------------- */
	err = Pa_OpenStream(
			  &stream,
			  &inputParameters,
			  NULL,                  /* &outputParameters, */
			  SAMPLE_RATE,
			  0,
			  paClipOff,      /* we won't output out of range samples so don't bother clipping them */
			  stream_record_callback,
			  NULL);
	if( err != paNoError ) return 1;

	err = Pa_StartStream(stream);
	if( err != paNoError ) return 1;
	printf("\n=== Now recording!! Please speak into the microphone (%s). ===\n", Pa_GetDeviceInfo(inputParameters.device)->name); fflush(stdout);
#endif

	// whisper init
	if (whisper_lang_id(language.c_str()) == -1) {
		printf("error: unknown language '%s'\n", language.c_str());
		exit(0);
	}
	// whisper init
	if (whisper_lang_id(language_prompt.c_str()) == -1) {
		printf("error: unknown language_prompt '%s'\n", language_prompt.c_str());
		exit(0);
	}
	
	whisper_context_params cparams = whisper_context_default_params();
	cparams.use_gpu = DO_PROMPT_GPU;
	printf("Load prompt model on %s from %s\n", cparams.use_gpu ? "GPU" : "CPU", model_prompt);
	whisper_context* ctx_prompt = whisper_init_from_file_with_params(model_prompt, cparams);

	cparams.use_gpu = DO_TRANSCRIPT_GPU;
	printf("Load transcription model on %s from %s\n", cparams.use_gpu ? "GPU" : "CPU", model_transcribe);
	whisper_context* ctx        = whisper_init_from_file_with_params(model_transcribe, cparams);

	// print some info about the processing
	{
		if (!whisper_is_multilingual(ctx)) {
			if (language != "en" || translate) {
				language = "en";
				translate = false;
				printf("%s: WARNING: model is not multilingual, ignoring language and translation options\n", __func__);
			}
		}
		if (!whisper_is_multilingual(ctx_prompt)) {
			if (language_prompt != "en" || translate_prompt) {
				language_prompt = "en";
				translate_prompt = false;
				printf("%s: WARNING: model_prompt is not multilingual, ignoring language and translation options\n", __func__);
			}
		}
	}

#if DO_CHAT
	chat_init();
#endif

#if DO_SERVER
	while (running)
	{
		SOCKET s = accept_connection();
		if (s == INVALID_SOCKET)
			break;
		if (!running) break;

		bool r = do_transcription(
			ctx, ctx_prompt,
			initial_skip, prompt_errors, rt_factor, s);
		net_close_socket(s);
		if (!r)
			break;
	}
#else
	bool r = do_transcription(
		ctx, ctx_prompt,
		initial_skip, prompt_errors, rt_factor, INVALID_SOCKET);
#endif

#if DO_CHAT
	chat_close();
#endif

#if !DO_INPUT_WAV && !DO_SERVER
	if (stream)
		Pa_CloseStream(stream);
	Pa_Terminate();
#endif

	whisper_free(ctx_prompt);
	whisper_free(ctx);

	return 0;
}

bool do_transcription(
		whisper_context* ctx, whisper_context* ctx_prompt,
		float initial_skip, float* prompt_errors, float* rt_factor, SOCKET s)
{
	s64 samples_eaten = 0;
#if WRITE_TO_FILE
	std::vector<short> pcmf32_recording_16;
	std::vector<short> pcmf32_recording_debug_16;
	std::vector<short> pcmf32_recording_prompts_16;
#endif

#if DO_INPUT_WAV
	{
		//#input
		read_wav(INPUT_WAV, pcmf32_buffer_16, SAMPLE_RATE);


		s64 skip_frames = (s64)(initial_skip*SAMPLE_RATE);
		//pcmf32_buffer_16.erase(pcmf32_buffer_16.begin(), pcmf32_buffer_16.begin()+(s64)(301.3*SAMPLE_RATE));
		pcmf32_buffer_16.erase(pcmf32_buffer_16.begin(), pcmf32_buffer_16.begin()+skip_frames);
		samples_eaten += skip_frames;

		//if (pcmf32_buffer_16.size() > 30*SAMPLE_RATE) pcmf32_buffer_16.resize(30*SAMPLE_RATE);

		
#if WRITE_TO_FILE
		std::vector<short> append(skip_frames, 0);
		pcmf32_recording_16.insert(pcmf32_recording_16.end(), append.begin(), append.begin());
		pcmf32_recording_debug_16.insert(pcmf32_recording_debug_16.end(), append.begin(), append.end());
#endif

		/*{
			double average_amplitude_bias = 0;
			for (int i = 0; i < pcmf32_buffer_16.size(); i++)
			{
				double a = pcmf32_buffer_16[i];
				average_amplitude_bias = average_amplitude_bias*0.9999 + a*0.0001;
				pcmf32_buffer_16[i] -= (short)average_amplitude_bias;
			}
		}*/
		//for (int i = 0; i < pcmf32_buffer_16.size(); i++) pcmf32_buffer_16[i] *= 3;
	}

#endif

	std::vector<float> pcmf32;

	std::vector<int> found_prompts;

	int prompt_step = 0;
#if DO_PROMPT_SAVE == 1
	FILE* prompt_file = fopen("saved_prompts.bin", "wb");
#elif DO_PROMPT_SAVE == 2
	FILE* prompt_file = fopen("saved_prompts.bin", "rb");
#endif

	bool found_prompt = false;
	int num_ignore_prompts = 0;
	
	//DenoiseState* st = rnnoise_create(NULL);
	int64_t t_start = ggml_time_ms();
	int num_prompt_evaluations = 0;

	// main audio loop
	while (true)
	{
		if (!running)
			break;

#if DO_SERVER
		{
			// Receive audio samples from client and put them into the buffer
			buffer_mutex.lock();
			bool fail = false;
			while (true)
			{
				if (!net_can_read_without_blocking(s))
					break;

				const int receive_request_size = 1024;
				char buffer[receive_request_size];
				int r = net_recv(s, buffer, receive_request_size);
				//wprintf(L"Done\n");
				if (r <= 0)
				{
					fail = true;
					break;
				}
				if (r%2 != 0)
				{
					printf("Got uneven amount of bytes!\n");
					fail = true;
					break;
				}
				int size = (int)pcmf32_buffer_16.size();
				pcmf32_buffer_16.resize(size+r/2);
				for (int i = 0; i < r/2; i++)
					pcmf32_buffer_16[size+i] = ((short*)buffer)[i];
			}
			buffer_mutex.unlock();
			if (fail)
				break;
		}

#endif

		// process new audio
		int prompt_start;
		{
			// Get next buffer for prompt search
			//if (!found_prompt)
			bool is_silence = false;
			bool is_prompt = false;
			bool ignore_this_prompt = false;
			prompt_start = prompt_step*(n_samples_prompt-n_samples_prompt_overlap);
			int prompt_end   = prompt_start+n_samples_prompt;
			{
				pcmf32.resize(n_samples_prompt);
				{
					buffer_mutex.lock();
					int available = (int)pcmf32_buffer_16.size();
					if (available < prompt_end)
					{
						buffer_mutex.unlock();
#if DO_INPUT_WAV
						break;
#endif
						//printf("sleep %dms\n", int(s64(buffer_pos)*1000/SAMPLE_RATE));
						imprecise_sleep(0.1);
						continue;
					}
					for (int i = 0; i < pcmf32.size(); i++)
						pcmf32[i] = float(pcmf32_buffer_16[prompt_start+i])/32768.0f;
					buffer_mutex.unlock();
#if !DO_INPUT_WAV
					if (available > prompt_end+pcmf32.size())
					{
						printf("Lagging %.02fs ", float(available-(int)pcmf32.size()-prompt_end)/WHISPER_SAMPLE_RATE);
					}
#endif
				}

				if (num_ignore_prompts)
				{
					num_ignore_prompts--;
					ignore_this_prompt = true;
				}

				// run the inference to search for prompt
#if DO_PROMPT_SAVE == 2
				ignore_this_prompt = true;
#endif
				if (!ignore_this_prompt)
				{
					whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

					wparams.print_progress   = false;
					wparams.print_special    = false;
					wparams.print_realtime   = false;
					wparams.print_timestamps = false;
					wparams.translate        = translate_prompt;
					wparams.single_segment   = true;
					wparams.max_tokens       = 0;
					wparams.language         = language_prompt.c_str();
					wparams.n_threads        = n_threads;

					wparams.audio_ctx        = audio_ctx_prompt;
					wparams.speed_up         = false;

					// disable temperature fallback
					wparams.temperature_inc  = -1.0f;

					wparams.prompt_tokens    = nullptr;
					wparams.prompt_n_tokens  = 0;
			
					//wparams.token_timestamps = true;
					//wparams.max_len = 1;

					if (whisper_full(ctx_prompt, wparams, pcmf32.data(), (int)pcmf32.size()) != 0) {
						printf("Failed to process audio\n");
						return false;
					}
					num_prompt_evaluations++;

					// print result;
					const int t0 = (int)((samples_eaten+prompt_start)*1000/SAMPLE_RATE);
					const int t1 = (int)((samples_eaten+prompt_end  )*1000/SAMPLE_RATE);
					const int t_ = (t0+t1)/2;

#if PRINT_PROMT_SEARCH
					printf("Search prompt %dms %02d:%02d:%03d --> %02d:%02d:%03d: ",
							t_,
							t0/1000/60, t0/1000%60, t0%1000,
							t1/1000/60, t1/1000%60, t1%1000);
#endif

					const int n_segments = whisper_full_n_segments(ctx_prompt);
					for (int i = 0; i < n_segments; ++i)
					{
						const char * text = whisper_full_get_segment_text(ctx_prompt, i);

						//const s64 t0 = whisper_full_get_segment_t0(ctx_prompt, i);
						//const s64 t1 = whisper_full_get_segment_t1(ctx_prompt, i);
						//printf ("[%s --> %s]  %s\n", to_timestamp(t0).c_str(), to_timestamp(t1).c_str(), text);
						//printf ("%s\n", text);
					
						auto text_words = get_words(text);
						int prompt_end_index = -1;
						is_prompt = text_contains_any_prompt(text_words, prompts, &prompt_end_index);
						if (is_prompt)
						{
							std::string before = combine_words(text_words, 0, prompt_end_index);
							std::string after = combine_words(text_words, prompt_end_index, (int)text_words.size()-prompt_end_index);
#if PRINT_PROMT_SEARCH
							printf ("%s [PROMPT]%s\n", before.c_str(), after.c_str());
#endif
							found_prompts.push_back((int)((samples_eaten+prompt_start+n_samples_prompt/2)*1000/SAMPLE_RATE));
							break;
						}
						else
						{
							if (text[0] &&
								(text[1] == '[' || text[1] == '(') &&
								!starts_with(text+2, " [") &&
								!starts_with(text+2, " inaudible") &&
								!starts_with(text+2, "inaudible") &&
								!starts_with(text+2, " indistinct") &&
								!starts_with(text+2, "indistinct") &&
								!starts_with(text+2, " speaking in") &&
								!starts_with(text+2, "speaking in")) // something like: (speaking in foreign language)
							{
								is_silence = true;
#if PRINT_PROMT_SEARCH
								printf ("SILENCE: %s\n", text);
#endif
							}
							else
							{
#if PRINT_PROMT_SEARCH
								printf ("%s\n", text);
#endif
							}
						}

						/*const int token_count = whisper_full_n_tokens(ctx_prompt, i);
						for (int j = 0; j < token_count; ++j)
						{
							whisper_token_data token = whisper_full_get_token_data(ctx_prompt, i, j);
							printf("%s %lld %lld\n", whisper_token_to_str(ctx_prompt, token.id), token.t0, token.t1);
						}*/
					}
				}
				else
				{
#if PRINT_PROMT_SEARCH && DO_PROMPT_SAVE != 2
					printf("[IGNORING]\n");
#endif
				}
				char val = 0;
#if DO_PROMPT_SAVE == 1
				if (is_prompt)       val = 2;
				else if (is_silence) val = 1;
				fwrite(&val, 1, 1, prompt_file); 
#elif DO_PROMPT_SAVE == 2
				fread(&val, 1, 1, prompt_file); 
				if (val == 1) is_silence = true;
				if (val == 2) is_prompt = true;
				ignore_this_prompt = false;
#endif
				
				//if (!found_prompt)
				{
					buffer_mutex.lock();
					int available = (int)pcmf32_buffer_16.size();
					assert(available >= n_samples_prompt);
#if WRITE_TO_FILE
					// Output original recording
					pcmf32_recording_16.insert(pcmf32_recording_16.end(), pcmf32_buffer_16.begin()+prompt_start, pcmf32_buffer_16.begin()+(prompt_start+n_samples_prompt-n_samples_prompt_overlap));
#endif
					if (is_prompt)
					{
						pcmf32_buffer_16.erase(pcmf32_buffer_16.begin(), pcmf32_buffer_16.begin() + prompt_start);
						samples_eaten += prompt_start;
						prompt_step = 1;
					}
					else if (is_silence && !found_prompt)
					{
						pcmf32_buffer_16.erase(pcmf32_buffer_16.begin(), pcmf32_buffer_16.begin() + prompt_end-n_samples_prompt_overlap);
						samples_eaten += prompt_end-n_samples_prompt_overlap;
						prompt_step = 0;
					}
					else
						prompt_step++;
					buffer_mutex.unlock();

#if WRITE_TO_FILE
					short value;
					if (is_prompt)               value = 30000;
					else if (is_silence)         value = 10000;
					else if (ignore_this_prompt) value = -10000;
					else                         value = 20000;
					std::vector<short> append(n_samples_prompt-n_samples_prompt_overlap, value);
					append[0] = 0;
					pcmf32_recording_debug_16.insert(pcmf32_recording_debug_16.end(), append.begin(), append.end());
#endif

					if (is_prompt)
						found_prompt = true;
				}
			}
			if (!found_prompt || !is_silence)
			{
				continue;
			}

#if DO_SERVER
			// Send confirmation that a prompt has been detected.
			// The robot then will play music to indicate that
			// an answer will come soon.
			const char command[] = "%play_music%#";
			net_send_all(s, command, sizeof(command)-1);
#endif

			{
				const int t0 = (int)((samples_eaten             )*1000/SAMPLE_RATE);
				const int t1 = (int)((samples_eaten+prompt_start)*1000/SAMPLE_RATE);
				//printf("### Transcribe %dms --> %dms: ", t0, t1);
				printf("### Transcribe %02d:%02d:%03d --> %02d:%02d:%03d: ",
						t0/1000/60, t0/1000%60, t0%1000,
						t1/1000/60, t1/1000%60, t1%1000);
			}

			pcmf32.resize(prompt_start);
			{
				buffer_mutex.lock();
				int available = (int)pcmf32_buffer_16.size();
				assert(available >= prompt_start);
				for (int i = 0; i < pcmf32.size(); i++)
					pcmf32[i] = float(pcmf32_buffer_16[i])/32768.0f;
#if WRITE_TO_FILE
				pcmf32_recording_prompts_16.insert(pcmf32_recording_prompts_16.end(), pcmf32_buffer_16.begin(), pcmf32_buffer_16.begin()+prompt_start);
				std::vector<short> append(WHISPER_SAMPLE_RATE, 0);
				append[0] = 10000;
				append.back() = 10000;
				pcmf32_recording_prompts_16.insert(pcmf32_recording_prompts_16.end(), append.begin(), append.end());
#endif
				pcmf32_buffer_16.erase(pcmf32_buffer_16.begin(), pcmf32_buffer_16.begin() + prompt_end-n_samples_prompt_overlap);
				samples_eaten += prompt_end-n_samples_prompt_overlap;
				buffer_mutex.unlock();
			}
			prompt_step = 0;
			found_prompt = false;
		}
		

		//printf("\n"); continue;

		// run the inference
		{
			int64_t transcription_start_time = ggml_time_ms();
			whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);

			wparams.print_progress   = false;
			wparams.print_special    = false;
			wparams.print_realtime   = false;
			wparams.print_timestamps = false;
			wparams.translate        = translate;
			wparams.single_segment   = true;
			wparams.max_tokens       = 0;
			wparams.language         = language.c_str();
			wparams.n_threads        = n_threads;
			wparams.speed_up         = false;

			if (prompt_start/SAMPLE_RATE > 15)
			{
				// When the audio reaches a certain length, a small context makes the transcription much worse
				// in the later parts. To prevent that, we use the full context.
				//printf("LONG TRANSC %ds\n", prompt_start/SAMPLE_RATE);
				wparams.audio_ctx = audio_ctx_long;
			}
			else
			{
				//printf("SHORT TRANSC %ds\n", prompt_start/SAMPLE_RATE);
				wparams.audio_ctx = audio_ctx;
			}

			// disable temperature fallback
			wparams.temperature_inc  = -1.0f;

			wparams.prompt_tokens    = nullptr;
			wparams.prompt_n_tokens  = 0      ;
			
			//wparams.token_timestamps = true;
			//wparams.max_len = 1;

			if (whisper_full(ctx, wparams, pcmf32.data(), (int)pcmf32.size()) != 0) {
				printf("failed to process audio\n");
				return false;
			}

			// print result;
			{
				const int n_segments = whisper_full_n_segments(ctx);
				for (int i = 0; i < n_segments; ++i)
				{
					const char * text = whisper_full_get_segment_text(ctx, i);

					const s64 t0 = whisper_full_get_segment_t0(ctx, i);
					const s64 t1 = whisper_full_get_segment_t1(ctx, i);

					//printf ("[%s --> %s]  %s\n", to_timestamp(t0).c_str(), to_timestamp(t1).c_str(), text);
					//printf ("%s\n", text);

					auto text_words = get_words(text);
					int prompt_end_index = -1;
					bool found_prompt_transcription = text_contains_any_prompt(text_words, prompts, &prompt_end_index);
					bool good_prompt = false;
					if (found_prompt_transcription)
					{
						std::string before = combine_words(text_words, 0, prompt_end_index);
						std::string after = combine_words(text_words, prompt_end_index, (int)text_words.size()-prompt_end_index);
						if (after.size() && after[0] == ' ')
							after.erase(0, 1);
						printf ("%s [PROMPT] %s\n", before.c_str(), after.c_str());
						if (after.size())
						{
#if DO_CHAT
							// Send transcription from microphone to ai
							std::vector<std::string> replies;
							auto answer_callback = [s, &replies](const char* ai_reply)
							{
								replies.push_back(ai_reply);
#if DO_SERVER
								// Send ai answer back to robot
								net_send_all(s, ai_reply, (int)strlen(ai_reply));
#endif
							};
							chat_reply(after.c_str(), answer_callback);
							for (int i = 0; i < (int)replies.size(); i++)
								printf("ai_reply: \'%s\'\n", replies[i].c_str());
#endif
#if DO_SERVER && !DO_CHAT
							after += "#";
							// If there is no ai, we just send the transcription back
							net_send_all(s, after.c_str(), (int)after.size());
#endif
#if DO_SERVER
							// Send confirmation that all replies are sent.
							//const char command[] = "%play_finish%#";
							//net_send_all(s, command, sizeof(command)-1);
#endif
							good_prompt = true;
						}
					}
					else
						printf ("%s\n", text);

#if DO_SERVER
					if (!good_prompt)
					{
						// Send indication that we failed so the music can stop playing.
						const char command[] = "%stop_music%#";
						net_send_all(s, command, sizeof(command)-1);
					}
#endif
				}
			}

#if DO_SERVER
			// Doing the transcription did cost a bit of time. That means we lag behing with the
			// prompt transcription.
			// Here we calculate how many prompts we must have missed and then set a counter
			// so we will ignore them. This should be fine, because the user will usually wait
			// for a response from the robot during this time and there is nothing to detect anyway.
			// Note that we cannot simply clear the audio queue here, because we didn't refill
			// the queue during the transcription. In server mode, that means the queue fills up
			// on the client side during that time.
			double transcription_calc_time = (ggml_time_ms() - transcription_start_time)/1000.0;
			double prompt_step_time = (n_samples_prompt-n_samples_prompt_overlap)/double(WHISPER_SAMPLE_RATE);
			num_ignore_prompts = (int)ceil(transcription_calc_time / prompt_step_time);
			//printf("num_ignore_prompts: %d\n", num_ignore_prompts);
#endif
		}
	}
	printf("\nClosing\n");

	//rnnoise_destroy(st);
#if WRITE_TO_FILE
	printf("Saving stream_rec.wav %f\n", float(pcmf32_recording_16.size())/WHISPER_SAMPLE_RATE);
	write_wav("stream_rec.wav", pcmf32_recording_16.data(), (int)pcmf32_recording_16.size(), WHISPER_SAMPLE_RATE);

	printf("Saving stream_debug.wav %f\n", float(pcmf32_recording_debug_16.size())/WHISPER_SAMPLE_RATE);
	write_wav("stream_debug.wav", pcmf32_recording_debug_16.data(), (int)pcmf32_recording_debug_16.size(), WHISPER_SAMPLE_RATE);

	printf("Saving stream_prompts.wav %f\n", float(pcmf32_recording_prompts_16.size())/WHISPER_SAMPLE_RATE);
	write_wav("stream_prompts.wav", pcmf32_recording_prompts_16.data(), (int)pcmf32_recording_prompts_16.size(), WHISPER_SAMPLE_RATE);
#endif

	// audiotest2
	//std::vector<int> prompts_gt = {4000, 11200, 18000, 40200, 48200, 61200, 78600, 99400, 105700, 118600, 127800};

	// milana_test_2
	std::vector<int> prompts_gt = {4900, 11400, 17900, 40000, 47800, 60800, 79000, 99800, 106300, 119300, 128400, 235000, 241500, 249300, 257100, 266200, 283100, 292200, 306500, 316900, 327300};

	// audiotest8
	//std::vector<int> prompts_gt = {2000, 7000, 13000, 20400, 27400, 36600, 44200, 49800};
	printf("gt    prompts (%d): ", (int)prompts_gt.size());
	for (int i = 0; i < (int)prompts_gt.size(); i++)
	{
		printf("%d, ", prompts_gt[i]);
	}
	printf("\n");

	printf("found prompts (%d): ", (int)found_prompts.size());
	for (int i = 0; i < (int)found_prompts.size(); i++)
	{
		printf("%d, ", found_prompts[i]);
	}
	printf("\n");

#if !DO_SERVER
	int max_dist = vec_vec_max_distance(found_prompts, prompts_gt);
	printf("prompts distance to gt: %dms\n", max_dist);
#endif

    whisper_log_set(nullptr, nullptr);
	printf("\nprompt:");fflush(stdout);
	whisper_print_timings(ctx_prompt);
	printf("\ntranscription:");fflush(stdout);
	whisper_print_timings(ctx);

#if !DO_SERVER
	int max_dist2 = vec_vec_max_distance2(found_prompts, prompts_gt);
	printf("prompt errors: %d\n", max_dist2);
	*prompt_errors = (float)max_dist2;
#endif

	const int64_t t_end = ggml_time_ms();
	double prompt_calc_time = (t_end - t_start)/1000.0 / num_prompt_evaluations;
	printf("prompt time: %fs / %d = %fs\n",
			(t_end - t_start)/1000.0f,
			num_prompt_evaluations,
			prompt_calc_time);
	double prompt_step_time = (n_samples_prompt-n_samples_prompt_overlap)/double(WHISPER_SAMPLE_RATE);
	printf("prompt evaluation time step: %fs\n", prompt_step_time);
	printf("realtime factor: %f\n", prompt_calc_time / prompt_step_time);
	*rt_factor = (float)(prompt_calc_time / prompt_step_time);

	pcmf32_buffer_16.clear();

#if DO_PROMPT_SAVE != 0
	fclose(prompt_file);
#endif

	return true;
}

void whisper_log_callback(ggml_log_level level, const char * text, void * user_data)
{
	if (level != GGML_LOG_LEVEL_INFO)
	{
		fputs(text, stderr);
		fflush(stderr);
	}
}

int main(int argc, char ** argv)
{
	//auto text_words = get_words("Okay, and");
	//int prompt_end_index;
	//bool is_prompt = text_contains_any_prompt(text_words, prompts, &prompt_end_index);

#ifdef _MSC_VER
    if (!SetConsoleCtrlHandler(consoleHandler, TRUE)) {
		printf("\nERROR: Could not set control handler"); 
		return 1;
	}
#else
   signal(SIGINT, my_handler);
#endif

#if DO_CHAT_ONLY_TEST
	chat_test();
	return 0;
#endif

    whisper_log_set(whisper_log_callback, nullptr);



#if DO_SERVER
	net_startup();
#endif
	
	const int use_openblas = 1;

	// Evaluation of multiple models
	if (0)
	{
		float prompt_errors_sum[num_models];
		float rt_factor_sum[num_models];
		const int num_runs = 10;
		for (int m = 0; m < num_models; m++)
		{
			char model_prompt[128];
			strcpy(model_prompt, MODEL_PATH);
			strcat(model_prompt, models[m]);
			printf("Evaluating model: %s\n", models[m]);
			fprintf(stderr, "Evaluating model: %s\n", models[m]);
			prompt_errors_sum[m] = 0;
			rt_factor_sum[m] = 0;
			for (int i = 0; i < num_runs; i++)
			{
				printf("Run: %d/%d\n", i+1, num_runs);
				fprintf(stderr, "Run: %d/%d\n", i+1, num_runs);
				float prompt_errors, rt_factor;
				do_stream(argc, argv, i*0.3257f, &prompt_errors, &rt_factor, model_prompt);
				prompt_errors_sum[m] += prompt_errors;
				rt_factor_sum[m] += rt_factor;
			}
			printf("average errors: %f\n", prompt_errors_sum[m] / num_runs);
			printf("average rt factor: %f\n", rt_factor_sum[m] / num_runs);

			fprintf(stderr, "%s\t%d\t%f\t%f\t%d\t%f\t%f\n",
					models[m]+5,
					audio_ctx_prompt,
					n_samples_prompt/float(WHISPER_SAMPLE_RATE),
					n_samples_prompt_overlap/float(WHISPER_SAMPLE_RATE),
					use_openblas,
					prompt_errors_sum[m] / num_runs,
					rt_factor_sum[m] / num_runs
					);
		}

		printf("%s\t%s\t%s\t%s\t%s\t%s\t%s\n",
				"model",
				"ctx",
				"timestep",
				"overlap",
				"openblas",
				"errors",
				"rtfactor"
				);
		for (int m = 0; m < num_models; m++)
		{
			printf("%s\t%d\t%f\t%f\t%d\t%f\t%f\n",
					models[m]+5,
					audio_ctx_prompt,
					n_samples_prompt/float(WHISPER_SAMPLE_RATE),
					n_samples_prompt_overlap/float(WHISPER_SAMPLE_RATE),
					use_openblas,
					prompt_errors_sum[m] / num_runs,
					rt_factor_sum[m] / num_runs
					);
		}
		return 0;
	}
	else
	{
		int m = 0;
		char model_prompt[128];
		strcpy(model_prompt, MODEL_PATH);
		strcat(model_prompt, models[m]);
		float prompt_errors, rt_factor;
		int r = do_stream(argc, argv, 0, &prompt_errors, &rt_factor, model_prompt);
		printf("%s\t%d\t%f\t%f\t%d\t%f\t%f\n",
				models[m]+5,
				audio_ctx_prompt,
				n_samples_prompt/float(WHISPER_SAMPLE_RATE),
				n_samples_prompt_overlap/float(WHISPER_SAMPLE_RATE),
				use_openblas,
				prompt_errors,
				rt_factor
				);
		return r;
	}
#if DO_SERVER
	net_shutdown();
#endif
	return 0;
}
