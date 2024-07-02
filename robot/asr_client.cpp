// asr client.
// Connets to asr_server and records audio from the microphone. It sends all the audio samples to the
// server and receives text back. The text is then sent to tts for speech output.

#include "asr_client.h"
#include "../common/common.h"
#include "../common/network.h"
#include "main.h"
#include "tts_client.h"

#include <alsa/asoundlib.h>

#include <cerrno>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <vector>


const char* asr_host_address = "insert_hostname_or_ip_here";

static SOCKET client = INVALID_SOCKET;
static NetConnecting* connecting = nullptr;

static std::vector<char> recv_queue;
const int recv_bytes_per_call = 100*1024;


// alsa sound code based on https://gist.github.com/ghedo/963382?permalink_comment_id=3465692
const char* record_device = "plughw:2,0";
using SampleType = s16;
const snd_pcm_format_t sample_format = SND_PCM_FORMAT_S16_LE;
static std::vector<char> audio_queue; // This is of type char because single bytes can be sent over the network.
const int record_sample_rate = 16000;
static snd_pcm_t* pcm_handle;
const int samples_per_frame = 1000;

static bool record_audio_init();

bool asr_client_init()
{
	if (!record_audio_init()) return false;
	
	return true;
}

static void disconnect_client()
{
	if (client != INVALID_SOCKET)
	{
		net_close_socket(client);
		client = INVALID_SOCKET;
		if (md.asr_client_connected)
			printf("asr disconnected\n");
	}
	//recv_queue.clear();
	//recv_queue.shrink_to_fit(); // free memory
	
	audio_queue.clear();
	audio_queue.shrink_to_fit(); // free memory
	
	md.asr_client_connected = false;
}


bool asr_client_update()
{
	u32_micros start_time = time_micros();

	// Transfer audio samples from microphone (with Alsa) into our queue.
	while (true)
	{
		static SampleType buffer[samples_per_frame];
		int r = snd_pcm_readi(pcm_handle, buffer, samples_per_frame);
		if (r == -EAGAIN)
			r = 0;
		else if (r == -EPIPE)
		{
			printf("broken record pipe.\n");
			//snd_pcm_prepare(pcm_handle);
			r = 0;
		}
		if (r < 0 || r > samples_per_frame)
		{
			printf("Error. Can't read from PCM device. %d %s\n", r, snd_strerror(r));
			return false;
		}
		if (r == 0)
			break;

		int size = audio_queue.size();
		audio_queue.resize(size+r*sizeof(SampleType));
		memcpy(audio_queue.data()+size, buffer, r*sizeof(SampleType));
	}

	// Async connect to server
	if (client == INVALID_SOCKET && !connecting)
	{
		connecting = net_connect(asr_host_address, asr_port, false, false);
		//printf("asr connecting...\n");
		//printf("asr connecting to %s : %i\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	}
	if (connecting && net_connect_is_done(connecting, &client))
	{
		net_connect_close(connecting);
		connecting = nullptr;
		if (client != INVALID_SOCKET)
		{
			printf("asr connected.\n");
			net_set_socket_non_blocking(client);
		}
	}
	md.asr_client_connected = client != INVALID_SOCKET;

	// Receive text from server
	while (client != INVALID_SOCKET)
	{
		char buffer[recv_bytes_per_call];
		errno = 0;
		int r = net_recv(client, buffer, recv_bytes_per_call);
		if (r == -1)
			break;
		if (r == 0)
		{
			//printf("asr recv error! %d %d\n", r, errno);
			disconnect_client();
			break;
		}

		//printf("asr recv: %d bytes\n", r);
		size_t size = recv_queue.size();
		recv_queue.resize(size+r);
		memcpy(recv_queue.data()+size, buffer, r);
	}

	// Get full sentences or command from the recv queue.
	// Don't do it if we are already processing something, so that we can easily assign
	// the samples we receive from the tts server to the text.
	md.asr_recv_text = false;
	if (!md.tts_calculating)
	{
		for (int pos = 0; pos < recv_queue.size(); pos++)
		{
			if (recv_queue[pos] == '#')
			{
				std::vector<char> s(recv_queue.begin(), recv_queue.begin()+pos);
				s.push_back(0);
				printf("asr recv: %s\n", s.data());
		
				tts_say_text(s.data());
				recv_queue = std::vector<char>(recv_queue.begin()+pos+1, recv_queue.end());
				md.asr_recv_text = true;
				break;
			}
		}
	}

	// send data in audio queue to server
	if (client != INVALID_SOCKET)
	{
		while (audio_queue.size())
		{
			int r = net_send(client, audio_queue.data(), audio_queue.size());
			if (r == -1)
				break;
			if (r == 0)
			{
				printf("asr send fail %d %d\n", r, (int)audio_queue.size());
				disconnect_client();
				break;
			}
			memmove(audio_queue.data(), audio_queue.data()+r, audio_queue.size()-r);
			audio_queue.resize(audio_queue.size()-r);
			//printf("asr sent %d bytes\n", r);
		}
	}
	else
	{
		// Don't queue up audio when we are not connected.
		// Otherwise the server will take a while to handle all that audio
		// once we are connected and cause delay.
		audio_queue.clear();
	}


	md.delta_time_asr = time_micros() - start_time;
	return true;
}

static void record_audio_close()
{
	if (pcm_handle)
		snd_pcm_close(pcm_handle);
	pcm_handle = nullptr;
}

void asr_client_close()
{
	disconnect_client();
	record_audio_close();
}

static void enum_record_devices()
{
	snd_pcm_stream_t direction = SND_PCM_STREAM_CAPTURE;

	snd_ctl_card_info_t* info;
	snd_ctl_card_info_malloc(&info);

	snd_pcm_info_t* pcminfo;
	snd_pcm_info_malloc(&pcminfo);

	int cid = -1;
	int ret = snd_card_next(&cid);
	if (ret == 0 && cid == -1)
	{
		printf("No audio devices found.\n");
		return;
	}
	for (; cid != -1 && ret >= 0; ret = snd_card_next(&cid))
	{
		char hwname[128];
		snprintf(hwname, 128, "hw:%d", cid);
		snd_ctl_t* handle;
		ret = snd_ctl_open(&handle, hwname, 0);
		if (ret < 0)
		{
			printf("Could not open card %d: %s", cid, snd_strerror(ret));
			continue;
		}
		ret = snd_ctl_card_info(handle, info);
		if (ret < 0) {
			fprintf(stderr, "Could not get info for card %d: %s",
				cid, snd_strerror(ret));
			snd_ctl_close(handle);
			continue;
		}
		int dev = -1;
		ret = snd_ctl_pcm_next_device(handle, &dev);
		if (ret >= 0 && dev == -1)
		{
			fprintf(stderr, "Warning: No devices found on card %d\n", cid);
		}

		for (; dev != -1 && ret >= 0; ret = snd_ctl_pcm_next_device(handle, &dev))
		{
			snd_pcm_info_set_device(pcminfo, dev);
			snd_pcm_info_set_subdevice(pcminfo, 0);
			snd_pcm_info_set_stream(pcminfo, direction);
			ret = snd_ctl_pcm_info(handle, pcminfo);
			if (ret < 0)
			{
				//fprintf(stderr, "error getting device info [%d, %d]: %s\n", cid, dev, snd_strerror(ret));
				continue;
			}
			printf("plughw:%d,%d ", cid, dev);
			printf("dev_id: %s ", snd_ctl_card_info_get_id(info));
			printf("dev_name = %s\n", snd_ctl_card_info_get_name(info));
		}

		if (ret == -1)
		{
			fprintf(stderr, "Error reading next sound device on card %d\n", cid);
		}
		snd_ctl_close(handle);
	}
	if (ret == -1)
	{
		fprintf(stderr, "Error reading next sound card\n");
	}
	snd_ctl_card_info_free(info);
	snd_pcm_info_free(pcminfo);
}

static bool record_audio_init()
{
	//enum_record_devices();
	//return false;

	// Open the PCM device in recording mode
	int r = snd_pcm_open(&pcm_handle, record_device,
				         SND_PCM_STREAM_CAPTURE, SND_PCM_NONBLOCK);
	if (r < 0)
	{
		printf("Error: Can't open microphone \"%s\". %s\n", record_device, snd_strerror(r));
		return false;
	}

	// Allocate parameters object and fill it with default values.
	snd_pcm_hw_params_t *params;
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(pcm_handle, params);

	// Set parameters
	unsigned int rate = record_sample_rate;
	if ((r = snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) goto error;
	if ((r = snd_pcm_hw_params_set_format(pcm_handle, params, sample_format)) < 0) goto error;
	if ((r = snd_pcm_hw_params_set_channels(pcm_handle, params, 1)) < 0) goto error;
	if ((r = snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, 0)) < 0) goto error;
	if ((r = snd_pcm_hw_params(pcm_handle, params)) < 0) goto error;

#if 0
	// Print information
	printf("Record PCM name: '%s'\n", snd_pcm_name(pcm_handle));
	printf("PCM state: %s\n", snd_pcm_state_name(snd_pcm_state(pcm_handle)));
	{
		uint tmp;
		snd_pcm_hw_params_get_rate(params, &tmp, 0);
		printf("rate: %d bps\n", tmp);
	}
#endif

	return true;
	error:
	printf("Error recording PCM: %s\n", snd_strerror(r));
	record_audio_close();
	return false;
}
