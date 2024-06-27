// Here we connect with the tts server.
// When we want to output a text, we send that text to the server and wait for the
// audio samples to come back. We then play those with Alsa.
// (We use Alsa instead of Portaudio because Portaudio produces some strange console output)
// The tts server either runs locally on the robot (coqui_tts) in a docker container or remotely
// on another machine (style_tts2).

#include "tts_client.h"
#include "../common/common.h"
#include "main.h"
#include "../common/helper.h"
#include "../common/network.h"
#include "command.h"

#include <alsa/asoundlib.h>

#include <cerrno>
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <vector>

// Run coqui_tts locally in a docker container. A bit slow and quite difficult to set up
//#define USE_LOCAL_TTS 1

// Run style_tts2 on a machine with a gpu. Much faster, better quality and easier to set up.
#define USE_LOCAL_TTS 0

#if USE_LOCAL_TTS
const char* tts_host_address = "127.0.0.1";
#else
const char* tts_host_address = "insert_hostname_or_ip_here";
#endif

static SOCKET client = INVALID_SOCKET;
static NetConnecting* connecting = nullptr;

#if USE_LOCAL_TTS
static bool started_docker = false;
#endif
static float docker_timer = 20;

static std::vector<char> send_buffer;
const int recv_bytes_per_call = 100*1024;


// alsa sound code based on https://gist.github.com/ghedo/963382?permalink_comment_id=3465692
const char* playback_device = "default";
using SampleType = s16;
const snd_pcm_format_t sample_format = SND_PCM_FORMAT_S16_LE;
static std::vector<char> audio_queue; // This is of type char because single bytes can be received over the network.
//const int playback_sample_rate = 22050; // This is what coqui_tts outputs
const int playback_sample_rate = 24000; // This is what style_tts2 outputs
snd_pcm_t* pcm_handle;
const int period_frames = 1000; // Number of samples transmitted to Alsa per call.

std::vector<SampleType> music_samples;
bool playing_music = false;

std::vector<SampleType> finish_samples;

float received_audio_samples_for_text_timer = 0;
bool wait_with_playback_until_commands_are_done = false;

static bool playback_init();

bool tts_client_init()
{
	if (!playback_init()) return false;

	if (!read_wav("tts/elevatormusic_mono.wav", music_samples, playback_sample_rate)) return false;
	if (!read_wav("tts/finish.wav", finish_samples, playback_sample_rate)) return false;
	
	return true;
}

static void disconnect_client()
{
	if (client != INVALID_SOCKET)
	{
		net_close_socket(client);
		client = INVALID_SOCKET;
		if (md.tts_client_connected)
			printf("tts disconnected\n");
	}
	//send_buffer.clear();
	//send_buffer.shrink_to_fit(); // free memory
	
	audio_queue.clear();
	audio_queue.shrink_to_fit(); // free memory
	
	md.tts_client_connected = false;
}

void tts_play_finish()
{
	printf("Playing finish\n");
	audio_queue.insert(audio_queue.end(), (char*)finish_samples.data(),
		((char*)finish_samples.data())+finish_samples.size()*sizeof(SampleType));
}

void tts_say_text(const char* text)
{
	for (int i = 0; i < num_commands; i++)
	{
		if (strcmp(text, command_names[i]) == 0)
		{
			if (i == C_Play_music)
			{
				printf("Playing music\n");
				playing_music = true;

				audio_queue.insert(audio_queue.end(), (char*)music_samples.data(),
					((char*)music_samples.data())+music_samples.size()*sizeof(SampleType));
			}
			else if (i == C_Stop_music)
			{
				if (playing_music)
				{
					printf("Stop music\n");
					playing_music = false;
					audio_queue.clear();
				}
			}
			else
				command_add((Command)i);
			return;
		}
	}
	printf("tts send %s\n", text);
	send_buffer.insert(send_buffer.end(), text, text+strlen(text));
	send_buffer.push_back('#');
	md.tts_calculating = true;
	received_audio_samples_for_text_timer = 0;
	wait_with_playback_until_commands_are_done = command_is_pending();
}

int play_samples(SampleType* samples, int num_samples)
{
	int num_samples2 = std::min(num_samples, period_frames);
	static SampleType buffer[period_frames];
	for (int i = 0; i < num_samples2; i++)
		buffer[i] = (SampleType)(samples[i] * cd.tts_volume);

	// If we don't have enough samples available, we send silence.
	// This is better than sending nothing because otherwise the 
	// internal "pipe" gets broken.
	for (int i = num_samples2; i < period_frames; i++)
		buffer[i] = 0;
	int r = snd_pcm_writei(pcm_handle, buffer, period_frames);
	if (r == -EAGAIN)
		r = 0;
	else if (r == -EPIPE)
	{
		printf("playback broken pipe.\n");
		snd_pcm_prepare(pcm_handle);
		r = 0;
	}
	if (r < 0 || r > period_frames)
	{
		printf("Error. Can't write to PCM device. %d %s\n", r, snd_strerror(r));
		return -1;
	}

	// Return how many of the supplied samples were sent to Alsa.
	return std::min(r, num_samples);
}

bool tts_client_update()
{
	u32_micros start_time = time_micros();

	docker_timer += md.delta_time;

	static int last_tts_say_thing_trigger;
	if (cd.tts_say_thing_trigger-last_tts_say_thing_trigger == 1)
	{
		if (cd.tts_say_thing_index >= 0 && cd.tts_say_thing_index < tts_say_things_num)
		{
			tts_say_text(tts_say_things[cd.tts_say_thing_index]);
		}
		else if (-cd.tts_say_thing_index-1 >= 0 && -cd.tts_say_thing_index-1 < num_commands)
		{
			tts_say_text(command_names[-cd.tts_say_thing_index-1]);
		}
		else
		{
			assert(0);
		}
	}
	last_tts_say_thing_trigger = cd.tts_say_thing_trigger;

	if (client == INVALID_SOCKET && !connecting && docker_timer > 20)
	{
		connecting = net_connect(tts_host_address, tts_port, false, false);
		//printf("tts connecting...\n");
		//printf("tts connecting to %s : %i\n", inet_ntoa(addr.sin_addr), ntohs(addr.sin_port));
	}

	bool just_connected = false;
	if (connecting)
	{
		if (net_connect_is_done(connecting, &client))
		{
			net_connect_close(connecting);
			connecting = nullptr;
#if USE_LOCAL_TTS
			// When docker opens this port, but the container is not listening, connect()
			// will somehow succeed. But then the first recv call will fail for some reason.
			// We catch that here to prevent spamming the console.
			just_connected = true;
#endif
		}
	}
	md.tts_client_connected = client != INVALID_SOCKET;

#if USE_LOCAL_TTS
	if (!md.tts_client_connected && docker_timer > 20 && just_connected && !started_docker)
	{
		//printf("tts connect fail %d %d\n", r, errno);

		started_docker = true;
		docker_timer = 0;

		const char* command = R"--(nvidia-docker run -p 60124:60124 --gpus all --runtime nvidia \
	--cpu-period=1000 --cpu-quota=3000 \
    --volume $HOME/coqui_tts:/coqui_tts \
    --volume $HOME/coqui_tts/TTS:/coqui/TTS \
    --volume $HOME/coqui_tts/model_cache:/root/.local/share/tts \
	jetson-coqui \
    python3 /coqui_tts/synthesize_server.py \
	--model_name "tts_models/en/ljspeech/glow-tts" \
	--vocoder_name "vocoder_models/en/ljspeech/multiband-melgan" \
	</dev/null >/dev/null 2>&1 &)--";

		//printf("start docker: %s\n", command);
		printf("start tts docker\n");
		system(command);
	}
#endif

	bool received_audio_samples = false;
	while (client != INVALID_SOCKET)
	{
		if (wait_with_playback_until_commands_are_done && command_is_pending())
		{
			// Don't receive samples of this text yet!
			// Some commands need to be executed first.
			break;
		}
		char buffer[recv_bytes_per_call];
		int r = net_recv(client, buffer, recv_bytes_per_call);
		if (r == -1)
			break;
		if (r == 0)
		{
			if (just_connected)
			{
				just_connected = false;
				//printf("connection fail\n");
				net_close_socket(client);
				client = INVALID_SOCKET;
			}
			else
				printf("tts recv error! %d %d\n", r, errno);
			disconnect_client();
			break;
		}
		//printf("tts recv: %d bytes\n", r);
#if USE_LOCAL_TTS
		started_docker = true;
#endif
		received_audio_samples = true;
		md.tts_playback = true;
		if (playing_music)
		{
			playing_music = false;
			audio_queue.clear();
		}
		audio_queue.insert(audio_queue.end(), buffer, buffer+r);
	}
	while (send_buffer.size() && client != INVALID_SOCKET)
	{
		int r = net_send(client, send_buffer.data(), send_buffer.size());
		if (r == -1)
			break;
		if (r == 0)
		{
			printf("tts send fail %d %d\n", r, (int)send_buffer.size());
			disconnect_client();
			break;
		}
		send_buffer.erase(send_buffer.begin(), send_buffer.begin()+r);
		//printf("tts sent %d bytes\n", r);
	}

	if (just_connected)
		printf("tts connected.\n");

	md.tts_recv_audio = received_audio_samples;
	md.tts_queue_size = (int)audio_queue.size();

	if (md.tts_calculating)
	{
		// We set tts_calculating to false when we have received all the audio samples
		// since we sent the text to the tts server.
		// Sadly the server sends no indication when it's done, so we use some heuristics.
		if (received_audio_samples)
			received_audio_samples_for_text_timer = 2.0f;
		if (received_audio_samples_for_text_timer != 0)
		{
			received_audio_samples_for_text_timer -= md.delta_time;
			if (received_audio_samples_for_text_timer <= 0)
			{
				received_audio_samples_for_text_timer = 0;
				md.tts_calculating = false;
			}
		}
	}

	// Transfer audio frames from our queue into Alsa, which will play it.
	{
		int r = play_samples((SampleType*)audio_queue.data(), audio_queue.size()/sizeof(SampleType));
		if (r == -1) return false;
		memmove(audio_queue.data(), audio_queue.data()+r*sizeof(SampleType), audio_queue.size()-r*sizeof(SampleType));
		audio_queue.resize(audio_queue.size()-r*sizeof(SampleType));
		if (audio_queue.size() == 0)
		{
			md.tts_playback = false;
			playing_music = false;
		}
	}

	md.delta_time_tts = time_micros() - start_time;
	return true;
}

static void playback_close()
{
	if (pcm_handle)
		snd_pcm_close(pcm_handle);
	pcm_handle = nullptr;
}

void tts_client_close()
{
	disconnect_client();
	if (connecting)
	{
		net_connect_close(connecting);
		connecting = nullptr;
	}
	playback_close();
}

static bool playback_init()
{
	// Open the PCM device in playback mode
	int r = snd_pcm_open(&pcm_handle, playback_device,
				         SND_PCM_STREAM_PLAYBACK, SND_PCM_NONBLOCK);
	if (r < 0)
	{
		printf("Error: Can't open speaker \"%s\". %s\n", playback_device, snd_strerror(r));
		return false;
	}

	// Allocate parameters object and fill it with default values.
	snd_pcm_hw_params_t *params;
	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(pcm_handle, params);

	// Set parameters
	unsigned int rate = playback_sample_rate;
	if ((r = snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) goto error;
	if ((r = snd_pcm_hw_params_set_format(pcm_handle, params, sample_format)) < 0) goto error;
	if ((r = snd_pcm_hw_params_set_channels(pcm_handle, params, 1)) < 0) goto error;
	if ((r = snd_pcm_hw_params_set_rate_near(pcm_handle, params, &rate, 0)) < 0) goto error;
	if ((r = snd_pcm_hw_params(pcm_handle, params)) < 0) goto error;

	/*{
		snd_pcm_uframes_t tmp;
		// This seems to be the number of frames you should supply per call.
		snd_pcm_hw_params_get_period_size(params, &tmp, 0);
		period_frames = tmp*2;
	}*/

#if 0
	// Print information
	printf("PCM name: '%s'\n", snd_pcm_name(pcm_handle));
	printf("PCM state: %s\n", snd_pcm_state_name(snd_pcm_state(pcm_handle)));
	{
		uint tmp;
		snd_pcm_hw_params_get_rate(params, &tmp, 0);
		printf("rate: %d bps\n", tmp);
	}
	printf("period_frames: %d\n", period_frames);
#endif

	return true;
	error:
	printf("Error PCM: %s\n", snd_strerror(r));
	playback_close();
	return false;
}

