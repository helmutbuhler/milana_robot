
# Automatic Speech Recognition Server

## Overview
This is a server that gives the robot access to Automatic Speech Recognition (optionally with a LLM).

The server basically does this:
- Receives audio samples.
- Passes those audio samples to [whisper.cpp](https://github.com/ggerganov/whisper.cpp) (the [model](https://github.com/openai/whisper) from OpenAI) to translate it into text.
- Optionally passes that text into a LLM to generate a text response (using [llama.cpp](https://github.com/ggerganov/llama.cpp)).
- Returns either the original text or the response back to the robot.

Not everything what is heard in the input audio stream is processed. First a passphrase must be said ("OK, Milana"), then the following audio is further processed until silence is detected for a few seconds.

This passphrase detection, and the silence detection, is done with a small whisper model (ggml-small.bin). Once a passphrase is detected, the audio after the passphrase is processed with a bigger model (ggml-large-v3.bin).

I ran this on a NVIDIA GeForce RTX 2080 Ti with 11GB. If you want to run it with a LLM, you need at least that much GPU memory. You also need about 32HB of RAM. Sadly, the Jetson Nano isn't capable of running this.

This program can optionally be built so that is receives audio directly from the local PC. See the defines at the top of `asr_server.cpp`.

You need to download some models to use this. Go to the models folder for more details.

## Build
You can build this on Windows, Ubuntu on WSL or on a real Ubuntu.

### Windows
Download and install the [NVidia Cuda Toolkit](https://developer.nvidia.com/cuda-downloads). Open `milana_robot.sln` in Visual Studio 2019 or up and it should build everything.

### Ubuntu
If you want to use WSL, you need to give WSL access to more RAM. Open `C:\Users\*username*\.wslconfig` and add this:
```
[wsl2]
memory=30GB
```
You also need to [install Cuda](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=WSL-Ubuntu&target_version=2.0&target_type=deb_local) in WSL.

If you want to use the real Ubuntu, install the nvidia driver and cuda-toolkit with:
```
sudo apt install nvidia-cuda-toolkit nvidia-cuda-toolkit-gcc
```
See [here](https://www.cherryservers.com/blog/install-cuda-ubuntu) if that doesn't work.

Next you need to adjust the CUDA_ARCHITECTURES number in `CMakeLists.txt`. Find out your gpu capability by running:
```
nvidia-smi --query-gpu=compute_cap --format=csv
```
Then enter that number without the dot after CUDA_ARCHITECTURES in `CMakeLists.txt`.
After that go to the root of the repository and build it with:
```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
```

And run it with `./asr_server`
