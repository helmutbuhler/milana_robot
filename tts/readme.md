# TTS Server

## Overview
Here is the code for running the TTS Server. It allows the robot to convert text into speech. The protocol is very simple: it receives the text and sends the audio signal back. Sadly I didn't find a way to make this application native, it uses Python.

There are two implementations here of this server:
- Implementation based on [StyleTTS 2](https://github.com/yl4579/StyleTTS2). It must run on a separate machine with a gpu and is much faster, has better quality and is easier to set up than the other implementation.
- Implementation based on [Coqui TTS](https://github.com/coqui-ai/TTS.git). This one can run locally on the Jetson Nano. But it is a bit slow and quite difficult to set up. It needs to run in a docker container to make use of Jetsons acceleration hardware.

## StyleTTS 2

This implementation requires Python, either on a Linux or Windows machine. I used [Anaconda](https://www.anaconda.com/download) to create a Python Environment and used this to install the dependencies:

```
conda create -n tts
conda activate tts

conda install conda-libmamba-solver
conda install python=3.11 mamba -c conda-forge --solver=libmamba

# pytorch - current nightly works w/ Python 3.11 but not 3.12
# pick your version here: https://pytorch.org/get-started/locally/
mamba install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch-nightly -c nvidia

# reqs - torch stuff already installed 
pip install SoundFile munch pydub pyyaml librosa nltk matplotlib accelerate transformers phonemizer einops einops-exts tqdm typing typing-extensions git+https://github.com/resemble-ai/monotonic_align.git
python -m nltk.downloader punkt
```
(This is partly based on this [Guide](https://llm-tracker.info/books/howto-guides/page/styletts-2-setup-guide))

Next we need to clone [StyleTTS 2](https://github.com/yl4579/StyleTTS2) into the tts/style_tts2 folder and download its model files:
```
cd tts/style_tts2
git clone https://github.com/yl4579/StyleTTS2.git
# (I got Revision: cc4025cd69462b33c834d4b87dfaf307e9fe29bf, but master probably works, too)
cd StyleTTS2

# get models (gdown may not work, try manually downloading it in browser then)
pip install gdown
gdown 'https://drive.google.com/file/d/1K3jt1JEbtohBLUA0X75KLw36TW7U1yxq/view?usp=sharing'
unzip Models.zip

# (optional, if you want to experiment with the notebook)
mamba install jupyter notebook
```
On Windows, we also need to download eSpeak NG into `tts/style_tts2/eSpeak NG` from [here](https://github.com/espeak-ng/espeak-ng/releases/tag/1.51).

Finally we can start the server:
```
cd tts/style_tts2
python Inference_LJSpeech_Server
```

## Coqui TTS

Running [Coqui TTS](https://github.com/coqui-ai/TTS.git) on the Jetson Nano is a bit difficult: The Ubuntu version on the Jetson Nano only supports Python 3.6, which means that we need to use an older version of Coqui TTS. Also installing Pytorch (which Coqui TTS needs) is basically impossible without a Docker Container.

I forked the version of Coqui TTS that still works with Python 3.6 [here](todo). It also includes some bugfixes. Clone it into `tts/coqui_tts`.

To create the Docker container, run:
```
cd tts/coqui_tts
./build.sh
```
If you don't have docker installed on your Jetson Nano, you need to install it with the sdkmanager. Building the image can take about 30 minutes and requires about 15GB of diskspace. So make sure to have a big enough SD card installed.

Once it is build, you can run it with:
```
cd tts
coqui_tts/run.sh
```
It will download the required models automatically on the first run. After it has done so, it will also work offline.

## Other TTS libraries
Here are some other libraries that I tried:
```
https://github.com/dusty-nv/jetson-voice
https://github.com/snakers4/silero-models#text-to-speech
https://r9y9.github.io/deepvoice3_pytorch/
https://github.com/neonbjb/tortoise-tts
https://github.com/suno-ai/bark
```
All of them are either too slow or too unreliable for the Jetson Nano.

## Note
The `elevatormusic_mono.wav` file in this folder is used by the robot to indicate that a response is coming soon. It's from [here](
https://pixabay.com/music/bossa-nova-waiting-music-116216). `finish.wav` is used to indicate that the response is finished (the file is from `alarm07.wav` in the Windows Media folder).