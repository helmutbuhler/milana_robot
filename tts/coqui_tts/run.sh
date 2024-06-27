#!/usr/bin/env bash
#
# note: this command is copied with minor changes in robot/tts_client.cpp
sudo nvidia-docker run -p 60124:60124 --gpus all --runtime nvidia -it \
        --cpu-period=1000 --cpu-quota=3000 \
        --volume $HOME/coqui_tts:/coqui_tts \
        --volume $HOME/coqui_tts/TTS:/coqui/TTS \
        --volume $HOME/coqui_tts/model_cache:/root/.local/share/tts \
        jetson-coqui \
        python3 /coqui_tts/synthesize_server.py \
        --model_name "tts_models/en/ljspeech/glow-tts" \
        --vocoder_name "vocoder_models/en/ljspeech/multiband-melgan"
