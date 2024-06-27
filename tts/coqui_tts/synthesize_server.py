#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# This is based on coqui_tts/TTS/TTS/bin/synthesize.py
# The source version is 0.6.1, which is the one of the last ones to work with Python 3.6.
# (Jetson Nano only supports Python up to version 3.6)
#
# This file starts a server and waits for text.
# It reads from the socket until the '#' character designates the end of the string.
# It then splits the received string into sentences, calculates the audio of each
# sentence and sends the 16-bit samples back.

import argparse
import sys
import io
from argparse import RawTextHelpFormatter

# pylint: disable=redefined-outer-name, unused-argument
from pathlib import Path

from TTS.utils.manage import ModelManager
from TTS.utils.synthesizer import Synthesizer

# This make inference faster on the Jetson Nano
import torch
torch.set_num_threads(1)

import numpy as np
import socket
tts_port = 60124  # port to listen on (non-privileged ports are > 1023)

def do_tts(synthesizer, args, text, conn):
    print("ttsserver:", text)
    # Split input text into sentences to reduce delay.
    sens = synthesizer.split_into_sentences(text)
    for sen in sens:
        print("ttsserver sen:", sen)
        if len(sen) <= 1:
            continue
        wav = synthesizer.tts(sen, args.speaker_idx, args.language_idx, args.speaker_wav)
        wav = np.array(wav)
        wav = wav * (32767 / max(0.01, np.max(np.abs(wav))))
        wav = wav.astype(np.int16).tobytes()
        if conn is not None:
            conn.sendall(wav)
            print("send done")

def run_server(synthesizer, args):
    
    # We need to reconfigure console output to be utf8, otherwise printing strings with non-ascii
    # characters in them will crash.
    #sys.stdout.reconfigure(encoding='utf-8') # only works with python 3.7 and up
    sys.stdout = io.open(sys.stdout.fileno(), 'w', encoding='utf8')
    
    do_tts(synthesizer, args, "warmup", None)
    print(f"Generating audio with {synthesizer.output_sample_rate}Hz")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

	    # This prevents: Address already in use
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        s.bind((socket.gethostbyname("0.0.0.0"), tts_port))
        s.listen()

        while True:
            print("Waiting for connection")
            conn, addr = s.accept()
            input_string = ""
            try:
                with conn:
                    print(f"Connected by {addr}")
                    while True:
                        data = conn.recv(1024)
                        if not data:
                            break
                        input_string += data.decode('utf-8')
                        #print(input_string)
                        splits = input_string.split('#')
                        if len(splits) > 1:
                            for i in range(len(splits)-1):
                                do_tts(synthesizer, args, splits[i], conn)
                            input_string = splits[-1]
            except (ConnectionResetError, BrokenPipeError):
                pass
            print("Closed.")

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    if v.lower() in ("no", "false", "f", "n", "0"):
        return False
    raise argparse.ArgumentTypeError("Boolean value expected.")

def main():
    description = """Synthesize speech on command line.

You can either use your trained model or choose a model from the provided list.

If you don't specify any models, then it uses LJSpeech based English model.

## Example Runs

### Single Speaker Models

- List provided models:

    ```
    $ tts --list_models
    ```

- Run TTS with default models:

    ```
    $ tts --text "Text for TTS"
    ```

- Run a TTS model with its default vocoder model:

    ```
    $ tts --text "Text for TTS" --model_name "<language>/<dataset>/<model_name>
    ```

- Run with specific TTS and vocoder models from the list:

    ```
    $ tts --text "Text for TTS" --model_name "<language>/<dataset>/<model_name>" --vocoder_name "<language>/<dataset>/<model_name>" --output_path
    ```

- Run your own TTS model (Using Griffin-Lim Vocoder):

    ```
    $ tts --text "Text for TTS" --model_path path/to/model.pth.tar --config_path path/to/config.json --out_path output/path/speech.wav
    ```

- Run your own TTS and Vocoder models:
    ```
    $ tts --text "Text for TTS" --model_path path/to/config.json --config_path path/to/model.pth.tar --out_path output/path/speech.wav
        --vocoder_path path/to/vocoder.pth.tar --vocoder_config_path path/to/vocoder_config.json
    ```

### Multi-speaker Models

- List the available speakers and choose as <speaker_id> among them:

    ```
    $ tts --model_name "<language>/<dataset>/<model_name>"  --list_speaker_idxs
    ```

- Run the multi-speaker TTS model with the target speaker ID:

    ```
    $ tts --text "Text for TTS." --out_path output/path/speech.wav --model_name "<language>/<dataset>/<model_name>"  --speaker_idx <speaker_id>
    ```

- Run your own multi-speaker TTS model:

    ```
    $ tts --text "Text for TTS" --out_path output/path/speech.wav --model_path path/to/config.json --config_path path/to/model.pth.tar --speakers_file_path path/to/speaker.json --speaker_idx <speaker_id>
    ```
    """
    # We remove Markdown code formatting programmatically here to allow us to copy-and-paste from main README to keep
    # documentation in sync more easily.
    parser = argparse.ArgumentParser(
        description=description.replace("    ```\n", ""),
        formatter_class=RawTextHelpFormatter,
    )

    parser.add_argument(
        "--list_models",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
        help="list available pre-trained TTS and vocoder models.",
    )
    #parser.add_argument("--text", type=str, default=None, help="Text to generate speech.")

    # Args for running pre-trained TTS models.
    parser.add_argument(
        "--model_name",
        type=str,
        default="tts_models/en/ljspeech/tacotron2-DDC",
        help="Name of one of the pre-trained TTS models in format <language>/<dataset>/<model_name>",
    )
    parser.add_argument(
        "--vocoder_name",
        type=str,
        default=None,
        help="Name of one of the pre-trained  vocoder models in format <language>/<dataset>/<model_name>",
    )

    # Args for running custom models
    parser.add_argument("--config_path", default=None, type=str, help="Path to model config file.")
    parser.add_argument(
        "--model_path",
        type=str,
        default=None,
        help="Path to model file.",
    )
    """
    parser.add_argument(
        "--out_path",
        type=str,
        default="tts_output.wav",
        help="Output wav file path.",
    )
    """
    parser.add_argument("--use_cuda", type=bool, help="Run model on CUDA.", default=False)
    parser.add_argument(
        "--vocoder_path",
        type=str,
        help="Path to vocoder model file. If it is not defined, model uses GL as vocoder. Please make sure that you installed vocoder library before (WaveRNN).",
        default=None,
    )
    parser.add_argument("--vocoder_config_path", type=str, help="Path to vocoder model config file.", default=None)
    parser.add_argument(
        "--encoder_path",
        type=str,
        help="Path to speaker encoder model file.",
        default=None,
    )
    parser.add_argument("--encoder_config_path", type=str, help="Path to speaker encoder config file.", default=None)

    # args for multi-speaker synthesis
    parser.add_argument("--speakers_file_path", type=str, help="JSON file for multi-speaker model.", default=None)
    parser.add_argument("--language_ids_file_path", type=str, help="JSON file for multi-lingual model.", default=None)
    parser.add_argument(
        "--speaker_idx",
        type=str,
        help="Target speaker ID for a multi-speaker TTS model.",
        default=None,
    )
    parser.add_argument(
        "--language_idx",
        type=str,
        help="Target language ID for a multi-lingual TTS model.",
        default=None,
    )
    parser.add_argument(
        "--speaker_wav",
        nargs="+",
        help="wav file(s) to condition a multi-speaker TTS model with a Speaker Encoder. You can give multiple file paths. The d_vectors is computed as their average.",
        default=None,
    )
    parser.add_argument("--gst_style", help="Wav path file for GST stylereference.", default=None)
    parser.add_argument(
        "--list_speaker_idxs",
        help="List available speaker ids for the defined multi-speaker model.",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
    )
    parser.add_argument(
        "--list_language_idxs",
        help="List available language ids for the defined multi-lingual model.",
        type=str2bool,
        nargs="?",
        const=True,
        default=False,
    )
    # aux args
    parser.add_argument(
        "--save_spectogram",
        type=bool,
        help="If true save raw spectogram for further (vocoder) processing in out_path.",
        default=False,
    )

    args = parser.parse_args()

    # print the description if either text or list_models is not set
    #if args.text is None and not args.list_models and not args.list_speaker_idxs and not args.list_language_idxs:
    #    parser.parse_args(["-h"])

    # load model manager
    #path = Path(__file__).parent / "../.models.json"
    path = Path(__file__).parent / "TTS/TTS/.models.json"
    manager = ModelManager(path)

    model_path = None
    config_path = None
    speakers_file_path = None
    language_ids_file_path = None
    vocoder_path = None
    vocoder_config_path = None
    encoder_path = None
    encoder_config_path = None

    # CASE1: list pre-trained TTS models
    if args.list_models:
        manager.list_models()
        sys.exit()

    # CASE2: load pre-trained model paths
    if args.model_name is not None and not args.model_path:
        model_path, config_path, model_item = manager.download_model(args.model_name)
        args.vocoder_name = model_item["default_vocoder"] if args.vocoder_name is None else args.vocoder_name

    if args.vocoder_name is not None and not args.vocoder_path:
        vocoder_path, vocoder_config_path, _ = manager.download_model(args.vocoder_name)

    # CASE3: set custom model paths
    if args.model_path is not None:
        model_path = args.model_path
        config_path = args.config_path
        speakers_file_path = args.speakers_file_path
        language_ids_file_path = args.language_ids_file_path

    if args.vocoder_path is not None:
        vocoder_path = args.vocoder_path
        vocoder_config_path = args.vocoder_config_path

    if args.encoder_path is not None:
        encoder_path = args.encoder_path
        encoder_config_path = args.encoder_config_path

    # load models
    synthesizer = Synthesizer(
        model_path,
        config_path,
        speakers_file_path,
        language_ids_file_path,
        vocoder_path,
        vocoder_config_path,
        encoder_path,
        encoder_config_path,
        args.use_cuda,
    )

    # query speaker ids of a multi-speaker model.
    if args.list_speaker_idxs:
        print(
            " > Available speaker ids: (Set --speaker_idx flag to one of these values to use the multi-speaker model."
        )
        print(synthesizer.tts_model.speaker_manager.speaker_ids)
        return

    # query langauge ids of a multi-lingual model.
    if args.list_language_idxs:
        print(
            " > Available language ids: (Set --language_idx flag to one of these values to use the multi-lingual model."
        )
        print(synthesizer.tts_model.language_manager.language_id_mapping)
        return

    # check the arguments against a multi-speaker model.
    if synthesizer.tts_speakers_file and (not args.speaker_idx and not args.speaker_wav):
        print(
            " [!] Looks like you use a multi-speaker model. Define `--speaker_idx` to "
            "select the target speaker. You can list the available speakers for this model by `--list_speaker_idxs`."
        )
        return

    run_server(synthesizer, args)

    """
    # RUN THE SYNTHESIS
    print(" > Text: {}".format(args.text))

    # kick it
    wav = synthesizer.tts(args.text, args.speaker_idx, args.language_idx, args.speaker_wav)

    # save the results
    print(" > Saving output to {}".format(args.out_path))
    synthesizer.save_wav(wav, args.out_path)
    """

if __name__ == "__main__":
    main()
