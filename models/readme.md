# Models

## Overview
This is the place for the models the ASR Server uses.

## Download
We need the ggml whisper models `small` and `large-v3` for ASR. Execute this to download them into this folder:
```
download-ggml-model small
download-ggml-model large-v3
```

If you want to use the llm model for chat generation, you can download [Mistral-7B-Instruct-v0.1](https://huggingface.co/TheBloke/Mistral-7B-Instruct-v0.1-GGUF). [Here](https://huggingface.co/TheBloke/Mistral-7B-Instruct-v0.1-GGUF/blob/main/mistral-7b-instruct-v0.1.Q5_K_M.gguf) is a link to the specific version I used. But pretty much any model that llama.cpp can load should work. `llm/chat.cpp` references some other models that you can try out too.
