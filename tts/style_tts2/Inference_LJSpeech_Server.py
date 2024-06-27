# This is based on StyleTTS2/Demo/Inference_LJSpeech.ipynb
# and adds a server interface. It is no longer a notebook, but a python script now.
# I also fixed a crash when the supplied text is too long.

#!/usr/bin/env python
# coding: utf-8

# # StyleTTS 2 Demo (LJSpeech)
# 

# ### Utils

# In[1]:


import torch
#torch.manual_seed(0)
#torch.backends.cudnn.benchmark = False
#torch.backends.cudnn.deterministic = True

import random
#random.seed(0)

import numpy as np
#np.random.seed(0)


# In[2]:


#get_ipython().run_line_magic('cd', '..')
import sys, os
from os.path import dirname
style_tts2_repo = 'StyleTTS2'
sys.path.append(os.path.join(dirname(__file__), style_tts2_repo))

# On Windows espeak isn't automatically installed with the python dependencies. Set path manually here.
if os.name == 'nt':
    os.environ["PHONEMIZER_ESPEAK_LIBRARY"] = "eSpeak NG\\libespeak-ng.dll"

# In[3]:


# load packages
import time
import random
import yaml
from munch import Munch
import numpy as np
import torch
from torch import nn
import torch.nn.functional as F
import torchaudio
import librosa
from nltk.tokenize import word_tokenize

from models import *
from utils import *
from text_utils import TextCleaner
textclenaer = TextCleaner()

#get_ipython().run_line_magic('matplotlib', 'inline')


# In[4]:


device = 'cuda' if torch.cuda.is_available() else 'cpu'
#device = 'cpu'

print("Using device", device)


# In[6]:


to_mel = torchaudio.transforms.MelSpectrogram(
    n_mels=80, n_fft=2048, win_length=1200, hop_length=300)
mean, std = -4, 4

def length_to_mask(lengths):
    mask = torch.arange(lengths.max()).unsqueeze(0).expand(lengths.shape[0], -1).type_as(lengths)
    mask = torch.gt(mask+1, lengths.unsqueeze(1))
    return mask

def preprocess(wave):
    wave_tensor = torch.from_numpy(wave).float()
    mel_tensor = to_mel(wave_tensor)
    mel_tensor = (torch.log(1e-5 + mel_tensor.unsqueeze(0)) - mean) / std
    return mel_tensor

def compute_style(ref_dicts):
    reference_embeddings = {}
    for key, path in ref_dicts.items():
        wave, sr = librosa.load(path, sr=24000)
        audio, index = librosa.effects.trim(wave, top_db=30)
        if sr != 24000:
            audio = librosa.resample(audio, sr, 24000)
        mel_tensor = preprocess(audio).to(device)

        with torch.no_grad():
            ref = model.style_encoder(mel_tensor.unsqueeze(1))
        reference_embeddings[key] = (ref.squeeze(1), audio)
    
    return reference_embeddings


# ### Load models

# load phonemizer
import phonemizer
global_phonemizer = phonemizer.backend.EspeakBackend(language='en-us', preserve_punctuation=True,  with_stress=True)
#global_phonemizer = phonemizer.backend.EspeakBackend(language='de', preserve_punctuation=True,  with_stress=True)


# In[11]:


config = yaml.safe_load(open(os.path.join(style_tts2_repo, "Models/LJSpeech/config.yml")))

# load pretrained ASR model
ASR_config = config.get('ASR_config', False)
ASR_path = config.get('ASR_path', False)
text_aligner = load_ASR_models(os.path.join(style_tts2_repo, ASR_path), os.path.join(style_tts2_repo, ASR_config))

# load pretrained F0 model
F0_path = config.get('F0_path', False)
pitch_extractor = load_F0_models(os.path.join(style_tts2_repo, F0_path))

# load BERT model
from Utils.PLBERT.util import load_plbert
BERT_path = config.get('PLBERT_dir', False)
plbert = load_plbert(os.path.join(style_tts2_repo, BERT_path))


# In[12]:


model = build_model(recursive_munch(config['model_params']), text_aligner, pitch_extractor, plbert)
_ = [model[key].eval() for key in model]
_ = [model[key].to(device) for key in model]


# In[13]:


params_whole = torch.load(os.path.join(style_tts2_repo, "Models/LJSpeech/epoch_2nd_00100.pth"), map_location='cpu')
params = params_whole['net']


# In[14]:


for key in model:
    if key in params:
        print('%s loaded' % key)
        try:
            model[key].load_state_dict(params[key])
        except:
            from collections import OrderedDict
            state_dict = params[key]
            new_state_dict = OrderedDict()
            for k, v in state_dict.items():
                name = k[7:] # remove `module.`
                new_state_dict[name] = v
            # load params
            model[key].load_state_dict(new_state_dict, strict=False)
#             except:
#                 _load(params[key], model[key])
_ = [model[key].eval() for key in model]


# In[15]:


from Modules.diffusion.sampler import DiffusionSampler, ADPM2Sampler, KarrasSchedule


# In[16]:


sampler = DiffusionSampler(
    model.diffusion.diffusion,
    sampler=ADPM2Sampler(),
    sigma_schedule=KarrasSchedule(sigma_min=0.0001, sigma_max=3.0, rho=9.0), # empirical parameters
    clamp=False
)


# ### Synthesize speech

# In[17]:


# synthesize a text
text = ''' StyleTTS 2 is a text-to-speech model that leverages style diffusion and adversarial training with large speech language models to achieve human-level text-to-speech synthesis. '''


# In[18]:


def inference(text, noise, diffusion_steps=5, embedding_scale=1):
    text = text.strip()
    text = text.replace('"', '')
    ps = global_phonemizer.phonemize([text])
    ps = word_tokenize(ps[0])
    ps = ' '.join(ps)

    tokens = textclenaer(ps)
    tokens.insert(0, 0)

    if len(tokens) > 512:
        print("############ Sentence is too long! Will be shortened.", len(tokens))
        tokens = tokens[:512]

    tokens = torch.LongTensor(tokens).to(device).unsqueeze(0)
    
    with torch.no_grad():
        input_lengths = torch.LongTensor([tokens.shape[-1]]).to(tokens.device)
        text_mask = length_to_mask(input_lengths).to(tokens.device)

        t_en = model.text_encoder(tokens, input_lengths, text_mask)
        bert_dur = model.bert(tokens, attention_mask=(~text_mask).int())
        d_en = model.bert_encoder(bert_dur).transpose(-1, -2) 

        s_pred = sampler(noise, 
              embedding=bert_dur[0].unsqueeze(0), num_steps=diffusion_steps,
              embedding_scale=embedding_scale).squeeze(0)

        s = s_pred[:, 128:]
        ref = s_pred[:, :128]

        d = model.predictor.text_encoder(d_en, s, input_lengths, text_mask)

        x, _ = model.predictor.lstm(d)
        duration = model.predictor.duration_proj(x)
        duration = torch.sigmoid(duration).sum(axis=-1)
        pred_dur = torch.round(duration.squeeze()).clamp(min=1)

        pred_dur[-1] += 5

        pred_aln_trg = torch.zeros(input_lengths, int(pred_dur.sum().data))
        c_frame = 0
        for i in range(pred_aln_trg.size(0)):
            pred_aln_trg[i, c_frame:c_frame + int(pred_dur[i].data)] = 1
            c_frame += int(pred_dur[i].data)

        # encode prosody
        en = (d.transpose(-1, -2) @ pred_aln_trg.unsqueeze(0).to(device))
        F0_pred, N_pred = model.predictor.F0Ntrain(en, s)
        out = model.decoder((t_en @ pred_aln_trg.unsqueeze(0).to(device)), 
                                F0_pred, N_pred, ref.squeeze().unsqueeze(0))
        
    return out.squeeze().cpu().numpy()


import socket
tts_port = 60124  # port to listen on (non-privileged ports are > 1023)

def do_tts(text, conn):
    print("##################")
    print("ttsserver:", text)

    start = time.time()
    noise = torch.randn(1,1,256).to(device)
    wav = inference(text, noise, diffusion_steps=5, embedding_scale=1)
    end = time.time()

    rtf = (end - start) / (len(wav) / 24000)
    print(f"Clip duration: {len(wav)/24000:.2f}s")
    print(f"Inference time: {end-start:.2f}s")
    print(f"RTF = {rtf:5f}")

    wav = wav * (32767 / max(1, np.max(np.abs(wav))))
    wav = wav.astype(np.int16)
    #import soundfile as sf
    #sf.write('output.df5.wav', wav, 24000)
    if conn is not None:
        conn.sendall(wav.tobytes())
        print("send done")

def run_server():
        
    do_tts("warmup", None)
    do_tts("The first manned Moon landing was Apollo 11 on July, 20 1969. The first human to step on the Moon was astronaut Neil Armstrong followed second by Buzz Aldrin. They landed in the Sea of Tranquility with their lunar module the Eagle. They were on the lunar surface for 2.25 hours and collected 50 pounds of moon rocks.", None)

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:

	    # This prevents: Address already in use
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        #accept() blocks and prevents ctrl+c from working, so we busy wait with a timeout
        s.settimeout(1.0)

        s.bind((socket.gethostbyname("0.0.0.0"), tts_port))
        s.listen()

        while True:
            print("Waiting for connection")
            while True:
                try:
                    conn, addr = s.accept()
                    break
                except socket.timeout:
                    pass
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
                                do_tts(splits[i], conn)
                            input_string = splits[-1]
            except (ConnectionResetError, BrokenPipeError):
                pass
            print("Closed.")


run_server()
exit(0)







# #### Basic synthesis (5 diffusion steps)

start = time.time()
noise = torch.randn(1,1,256).to(device)
wav = inference(text, noise, diffusion_steps=5, embedding_scale=1)
#rtf = (time.time() - start) / (len(wav) / 24000)
#print(f"RTF = {rtf:5f}")

end = time.time()
rtf = (end - start) / (len(wav) / 24000)
print(f"Clip duration: {len(wav)/24000:.2f}s")
print(f"Inference time: {end-start:.2f}s")
print(f"RTF = {rtf:5f}")
print(f"RTX = {1/rtf:.2f}")

#import IPython.display as ipd
#display(ipd.Audio(wav, rate=24000))
import soundfile as sf
sf.write('output.df5.wav', wav, 24000)
print("Done!")
exit(0)

# #### With higher diffusion steps (more diverse)
# Since the sampler is ancestral, the higher the stpes, the more diverse the samples are, with the cost of slower synthesis speed.

# In[ ]:


start = time.time()
noise = torch.randn(1,1,256).to(device)
wav = inference(text, noise, diffusion_steps=10, embedding_scale=1)
rtf = (time.time() - start) / (len(wav) / 24000)
print(f"RTF = {rtf:5f}")
import IPython.display as ipd
display(ipd.Audio(wav, rate=24000))


# ### Speech expressiveness
# The following section recreates the samples shown in [Section 6](https://styletts2.github.io/#emo) of the demo page.

# #### With embedding_scale=1
# This is the classifier-free guidance scale. The higher the scale, the more conditional the style is to the input text and hence more emotional. 

# In[ ]:


texts = {}
texts['Happy'] = "We are happy to invite you to join us on a journey to the past, where we will visit the most amazing monuments ever built by human hands."
texts['Sad'] = "I am sorry to say that we have suffered a severe setback in our efforts to restore prosperity and confidence."
texts['Angry'] = "The field of astronomy is a joke! Its theories are based on flawed observations and biased interpretations!"
texts['Surprised'] = "I can't believe it! You mean to tell me that you have discovered a new species of bacteria in this pond?"

for k,v in texts.items():
    noise = torch.randn(1,1,256).to(device)
    wav = inference(v, noise, diffusion_steps=10, embedding_scale=1)
    print(k + ": ")
    display(ipd.Audio(wav, rate=24000, normalize=False))


# #### With embedding_scale=2

# In[ ]:


texts = {}
texts['Happy'] = "We are happy to invite you to join us on a journey to the past, where we will visit the most amazing monuments ever built by human hands."
texts['Sad'] = "I am sorry to say that we have suffered a severe setback in our efforts to restore prosperity and confidence."
texts['Angry'] = "The field of astronomy is a joke! Its theories are based on flawed observations and biased interpretations!"
texts['Surprised'] = "I can't believe it! You mean to tell me that you have discovered a new species of bacteria in this pond?"

for k,v in texts.items():
    noise = torch.randn(1,1,256).to(device)
    wav = inference(v, noise, diffusion_steps=10, embedding_scale=2) # embedding_scale=2 for more pronounced emotion
    print(k + ": ")
    display(ipd.Audio(wav, rate=24000, normalize=False))


# ### Long-form generation
# This section includes basic implementation of Algorithm 1 in the paper for consistent longform audio generation. The example passage is taken from [Section 5](https://styletts2.github.io/#long) of the demo page. 

# In[ ]:


passage = '''If the supply of fruit is greater than the family needs, it may be made a source of income by sending the fresh fruit to the market if there is one near enough, or by preserving, canning, and making jelly for sale. To make such an enterprise a success the fruit and work must be first class. There is magic in the word "Homemade," when the product appeals to the eye and the palate; but many careless and incompetent people have found to their sorrow that this word has not magic enough to float inferior goods on the market. As a rule large canning and preserving establishments are clean and have the best appliances, and they employ chemists and skilled labor. The home product must be very good to compete with the attractive goods that are sent out from such establishments. Yet for first-class homemade products there is a market in all large cities. All first-class grocers have customers who purchase such goods.'''


# In[ ]:


def LFinference(text, s_prev, noise, alpha=0.7, diffusion_steps=5, embedding_scale=1):
    text = text.strip()
    text = text.replace('"', '')
    ps = global_phonemizer.phonemize([text])
    ps = word_tokenize(ps[0])
    ps = ' '.join(ps)

    tokens = textclenaer(ps)
    tokens.insert(0, 0)
    tokens = torch.LongTensor(tokens).to(device).unsqueeze(0)
    
    with torch.no_grad():
        input_lengths = torch.LongTensor([tokens.shape[-1]]).to(tokens.device)
        text_mask = length_to_mask(input_lengths).to(tokens.device)

        t_en = model.text_encoder(tokens, input_lengths, text_mask)
        bert_dur = model.bert(tokens, attention_mask=(~text_mask).int())
        d_en = model.bert_encoder(bert_dur).transpose(-1, -2) 

        s_pred = sampler(noise, 
              embedding=bert_dur[0].unsqueeze(0), num_steps=diffusion_steps,
              embedding_scale=embedding_scale).squeeze(0)
        
        if s_prev is not None:
            # convex combination of previous and current style
            s_pred = alpha * s_prev + (1 - alpha) * s_pred
        
        s = s_pred[:, 128:]
        ref = s_pred[:, :128]

        d = model.predictor.text_encoder(d_en, s, input_lengths, text_mask)

        x, _ = model.predictor.lstm(d)
        duration = model.predictor.duration_proj(x)
        duration = torch.sigmoid(duration).sum(axis=-1)
        pred_dur = torch.round(duration.squeeze()).clamp(min=1)

        pred_aln_trg = torch.zeros(input_lengths, int(pred_dur.sum().data))
        c_frame = 0
        for i in range(pred_aln_trg.size(0)):
            pred_aln_trg[i, c_frame:c_frame + int(pred_dur[i].data)] = 1
            c_frame += int(pred_dur[i].data)

        # encode prosody
        en = (d.transpose(-1, -2) @ pred_aln_trg.unsqueeze(0).to(device))
        F0_pred, N_pred = model.predictor.F0Ntrain(en, s)
        out = model.decoder((t_en @ pred_aln_trg.unsqueeze(0).to(device)), 
                                F0_pred, N_pred, ref.squeeze().unsqueeze(0))
        
    return out.squeeze().cpu().numpy(), s_pred


# In[ ]:


sentences = passage.split('.') # simple split by comma
wavs = []
s_prev = None
for text in sentences:
    if text.strip() == "": continue
    text += '.' # add it back
    noise = torch.randn(1,1,256).to(device)
    wav, s_prev = LFinference(text, s_prev, noise, alpha=0.7, diffusion_steps=10, embedding_scale=1.5)
    wavs.append(wav)
display(ipd.Audio(np.concatenate(wavs), rate=24000, normalize=False))

