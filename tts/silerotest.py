import os
import torch
import time
import re
import inflect

number_regex = re.compile(r'\d+(?:,\d+)?')  # https://stackoverflow.com/a/16321189
number_inflect = inflect.engine()
def numbers_to_words(text):
    """
    Convert instances of numbers to words in the text.
    For example:  "The answer is 42" -> "The answer is forty two."
    """
    number_tokens = number_regex.findall(text)
    
    for number_token in number_tokens:
        # TODO test/handle floating-point numbers
        word_text = number_inflect.number_to_words(number_token)              
        num_begin = text.index(number_token)

        # insert the words back at the old location
        text = text[:num_begin] + word_text + text[num_begin + len(number_token):]
        
    return text

device = torch.device('cpu')
#device = torch.device('cuda')
torch.set_num_threads(1)
local_file = 'v3_en.pt'
#local_file = 'v3_en_indic.pt'

if not os.path.isfile(local_file):
    torch.hub.download_url_to_file('https://models.silero.ai/models/tts/en/'+local_file,
                                   local_file)  

model = torch.package.PackageImporter(local_file).load_pickle("tts_models", "model")
model.to(device)

example_text = 'The first manned Moon landing was Apollo 11 on July, 20 1969. The first human to step on the Moon was astronaut Neil Armstrong followed second by Buzz Aldrin. They landed in the Sea of Tranquility with their lunar module the Eagle. They were on the lunar surface for 2.25 hours and collected 50 pounds of moon rocks.'
#example_text = "Hello! How are you doing? I don't know what to say now. 1 plus 2 is not equal to 42. The Jetson Nano is a nice device, but it is so painful to program it."

example_text = numbers_to_words(example_text)
sample_rate = 48000
#speaker='en_28'
speaker='en_43'
#speaker='tamil_female'
#print(model)
#for i in range(118):
#    speaker='en_'+str(i)
for _ in range(2):
    print(speaker)
    start_time = time.time()
    audio_paths = model.save_wav(text=example_text,
                             speaker=speaker,
                             sample_rate=sample_rate)
    print("time:", time.time()-start_time)
    #os.rename('test.wav', 'test_'+str(i)+'.wav')
