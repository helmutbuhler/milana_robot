# install anaconda
# on linux, run:
conda init

# based on (https://llm-tracker.info/books/howto-guides/page/styletts-2-setup-guide)

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

# checkout codebase
git clone https://github.com/yl4579/StyleTTS2.git
# (I got Revision: cc4025cd69462b33c834d4b87dfaf307e9fe29bf Date: 09.12.2023 17:50:13)
cd StyleTTS2

# get models (gdown may not work, try manually downloading it in browser then)
pip install gdown
gdown 'https://drive.google.com/file/d/1K3jt1JEbtohBLUA0X75KLw36TW7U1yxq/view?usp=sharing'
unzip Models.zip

# (optional, if you want to experiment with the notebook)
mamba install jupyter notebook

#On Windows, download eSpeak NG into this folder (https://github.com/espeak-ng/espeak-ng/releases/tag/1.51)



