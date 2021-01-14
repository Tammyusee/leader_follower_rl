virtualenv /usr/local/share/venvs/py3venv --python=python3 \
ln -s /usr/local/share/venvs/py3venv ~/.py3venv \
source ~/.py3venv/bin/activate \
pip install pyaml rospkg \
pip install torch torchvision \
pip install --upgrade cython \
pip install tqdm cython pycocotools \
pip install matplotlib \
pip install opencv-python \
pip uninstall em \
pip install empy