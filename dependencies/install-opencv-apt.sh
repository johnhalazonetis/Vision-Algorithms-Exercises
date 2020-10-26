# Install OpenCV on Linux distros with apt package manager

# First we update the system and install the dependencies:
sudo apt-get update

sudo apt-get upgrade

sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

# Install OpenCV from source
git clone https://github.com/opencv/opencv.git

cd opencv && mkdir build && cd build

cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..

make -j3

sudo make install

cd .. && rm -r opencv