# Install OpenCV on Linux distros with apt package manager

# First we update the system and install the dependencies:
sudo apt-get update

sudo apt-get upgrade

sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libvtk7-dev

sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"

sudo apt-get update

sudo apt-get install -y libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libdc1394-22-dev libeigen3-dev libtheora-dev libvorbis-dev libxvidcore-dev libx264-dev sphinx-common libtbb-dev yasm libfaac-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenexr-dev libgstreamer-plugins-base1.0-dev libavutil-dev libavfilter-dev libavresample-dev


# Then we download the latest version of OpenCV:
mkdir ~/install_opencv && cd ~/install_opencv

git clone https://github.com/opencv/opencv.git

git clone https://github.com/opencv/opencv_contrib.git


# Then we build the library to be used in our system
cd opencv

mkdir release && cd release

sudo cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=ON -D WITH_OPENCL=ON -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=ON -D WITH_V4L=ON -D WITH_VTK=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=~/install_opencv/opencv_contrib/modules ~/install_opencv/opencv/

# Ask how many threads we want to use to make the library
read -p "How many threads to do you want to use to make OpenCV?: " ThreadNumber
sudo make -j$ThreadNumber

sudo make install

rm -rf ~/install_opencv
