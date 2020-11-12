xcode-select --install

mkdir ~/install_opencv && cd ~/install_opencv
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git

# Then we build the library to be used in our system
cd opencv
mkdir release && cd release

sudo cmake -D BUILD_TIFF=ON -D WITH_CUDA=OFF -D ENABLE_AVX=OFF -D WITH_OPENGL=ON -D WITH_OPENCL=ON -D WITH_IPP=OFF -D WITH_TBB=ON -D BUILD_TBB=ON -D WITH_EIGEN=ON -D WITH_V4L=ON -D WITH_VTK=ON -D BUILD_TESTS=OFF -D BUILD_PERF_TESTS=OFF -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local -D OPENCV_EXTRA_MODULES_PATH=~/install_opencv/opencv_contrib/modules ~/install_opencv/opencv/

read -p "How many threads to do you want to use to make OpenCV?: " ThreadNumber
sudo make -j$ThreadNumber

sudo make install

sudo rm -r ~/install_opencv