# Install OpenCV on Linux distros with apt package manager

# First we update the system and install the dependencies:
sudo pacman -Syu

sudo pacman -S java-runtime cblas ffmpeg gst-plugins-base intel-tbb jasper lapack libdc1394 libgphoto2 openexr hdf5 opencv-samples python-numpy qt5-base vtk ant cmake eigen glew lapacke mesa python-setuptools

# Then we download the latest version of OpenCV:
yay -S opencv3-opt