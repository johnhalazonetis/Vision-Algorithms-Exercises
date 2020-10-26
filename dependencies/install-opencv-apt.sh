# Install OpenCV on Linux distros with apt package manager

# First we update the system and install the dependencies:
sudo apt-get update

sudo apt-get upgrade

sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev libvtk7-dev

sudo apt-get install -y libopencv-dev python3-opencv