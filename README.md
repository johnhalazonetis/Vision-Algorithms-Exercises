# Vision-Algorithms-Exercises
This repo is where you will find the code for the 

## Cloning and Installing Dependencies
*Note: This guide to installing dependencies only works for macOS machines or Linux computer that use either `pacman` or `apt` as package managers.*

Start by cloning the repo onto your computer: `git clone https://github.com/johnhalazonetis/Vision-Algorithms-Exercises`

After cloning this repo, you can install the dependencies or this project by running one of the three scripts located in the `dependencies` directory. Currently, the only C++ libraries that these scripts will install are:

* Latest version of OpenCV

The installation scripts will also download all of the images and files from my cloud server and add them into the `Vision-Algorithms-Exercises` folder. Please do not move these files around, unless you want to go into the code and modify their location.

## Setting Things Up

Go into the folder: `cd Vision-Algorithms-Exercises`

Make build directory and go cd into it: `mkdir build && cd build`

After installing the required dependencies (see above) you can build the project: `cmake .. && make`

You can then run any of the scripts that have been written for the exercises, or the script that has been written for the project.