# Vision-Algorithms-Exercises
This repo is where you will find the code for the exercises and mini-project of the VAMR Course (Autumn 2020 Semester).

## Cloning and Installing Dependencies
*Note: This guide to installing dependencies only works for macOS machines or Linux computer that use either `pacman` or `apt` as package managers.*

Start by cloning the repo onto your computer: `git clone https://github.com/johnhalazonetis/Vision-Algorithms-Exercises`

After cloning this repo, you can install the dependencies or this project by running one of the three scripts located in the `dependencies` directory. Currently, the only C++ libraries that these scripts will install are:

* Latest version of OpenCV (for C++)
* Latest version of Eigen3

The script will ask you if you want to install these libraries, you can answer "No" if you already have them installed on your computer.

The installation scripts can also download all of the images and files from my cloud server and add them into the `Vision-Algorithms-Exercises` folder. If you already have them installed, you can answer "No" when the installer asks, and the installer will ask you where the data files are located on your computer.

## Setting Things Up

Go into the folder: `cd Vision-Algorithms-Exercises`

Make build directory and go cd into it: `mkdir build && cd build`

After installing the required dependencies (see above) you can build the project: `cmake .. && make`

You can then run any of the scripts that have been written for the exercises, or the script that has been written for the project.