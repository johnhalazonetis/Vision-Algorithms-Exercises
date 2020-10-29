#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <stdint.h>
#include <math.h>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

// Include the functions that were made during exercises
#include "base-functions.h"
#include "ex5-functions.h"

int main(int argc, char** argv)
{
    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 5 - Stereo Dense Reconstruction/data/";    // Define path to data (for exercise 5)

    Matx33d calibrationMatrix = getCalibrationMatrix("K.txt");      // Get the calibration matrix from file

    // Given by the KITTI dataset:
    double baseline = 0.54;

    // Carefully tuned by the TAs:
    int patchRadius = 5;
    int minDisparity = 5;
    int maxDisparity = 50;
    Vec2i xlims(7, 20);
    Vec2i ylims(-6, 10);
    Vec2i zlims(-5, 5);

    Mat leftImage, rightImage, stereoImage[2];
    VideoCapture leftCap(datapath + "left/%06d.png"), rightCap(datapath + "right/%06d.png");    // Start video capture from images found in folder
    while( leftCap.isOpened() )                             // Loop while we are receiving images from folder
    {
        readStereoImage(leftCap, rightCap, 0.5, stereoImage);    // Read the right and left image

        showStereoImage(stereoImage, "STEREO");             // Show the image output

        waitKey(25);                                        // Wait for X ms
    }

    destroyAllWindows();                                    // Close all windows
    return 0;
}