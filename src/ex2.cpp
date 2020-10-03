#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>	
#include <Eigen/SVD>
#include <Eigen/QR>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
using namespace Eigen;

// Include the functions that were made during course work
#include "base-functions.h"
#include "ex2-functions.h"

int main(int argc, char** argv)
{
    // Initlialization of main:

    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 2 - PnP/data/";     // Define path to data (for exercise 2)

    string calibrationMatrixFile = "K.txt";                                             // Define the name of the calibration matrix file
    Matrix3d calibrationMatrix = getCalibrationMatrix(calibrationMatrixFile, datapath);      // Call function to read the calibration matrix file and make the calibration matrix

    Vector4d q; q << 1, 1, 1, 1;
    
    Matrix2d R = quat2RotMatrix(q);

    cout << "R_after = " << R << endl;
}