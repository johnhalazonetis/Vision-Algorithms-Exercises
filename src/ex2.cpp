#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Core>               // Importing general Eigen library
#include <Eigen/SVD>                // Allows to calculate SVD (using Jacobi or faster BDC methods)
#include <Eigen/Dense>              // Allows to calulate the inverse of Eigen::Matrix

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
using namespace Eigen;

// Include the functions that were made during exercises
#include "base-functions.h"
#include "ex2-functions.h"

int main(int argc, char** argv)
{
    // Initlialization of main:
    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 2 - PnP/data/";    // Define path to data (for exercise 2)
    int numberOfDetectedCornersPerFrame = 12;

    string calibrationMatrixFile = "K.txt";                                                                                     // Define the name of the calibration matrix file
    Matrix3d calibrationMatrix = getCalibrationMatrix(calibrationMatrixFile, datapath);                                         // Call function to read the calibration matrix file and make the calibration matrix

    MatrixXd worldCoordinates = loadWorldCoordinatePoints("p_W_corners.txt", datapath, numberOfDetectedCornersPerFrame);        // Get world coordinates of detected points per frame

    string detectedCornersFileName = "detected_corners.txt";                                                                    // Define the name of the poses text file
    ifstream detectedCornersFile;                                                                                               // Make an inward stream of data called 'detectedCornersFile'
    detectedCornersFile.open (datapath + detectedCornersFileName);                                                              // Open the file in question
    MatrixXd currentDetectedCorners(2, numberOfDetectedCornersPerFrame);                                                        // Create current pose matrix

    VideoCapture cap(datapath + "images_undistorted/img_%04d.jpg");                                                             // Start video capture from images found in folder
    while( cap.isOpened() )                                                                                                     // Loop while we are receiving images from folder
    {
        Mat image;                                                                                                                  // Define image as matrix
        cap.read(image);                                                                                                            // Read image

        if (!image.data)                                                                                                            // If statement in case the file cannot be opened or does not exist
        {
            printf("No image data \n");
            return -1;                                                                                                              // End program
        }

        currentDetectedCorners = getCameraCoordinates(detectedCornersFile, numberOfDetectedCornersPerFrame);                        // Detect corners of the current frame

        drawPointCloud(image, currentDetectedCorners, numberOfDetectedCornersPerFrame);                                             // Draw the detected points in the image

        MatrixXd currentPose = estimatePoseDLT(currentDetectedCorners, worldCoordinates, calibrationMatrix);                        // Estimate the current pose using DLT algorithm

        reprojectPoints(worldCoordinates, currentPose, calibrationMatrix, image);                                                   // Reproject the points using the current pose estimate that we have from estimatePoseDLT
        
        imshow("Display Image", image);                                                                                             // Show the input image

        waitKey(17);                                                                                                                // Wait for X ms
    }

    detectedCornersFile.close();                                                                                                // Close detected corners file

    destroyAllWindows();                                                                                                        // Close all windows
    return 0;
}