#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

// Include the functions that were made during course work
#include "base-functions.h"

int main(int argc, char** argv)
{
    // Initialisation of main:

    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 1 - Augmented Reality Wireframe Cube/data/";     // Define path to data (for exercise 1)

    Matx33d calibrationMatrix = getCalibrationMatrix(datapath + "K.txt"); // Call function to read the calibration matrix file and make the calibration matrix

    Matx34d transformationMatrix;                                // Define the transformation matrix for function 'makeTranslationMatrix'
    string posesFile = "poses.txt";                                     // Define the name of the poses text file
    ifstream poseFile;                                                  // Make an inward stream of data called 'poseFile'
    poseFile.open (datapath + posesFile);                               // Open the file in question (we sum up the datapath and the filename)
    
    Vec3d inputWorldPoints;                                          // Define input points vector (in world coordinates)
    Vec2i outputCameraPoints;                                        // Define output points vector (in camera frame)

    Vec3d cubeOrigin = (0.04, 0.04, 0);                   // Define where we want to place the cube on the chessboard
    double cubeLength = 0.08;                                           // Define length of cube side

    double distortionArray[1];
    getLensDistortionValues(datapath + "D.txt", 1, 2, distortionArray);

    VideoCapture cap(datapath + "images/img_%04d.jpg");                 // Start video capture from images found in folder
    while( cap.isOpened() )                                             // Loop while we are receiving images from folder
    {
        Mat image;                                                          // Define image as matrix
        cap.read(image);                                                    // Read image

        if (!image.data)                                                    // If statement in case the file cannot be opened or does not exist
        {
            printf("No image data \n");
            return -1;                                                      // End program
        }

        Vec6d currentPose = getPose(poseFile);                           // Call function to get the current pose
        transformationMatrix = makeTransforationMatrix(currentPose);        // Call function to read the poses and create a transformation matrix
        
        outputCameraPoints = projectPoints(calibrationMatrix, transformationMatrix, inputWorldPoints, distortionArray); // Call function to project points from world frame to camera frame
        
        drawCube(image, cubeOrigin, cubeLength, calibrationMatrix, transformationMatrix, distortionArray);              // Call function to draw cube at given cubeOrigin

        imshow("Display Image", image);                                     // Show the input image

        waitKey(17);                                                        // Wait for X ms
    }
    poseFile.close();                                                   // Close the poses file

    destroyAllWindows();                                                // Close all windows
    return 0;
}