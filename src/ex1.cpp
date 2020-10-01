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

void drawCube(Mat image, Mat cubeOrigin, double length, Mat calibrationMatrix, Mat transformationMatrix, Mat distortionArray)                 // Function to draw a cube on top of the current frame
{
    // Start by computing the position of the other 7 points in the world frame (keeping all of the same edge lengths)
    Mat xTranslation = Mat::eye(3, 3, CV_64F).col(0);                                                                                       // Define a translation on the x axis
    Mat yTranslation = Mat::eye(3, 3, CV_64F).col(1);                                                                                       // Define a translation on the y axis
    Mat zTranslation = Mat::eye(3, 3, CV_64F).col(2);                                                                                       // Define a translation on the z axis
    
    Mat cubeWorldCorners = Mat::zeros(3, 7, CV_64F);                                                                                        // Define the world coordinates of the corners of the cube

    // Compute the world coordinates of the cube by using simple x, y and z translations in world frame
    cubeWorldCorners.col(0) = cubeOrigin + length * xTranslation;
    cubeWorldCorners.col(1) = cubeOrigin + length * xTranslation + length * yTranslation;
    cubeWorldCorners.col(2) = cubeOrigin + length * yTranslation;
    cubeWorldCorners.col(3) = cubeOrigin - length * zTranslation;
    cubeWorldCorners.col(4) = cubeOrigin + length * xTranslation - length * zTranslation;
    cubeWorldCorners.col(5) = cubeOrigin + length * xTranslation + length * yTranslation - length * zTranslation;
    cubeWorldCorners.col(6) = cubeOrigin + length * yTranslation - length * zTranslation;

    hconcat(cubeOrigin, cubeWorldCorners, cubeWorldCorners);                                                                                // Add the cube origin to the world coordinates of the cube (wasn't added previously)

    Mat outputCameraCorners(2, 8, CV_64F);                                                                                                  // Define output variable
    Mat tempInputWorldCoords(3, 1, CV_64F);                                                                                                 // Define temporary input variable
    Mat tempOutputCameraCoords(2, 1, CV_64F);                                                                                               // Define temporary output variable

    for (int cornerNumber = 0; cornerNumber < 8; cornerNumber++)                                                                            // For loop to go over all of the corner
    {
        tempInputWorldCoords = cubeWorldCorners.col(cornerNumber);                                                                          // Put the current input world coordinates in a temporary input variable
        tempOutputCameraCoords = projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, distortionArray);             // Output the camera coordinates in temporary variable
        
        outputCameraCorners.at<double>(0, cornerNumber) = tempOutputCameraCoords.at<double>(0, 0);                                          // Put the first value of the temporary result in the output array
        outputCameraCorners.at<double>(1, cornerNumber) = tempOutputCameraCoords.at<double>(1, 0);                                          // Put the second value of the temporary result in the output array
    }
    
    // Draw lines of the cube on the image
    line(image, Point(outputCameraCorners.col(0)), Point(outputCameraCorners.col(1)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(1)), Point(outputCameraCorners.col(2)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(2)), Point(outputCameraCorners.col(3)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(3)), Point(outputCameraCorners.col(0)), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(outputCameraCorners.col(4)), Point(outputCameraCorners.col(5)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(5)), Point(outputCameraCorners.col(6)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(6)), Point(outputCameraCorners.col(7)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(7)), Point(outputCameraCorners.col(4)), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(outputCameraCorners.col(0)), Point(outputCameraCorners.col(4)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(1)), Point(outputCameraCorners.col(5)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(2)), Point(outputCameraCorners.col(6)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners.col(3)), Point(outputCameraCorners.col(7)), Scalar(0, 0, 255), 2, LINE_8);
    
}

int main(int argc, char** argv)
{
    // Initlialization of main:

    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 1 - Augmented Reality Wireframe Cube/data/";     // Define path to data (for exercise 1)

    string calibrationMatrixFile = "K.txt";                             // Define the name of the calibration matrix file
    Mat calibrationMatrix = getCalibrationMatrix(calibrationMatrixFile, datapath);// Call function to read the calibration matrix file and make the calibration matrix

    Mat transformationMatrix(3, 4, CV_64F);                             // Define the transformation matrix for function 'makeTranslationMatrix'
    string posesFile = "poses.txt";                                     // Define the name of the poses text file
    ifstream poseFile;                                                  // Make an inward stream of data called 'poseFile'
    poseFile.open (datapath + posesFile);                               // Open the file in question (we sum up the datapath and the filename)
    Mat currentPose(6, 1, CV_64F);                                      // Create current pose matrix
    
    Mat inputWorldPoints = Mat::zeros(3,1, CV_64F);                     // Define input points vector (in world coordinates)
    Mat outputCameraPoints(2, 1, CV_64F);                               // Define output points vector (in camera frame)
    bool lensDistortion = 1;                                            // Define whether we want to have lens distortion on or off

    Mat cubeOrigin = 0.04*Mat::eye(3, 1, CV_64F);                       // Define where we want to place the cube on the chessboard
    double cubeLength = 0.08;                                           // Define length of cube side

    string distortionValuesFile = "D.txt";
    Mat distortionArray = getLensDistortionValues(distortionValuesFile, lensDistortion, datapath);

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

        getPose(poseFile, currentPose);                                     // Call function to get the current pose
        transformationMatrix = makeTransforationMatrix(currentPose);        // Call function to read the poses and create a transformation matrix

        outputCameraPoints = projectPoints(calibrationMatrix, transformationMatrix, inputWorldPoints, distortionArray); // Call function to project points from world frame to camera frame

        circle(image, Point(outputCameraPoints.col(0)), 3, Scalar( 0, 0, 255 ), FILLED, LINE_8 );                       // Draw point at inputWorldCoordinates

        drawCube(image, cubeOrigin, cubeLength, calibrationMatrix, transformationMatrix, distortionArray);              // Call function to draw cube at given cubeOrigin

        imshow("Display Image", image);                                     // Show the input image

        waitKey(20);                                                        // Wait for 15 ms
    }
    poseFile.close();

    destroyAllWindows();                                                    // Close all windows
    return 0;
}