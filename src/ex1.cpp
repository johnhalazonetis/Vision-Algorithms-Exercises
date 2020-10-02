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

void drawCube(Mat& image, Vector3d& cubeOrigin, double& length, Matrix3d& calibrationMatrix, MatrixXd& transformationMatrix, VectorXd& distortionArray)   // Function to draw a cube on top of the current frame
{
    // Start by computing the position of the other 7 points in the world frame (keeping all of the same edge lengths)
    Vector3d xTranslation; xTranslation << 1, 0, 0;                                                                                                 // Define a translation on the x axis
    Vector3d yTranslation; yTranslation << 0, 1, 0;                                                                                                 // Define a translation on the y axis
    Vector3d zTranslation; zTranslation << 0, 0, 1;                                                                                                 // Define a translation on the z axis
    
    MatrixXd cubeWorldCorners = MatrixXd::Zero(3, 8);                                                                                               // Define the world coordinates of the corners of the cube

    // Compute the world coordinates of the cube by using simple x, y and z translations in world frame
    cubeWorldCorners.col(1) = cubeOrigin + length * xTranslation;
    cubeWorldCorners.col(2) = cubeOrigin + length * xTranslation + length * yTranslation;
    cubeWorldCorners.col(3) = cubeOrigin + length * yTranslation;
    cubeWorldCorners.col(4) = cubeOrigin - length * zTranslation;
    cubeWorldCorners.col(5) = cubeOrigin + length * xTranslation - length * zTranslation;
    cubeWorldCorners.col(6) = cubeOrigin + length * xTranslation + length * yTranslation - length * zTranslation;
    cubeWorldCorners.col(7) = cubeOrigin + length * yTranslation - length * zTranslation;

    MatrixXi outputCameraCorners(2, 8);                                                                                                             // Define output variable
    Vector3d tempInputWorldCoords;                                                                                                                  // Define temporary input variable
    Vector2i tempOutputCameraCorners;

    for (int cornerNumber = 0; cornerNumber < 8; cornerNumber++)                                                                                    // For loop to go over all of the corner
    {
        tempInputWorldCoords = cubeWorldCorners.col(cornerNumber);                                                                                  // Put the current input world coordinates in a temporary input variable
        tempOutputCameraCorners = projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, distortionArray);      // Output the camera coordinates in temporary variable
        outputCameraCorners.col(cornerNumber) = tempOutputCameraCorners;
    }

    //cout << outputCameraCorners << endl;

    Mat cvOutputCameraCorners(2, 8, CV_64F);
    cvOutputCameraCorners = eigenMatInt2cvMat(outputCameraCorners);
    
    // Draw lines of the cube on the image
    line(image, Point(cvOutputCameraCorners.col(0)), Point(cvOutputCameraCorners.col(1)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(1)), Point(cvOutputCameraCorners.col(2)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(2)), Point(cvOutputCameraCorners.col(3)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(3)), Point(cvOutputCameraCorners.col(0)), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(cvOutputCameraCorners.col(4)), Point(cvOutputCameraCorners.col(5)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(5)), Point(cvOutputCameraCorners.col(6)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(6)), Point(cvOutputCameraCorners.col(7)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(7)), Point(cvOutputCameraCorners.col(4)), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(cvOutputCameraCorners.col(0)), Point(cvOutputCameraCorners.col(4)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(1)), Point(cvOutputCameraCorners.col(5)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(2)), Point(cvOutputCameraCorners.col(6)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(3)), Point(cvOutputCameraCorners.col(7)), Scalar(0, 0, 255), 2, LINE_8);
}

int main(int argc, char** argv)
{
    // Initlialization of main:

    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 1 - Augmented Reality Wireframe Cube/data/";     // Define path to data (for exercise 1)

    string calibrationMatrixFile = "K.txt";                             // Define the name of the calibration matrix file
    Matrix3d calibrationMatrix = getCalibrationMatrix(calibrationMatrixFile, datapath); // Call function to read the calibration matrix file and make the calibration matrix

    MatrixXd transformationMatrix(3, 4);                                // Define the transformation matrix for function 'makeTranslationMatrix'
    string posesFile = "poses.txt";                                     // Define the name of the poses text file
    ifstream poseFile;                                                  // Make an inward stream of data called 'poseFile'
    poseFile.open (datapath + posesFile);                               // Open the file in question (we sum up the datapath and the filename)
    VectorXd currentPose(6, 1);                                         // Create current pose matrix
    
    Vector3d inputWorldPoints;                                          // Define input points vector (in world coordinates)
    Vector2i outputCameraPoints;                                        // Define output points vector (in camera frame)
    bool lensDistortion = 1;                                            // Define whether we want to have lens distortion on or off

    Vector3d cubeOrigin; cubeOrigin << 0.04, 0.04, 0.04;                // Define where we want to place the cube on the chessboard
    double cubeLength = 0.08;                                           // Define length of cube side

    string distortionValuesFile = "D.txt";
    VectorXd distortionArray = getLensDistortionValues(distortionValuesFile, lensDistortion, datapath);
    cout << distortionArray << endl;

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
        //cout << transformationMatrix << endl;
        outputCameraPoints = projectPoints(calibrationMatrix, transformationMatrix, inputWorldPoints, distortionArray); // Call function to project points from world frame to camera frame

        drawCube(image, cubeOrigin, cubeLength, calibrationMatrix, transformationMatrix, distortionArray);              // Call function to draw cube at given cubeOrigin

        imshow("Display Image", image);                                     // Show the input image

        waitKey(20);                                                        // Wait for 15 ms
    }
    poseFile.close();                                                   // Close the poses file

    destroyAllWindows();                                                // Close all windows
    return 0;
}