#include <Eigen/Core>               // Importing general Eigen library
#include <Eigen/SVD>                // Allows to calculate SVD (using Jacobi or faster BDC methods)
#include <Eigen/Dense>              // Allows to calulate the inverse of Eigen::Matrix
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/viz.hpp>          // Import the viz module for showing the camera in 3D space

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
    // Initialisation of main:
    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 2 - PnP/data/";    // Define path to data (for exercise 2)
    int numberOfDetectedCornersPerFrame = 12;

    Matx33d calibrationMatrix = getCalibrationMatrix(datapath + "K.txt");                                       // Call function to read the calibration matrix file and make the calibration matrix

    viz::Viz3d viz_window("Camera Position");                                                                   // Name the window (useful for accessing in the future)
    viz_window.setBackgroundColor();                                                                            // Set background color (default is black)
    viz_window.showWidget("Coordinate Widget", viz::WCoordinateSystem());                                       // Show coordinate widget in 3D space
    
    Vec3d worldCoordinates[numberOfDetectedCornersPerFrame];
    loadWorldCoordinatePoints(datapath + "p_W_corners.txt", worldCoordinates);                                  // Get world coordinates of detected points per frame

    string detectedCornersFileName = "detected_corners.txt";                                                    // Define the name of the poses text file
    ifstream detectedCornersFile;                                                                               // Make an inward stream of data called 'detectedCornersFile'
    detectedCornersFile.open (datapath + detectedCornersFileName);                                              // Open the file in question
    Vec2i currentDetectedCorners[numberOfDetectedCornersPerFrame];                                        // Create current pose matrix

    draw3DPointCloud(worldCoordinates, viz_window);

    VideoCapture cap(datapath + "images_undistorted/img_%04d.jpg");                                             // Start video capture from images found in folder
    while( cap.isOpened() )                                                                                     // Loop while we are receiving images from folder
    {
        Mat image;                                                                                              // Define image as matrix
        cap.read(image);                                                                                        // Read image

        if (!image.data)                                                                                        // If statement in case the file cannot be opened or does not exist
        {
            printf("No image data \n");
            return -1;                                                                                          // End program
        }

        getCameraCoordinates(detectedCornersFile, numberOfDetectedCornersPerFrame, currentDetectedCorners);     // Detect corners of the current frame

        drawPointCloud(image, currentDetectedCorners, numberOfDetectedCornersPerFrame);                         // Draw the detected points in the image

        MatrixXd currentPose = estimatePoseDLT(currentDetectedCorners, worldCoordinates, calibrationMatrix);    // Estimate the current pose using DLT algorithm

        reprojectPoints(worldCoordinates, currentPose, calibrationMatrix, image);                               // Re-project the points using the current pose estimate that we have from estimatePoseDLT
        
        imshow("Display Image", image);                                                                         // Show the input image

        viz::WCameraPosition cp_frustum(cvCalibrationMatrix, 1, viz::Color::yellow());                          // Create the camera frustum widget

        MatrixXd currentTransform = MatrixXd::Identity(4, 4); currentTransform.block<3, 4>(0, 0) = currentPose; // Create "currentTransform" matrix as homogenous transf. of currentPose
        Matx44d cvCurrentPose; eigen2cv(currentTransform, cvCurrentPose);                                       // Create equivalent matrix in OpenCV and convert currentTransform to OpenCV Matx44d
        viz_window.showWidget("CP_Frustum", cp_frustum, cvCurrentPose);                                         // Move the camera widget with respect to the calculated current pose of the camera

        viz_window.spinOnce(1, true);                                                                           // Update viz window
        waitKey(17);                                                                                            // Wait for X ms
    }

    detectedCornersFile.close();                                                                                // Close detected corners file

    destroyAllWindows();                                                                                        // Close all windows
    return 0;
}