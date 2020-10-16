#include <Eigen/Core>               // Importing general Eigen library
#include <Eigen/SVD>                // Allows to calculate SVD (using Jacobi or faster BDC methods)
#include <Eigen/Dense>              // Allows to calulate the inverse of Eigen::Matrix
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;
using namespace Eigen;

// Include the functions that were made during exercises
#include "base-functions.h"
#include "ex3-functions.h"

int main(int argc, char** argv)
{
    // Initialisation of main:
    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 3 - Simple Keypoint Tracker/data/";    // Define path to data (for exercise 3)

    // Randomly chosen parameters that seem to work well - can you find better ones?
    int cornerPatchSize = 9;
    double harrisKappa = 0.08;
    int numKeypoints = 200;
    int nonmaximumSupressionRadius = 8;
    int descriptorRadius = 9;
    int matchLambda = 4;

    Mat scores, keypoints, desc, previousDesc, previousKeypoints;
    VectorXd matches;
    int frameNumber = 0;

    VideoCapture cap(datapath + "%06d.png");                                                // Start video capture from images found in folder
    while( cap.isOpened() )                                                                 // Loop while we are receiving images from folder
    {
        Mat image;                                                                          // Define image as matrix
        cap.read(image);                                                                    // Read image

        if (!image.data)                                                                    // If statement in case the file cannot be opened or does not exist
        {
            printf("No image data \n");
            return -1;                                                                      // End program
        }

        scores = shiTomasi(image, cornerPatchSize);
        // keypoints = selectKeypoints(scores, numKeypoints, nonmaximumSupressionRadius);
        // (plot the keypoints)

        /* desc = describeKeypoints(image, cornerPatchSize, harrisKappa);

        if (frameNumber != 0)
        {
            matches = matchDescriptors(desc, previousDesc, matchLambda);
            plotMatches(matches, keypoints, previousKeypoints);
        }

        previousKeypoints = keypoints;
        previousDesc = desc; */

        imshow("Display Image", scores);                                                    // Show the input image

        waitKey(1);                                                                         // Wait for X ms
        frameNumber++;
    }

    destroyAllWindows();                                                                    // Close all windows
    return 0;
}