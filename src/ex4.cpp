#include <Eigen/Core>               // Importing general Eigen library
#include <Eigen/SVD>                // Allows to calculate SVD (using Jacobi or faster BDC methods)
#include <Eigen/Dense>              // Allows to calulate the inverse of Eigen::Matrix
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
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
using namespace Eigen;

// Include the functions that were made during exercises
#include "base-functions.h"
#include "ex4-functions.h"

int main(int argc, char** argv)
{
    // Initialisation of main:
    string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 4/images/";    // Define path to data (for exercise 4)

    // Tunable parameters & constants:
    double rotationInv = 1;
    double rotationImage2 = 0;

    int numberOfScales = 3;             // Number of scales per octave
    int numberOfOctaves = 5;            // Number of octaves
    double sigma = 1.6;
    double constrastThreshold = 0.04;
    double rescaleFactor = 0.2;         // Rescaling of the original image for speed

    // Read both images in directory
    MatrixXd leftImage = importImageToEigen(datapath+"img_1.jpg", rescaleFactor);
    MatrixXd rightImage = importImageToEigen(datapath+"img_2.jpg", rescaleFactor);

    // To test rotation invariance of SIFT
    if (rotationImage2 != 0)
    {
        //...
    }
    

    // Loop through the two images and
    for (int imageI = 0; imageI < 2; imageI++)                  // Loop through the two images
    {
        // Write code to compute:
        // 1)    image pyramid. Number of images in the pyarmid equals
        //       'num_octaves'.
        // 2)    blurred images for each octave. Each octave contains
        //       'num_scales + 3' blurred images.
        // 3)    'num_scales + 2' difference of Gaussians for each octave.
        // 4)    Compute the keypoints with non-maximum suppression and
        //       discard candidates with the contrast threshold.
        // 5)    Given the blurred images and keypoints, compute the
        //       descriptors. Discard keypoints/descriptors that are too close
        //       to the boundary of the image. Hence, you will most likely
        //       lose some keypoints that you have computed earlier.
    }
    
    // Finally, match the descriptors using the function 'matchFeatures' and
    // visualize the matches with the function 'showMatchedFeatures'.
    // If you want, you can also implement the matching procedure yourself using
    // 'knnsearch'.

    waitKey(0);
    return 0;
}