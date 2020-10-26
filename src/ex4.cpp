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
    double sigma0 = 1.6;                // Base sigma used for image smoothing in image pyramids
    double constrastThreshold = 0.04;   // Constant threshold for point suppression
    double rescaleFactor = 0.2;         // Rescaling of the original image for speed

    // Read both images in directory
    Mat inputImages[2];
    inputImages[0] = importImageGreyRescale(datapath + "img_1.jpg", rescaleFactor);
    inputImages[1] = importImageGreyRescale(datapath + "img_2.jpg", rescaleFactor);

    /*/ To test rotation invariance of SIFT
    if (rotationImage2 != 0)
    {
        //...
    }*/

    Mat blurredImagePyramid[2][numberOfOctaves][numberOfScales+3];                                                                      // Create 2D array for the blurred images pyramid
    Mat dogImagePyramid[2][numberOfOctaves][numberOfScales+2];                                                                          // Create 2D array for the DoG image pyramid

    // Loop through the two images
    for (int imageN = 0; imageN < 2; imageN++)                                                                                          // Loop through the two images
    {
        // Compute the image pyramids
        for (int octaveN = 0; octaveN < numberOfOctaves-1; octaveN++)                                                                   // Loop over the number of ocatves for each image
        {
            for (int scaleN = -1; scaleN < numberOfScales+2; scaleN++)                                                                  // Loop over the number of scales for each image
            {
                Mat tempImage;                                                                                                          // Define a temporary image
                resize(inputImages[imageN], tempImage, Size(), pow(2, -octaveN), pow(2, -octaveN));                                     // Resize the image with the given rescale factor
                double sigma = sigma0 * pow(2, (double)scaleN/(double)numberOfScales);                                                  // Compute sigma for smoothing
                GaussianBlur(tempImage, tempImage, Size(0, 0), sigma, sigma);                                                           // Compute the Gaussian blur
                blurredImagePyramid[imageN][octaveN][scaleN+1] = tempImage;                                                             // Put the blurred image into the image pyramid

                if (scaleN != -1)                                                                                                       // Wait until the first iteration is over
                {
                    dogImagePyramid[imageN][octaveN][scaleN] = blurredImagePyramid[octaveN][scaleN] - blurredImagePyramid[octaveN][scaleN+1];   // Compute the DoG
                }
            }
        }

        
        // Write code to compute:
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