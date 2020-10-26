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
    double sigma0 = 1.6;                // Base sigma that we use for image smoothing in image pyramids
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

    // Loop through the two images
    for (int imageI = 0; imageI < 2; imageI++)                                          // Loop through the two images
    {
        // Compute the image pyramids
        Mat blurredImagePyramid[numberOfOctaves][numberOfScales+3];                     // Create 2D array for the blurred images pyramid
        for (int r = 0; r < numberOfOctaves-1; r++)                                     // Loop over the number of ocatves we have for each image
        {
            for (int s = -1; s < numberOfScales+2; s++)                                 // Loop over the number of scales we have for each image
            {
                Mat tempImage;                                                          // Define a temporary image
                resize(inputImages[imageI], tempImage, Size(), pow(2, -r), pow(2, -r)); // Resize the image with the given rescale factor
                double sigma = sigma0 * pow(2, (double)s/(double)numberOfScales);       // Compute sigma for smoothing
                GaussianBlur(tempImage, tempImage, Size(0, 0), sigma, sigma);           // Compute the Gaussian blur
                blurredImagePyramid[r][s+1] = tempImage;                                // Put the blurred image into the image pyramid
            }
        }

        Mat dogImagePyramid[numberOfOctaves][numberOfScales+2];                         // Create 2D array for the DoG image pyramid
        for (int octaveN = 0; octaveN < numberOfOctaves-1; octaveN++)                   // Loop through the octaves of the blurredImagePyramid array
        {
            for (int scaleN = 0; scaleN < numberOfScales+2; scaleN++)                   // Loop through the number of scales of the blurredImagePyramid array
            {
                dogImagePyramid[octaveN][scaleN] = blurredImagePyramid[octaveN][scaleN] - blurredImagePyramid[octaveN][scaleN+1];   // Compute the DoG
            }
        }
        
        //imshow("dogImagePyramid[0][0]", dogImagePyramid[0][0]);

        
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