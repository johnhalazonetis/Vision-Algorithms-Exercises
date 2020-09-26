#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float arucoSquareDimension = 0.04f;           // Dimension of side of one aruco square [m]
const Size chessboardDimensions = Size(6, 9);       // Number of square on chessboard calibration page

const string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 1 - Augmented Reality Wireframe Cube/data/";

int main(int argc, char** argv) {
    Mat image = imread(datapath + "images_undistorted/img_0001.jpg");

    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }

    namedWindow("Display Image", WINDOW_KEEPRATIO);
    imshow("Display Image", image);

    resizeWindow("Display Image", image.cols, image.rows);

    moveWindow("Display Image", 700, 500);

    waitKey(0);

    return 0;
}