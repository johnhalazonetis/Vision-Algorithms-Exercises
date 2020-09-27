#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float chessboardSquareDimension = 0.04f;      // Dimension of side of one aruco square [m]
const Size chessboardDimensions = Size(6, 9);       // Number of square on chessboard calibration page

const string datapath = "/home/john/Nextcloud/Me/ETH/Master 4 (Fall 2020)/Vision Algorithms/Exercises/Exercise 1 - Augmented Reality Wireframe Cube/data/";     // Define path to data (for exercise 1)

void readCalibrationMatrix(string& filename, Mat calibrationMatrix)         // Function to read calibration matrix from txt file
{
    ifstream file;                                                      // Make an inward stream of data called 'file'
    file.open (datapath + filename);                                    // Open the file in question (we sum up the datapath and the filename)
    if (!file.is_open()) return;                                        // Insurance in case the file is not opened or doesn't exist
    double value;                                                       // Make variable 'value', where we put the values that we get from file                                  
    
    for (int r = 0; r < 3; r++)                                         // Make for loop to go through the rows of our calibration matrix
    {
        for (int c = 0; c < 3; c++)                                     // Make for loop to go through the columns of our calibration matrix
        {
            file >> value;                                              // Get the value from our file
            calibrationMatrix.at<double>(r, c) = value;                 // Place each new value from the file into the calibration matrix
        }
    }
}

void makeTransforationMatrix(string& filename, Mat transformationMatrix)    // Function to get the poses from a text file and make a transformation matrix
{
    ifstream file;                                                      // Make an inward stream of data called 'file'
    file.open (datapath + filename);                                    // Open the file in question (we sum up the datapath and the filename)
    if(!file.is_open()) return;                                         // Insurance in case the file is not opened or does not exist
    double value;                                                       // Make a variable 'value', where we put the values that we get from file

    // Get rotation values and make rotation matrix (Rodrigues formula)
    Mat omega(3, 1, CV_64F);                                            // Make a vector called 'omega', where we put out values of omega
    for (int r = 0; r < 3; r++)                                         // Make a for loop to go through the rows of our vector 'omega'
    {
        file >> value;                                                  // Get the value from our file
        omega.at<double>(r, 0) = value;                                 // Put value from file in the 'omega' vector
    }
    double normOmega = sqrt(pow(omega.at<double>(0, 0) , 2) + pow(omega.at<double>(1, 0) , 2) + pow(omega.at<double>(2, 0) , 2));   // Calulate the norm of the vector 'omega'

    Mat rotationMatrix(3, 3, CV_64F);                                   // Define the roation matrix in our transformation
    Mat kCrossProduct(3, 3, CV_64F);                                    // Define the k cross product matrix (we use the Rodrigues formula to find the rotation matrix)
    
    // Manually input values of k cross product matrix
    kCrossProduct.at<double>(0, 0) = 0;
    kCrossProduct.at<double>(0, 1) = -omega.at<double>(2, 0)/normOmega;
    kCrossProduct.at<double>(0, 2) = omega.at<double>(1, 0)/normOmega;
    kCrossProduct.at<double>(1, 0) = omega.at<double>(2, 0)/normOmega;
    kCrossProduct.at<double>(1, 1) = 0;
    kCrossProduct.at<double>(1, 2) = -omega.at<double>(0, 0)/normOmega;
    kCrossProduct.at<double>(2, 0) = -omega.at<double>(1, 0)/normOmega;
    kCrossProduct.at<double>(2, 1) = omega.at<double>(0, 0)/normOmega;
    kCrossProduct.at<double>(2, 2) = 0;

    rotationMatrix = Mat::eye(3, 3, CV_64F) + (sin(normOmega)) * kCrossProduct + (1 - cos(normOmega)) * (kCrossProduct * kCrossProduct);    // Apply Rodrigues formula to calculate the rotation matrix

    // Making the translation matrix out of the next three values from the poses.txt file
    Mat translationMatrix(3, 1, CV_64F);                                // Define the translation vector
    for (int r = 0; r < 3; r++)                                         // For loop to go through the elements of our translation matrix
    {
        file >> value;                                                  // Get the latest value from our text file
        translationMatrix.at<double>(r, 0) = value;                     // Put value in translation matrix
    }

    hconcat(rotationMatrix, translationMatrix, transformationMatrix);   // Concatenate the rotation matrix and the translation vector to the transformation matrix
}

void projectPoints(Mat& calibrationMatrix, Mat& transformationMatrix, Mat& inputWorldPoints, bool& lensDistortion, Mat outputCameraPoints)
{
    Mat worldPointsHomo = Mat::zeros(4, 1, CV_64F);                     // Define homogeneous vector for world points (empty)
    vconcat(inputWorldPoints, Mat::eye(1, 1, CV_64F), worldPointsHomo); // Create homogeneous coordinates vector for world points

    Mat cameraPointsHomo = Mat::zeros(4, 1, CV_64F);                    // Define the camera points vector in homogeneous coordinates
    cameraPointsHomo = transformationMatrix * worldPointsHomo;          // Calculate the camera points in homogeneous coordinates

    Mat normalizedCoordinates(2, 1 ,CV_64F);                            // Define the normalized coordinates vector
    normalizedCoordinates.at<double>(0, 0) = cameraPointsHomo.at<double>(0, 0)/cameraPointsHomo.at<double>(2, 0);   // Calculate first element of the normalized coordate
    normalizedCoordinates.at<double>(1, 0) = cameraPointsHomo.at<double>(1, 0)/cameraPointsHomo.at<double>(2, 0);   // Calculate second element of the normalized coordate

    if (lensDistortion)                                                 // If we want to account for lens distortion as well
    {
        double r = sqrt(pow(normalizedCoordinates.at<double>(0, 0), 2) + pow(normalizedCoordinates.at<double>(1, 0), 2));   // Compute the radial component of normalized coordinates

        ifstream file;                                                  // Make an inward stream of data called 'file'
        file.open (datapath + "D.txt");                                 // Open the file in question (we sum up the datapath and the filename)
        if(!file.is_open()) return;                                     // Insurance in case the file is not opened or does not exist
        double k1, k2;                                                  // Make a variable 'k1' and 'k2', where we put the values that we get from file
        file >> k1; file >> k2;                                         // Put values from distortion file into variables k1 and k2

        normalizedCoordinates = (1 + k1*pow(r, 2) + k2*pow(r, 4)) * normalizedCoordinates;  // Compute the distorted normalized coordinates
    }
    
    Mat outputCameraPointsHomo(3, 1, CV_64F);                           // Define output camera points vector in homogeneous coordinates
    Mat normalizedCoordinatesHomo(3, 1, CV_64F);                        // Define normalized camera coordinates vector in homogeneous coordinates
    vconcat(normalizedCoordinates, Mat::eye(1, 1, CV_64F), normalizedCoordinatesHomo);  // Make normalized camera coordinates vector in homogeneous coordinates

    outputCameraPointsHomo = calibrationMatrix * normalizedCoordinatesHomo; // Calculate output camera points in homogenous coordinates
    
    int cameraCoordinate0 = outputCameraPointsHomo.at<double>(0, 0) / outputCameraPointsHomo.at<double>(2, 0);  // Normalize output camera coordinates and convert to integer (for pixel coordinates)
    int cameraCoordinate1 = outputCameraPointsHomo.at<double>(1, 0) / outputCameraPointsHomo.at<double>(2, 0);  // Normalize output camera coordinates and convert to integer (for pixel coordinates)

    outputCameraPoints.at<double>(0, 0) = cameraCoordinate0;            // Input integer values to output camera coordinates vector
    outputCameraPoints.at<double>(1, 0) = cameraCoordinate1;            // Input integer values to output camera coordinates vector
}

void drawCube(Mat& image, Mat& cubeOrigin, float& length, Mat& calibrationMatrix, Mat& transformationMatrix, bool& lensDistortion)                 // Function to draw a cube on top of the current frame
{
    // Start by computing the position of the other 7 points in the world frame (keeping all of the same edge lengths)
    Mat cubeWorldCorners(3, 8, CV_64F);
    cubeWorldCorners.col(0) = cubeOrigin;
    cubeWorldCorners.col(1) = cubeOrigin + length * Mat::eye(3, 3, CV_64F).col(0);
    cubeWorldCorners.col(2) = cubeOrigin + length * Mat::eye(3, 3, CV_64F).col(1);
    cubeWorldCorners.col(3) = cubeOrigin + length * Mat::eye(3, 3, CV_64F).col(0) + length * Mat::eye(3, 3, CV_64F).col(1);
    cubeWorldCorners.col(4) = cubeOrigin - length * Mat::eye(3, 3, CV_64F).col(2);
    cubeWorldCorners.col(5) = cubeOrigin + length * Mat::eye(3, 3, CV_64F).col(0) - length * Mat::eye(3, 3, CV_64F).col(2);
    cubeWorldCorners.col(6) = cubeOrigin + length * Mat::eye(3, 3, CV_64F).col(1) - length * Mat::eye(3, 3, CV_64F).col(2);
    cubeWorldCorners.col(7) = cubeOrigin + length * Mat::eye(3, 3, CV_64F).col(0) + length * Mat::eye(3, 3, CV_64F).col(1) - length * Mat::eye(3, 3, CV_64F).col(2);

    Mat outputCameraCorners(2, 8, CV_64F);
    Mat tempInputWorldCoords(3, 1, CV_64F);
    Mat tempOutputCameraCoords(2, 1, CV_64F);
    for (int c = 0; c <= 8; c++)
    {
        tempInputWorldCoords = cubeWorldCorners.col(c);
        projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, tempOutputCameraCoords);
        outputCameraCorners.col(c) = tempOutputCameraCoords;
    }

    cout << outputCameraCorners << endl;
    
    //line(image, Point(outputCameraCorners.col(0)), Point(outputCameraCorners.col(1)), Scalar(0, 0, 255), 3, LINE_8);
}

int main(int argc, char** argv)
{
    Mat image = imread(datapath + "images/img_0001.jpg");               // Define image in file

    if (!image.data)                                                    // If statement in case the file cannot be opened or does not exist
    {
        printf("No image data \n");
        return -1;                                                      // End program
    }

    Mat calibrationMatrix(3, 3, CV_64F);                                // Define calibration matrix for function 'readCalibrationMatrixFile'
    string calibrationMatrixFile = "K.txt";                             // Define the name of the calibration matrix file
    readCalibrationMatrix(calibrationMatrixFile, calibrationMatrix);    // Call function to read the calibration matrix file and make the calibration matrix

    Mat transformationMatrix(3, 4, CV_64F);                             // Define the transformation matrix for function 'makeTranslationMatrix'
    string posesFile = "poses.txt";                                     // Define the name of the poses text file
    makeTransforationMatrix(posesFile, transformationMatrix);           // Call function to read the poses and create a transformation matrix

    Mat projectionMatrix(3, 4, CV_64F);                                 // Define projection matrix
    projectionMatrix = calibrationMatrix * transformationMatrix;        // Multiply calibration matrix by transformation matrix to make projection matrix

    Mat inputWorldPoints = Mat::zeros(3,1, CV_64F);                     // Define input points vector (in world coordinates)
    Mat outputCameraPoints(2, 1, CV_64F);                               // Define output points vector (in camera frame)
    bool lensDistortion = 1;                                            // Define whether we want to have lens distortion on or off
    projectPoints(calibrationMatrix, transformationMatrix, inputWorldPoints, lensDistortion, outputCameraPoints);  // Call function to project points from world frame to camera frame

    circle(image, Point(outputCameraPoints.at<double>(0,0), outputCameraPoints.at<double>(1,0)), 3, Scalar( 0, 0, 255 ), FILLED, LINE_8 );  // Draw point at inputWorldCoordinates

    Mat cubeOrigin = Mat::zeros(3, 1, CV_64F);
    float length = 0.08f;
    drawCube(image, cubeOrigin, length, calibrationMatrix, transformationMatrix, lensDistortion);

    namedWindow("Display Image", WINDOW_KEEPRATIO);                     // Create window where the ratio of input image is kept
    imshow("Display Image", image);                                     // show the input image

    resizeWindow("Display Image", image.cols, image.rows);              // Resize the window to the image dimensions

    moveWindow("Display Image", 500, 500);                              // Move the window in the screen

    waitKey(0);

    return 0;
}