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
    file.close();
}

void makeTransforationMatrix(Mat& currentPose, Mat transformationMatrix)    // Function to get the poses from a text file and make a transformation matrix
{
    // Get rotation values and make rotation matrix (Rodrigues formula)
    Mat omega = currentPose.rowRange(0,3);                                            // Make a vector called 'omega', where we put out values of omega

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
    Mat translationMatrix = currentPose.rowRange(3,6);                                // Define the translation vector

    hconcat(rotationMatrix, translationMatrix, transformationMatrix);   // Concatenate the rotation matrix and the translation vector to the transformation matrix
}

void projectPoints(Mat& calibrationMatrix, Mat& transformationMatrix, Mat& inputWorldPoints, bool& lensDistortion, Mat outputCameraPoints)      // Function to project world coordinates onto image plane (with lens distortion correction if requested)
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
        file.close();

        normalizedCoordinates = (1 + k1*pow(r, 2) + k2*pow(r, 4)) * normalizedCoordinates;  // Compute the distorted normalized coordinates
    }
    
    Mat outputCameraPointsHomo(3, 1, CV_64F);                           // Define output camera points vector in homogeneous coordinates
    Mat normalizedCoordinatesHomo(3, 1, CV_64F);                        // Define normalized camera coordinates vector in homogeneous coordinates
    vconcat(normalizedCoordinates, Mat::eye(1, 1, CV_64F), normalizedCoordinatesHomo);      // Make normalized camera coordinates vector in homogeneous coordinates

    outputCameraPointsHomo = calibrationMatrix * normalizedCoordinatesHomo;                 // Calculate output camera points in homogenous coordinates
    
    int cameraCoordinate0 = outputCameraPointsHomo.at<double>(0, 0) / outputCameraPointsHomo.at<double>(2, 0);  // Normalize output camera coordinates and convert to integer (for pixel coordinates)
    int cameraCoordinate1 = outputCameraPointsHomo.at<double>(1, 0) / outputCameraPointsHomo.at<double>(2, 0);  // Normalize output camera coordinates and convert to integer (for pixel coordinates)

    outputCameraPoints.at<double>(0, 0) = cameraCoordinate0;            // Input integer values to output camera coordinates vector
    outputCameraPoints.at<double>(1, 0) = cameraCoordinate1;            // Input integer values to output camera coordinates vector
}

void drawCube(Mat& image, Mat& cubeOrigin, double& length, Mat& calibrationMatrix, Mat& transformationMatrix, bool& lensDistortion)                 // Function to draw a cube on top of the current frame
{
    // Start by computing the position of the other 7 points in the world frame (keeping all of the same edge lengths)
    Mat xTranslation = Mat::eye(3, 3, CV_64F).col(0);
    Mat yTranslation = Mat::eye(3, 3, CV_64F).col(1);
    Mat zTranslation = Mat::eye(3, 3, CV_64F).col(2);
    
    Mat cubeWorldCorners = Mat::zeros(3, 7, CV_64F);
    cubeWorldCorners.col(0) = cubeOrigin + length * xTranslation;
    cubeWorldCorners.col(1) = cubeOrigin + length * xTranslation + length * yTranslation;
    cubeWorldCorners.col(2) = cubeOrigin + length * yTranslation;
    cubeWorldCorners.col(3) = cubeOrigin - length * zTranslation;
    cubeWorldCorners.col(4) = cubeOrigin + length * xTranslation - length * zTranslation;
    cubeWorldCorners.col(5) = cubeOrigin + length * xTranslation + length * yTranslation - length * zTranslation;
    cubeWorldCorners.col(6) = cubeOrigin + length * yTranslation - length * zTranslation;
    hconcat(cubeOrigin, cubeWorldCorners, cubeWorldCorners);

    Mat outputCameraCorners = Mat::zeros(2, 8, CV_64F);
    Mat tempInputWorldCoords(3, 1, CV_64F);
    Mat tempOutputCameraCoords(2, 1, CV_64F);
    
    tempInputWorldCoords = cubeWorldCorners.col(0);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(0));
    tempInputWorldCoords = cubeWorldCorners.col(1);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(1));
    tempInputWorldCoords = cubeWorldCorners.col(2);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(2));
    tempInputWorldCoords = cubeWorldCorners.col(3);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(3));
    tempInputWorldCoords = cubeWorldCorners.col(4);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(4));
    tempInputWorldCoords = cubeWorldCorners.col(5);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(5));
    tempInputWorldCoords = cubeWorldCorners.col(6);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(6));
    tempInputWorldCoords = cubeWorldCorners.col(7);
    projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, lensDistortion, outputCameraCorners.col(7));
    
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

    Mat calibrationMatrix(3, 3, CV_64F);                                // Define calibration matrix for function 'readCalibrationMatrixFile'
    string calibrationMatrixFile = "K.txt";                             // Define the name of the calibration matrix file
    readCalibrationMatrix(calibrationMatrixFile, calibrationMatrix);    // Call function to read the calibration matrix file and make the calibration matrix
    cout << "Calibration Matrix (K) =" << endl << calibrationMatrix << endl;

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

    VideoCapture cap(datapath + "images/img_%04d.jpg");                 // Start video capture from images found in folder
    int imageNumber = 1;                                                // Start counting images that we get as inputs
    while( cap.isOpened() )                                             // Loop while we are receiving images from folder
    {
        Mat image;                                                          // Define image as matrix
        cap.read(image);                                                    // Read image

        if (!image.data)                                                    // If statement in case the file cannot be opened or does not exist
        {
            printf("No image data \n");
            return -1;                                                      // End program
        }

        for (int r = 0; r < 6; r++)                                         // For loop to go through the elements of our translation matrix
        {
            poseFile >> currentPose.at<double>(r, 0);                       // Get the latest value from our text file 'poseFile'
        }
        makeTransforationMatrix(currentPose, transformationMatrix);         // Call function to read the poses and create a transformation matrix

        projectPoints(calibrationMatrix, transformationMatrix, inputWorldPoints, lensDistortion, outputCameraPoints);   // Call function to project points from world frame to camera frame

        circle(image, Point(outputCameraPoints.col(0)), 3, Scalar( 0, 0, 255 ), FILLED, LINE_8 );                       // Draw point at inputWorldCoordinates

        drawCube(image, cubeOrigin, cubeLength, calibrationMatrix, transformationMatrix, lensDistortion);               // Call function to draw cube at given cubeOrigin

        imshow("Display Image", image);                                     // Show the input image

        waitKey(15);                                                        // Wait for 15 ms
        imageNumber++;                                                      // Increase imageNumber by 1
    }
    poseFile.close();

    return 0;
}