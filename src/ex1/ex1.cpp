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

Mat getCalibrationMatrix(string& filename)                  // Function to read calibration matrix from txt file
{
    Mat calibrationMatrix(3, 3, CV_64F);
    ifstream file;                                          // Make an inward stream of data called 'file'
    file.open (datapath + filename);                        // Open the file in question (we sum up the datapath and the filename)
    double value;                                           // Make variable 'value', where we put the values that we get from file                                  
    
    for (int r = 0; r < 3; r++)                             // Make for loop to go through the rows of our calibration matrix
    {
        for (int c = 0; c < 3; c++)                         // Make for loop to go through the columns of our calibration matrix
        {
            file >> value;                                  // Get the value from our file
            calibrationMatrix.at<double>(r, c) = value;     // Place each new value from the file into the calibration matrix
        }
    }
    file.close();                                           // Close the file (we're not savages)

    return calibrationMatrix;
}

Mat makeTransforationMatrix(Mat& currentPose)    // Function to get the poses from a text file and make a transformation matrix
{
    Mat transformationMatrix(3, 4, CV_64F);
    
    // Get rotation values and make rotation matrix (Rodrigues formula)
    Mat omega = currentPose.rowRange(0,3);                                                                                                  // Make a vector called 'omega', where we put out values of omega

    double normOmega = sqrt(pow(omega.at<double>(0, 0) , 2) + pow(omega.at<double>(1, 0) , 2) + pow(omega.at<double>(2, 0) , 2));           // Calulate the norm of the vector 'omega'

    Mat rotationMatrix(3, 3, CV_64F);                                                                                                       // Define the roation matrix in our transformation
    Mat kCrossProduct(3, 3, CV_64F);                                                                                                        // Define the k cross product matrix (we use the Rodrigues formula to find the rotation matrix)
    
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
    Mat translationMatrix = currentPose.rowRange(3,6);                                                                                      // Define the translation vector

    hconcat(rotationMatrix, translationMatrix, transformationMatrix);                                                                       // Concatenate the rotation matrix and the translation vector to the transformation matrix

    return transformationMatrix;
}

Mat projectPoints(Mat& calibrationMatrix, Mat& transformationMatrix, Mat& inputWorldPoints, Mat& distortionArray)           // Function to project world coordinates onto image plane (with lens distortion)
{
    Mat outputCameraPoints(2, 1, CV_64F);
    Mat worldPointsHomo = Mat::zeros(4, 1, CV_64F);                                                                         // Define homogeneous vector for world points (empty)
    vconcat(inputWorldPoints, Mat::eye(1, 1, CV_64F), worldPointsHomo);                                                     // Create homogeneous coordinates vector for world points

    Mat cameraPointsHomo = Mat::zeros(4, 1, CV_64F);                                                                        // Define the camera points vector in homogeneous coordinates
    cameraPointsHomo = transformationMatrix * worldPointsHomo;                                                              // Calculate the camera points in homogeneous coordinates

    Mat normalizedCoordinates(2, 1 ,CV_64F);                                                                                // Define the normalized coordinates vector
    normalizedCoordinates.at<double>(0, 0) = cameraPointsHomo.at<double>(0, 0)/cameraPointsHomo.at<double>(2, 0);           // Calculate first element of the normalized coordate
    normalizedCoordinates.at<double>(1, 0) = cameraPointsHomo.at<double>(1, 0)/cameraPointsHomo.at<double>(2, 0);           // Calculate second element of the normalized coordate

    double r = sqrt(pow(normalizedCoordinates.at<double>(0, 0), 2) + pow(normalizedCoordinates.at<double>(1, 0), 2));       // Compute the radial component of normalized coordinates
    double radialDistortionConstant = 1;                                                                                    // Define radial distortion constant

    for (int distRow = 0; distRow < distortionArray.rows; distRow++)                                                        // For loop to go through all radial distortion constants
    {
        radialDistortionConstant = radialDistortionConstant + distortionArray.at<double>(distRow, 0)*pow(r, 2*(distRow+1)); // Computing radial distortion constant (to then multiply by normalized coordinates)
    }

    normalizedCoordinates = radialDistortionConstant * normalizedCoordinates;                                               // Compute the distorted normalized coordinates
    
    Mat outputCameraPointsHomo(3, 1, CV_64F);                                                                               // Define output camera points vector in homogeneous coordinates
    Mat normalizedCoordinatesHomo(3, 1, CV_64F);                                                                            // Define normalized camera coordinates vector in homogeneous coordinates
    vconcat(normalizedCoordinates, Mat::eye(1, 1, CV_64F), normalizedCoordinatesHomo);                                      // Make normalized camera coordinates vector in homogeneous coordinates

    outputCameraPointsHomo = calibrationMatrix * normalizedCoordinatesHomo;                                                 // Calculate output camera points in homogenous coordinates
    
    int cameraCoordinate0 = outputCameraPointsHomo.at<double>(0, 0) / outputCameraPointsHomo.at<double>(2, 0);              // Normalize output camera coordinates and convert to integer (for pixel coordinates)
    int cameraCoordinate1 = outputCameraPointsHomo.at<double>(1, 0) / outputCameraPointsHomo.at<double>(2, 0);              // Normalize output camera coordinates and convert to integer (for pixel coordinates)

    outputCameraPoints.at<double>(0, 0) = cameraCoordinate0;                                                                // Input integer values to output camera coordinates vector
    outputCameraPoints.at<double>(1, 0) = cameraCoordinate1;                                                                // Input integer values to output camera coordinates vector

    return outputCameraPoints;
}

void drawCube(Mat& image, Mat& cubeOrigin, double& length, Mat& calibrationMatrix, Mat& transformationMatrix, Mat& distortionArray)                 // Function to draw a cube on top of the current frame
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

Mat getLensDistortionValues(string& filename, bool& lensDistortion)     // Function to get the parameters of lens distortion without having to open and close the file many times
{
    Mat distortionArray;
    if (lensDistortion){
        ifstream distortionFile;                                        // Open an inward stream of data called distortionFile
        distortionFile.open (datapath + filename);                      // Open the file with the distortion data
        double tempValue;                                               // Declare temporary variable

        while (!distortionFile.eof())                                   // While we have not reached the end of the file
        {
            distortionFile >> tempValue;                                // Input the value from the text file into the tempValue
            if (distortionFile.eof()) break;                            // If we have reached the end of the file, break the loop (otherwise we get two times the same variable at the end)
            distortionArray.push_back(tempValue);                       // Append the new value to the array
        }
        distortionFile.close();                                         // Close the file
    }
    else                                                                // If there is no camera distortion
    {
        distortionArray = Mat::zeros(1, 1, CV_64F);                     // Set the distortion array to zero, we can then still propagate the distortion array throughout the code, but the code will still output an undistorted result
    }

    return distortionArray;
}

void getPose(ifstream& poseFile, Mat currentPose)                       // Function to get the current pose of the camera from the input file
{
    for (int r = 0; r < 6; r++)                                         // For loop to go through the elements of our translation matrix
    {
        poseFile >> currentPose.at<double>(r, 0);                       // Get the latest value from our text file 'poseFile'
    }
}

int main(int argc, char** argv)
{
    // Initlialization of main:

    string calibrationMatrixFile = "K.txt";                             // Define the name of the calibration matrix file
    Mat calibrationMatrix = getCalibrationMatrix(calibrationMatrixFile);// Call function to read the calibration matrix file and make the calibration matrix

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
    Mat distortionArray = getLensDistortionValues(distortionValuesFile, lensDistortion);

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