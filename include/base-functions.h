Mat getCalibrationMatrix(string& filename, string& datapath)    // Function to read calibration matrix from txt file
{
    Mat calibrationMatrix(3, 3, CV_64F);
    ifstream kMatrixFile;                                       // Make an inward stream of data called 'file'
    kMatrixFile.open (datapath + filename);                     // Open the file in question (we sum up the datapath and the filename)
    double value;                                               // Make variable 'value', where we put the values that we get from file                                  
    
    for (int r = 0; r < 3; r++)                                 // Make for loop to go through the rows of our calibration matrix
    {
        for (int c = 0; c < 3; c++)                             // Make for loop to go through the columns of our calibration matrix
        {
            kMatrixFile >> value;                               // Get the value from our file
            calibrationMatrix.at<double>(r, c) = value;         // Place each new value from the file into the calibration matrix
        }
    }
    kMatrixFile.close();                                        // Close the file (we're not savages)

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

Mat getLensDistortionValues(string& filename, bool& lensDistortion, string& datapath)     // Function to get the parameters of lens distortion without having to open and close the file many times
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

void getPose(ifstream& poseFile, Mat currentPose)                       // Function to get the current pose of the camera from an input file
{
    for (int r = 0; r < 6; r++)                                         // For loop to go through the elements of our translation matrix
    {
        poseFile >> currentPose.at<double>(r, 0);                       // Get the latest value from our text file 'poseFile'
    }
}