Matrix3d getCalibrationMatrix(string& filename, string& datapath)    // Function to read calibration matrix from txt file
{
    Matrix3d calibrationMatrix;
    ifstream kMatrixFile;                           // Make an inward stream of data called 'file'
    kMatrixFile.open (datapath + filename);         // Open the file in question (we sum up the datapath and the filename)
    
    for (int r = 0; r < 3; r++)                     // Loop over the rows
    {
        for (int c = 0; c < 3; c++)                 // Loop over the columns
        {
            kMatrixFile >> calibrationMatrix(r, c); // Input the values into the calibration matrix
        }
    }
    
    kMatrixFile.close();                            // Close the file (we're not savages)

    return calibrationMatrix;
}

MatrixXd makeTransforationMatrix(VectorXd& currentPose)    // Function to get the poses from a text file and make a transformation matrix
{
    MatrixXd transformationMatrix(3, 4);
    
    // Get rotation values and make rotation matrix (Rodrigues formula)
    Vector3d omega = currentPose.head(3);           // Make a vector called 'omega', with first 3 values of omega

    double normOmega = omega.norm();                // Calulate the norm of the vector 'omega'

    Matrix3d rotationMatrix;                        // Define the roation matrix in our transformation
    Matrix3d kCrossProduct;                         // Define the k cross product matrix (we use the Rodrigues formula to find the rotation matrix)
    
    // Manually input values of k cross product matrix
    kCrossProduct(0, 0) = 0;
    kCrossProduct(0, 1) = -omega(2, 0)/normOmega;
    kCrossProduct(0, 2) = omega(1, 0)/normOmega;
    kCrossProduct(1, 0) = omega(2, 0)/normOmega;
    kCrossProduct(1, 1) = 0;
    kCrossProduct(1, 2) = -omega(0, 0)/normOmega;
    kCrossProduct(2, 0) = -omega(1, 0)/normOmega;
    kCrossProduct(2, 1) = omega(0, 0)/normOmega;
    kCrossProduct(2, 2) = 0;

    rotationMatrix = MatrixXd::Identity(3, 3) + (sin(normOmega)) * kCrossProduct + (1 - cos(normOmega)) * (kCrossProduct * kCrossProduct);  // Apply Rodrigues formula to calculate the rotation matrix

    // Making the translation matrix out of the next three values from the poses.txt file
    Vector3d translationMatrix = currentPose.tail<3>();             // Define the translation vector

    transformationMatrix << rotationMatrix, translationMatrix;          // Concatenate the rotation matrix and the translation vector to the transformation matrix
    
    return transformationMatrix;
}

Vector2i projectPoints(Matrix3d calibrationMatrix, MatrixXd transformationMatrix, Vector3d inputWorldPoints, VectorXd distortionArray)  // Function to project world coordinates onto image plane (with lens distortion)
{
    Vector2i outputCameraPoints;
    VectorXd worldPointsHomo(4, 1);                                                                                       // Define homogeneous vector for world points (empty)
    worldPointsHomo << inputWorldPoints, 1;                                                                         // Create homogeneous coordinates vector for world points

    Vector3d cameraPointsHomo;                                                                                      // Define the camera points vector in homogeneous coordinates
    cameraPointsHomo = transformationMatrix * worldPointsHomo;                                                      // Calculate the camera points in homogeneous coordinates

    Vector2d normalizedCoordinates;                                                                                 // Define the normalized coordinates vector
    normalizedCoordinates[0] = cameraPointsHomo[0]/cameraPointsHomo[2];                                             // Calculate first element of the normalized coordate
    normalizedCoordinates[1] = cameraPointsHomo[1]/cameraPointsHomo[2];                                             // Calculate second element of the normalized coordate

    double radius = normalizedCoordinates.norm();                                                                   // Compute the radial component of normalized coordinates
    double radialDistortionConstant = 1;                                                                            // Define radial distortion constant

    for (int distRow = 0; distRow < distortionArray.rows(); distRow++)                                              // For loop to go through all radial distortion constants
    {
        radialDistortionConstant = radialDistortionConstant + distortionArray[distRow]*pow(radius, 2*(distRow+1));  // Computing radial distortion constant (to then multiply by normalized coordinates)
    }

    normalizedCoordinates = radialDistortionConstant * normalizedCoordinates;                                       // Compute the distorted normalized coordinates
    
    Vector3d outputCameraPointsHomo;                                                                                // Define output camera points vector in homogeneous coordinates
    Vector3d normalizedCoordinatesHomo;                                                                             // Define normalized camera coordinates vector in homogeneous coordinates
    normalizedCoordinatesHomo << normalizedCoordinates, 1;                                                          // Make normalized camera coordinates vector in homogeneous coordinates

    outputCameraPointsHomo = calibrationMatrix * normalizedCoordinatesHomo;                                         // Calculate output camera points in homogenous coordinates
    
    int cameraCoordinate0 = outputCameraPointsHomo[0] / outputCameraPointsHomo[2];                                  // Normalize output camera coordinates and convert to integer (for pixel coordinates)
    int cameraCoordinate1 = outputCameraPointsHomo[1] / outputCameraPointsHomo[2];                                  // Normalize output camera coordinates and convert to integer (for pixel coordinates)

    outputCameraPoints << cameraCoordinate0, cameraCoordinate1;                                                     // Input integer values to output camera coordinates vector

    return outputCameraPoints;
}

VectorXd getLensDistortionValues(string& filename, bool& lensDistortion, string& datapath)     // Function to get the parameters of lens distortion without having to open and close the file many times
{
    VectorXd distortionArray(2, 1);                             // Initialize the distortion array of values
    
    if (lensDistortion)                                         // If there is lens distortion
    {
        ifstream distortionFile;                                // Open an inward stream of data called distortionFile
        distortionFile.open (datapath + filename);              // Open the file with the distortion data

        for (int c = 0; c < 2; c++)                 // Loop over the columns
        {
            distortionFile >> distortionArray[c]; // Input the values into the calibration matrix
        }
        distortionFile.close();                                 // Close the file
    }
    else                                                        // If there is no camera distortion
    {
        distortionArray = VectorXd::Zero(1, 1);                 // Set the distortion array to zero, we can then still propagate the distortion array throughout the code, but the code will still output an undistorted result
    }

    return distortionArray;
}

VectorXd getPose(ifstream& poseFile)  // Function to get the current pose of the camera from an input file
{
    VectorXd currentPose(6, 1);

    for (int r = 0; r < 6; r++)                         // For loop to go through the elements of our translation matrix
    {
        poseFile >> currentPose[r];                     // Get the latest value from our text file 'poseFile'
    }

    return currentPose;
}

Mat eigenMat2cvMat(MatrixXd& inputMatrix)               // Make a cv::Mat from an Eigen::MatrixXd
{
    Mat outputMatrix(inputMatrix.rows(), inputMatrix.cols(), CV_64F);
    double tempValue;

    for (int rowNumber = 0; rowNumber < inputMatrix.rows(); rowNumber++)
    {
        for (int colNumber = 0; colNumber < inputMatrix.cols(); colNumber++)
        {
            tempValue = inputMatrix(rowNumber, colNumber);
            outputMatrix.at<double>(rowNumber, colNumber) = tempValue;
        }
    }

    return outputMatrix;
}

Mat eigenMatInt2cvMat(MatrixXi& inputMatrix)            // Make a cv::Mat from an Eigen::MatrixXd
{
    Mat outputMatrix(inputMatrix.rows(), inputMatrix.cols(), CV_64F);
    int tempValue;

    for (int rowNumber = 0; rowNumber < inputMatrix.rows(); rowNumber++)
    {
        for (int colNumber = 0; colNumber < inputMatrix.cols(); colNumber++)
        {
            tempValue = inputMatrix(rowNumber, colNumber);
            outputMatrix.at<double>(rowNumber, colNumber) = tempValue;
        }
    }

    return outputMatrix;
}

Point eigenVec2cvPoint(Vector2i& inputVector)           // Make a cv::Point from an Eigen::Vector2i
{
    Point outputPoint;
    int tempValue1, tempValue2;

    tempValue1 = inputVector[0];
    tempValue1 = inputVector[1];
    outputPoint = Point(tempValue1, tempValue2);

    return outputPoint;
}

MatrixXd cvMat2eigenMat(Mat& inputMatrix)               // Make an Eigen::MatrixXd from a cv::Mat
{
    MatrixXd outputMatrix(inputMatrix.rows, inputMatrix.cols);
    double tempValue;

    for (int rowNumber = 0; rowNumber < inputMatrix.rows; rowNumber++)
    {
        for (int colNumber = 0; colNumber < inputMatrix.cols; colNumber++)
        {
            tempValue = inputMatrix.at<double>(rowNumber, colNumber);
            outputMatrix(rowNumber, colNumber) = tempValue;
        }
    }
    
    return outputMatrix;
}

VectorXd cvMat2eigenVec(Mat& inputVector)               // Make an Eigen::VectorXd from a cv::Mat
{
    VectorXd outputVector(inputVector.rows, 1);
    double tempValue;

    for (int rowNumber = 0; rowNumber < inputVector.rows; rowNumber++)
    {
        tempValue = inputVector.at<double>(rowNumber, 0);
        outputVector[rowNumber] = tempValue;
    }

    return outputVector;
}

Vector2i cvPoint2eigenVec(Point& inputPoint)            // Make an Eigen::Vector2i from a cv::Point
{
    Vector2i outputVector;
    int tempValue1, tempValue2;

    tempValue1 = inputPoint.x;
    tempValue2 = inputPoint.y;

    outputVector << tempValue1, tempValue2;

    return outputVector;
}

void drawCube(Mat& image, Vector3d& cubeOrigin, double& length, Matrix3d& calibrationMatrix, MatrixXd& transformationMatrix, VectorXd& distortionArray)   // Function to draw a cube on top of the current frame
{
    // Start by computing the position of the other 7 points in the world frame (keeping all of the same edge lengths)
    Vector3d xTranslation; xTranslation << 1, 0, 0;                                                                                 // Define a translation on the x axis
    Vector3d yTranslation; yTranslation << 0, 1, 0;                                                                                 // Define a translation on the y axis
    Vector3d zTranslation; zTranslation << 0, 0, 1;                                                                                 // Define a translation on the z axis
    
    MatrixXd cubeWorldCorners(3, 8);                                                                                                // Define the world coordinates of the corners of the cube

    // Compute the world coordinates of the cube by using simple x, y and z translations in world frame
    cubeWorldCorners.col(0) = cubeOrigin;
    cubeWorldCorners.col(1) = cubeOrigin + length * xTranslation;
    cubeWorldCorners.col(2) = cubeOrigin + length * xTranslation + length * yTranslation;
    cubeWorldCorners.col(3) = cubeOrigin + length * yTranslation;
    cubeWorldCorners.col(4) = cubeOrigin - length * zTranslation;
    cubeWorldCorners.col(5) = cubeOrigin + length * xTranslation - length * zTranslation;
    cubeWorldCorners.col(6) = cubeOrigin + length * xTranslation + length * yTranslation - length * zTranslation;
    cubeWorldCorners.col(7) = cubeOrigin + length * yTranslation - length * zTranslation;

    MatrixXi outputCameraCorners(2, 8);                                                                                             // Define output variable
    Vector3d tempInputWorldCoords;                                                                                                  // Define temporary input variable
    Vector2i tempOutputCameraCorners;

    for (int cornerNumber = 0; cornerNumber < 8; cornerNumber++)                                                                    // For loop to go over all of the corner
    {
        tempInputWorldCoords = cubeWorldCorners.col(cornerNumber);                                                                  // Put the current input world coordinates in a temporary input variable
        tempOutputCameraCorners = projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, distortionArray);    // Output the camera coordinates in temporary variable
        outputCameraCorners.col(cornerNumber) = tempOutputCameraCorners;
    }

    //cout << outputCameraCorners << endl;

    Mat cvOutputCameraCorners(2, 8, CV_64F);
    cvOutputCameraCorners = eigenMatInt2cvMat(outputCameraCorners);
    
    // Draw lines of the cube on the image
    line(image, Point(cvOutputCameraCorners.col(0)), Point(cvOutputCameraCorners.col(1)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(1)), Point(cvOutputCameraCorners.col(2)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(2)), Point(cvOutputCameraCorners.col(3)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(3)), Point(cvOutputCameraCorners.col(0)), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(cvOutputCameraCorners.col(4)), Point(cvOutputCameraCorners.col(5)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(5)), Point(cvOutputCameraCorners.col(6)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(6)), Point(cvOutputCameraCorners.col(7)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(7)), Point(cvOutputCameraCorners.col(4)), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(cvOutputCameraCorners.col(0)), Point(cvOutputCameraCorners.col(4)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(1)), Point(cvOutputCameraCorners.col(5)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(2)), Point(cvOutputCameraCorners.col(6)), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(cvOutputCameraCorners.col(3)), Point(cvOutputCameraCorners.col(7)), Scalar(0, 0, 255), 2, LINE_8);
}
