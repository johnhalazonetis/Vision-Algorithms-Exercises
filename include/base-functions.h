Matx33d getCalibrationMatrix(string filename)    // Function to read calibration matrix from txt file
{
    Matx33d calibrationMatrix;
    ifstream kMatrixFile;                           // Make an inward stream of data called 'kMatrixFile'
    kMatrixFile.open (filename);                    // Open the file
    
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

Matx34d makeTransforationMatrix(Vec6d& currentPose)    // Function to get the poses from a text file and make a transformation matrix
{
    Matx34d transformationMatrix(3, 4);
    
    // Get rotation values and make rotation matrix (Rodrigues formula)
    Vec3d omega = (currentPose[0], currentPose[1], currentPose[2]);     // Make a vector called 'omega', with first 3 values of omega

    double normOmega = norm(omega, NORM_INF, noArray());                // Calulate the norm of the vector 'omega'

    Matx33d rotationMatrix;                        // Define the roation matrix in our transformation
    Matx33d kCrossProduct;                         // Define the k cross product matrix (we use the Rodrigues formula to find the rotation matrix)
    
    // Manually input values of k cross product matrix
    kCrossProduct(0, 0) = 0;
    kCrossProduct(0, 1) = -omega[2]/normOmega;
    kCrossProduct(0, 2) = omega[1]/normOmega;
    kCrossProduct(1, 0) = omega[2]/normOmega;
    kCrossProduct(1, 1) = 0;
    kCrossProduct(1, 2) = -omega[0]/normOmega;
    kCrossProduct(2, 0) = -omega[1]/normOmega;
    kCrossProduct(2, 1) = omega[0]/normOmega;
    kCrossProduct(2, 2) = 0;

    rotationMatrix = Matx33d::eye() + (sin(normOmega)) * kCrossProduct + (1 - cos(normOmega)) * (kCrossProduct * kCrossProduct);  // Apply Rodrigues formula to calculate the rotation matrix

    // Making the translation matrix out of the next three values from the poses.txt file
    Vec3d translationMatrix(currentPose[3], currentPose[4], currentPose[5]);             // Define the translation vector
    hconcat(rotationMatrix, translationMatrix, transformationMatrix);
    
    return transformationMatrix;
}

Vec2i projectPoints(Matx33d& calibrationMatrix, Matx34d& transformationMatrix, Vec3d inputWorldPoints, double *distortionArray)  // Function to project world coordinates onto image plane (with lens distortion)
{
    Vec4d worldPointsHomo;                                                                                          // Define homogeneous vector for world points (empty)
    vconcat(inputWorldPoints, 1, worldPointsHomo);                                                                  // Create homogeneous coordinates vector for world points

    Vec3d cameraPointsHomo = transformationMatrix * worldPointsHomo;                                                // Define the camera points vector in homogeneous coordinates

    Vec2d normalizedCoordinates(cameraPointsHomo[0], cameraPointsHomo[1]);                                          // Define the normalized coordinates vector
    normalizedCoordinates = normalizedCoordinates/cameraPointsHomo[2];                   
    
    double radius = norm(normalizedCoordinates, NORM_INF, noArray());                                               // Compute the radial component of normalized coordinates
    double radialDistortionConstant = 1;                                                                            // Define radial distortion constant

    for (int distRow = 0; distRow < sizeof(distortionArray); distRow++)                                             // For loop to go through all radial distortion constants
    {
        radialDistortionConstant = radialDistortionConstant + distortionArray[distRow]*pow(radius, 2*(distRow+1));  // Computing radial distortion constant (to then multiply by normalized coordinates)
    }

    normalizedCoordinates = radialDistortionConstant * normalizedCoordinates;                                       // Compute the distorted normalized coordinates
    
    Vec3d outputCameraPointsHomo;                                                                                   // Define output camera points vector in homogeneous coordinates
    Vec3d normalizedCoordinatesHomo;                                                                                // Define normalized camera coordinates vector in homogeneous coordinates
    vconcat(normalizedCoordinates, 1, normalizedCoordinatesHomo);                                                   // Make normalized camera coordinates vector in homogeneous coordinates

    outputCameraPointsHomo = calibrationMatrix * normalizedCoordinatesHomo;                                         // Calculate output camera points in homogenous coordinates
    
    outputCameraPointsHomo = outputCameraPointsHomo/outputCameraPointsHomo[2];
    Vec2i outputCameraPoints((int)outputCameraPointsHomo[0], (int)outputCameraPointsHomo[1]);                       // Input integer values to output camera coordinates vector
    return outputCameraPoints;
}

void getLensDistortionValues(string filename, bool lensDistortion, int parameterNumber, double *distortionArray)     // Function to get lens distortion parameters
{
    if (lensDistortion)                             // If there is lens distortion
    {
        ifstream distortionFile;                    // Open an inward stream of data called distortionFile
        distortionFile.open (filename);             // Open the file with the distortion data

        for (int c = 0; c < parameterNumber; c++)   // Loop over the columns
        {
            distortionFile >> distortionArray[c];   // Input the values into the calibration matrix
        }
        distortionFile.close();                     // Close the file
    }
    else                                            // If there is no camera distortion
    {
        distortionArray[0] = 0;                     // Set the distortion array to zero, distortion array can still be propagate throughout the code, but code will still output an undistorted result
    }
}

Vec6d getPose(ifstream& poseFile)   // Function to get the current pose of the camera from an input file
{
    Vec6d currentPose;              // Define the currentPose vector

    for (int r = 0; r < 6; r++)     // For loop to go through the elements of our translation matrix
    {
        poseFile >> currentPose[r]; // Get the latest value from our text file 'poseFile'
    }

    return currentPose;
}

double computeSimilarity(string type, Mat& kernel, Mat& sample)  // Function to compute the zero mean 
{
    if (kernel.size() != sample.size())
    {
        cout << "computeSimilarity: Input kernel and sample size do not match" << endl;
        return EXIT_FAILURE;
    } 
    
    double similarity;
    if (type == "NCC")
    {
        Mat nominatorMat = kernel.mul(sample);
        similarity = sum(nominatorMat)[0]/(norm(kernel, NORM_L2) * norm(sample, NORM_L2));
    }
    if (type == "SSD")
    {
        similarity = norm(kernel, sample, NORM_L2SQR);
    }
    if (type == "SAD")
    {
        similarity = norm(kernel, sample, NORM_L1);
    }
    if (type == "ZNCC")
    {
        Scalar kernelMean = mean(kernel);
        Scalar sampleMean = mean(sample);
        Mat newKernel; subtract(kernel, kernelMean, newKernel);
        Mat newSample; subtract(sample, sampleMean, newSample);
        Mat nominatorMat = newKernel.mul(newSample);
        if (sum(nominatorMat)[0] == 0)
        {
            similarity = 0;
        }
        else
        {
            similarity = sum(nominatorMat)[0]/(norm(newKernel, NORM_L2) * norm(newSample, NORM_L2));
        }
    }
    if (type == "ZSSD")
    {
        Scalar kernelMean = mean(kernel);
        Scalar sampleMean = mean(sample);
        Mat newKernel; subtract(kernel, kernelMean, newKernel);
        Mat newSample; subtract(sample, sampleMean, newSample);
        similarity = norm(newKernel, newSample, NORM_L2SQR);
    }
    if (type == "ZSAD")
    {
        Scalar kernelMean = mean(kernel);
        Scalar sampleMean = mean(sample);
        Mat newKernel; subtract(kernel, kernelMean, newKernel);
        Mat newSample; subtract(sample, sampleMean, newSample);
        similarity = norm(newKernel, newSample, NORM_L1);
    }
    
    return similarity;
}

void drawCube(Mat& image, Vec3d& cubeOrigin, double& length, Matx33d& calibrationMatrix, Matx34d& transformationMatrix, double *distortionArray)   // Function to draw a cube on top of the current frame
{
    // Start by computing the position of the other 7 points in the world frame (keeping all of the same edge lengths)
    Vec3d xTranslation(1, 0, 0);                                                                                    // Define a translation on the x axis
    Vec3d yTranslation(0, 1, 0);                                                                                    // Define a translation on the y axis
    Vec3d zTranslation(0, 0, 1);                                                                                    // Define a translation on the z axis
    
    Vec3d cubeWorldCorners[8];                                                                                      // Define the world coordinates of the corners of the cube

    // Compute the world coordinates of the cube by using simple x, y and z translations in world frame
    cubeWorldCorners[0] = cubeOrigin;
    cubeWorldCorners[1] = cubeOrigin + length * xTranslation;
    cubeWorldCorners[2] = cubeOrigin + length * xTranslation + length * yTranslation;
    cubeWorldCorners[3] = cubeOrigin + length * yTranslation;
    cubeWorldCorners[4] = cubeOrigin - length * zTranslation;
    cubeWorldCorners[5] = cubeOrigin + length * xTranslation - length * zTranslation;
    cubeWorldCorners[6] = cubeOrigin + length * xTranslation + length * yTranslation - length * zTranslation;
    cubeWorldCorners[7] = cubeOrigin + length * yTranslation - length * zTranslation;

    Vec2i outputCameraCorners[8];                                                                                                   // Define output variable
    Vec3d tempInputWorldCoords;                                                                                                     // Define temporary input variable
    Vec2i tempOutputCameraCorners;

    for (int cornerNumber = 0; cornerNumber < 8; cornerNumber++)                                                                    // For loop to go over all of the corner
    {
        tempInputWorldCoords = cubeWorldCorners[cornerNumber];                                                                      // Put the current input world coordinates in a temporary input variable
        tempOutputCameraCorners = projectPoints(calibrationMatrix, transformationMatrix, tempInputWorldCoords, distortionArray);    // Output the camera coordinates in temporary variable
        //cout << tempOutputCameraCorners[cornerNumber] << endl;
        outputCameraCorners[cornerNumber] = tempOutputCameraCorners;
    }

    // Draw lines of the cube on the image
    line(image, Point(outputCameraCorners[0]), Point(outputCameraCorners[1]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[1]), Point(outputCameraCorners[2]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[2]), Point(outputCameraCorners[3]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[3]), Point(outputCameraCorners[0]), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(outputCameraCorners[4]), Point(outputCameraCorners[5]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[5]), Point(outputCameraCorners[6]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[6]), Point(outputCameraCorners[7]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[7]), Point(outputCameraCorners[4]), Scalar(0, 0, 255), 2, LINE_8);

    line(image, Point(outputCameraCorners[0]), Point(outputCameraCorners[4]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[1]), Point(outputCameraCorners[5]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[2]), Point(outputCameraCorners[6]), Scalar(0, 0, 255), 2, LINE_8);
    line(image, Point(outputCameraCorners[3]), Point(outputCameraCorners[7]), Scalar(0, 0, 255), 2, LINE_8);
}

void drawPointCloud(Mat& image, Vec2i *pointCloud)
{
    for (int pointN = 0; pointN < sizeof(pointCloud); pointN++)
    {
        circle(image, Point(pointCloud[pointN]), 2.5, Scalar(0, 255, 255), FILLED, LINE_8);
    }
}