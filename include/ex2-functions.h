void loadWorldCoordinatePoints(string worldCoordinatesFileName, Vec3d *worldCoordinates)    // Function to put all found world coordinates in text file into matrix
{
    Vec3d tempValue;                                                                    // Define tempValue Vec3d
    ifstream worldCoordinatesFile;                                                      // Make an inward stream of data called 'worldCoordinatesFile'
    worldCoordinatesFile.open(worldCoordinatesFileName);                                // Open the file

    for (int pointNumber = 0; pointNumber < sizeof(worldCoordinates); pointNumber++)    // For loop to go through all of the 3D points in the file
    {
        for (int axis = 0; axis < 3; axis++)                                            // For loop to go through the values of each point
        {
            worldCoordinatesFile >> tempValue[axis];                                    // Input world coordinates into the current vector
        }
        worldCoordinates[pointNumber] = tempValue;                                      // Input vector into the worldCoordinates array
    }

    worldCoordinatesFile.close();                                                       // Close the file
}

void getCameraCoordinates(ifstream& detectedCornersFile, int& numberOfWorldCoordinates, Vec2i *currentCameraCoordinates)
{
    Vec2i tempValue;
    for (int pointNumber = 0; pointNumber < numberOfWorldCoordinates; pointNumber++)
    {
        for (int axis = 0; axis < 2; axis++)
        {
            detectedCornersFile >> tempValue[axis];
        }
        currentCameraCoordinates[pointNumber]
    }
}

Matx34d estimatePoseDLT(Vec2i *currentDetectedPoints, Vec3d *worldPointCoordinates, Matx33d& calibrationMatrix) // Function to estimate pose using DLT algorithm
{
    Matx34d currentPose;                                                            // Define the current pose matrix
    int numberOfPoints = sizeof(currentDetectedPoints);                             // Find the number of points from the input matrix

    Matx Q = Matx::zeros(numberOfPoints*2, 12);                                     // Create matrix Q

    for (int pointN = 0; pointN < numberOfPoints; pointN++)                         // Loop through all of the points that we have (to make the Q matrix)
    {
        Vec4d worldCoords; vconcat(worldPointCoordinates[pointN], 1, worldCoords);  // Make homogeneous world coordinates
        Vec3d pixelCoords; vconcat(currentDetectedPoints[pointN], 1, pixelCoords);  // Make homogeneous pixel coordinates
        Vec3d normalizedCoords = calibrationMatrix.inv() * pixelCoords;             // Calculate normalized pixel coordinates using inverted K matrix
        normalizedCoords = 1/normalizedCoords[2] * normalizedCoords;                // Ensure that the last value of the normalized coordinates is 1

        // Build matrix Q using the formula found in course
        Q.block<1,4>(2*pointN, 0) = worldCoords.transpose();
        Q.block<1,4>(2*pointN + 1, 4) = worldCoords.transpose();
        Q.block<1,4>(2*pointN, 8) = -normalizedCoords[0] * worldCoords.transpose();
        Q.block<1,4>(2*pointN + 1, 8) = -normalizedCoords[1] * worldCoords.transpose();
    }

    BDCSVD<MatrixXd> svd( Q, ComputeThinU | ComputeThinV );                         // Solve the SVD for Q
    MatrixXd Mcol = svd.matrixV().rightCols(1);                                     // Take the last column of V matrix (S is in descending order)

    Mcol.resize(4, 3);                                                              // Transform solution into 4x3 matrix
    currentPose = Mcol.transpose();                                                 // Take the transpose to get intermediate result

    if (currentPose(2, 3) < 0) currentPose = - currentPose;                         // If last element is negative multiply M by -1

    MatrixXd R = currentPose.block(0, 0, 3, 3);                                     // Get the rotation matrix from M
    BDCSVD<MatrixXd> svdR( R, ComputeThinU | ComputeThinV );                        // Compute SVD of R to get matrices U and V
    currentPose.block<3, 3>(0, 0) = svdR.matrixU() * svdR.matrixV().transpose();    // Compute accurate R* = U * V^T
    
    double alpha = (currentPose.block(0, 0, 3, 3).norm())/R.norm();                 // Compute scaling factor alpha = norm(R*)/norm(R)

    currentPose.block<3, 1>(0, 3) = alpha * currentPose.block(0, 3, 3, 1);          // The projection matrix can be calculated as: M* = [R* | alpha*t]

    return currentPose;
}

void reprojectPoints(Vec3d *worldCoordinates, Matx34d& currentPose, Matx33d& calibrationMatrix, Mat& image)                 // Function to reproject points
{
    int numberOfPoints = sizeof(worldCoordinates);                                                                          // Determine the number of points we need to reproject
    double distortion[1] = 0;
    for (int pointN = 0; pointN < numberOfPoints; pointN++)                                                                 // For loop to loop through the points
    {
        Vec3d currentWorldCoordinate = worldCoordinates[pointN];                                                            // Define a 3D point taken from our world point cloud
        Vec2i currentReprojectedPoint = projectPoints(calibrationMatrix, currentPose, currentWorldCoordinate, distortion);  // Project the point onto the image using projectPoints function

        Point cvCurrentReprojectedPoint = eigenVec2cvPoint(currentReprojectedPoint);                                        // Transform the Eigen::Vector2i into a cv::Point
        drawMarker(image, Point(currentReprojectedPoint),  Scalar(0, 0, 255), MARKER_CROSS, 7, 1, 8);                       // Draw a marker at the projected point location
    }    
}

void draw3DPointCloud(Vec3d *pointCloud, viz::Viz3d& vizWindow)
{
    int numberOfPoints = sizeof(pointCloud);

    for (int pointN = 0; pointN < numberOfPoints; pointN++)
    {
        viz::WSphere point(Point3d(pointCloud[pointN]), 1, 10, viz::Color::white());
        vizWindow.showWidget("tempPoint", point);
    }
}