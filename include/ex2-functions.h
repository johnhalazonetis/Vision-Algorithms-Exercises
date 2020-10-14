void quat(Vector4d q, MatrixXd& Q, MatrixXd& Q_bar)
{
    Q << q[0], -q[1], -q[2], -q[3],
         q[1],  q[0], -q[3],  q[2],
         q[2],  q[3],  q[0], -q[1],
         q[3], -q[2],  q[1],  q[0];

    Q_bar << q[0], -q[1], -q[3], -q[3],
             q[1],  q[0],  q[2], -q[2],
             q[2], -q[3],  q[0],  q[1],
             q[3],  q[2], -q[1],  q[0];
}

Vector4d rotMatrix2Quat(MatrixXd M)          // Return quaternion from rotation matrix
{
     double i = 0, j = 1, k = 2;
     
     if (M(1, 1) > M(0, 0))
     {    
          i = 1; j = 2; k = 0;
     }

     if (M(2, 2) > M(i, i))
     {
          i = 2; j = 0; k = 1;
     }

     double t = M(i,i) -(M(j,j) + M(k,k)) + 1;
     Vector4d q = Vector4d::Zero();
     q(0) = M(k,j) - M(j,k);
     q(i+1) = t;
     q(j+1) = M(i,j) + M(j,i);
     q(k+1) = M(k,i) + M(i,k);
     q = q * 0.5 / sqrt(t);

     return q;
}

Matrix2d quat2RotMatrix(Vector4d q)     // Transforms a quaternion q into a rotation matrix
{
     MatrixXd Q(4, 4), Q_bar(4, 4);
     quat(q, Q, Q_bar);

     MatrixXd R_tmp(4, 4);
     R_tmp = Q_bar.transpose() * Q;
     Matrix2d R;
     R = R_tmp.block<2,2>(2,2);

     return R;
}

MatrixXd loadWorldCoordinatePoints(string worldCoordinatesFileName, string& datapath, int& numberOfWorldCoordinates)    // Function to put all found world coordinates in text file into matrix
{
    MatrixXd worldCoordinates(3, numberOfWorldCoordinates);                     // Define world coordinates matrix
    double tempValue;
    ifstream worldCoordinatesFile;                                              // Make an inward stream of data called 'worldCoordinatesFile'
    worldCoordinatesFile.open (datapath + worldCoordinatesFileName);            // Open the file

    for (int colNumber = 0; colNumber < numberOfWorldCoordinates; colNumber++)  // For loop to go through all of the 3D points in the file
    {
        for (int rowNumber = 0; rowNumber < 3; rowNumber++)                     // For loop to go through the values of each point
        {
            worldCoordinatesFile >> tempValue;
            worldCoordinates(rowNumber, colNumber) = tempValue;                 // Input the value into the matrix
        }        
    }

    worldCoordinatesFile.close();                                               // Close the file
    return worldCoordinates;                                                    // Return the resulting matrix
}

MatrixXd getCameraCoordinates(ifstream& detectedCornersFile, int& numberOfWorldCoordinates)
{
    MatrixXd currentCameraCoordinates(2, numberOfWorldCoordinates);

    for (int colNumber = 0; colNumber < numberOfWorldCoordinates; colNumber++)
    {
        for (int rowNumber = 0; rowNumber < 2; rowNumber++)
        {
            detectedCornersFile >> currentCameraCoordinates(rowNumber, colNumber);
        }
    }

    return currentCameraCoordinates;
}

MatrixXd estimatePoseDLT(MatrixXd& currentDetectedPoints, MatrixXd& worldPointCoordinates, Matrix3d& calibrationMatrix) // Function to estimate pose using DLT algorithm
{
    MatrixXd currentPose(3, 4);                                                     // Define the current pose matrix
    int numberOfPoints = currentDetectedPoints.cols();                              // Find the number of points from the input matrix

    MatrixXd Q = MatrixXd::Zero(numberOfPoints*2, 12);                              // Create matrix Q

    for (int pointN = 0; pointN < numberOfPoints; pointN++)                         // Loop through all of the points that we have (to make the Q matrix)
    {
        Vector4d worldCoords; worldCoords << worldPointCoordinates.col(pointN), 1;  // Make homogeneous world coordinates
        Vector3d pixelCoords; pixelCoords << currentDetectedPoints.col(pointN), 1;  // Make homogeneous pixel coordinates
        Vector3d normalizedCoords = calibrationMatrix.inverse() * pixelCoords;      // Calculate normalized pixel coordinates using inverted K matrix
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

void reprojectPoints(MatrixXd& worldCoordinates, MatrixXd& currentPose, Matrix3d& calibrationMatrix, Mat& image)                    // Function to reproject points
{
    int numberOfPoints = worldCoordinates.cols();                                                                                   // Determine the number of points we need to reproject

    for (int pointN = 0; pointN < numberOfPoints; pointN++)                                                                         // For loop to loop through the points
    {
        Vector3d currentWorldCoordinate = worldCoordinates.col(pointN);                                                             // Define a 3D point taken from our world point cloud
        Vector2i currentReprojectedPoint = projectPoints(calibrationMatrix, currentPose, currentWorldCoordinate, VectorXd::Zero(1));// Project the point onto the image using projectPoints function

        Point cvCurrentReprojectedPoint = eigenVec2cvPoint(currentReprojectedPoint);                                                // Transform the Eigen::Vector2i into a cv::Point
        drawMarker(image, cvCurrentReprojectedPoint,  Scalar(0, 0, 255), MARKER_CROSS, 7, 1, 8);                                    // Draw a marker at the projected point location
    }    
}

void draw3DPointCloud(MatrixXd& pointCloud, viz::Viz3d& vizWindow)
{
    int numberOfPoints = pointCloud.cols();
    Point3d tempPoint;
    Matx44d tempTransform;
    MatrixXd homoTransform = MatrixXd::Identity(4, 4);

    for (int pointN = 0; pointN < numberOfPoints; pointN++)
    {
        homoTransform.topRightCorner<3, 1>() = pointCloud.col(pointN);
        eigen2cv(homoTransform, tempTransform);
        tempPoint = Point3d(pointCloud(0, pointN), pointCloud(1, pointN), pointCloud(2, pointN));
        viz::WSphere point(tempPoint, 1, 10, viz::Color::white());
        vizWindow.showWidget("tempPoint", point, tempTransform);
    }
}