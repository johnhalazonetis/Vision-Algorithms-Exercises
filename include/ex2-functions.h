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

MatrixXd estimatePoseDLT(MatrixXd& currentDetectedPoints, MatrixXd& worldPointCoordinates, Matrix3d& calibrationMatrix)
{
    MatrixXd currentPose(3, 4);                                                     // Define the current pose matrix
    int numberOfPoints = currentDetectedPoints.cols();                              // Find the number of points from the input matrix
    Matrix3d invCalibrationMatrix = calibrationMatrix.inverse();

    MatrixXd Q = MatrixXd::Zero(numberOfPoints*2, 12);                              // Create matrix Q

    for (int pointN = 0; pointN < numberOfPoints; pointN++)                         // Loop through all of the points that we have (to make the Q matrix)
    {
        Vector4d worldCoords; worldCoords << worldPointCoordinates.col(pointN), 1;
        Vector3d pixelCoords; pixelCoords << currentDetectedPoints.col(pointN), 1;
        Vector3d normalizedCoords = invCalibrationMatrix * pixelCoords;
        normalizedCoords = 1/normalizedCoords[2] * normalizedCoords;

        Q.block<1,4>(2*pointN, 0) = worldCoords.transpose();
        Q.block<1,4>(2*pointN + 1, 4) = worldCoords.transpose();
        Q.block<1,4>(2*pointN, 8) = -normalizedCoords[0] * worldCoords.transpose();
        Q.block<1,4>(2*pointN + 1, 8) = -normalizedCoords[1] * worldCoords.transpose();
    }

    cout << Q << endl << endl;
    

    // Find normalized coordinates: [x,y,1]^T = K^-1 [u, v, 1]^T

    // Make matrix Q (see statement)

    // compute the last column of V in SVD decomposition (check S as well according to statement)

    // Convert The result to a 3x4 matrix (transpose of 4x3 matrix)

    // Check that M_34 is positive, other wise multiply M by -1, then we have M = [R | t]

    // Find true rotation matrix R*
        // Calculate SVD of R (from M) R = U*S*V^T, then R* = U * V^T
    
    // Find the scaling factor alpha
        // alpha = norm(R*)/norm(R)

    // The projection matrix can be calculated as: M* = [R* | alpha*t]

    return currentPose;
}

void reprojectPoints(MatrixXd& worldCoordinates, MatrixXd& currentPose, Matrix3d& calibrationMatrix)
{

}