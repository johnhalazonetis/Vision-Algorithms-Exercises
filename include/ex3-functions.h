void describeKeypoints(Mat& img, int& keypoints, int r, Vec3d *descriptors)
{
    /*
    Returns a (2r+1)^2xN matrix of image patch vectors based on image
    img and a 2xN matrix containing the keypoint coordinates.
    r is the patch "radius".
    */

   // TODO: start function
}

Mat harrisScores(Mat& img, int& patchSize, double& kappa)
{
    // Define kernels:
    Matx33d Sobelx(-1, 0, 1, -2, 0, 2, -1, 0, 1);
    Matx33d Sobely(-1, -2, -1, 0, 0, 0, 1, 2, 1);

    // Convolve the images with the given sobel kernels:
    Mat I_X; filter2D(img, I_X, -1, Sobelx);
    Mat I_Y; filter2D(img, I_Y, -1, Sobely);

    // Compute the M matrix with the given image patch size
    Mat I_X2; multiply(I_X, I_X, I_X2);
    Mat I_Y2; multiply(I_Y, I_Y, I_Y2);
    Mat I_XI_Y; multiply(I_X, I_Y, I_XI_Y);

    // Define a matrix of ones (for patch summing)
    Mat ones = 1/(pow(patchSize, 2)) * Mat::ones(patchSize, patchSize, CV_64F);

    // Sum the values of the patch using the ones matrix
    Mat I_X2Patch; filter2D(I_X2, I_X2Patch, -1, ones);
    Mat I_Y2Patch; filter2D(I_Y2, I_Y2Patch, -1, ones);
    Mat I_XI_YPatch; filter2D(I_XI_Y, I_XI_YPatch, -1, ones);
    
    // Add a border of (patchSize-1)/2 around each patch image
    int borderWidth = (patchSize-1)/2;
    copyMakeBorder(I_X2Patch, I_X2Patch, borderWidth, borderWidth, borderWidth, borderWidth, BORDER_CONSTANT);
    copyMakeBorder(I_Y2Patch, I_Y2Patch, borderWidth, borderWidth, borderWidth, borderWidth, BORDER_CONSTANT);
    copyMakeBorder(I_XI_YPatch, I_XI_YPatch, borderWidth, borderWidth, borderWidth, borderWidth, BORDER_CONSTANT);

    Mat tempDet1; multiply(I_X2Patch, I_Y2Patch, tempDet1);
    Mat tempDet2; multiply(I_XI_YPatch, I_XI_YPatch, tempDet2);
    Mat tempTrace = I_X2Patch - I_Y2Patch; multiply(tempTrace, tempTrace, tempTrace);

    // Compute the harris score for each pixel (from the M matrix)
    Mat scores = tempDet1 - tempDet2 - kappa*tempTrace;

    return scores;
}

void matchDescriptors(Vec3d *query_descriptors, Vec3d *databaseDescriptors, double lambda, double *matches)
{
    /*
    Returns a 1xQ matrix where the i-th coefficient is the index of the
    database descriptor which matches to the i-th query descriptor.
    The descriptor vectors are MxQ and MxD where M is the descriptor
    dimension and Q and D the amount of query and database descriptors
    respectively. matches(i) will be zero if there is no database descriptor
    with an SSD < lambda * min(SSD). No two non-zero elements of matches will
    be equal.
    */

   // TODO: start function

}

void plotMatches(double *matches, double *queryKeypoints, double *databaseKeypoints)
{

}

void selectKeypoints(Mat& scores, int numberOfKeypoints, int r, Vec2d *keyPoints)
{
    /*
    Selects the num best scores as keypoints and performs non-maximum 
    supression of a (2r + 1)*(2r + 1) box around the current maximum.
    */

    // TODO: start function

}

Mat shiTomasi(Mat& img, int& patchSize)
{
    // Define kernels:
    Matx33d Sobelx(-1, 0, 1, -2, 0, 2, -1, 0, 1);
    Matx33d Sobely(-1, -2, -1, 0, 0, 0, 1, 2, 1);

    // Convolve the images with the given sobel kernels:
    Mat I_X; filter2D(img, I_X, -1, Sobelx);
    Mat I_Y; filter2D(img, I_Y, -1, Sobely);

    // Compute the M matrix with the given image patch size
    Mat I_X2; multiply(I_X, I_X, I_X2);
    Mat I_Y2; multiply(I_Y, I_Y, I_Y2);
    Mat I_XI_Y; multiply(I_X, I_Y, I_XI_Y);

    // Define a matrix of ones (for patch summing)
    Mat boxFilter = Mat::ones(patchSize, patchSize, CV_64F);

    // Sum the values of the patch using the ones matrix
    Mat I_X2Patch; filter2D(I_X2, I_X2Patch, -1, boxFilter);
    Mat I_Y2Patch; filter2D(I_Y2, I_Y2Patch, -1, boxFilter);
    Mat I_XI_YPatch; filter2D(I_XI_Y, I_XI_YPatch, -1, boxFilter);
    
    // Add a border of (patchSize-1)/2 around each patch image
    int borderWidth = (patchSize-1)/2;
    copyMakeBorder(I_X2Patch, I_X2Patch, borderWidth, borderWidth, borderWidth, borderWidth, BORDER_CONSTANT);
    copyMakeBorder(I_Y2Patch, I_Y2Patch, borderWidth, borderWidth, borderWidth, borderWidth, BORDER_CONSTANT);
    copyMakeBorder(I_XI_YPatch, I_XI_YPatch, borderWidth, borderWidth, borderWidth, borderWidth, BORDER_CONSTANT);

    // Compute the trace and the determinant (we use them to find the eigenvalues)
    Mat tempDet1; multiply(I_X2Patch, I_Y2Patch, tempDet1);
    Mat tempDet2; multiply(I_XI_YPatch, I_XI_YPatch, tempDet2);
    Mat tempDet = tempDet1 - tempDet2;
    Mat tempTrace = I_X2Patch - I_Y2Patch; multiply(tempTrace, tempTrace, tempTrace);
    Mat tempTrace2; multiply(tempTrace, tempTrace, tempTrace2);
    Mat tempCalc = tempTrace2 - 4*tempDet;
    tempCalc.convertTo(tempCalc, CV_64F);
    tempTrace.convertTo(tempTrace, CV_64F);
    sqrt(tempCalc, tempCalc);

    // Calculate the two eigenvalues of the matrix
    Mat lambda1 = 0.5*(tempTrace + tempCalc);
    Mat lambda2 = 0.5*(tempTrace - tempCalc);

    // Compute the harris score for each pixel (from the M matrix)
    Mat scores = min(lambda1, lambda2);

    return scores;
}