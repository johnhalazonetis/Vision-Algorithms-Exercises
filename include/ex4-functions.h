class stereoImagePair {
  public:
    Mat leftImage;   
    Mat rightImage;
    //TODO: Start working with classes to simplify data passing and general organization of code
};

Mat importImageGreyRescale(string datapath, double rescaleFactor)                           // Function to read the input image file
{
    Mat inputImage = imread(datapath, IMREAD_GRAYSCALE);                                    // Try to read the input image file
    if (!inputImage.data)                                                                   // If statement in case the file cannot be opened or does not exist
    {
        cout << "importImageGreyRescale: No image data in image: " << datapath << endl;     // Output that there was a problem inporting the image
        EXIT_FAILURE;                                                                       // End program
    }

    resize(inputImage, inputImage, Size(), rescaleFactor, rescaleFactor);                   // Resize image to the specified rescale factor

    return inputImage;
}

Mat derotatePatch(Mat& image, Vec2d& location, int& patchSize, double orientation)
{
    int patchRadius = patchSize/2;
    Mat derotatedPatch = Mat::zeros(patchSize, patchSize, CV_64F);
    
    Mat paddedImage; copyMakeBorder(paddedImage, paddedImage, patchRadius, patchRadius, patchRadius, patchRadius, BORDER_CONSTANT);

    // Compute the derotated patch
    for (int px = 0; px < patchSize; px++)
    {
        for (int py = 0; py < patchSize; py++)
        {
            int xOrigin = px - patchRadius;
            int yOrigin = py - patchRadius;

            // Rotate patch by angle "orientation"
            double xRotated = cos(M_PI * orientation/180) * xOrigin - sin(M_PI * orientation / 180) * yOrigin;
            double yRotated = sin(M_PI * orientation/180) * xOrigin + cos(M_PI * orientation / 180) * yOrigin;

            // Move Coordinates to patch
            double xPatchRotated = location(1) + xRotated;
            double yPatchRotated = location(0) - yRotated;

            // Sample image (using nearest neighbor sampling as opposed to more accuracte bilinear sampling)
            derotatedPatch.at<double>(px, py) = paddedImage.at<double>(ceil(yPatchRotated + patchRadius), ceil(xPatchRotated + patchRadius));
        }
    }

    return derotatedPatch;
}

Mat rotateImage(Mat inputImage, int angle)
{
    // get rotation matrix for rotating the image around its center in pixel coordinates
    Point2f center((inputImage.cols-1)/2.0, (inputImage.rows-1)/2.0);
    Mat rotationMatrix = getRotationMatrix2D(center, angle, 1.0);
    
    // Determine bounding rectangle, center not relevant
    Rect2f bbox = RotatedRect(Point2f(), inputImage.size(), angle).boundingRect2f();
    
    // Adjust transformation matrix
    rotationMatrix.at<double>(0,2) += bbox.width/2.0 - inputImage.cols/2.0;
    rotationMatrix.at<double>(1,2) += bbox.height/2.0 - inputImage.rows/2.0;

    Mat rotatedImage;
    warpAffine(inputImage, rotatedImage, rotationMatrix, bbox.size());

    return rotatedImage;
}

void extractKeypoints(vector<vector<Mat>> &dogImagePyramid, double constrastThreshold, int numberOfOctaves, vector<Mat> &kptLocations)
{
    for (int octaveIdx = 0; octaveIdx < numberOfOctaves-1; octaveIdx++)
    {
        vector<Mat> currentDog = dogImagePyramid[octaveIdx];

        // DoG_max = imdilate(DoG, true(3, 3, 3));
       
        // Equivalent to this is:
        // DoG_max = movmax(movmax(movmax(DoG, 3, 1), 3, 2), 3, 3);

        // is_kpt = (DoG == DoG_max) & (DoG >= contrast_threshold);

        // We do not consider the extrema at the boundaries of the DoGs.

        // is_kpt(:, :, 1) = false;
        // is_kpt(:, :, end) = false;
        // [x, y, s] = ind2sub(size(is_kpt), find(is_kpt));
        // kpt_locations{oct_idx} = horzcat(x, y, s);
    }
    
}

/*VectorXd weightedHistC(VectorXd& vals, VectorXd& weights, VectorXd& edges)
{
    if (vals.rows() != weights.rows())
    {
        cout << "Vals and weights must be vectors of the same length!";
        EXIT_FAILURE;
    }

    int numberOfEdges = edges.rows();
    VectorXd h = VectorXd::Zero(numberOfEdges);

    for (int n = 0; n < numberOfEdges; n++)
    {
        VectorXd ind = ;//...
        if (condition)
        {
            code
        }
        
    }
    
    return h;
}*/