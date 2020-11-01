void readStereoImage(VideoCapture& leftCap, VideoCapture& rightCap, double rescaleFactor, Mat stereoImage[2]) // Function to read two input images and assemble then into array
{
    Mat leftImage, rightImage;                          // Define left and right image matrices

    leftCap.read(leftImage);                            // Read left image
    rightCap.read(rightImage);                          // Read left image

    resize(leftImage, leftImage, Size(), rescaleFactor, rescaleFactor, INTER_LINEAR_EXACT);                   // Resize image to the specified rescale factor
    resize(rightImage, rightImage, Size(), rescaleFactor, rescaleFactor, INTER_LINEAR_EXACT);                   // Resize image to the specified rescale factor

    if (!leftImage.data)                                // If statement in case the file cannot be opened or does not exist
    {
        printf("No image data from left image \n");
    }
    if (!rightImage.data)                               // If statement in case the file cannot be opened or does not exist
    {
        printf("No image data from right image \n");
    }

    stereoImage[0] = leftImage;                         // Put left image in first position of array
    stereoImage[1] = rightImage;                        // Put right image in second position of array
}

void showStereoImage(Mat *stereoImage, string option)   // Function to show the stereo image output
{
    if (option == "ONLY_LEFT")
    {
        imshow("Left Camera", stereoImage[0]);
    }
    if (option == "ONLY_RIGHT")
    {
        imshow("Left Camera", stereoImage[0]);
    }
    if (option == "STEREO")
    {
        Mat stereoFrame;
        hconcat(stereoImage[0], stereoImage[1], stereoFrame);
        imshow("Stereo Image", stereoFrame);
    }
}

int getDisparityValue(Mat& kernel, Mat& searchPatch, string normType)
{
    int disparity = searchPatch.cols - kernel.cols;
    if (kernel.rows != searchPatch.rows)
    {
        cout << "getDisparityValue: The number of rows of the kernel and search patch do not match!" << endl;
        return EXIT_FAILURE;
    }

    int disparityCounter = 0;
    for (int searchCol = 0; searchCol < searchPatch.cols - kernel.cols; searchCol++)
    {
        Mat samplePatch(searchPatch, Rect(0, searchCol, kernel.rows, kernel.cols));
        double similarityMeasure = computeSimilarity(normType, kernel, samplePatch);
        cout << "Iteration: " << searchCol << ", similarityMeasure = " << similarityMeasure << endl;
        
        if (similarityMeasure == 1 && (normType == "NCC" || normType == "ZNCC"))
        {
            disparity = disparityCounter;
            break;
        }
        if (similarityMeasure == 0 && (normType == "SSD" || normType == "ZSSD" || normType == "SAD" || normType == "ZSAD"))
        {
            disparity = disparityCounter;
            break;
        }
        
        disparityCounter++;
    }
    
    return disparity;
}

Mat getDisparityMap(Mat *stereoImage, int patchRadius, int minDisparity, int maxDisparity, string normType)
{
    int imageWidth = stereoImage[0].cols;
    int imageHeight = stereoImage[0].rows;

    Mat disparityMap(Size(imageWidth, imageHeight), CV_8U);
    int disparityRange = maxDisparity - minDisparity;

    int patchSize = pow(patchRadius*2+1, 2);
    int halfPatchSize = (patchSize -1)/2;

    // Loop through the pixels and possible patches and call getDisparityValue to compute disparity
    for (int currentRow = 0; currentRow < imageHeight; currentRow++)
    {
        for (int currentCol = 0; currentCol < imageWidth - patchSize; currentCol++)
        {
            Mat kernel(stereoImage[0], Rect(currentCol, currentRow, patchSize, patchSize));
            int searchPatchWidth = min(maxDisparity, imageWidth - (currentCol + patchSize));
            Mat searchPatch(stereoImage[1], Rect(currentCol + minDisparity, currentRow, patchSize + searchPatchWidth - minDisparity, patchSize));
            imshow("Kernel", kernel); imshow("searchPatch", searchPatch);
            waitKey(0);
            cout << "Total size: " << searchPatch.size() << "Number of columns: " << searchPatch.cols << endl;
            int disparity = getDisparityValue(kernel, searchPatch, normType);
            disparityMap.at<int>(currentRow + halfPatchSize, currentCol + halfPatchSize) = disparity;
        }
    }
    
    return disparityMap;
}

Mat getDisparityAdvanced(Mat *stereoImage, int patchRadius, int minDisparity, int maxDisparity)
{
    // left_img and right_img are both H x W and you should return a H x W
    // matrix containing the disparity d for each pixel of left_img. Set
    // disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
    // estimates rejected in Part 2. patch_radius specifies the SSD patch and
    // each valid d should satisfy min_disp <= d <= max_disp.

    int imageWidth = stereoImage[0].cols;
    int imageHeight = stereoImage[0].rows;

    Mat disparityMap(Size(imageWidth, imageHeight), CV_16S);
    int disparityRange = maxDisparity - minDisparity;

    /*int patchSize = pow(patchRadius*2+1, 2);
    int halfPatchSize = (patchSize -1)/2;
    

    // Use the minDisparity and maxDisparity values to shorten the loop (there is no point looking outside of the values that will be rejected anyway)
    for (int leftImageHeight = 0; leftImageHeight < imageHeight - patchSize; leftImageHeight++)
    {
        for (int leftImageWidth = 2*patchSize; leftImageWidth < imageWidth - patchSize - maxDisparity; leftImageWidth++)
        {
            Mat leftPatch(stereoImage[0], Rect(leftImageWidth, leftImageHeight, patchSize, patchSize));
            double differences[45];

            for (int distance = 0; distance < disparityRange; distance++)
            {
                Mat rightPatch(stereoImage[1], Rect(leftImageWidth + minDisparity + distance, leftImageHeight, patchSize, patchSize));
                differences[distance] = norm(leftPatch, rightPatch, NORM_L2);
            }
            
            int disparity = 0; double bestDiff = differences[0];
            cout << bestDiff << endl;
            for (int dispLoop = 1; dispLoop < disparityRange; dispLoop++)
            {
                cout << differences[dispLoop] << endl;
                if (differences[dispLoop] < bestDiff)
                {
                    bestDiff = differences[dispLoop];
                    disparity = dispLoop;
                    cout << "found new disp" << endl;
                }
            }
            waitKey(0);
        }
    }*/

    Mat overlapDifference[disparityRange];

    for (int overlapDiff = 0; overlapDiff < disparityRange; overlapDiff++)
    {
        Mat leftPatch = stereoImage[0].colRange(overlapDiff + minDisparity, stereoImage[0].cols);
        Mat rightPatch = stereoImage[1].colRange(0, stereoImage[1].cols - overlapDiff - minDisparity);
        overlapDifference[overlapDiff] = abs(leftPatch - rightPatch);
    }
    
    imshow("First difference", overlapDifference[0]);
    imshow("Middle difference", overlapDifference[(disparityRange-1)/2]);
    imshow("Last difference", overlapDifference[disparityRange-1]);
    waitKey(0);
    

    return disparityMap;
}

void disparityToPointCloud(Mat& disparityMap, Matx33d calibrationMatrix, double& baseline, Mat *stereoImage, Vec3d points[], double intensities)
{
    // points should be 3xN and intensities 1xN, where N is the amount of pixels
    // which have a valid disparity. I.e., only return points and intensities
    // for pixels of left_img which have a valid disparity estimate! The i-th
    // intensity should correspond to the i-th point.
}