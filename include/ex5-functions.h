void readStereoImage(VideoCapture& leftCap, VideoCapture& rightCap, double rescaleFactor, Mat stereoImage[2]) // Function to read two input images and assemble then into array
{
    Mat leftImage, rightImage;                          // Define left and right image matrices

    leftCap.read(leftImage);                            // Read left image
    rightCap.read(rightImage);                          // Read left image

    resize(leftImage, leftImage, Size(), rescaleFactor, rescaleFactor);                   // Resize image to the specified rescale factor
    resize(rightImage, rightImage, Size(), rescaleFactor, rescaleFactor);                   // Resize image to the specified rescale factor

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

Mat getDisparity(Mat *stereoImage, int& patchRadius, int& minDisparity, int& maxDisparity)
{
    // left_img and right_img are both H x W and you should return a H x W
    // matrix containing the disparity d for each pixel of left_img. Set
    // disp_img to 0 for pixels where the SSD and/or d is not defined, and for d
    // estimates rejected in Part 2. patch_radius specifies the SSD patch and
    // each valid d should satisfy min_disp <= d <= max_disp.

    Mat disparityMap;

    double patchSize = pow(patchRadius*2+1, 2);

    // Brute force method of finding the disparity
    for (int imageHeight = 0; imageHeight < stereoImage[0].rows - patchSize; imageHeight++)
    {
        for (int leftImageWidth = 0 ; leftImageWidth < stereoImage[0].cols - patchSize; leftImageWidth++)
        {
            Mat leftPatch = stereoImage[0](Range(imageHeight, imageHeight+patchSize), Range(leftImageWidth, leftImageWidth+patchSize));
            
            for (int rightImageWidth = 0 ; rightImageWidth < stereoImage[0].cols - patchSize; rightImageWidth++)
            {
                Mat rightPatch = stereoImage[1](Range(imageHeight, imageHeight+patchSize), Range(rightImageWidth, rightImageWidth+patchSize));
                //Mat disparityCandidates = 
            }
        }
    }

    return disparityMap;
}

void disparityToPointCloud(Mat& disparityMap, Matx33d calibrationMatrix, double& baseline, Mat *stereoImage, Vec3d points[], double intensities)
{
    // points should be 3xN and intensities 1xN, where N is the amount of pixels
    // which have a valid disparity. I.e., only return points and intensities
    // for pixels of left_img which have a valid disparity estimate! The i-th
    // intensity should correspond to the i-th point.
}