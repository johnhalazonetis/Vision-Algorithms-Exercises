// TODO: Convert all matrices to std::arrays or cv::Mat

Mat derotatePatch(Mat& image, Vector2d& location, int& patchSize, double orientation)
{
    int patchRadius = patchSize/2;
    MatrixXd derotatedPatch = MatrixXd::Zero(patchSize, patchSize);
    
    Mat paddedImage;// = MatrixXd::Zero(image.rows()+2*patchRadius, image.cols()+2*patchRadius);
    paddedImage.block(patchRadius, patchRadius, image.rows(), image.cols()) = image;

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
            derotatedPatch(px, py) = paddedImage(ceil(yPatchRotated + patchRadius), ceil(xPatchRotated + patchRadius));
        }
    }

    return derotatedPatch;
}

VectorXd weightedHistC(VectorXd& vals, VectorXd& weights, VectorXd& edges)
{
    if (vals.rows() != weights.rows())
    {
        cout << "Vals and weights must be vectors of the same length!";
        return -1;
    }

    int numberOfEdges = edges.rows();
    VectorXd h = VectorXd::Zero(numberOfEdges);

    for (int n = 0; n < numberOfEdges; n++)
    {
        VectorXd ind = 
    }
    
    return h;
}