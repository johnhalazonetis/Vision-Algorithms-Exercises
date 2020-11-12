void p3p(vector<Vec3d> worldPoints, vector<Vec3d>imageVectors, vector<Matx34d> poses)
{
    /*
    Copyright (c) 2011, Laurent Kneip, ETH Zurich
    All rights reserved.

    Input: worldPoints: 3x3 matrix with corresponding 3D world points (each column is a point)
           imageVectors: 3x3 matrix with UNITARY feature vectors (each column is a vector)
    Output: Poses: 3x16 matrix that will contain the solutions
            form: [ 3x1 position(solution1) 3x3 orientation(solution1) 3x1 position(solution2) 3x3 orientation(solution2) ... ]
            the obtained orientation matrices are defined as transforming points from the cam to the world frame
    */
    
    // Initialization of the solution matrix, extraction of world points and feature vectors
    Vec3d P1 = worldPoints[0];
    Vec3d P2 = worldPoints[1];
    Vec3d P3 = worldPoints[2];
    
    // Verification that world points are not colinear
    Vec3d vector1 = P2 - P1;
    Vec3d vector2 = P3 - P1;

    if (norm(vector1.cross(vector2), NORM_INF) == 0)
    {
        return;
    }

    // Creation of intermediate camera frame
    Vec3d f1 = imageVectors[0];
    Vec3d f2 = imageVectors[1];
    Vec3d f3 = imageVectors[2];

    Vec3d e1 = f1;
    Vec3d e3 = f1.cross(f2);
    e3 = e3/norm(e3, NORM_L2SQR);
    Vec3d e2 = e3.cross(e1);
    
    //TODO: Need to find a way to concatenate vectors to matrices in opencv
    //Matx33d T = (e1, e2, e3);
}