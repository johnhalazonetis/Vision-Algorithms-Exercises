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