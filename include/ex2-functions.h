MatrixXd quat(Vector4d& q, MatrixXd Q, MatrixXd Q_bar)
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